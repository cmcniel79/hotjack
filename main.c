/*******************************************************************************
 * File Name:   main.c
 *
 * Description: Code for drive motor pid controller
 *
 *******************************************************************************/

#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"
#include "cy_pdl.h"
#include "cycfg.h"

/*******************************************************************************
 * Macros
 *****************************************************************************/
/* Defined Ports  */
#define LEFT_ENCODER_PIN (P9_5)
#define RIGHT_ENCODER_PIN (P9_6)
#define DRIVE_MOTOR_PIN (P9_0) // only used in Device configurator
#define STEER_MOTOR_PIN (P6_2) // only used in Device configurator
#define LEFT_ECHO_PIN (P9_1)
#define CENTER_ECHO_PIN (P9_2)
#define RIGHT_ECHO_PIN (P9_3)
#define TRIGGER_PIN (P10_4) // only used in Device configurator

/* Interrupt Priorities  */
#define LEFT_ENCODER_PRIORITY (7u)
#define RIGHT_ENCODER_PRIORITY (7u)
#define CAL_BUTTON_PRIORITY (1u)
#define TRIG_PRIORITY (5u)
#define ECHO_PRIORITY (6u)

/* Encoder Constants  */
#define MAGNETS_PER_REV (5.0f)
#define TARGET_SPEED (0.5f)    // in [m/s]
#define WHEEL_RADIUS (0.0325f) // in [m]

/* Constants shared by Drive and Steering Motors PWMs */
#define MOTORS_PWM_PERIOD (20000u)       // PWM Period in microseconds = 20,000 us = 20 ms
#define MOTORS_PWM_WIDTH_NEUTRAL (1500u) // 1,500 us = 1.5 ms ---> drive motor does not run at this input
#define MOTORS_TIMER_CLOCK_HZ (50000)    // in [Hz]

/* ONLY FOR DRIVE MOTOR!! */
#define DRIVE_TIMER_PERIOD_SECONDS (0.25f) // Encoder timer period values
#define DRIVE_TIMER_PERIOD_HZ (DRIVE_TIMER_PERIOD_SECONDS * MOTORS_TIMER_CLOCK_HZ)
/* DRIVE MOTOR PWM Max and Min input for traveling forward (absolute range is 1550(min) - 1795(max) microseconds) */
#define DRIVE_PWM_PULSE_MAX_FORWARD (1590u)
#define DRIVE_PWM_PULSE_MIN_FORWARD (1555u)
/* PWM Max and Min input for traveling in reverse (absolute range is 1480(min) - 1250(max) microseconds)
    Reverse is confusing but remember that these are the max and min values with respect to the reverse speed*/
#define DRIVE_PWM_PULSE_MAX_REVERSE (1390u)
#define DRIVE_PWM_PULSE_MIN_REVERSE (1430u)

/* ONLY FOR STEERING MOTOR!! */
#define STEER_TIMER_PERIOD_SECONDS (0.1f) // Encoder timer period values
#define STEER_TIMER_PERIOD_HZ (STEER_TIMER_PERIOD_SECONDS * MOTORS_TIMER_CLOCK_HZ)
// STEERING MOTOR PWM Values for turning left and right
#define STEER_PWM_PULSE_MAX_LEFT (1750u)
#define STEER_PWM_PULSE_MAX_RIGHT (1250u)
#define STEER_PWM_PULSE_NEUTRAL_OFFSET (100u)
#define STEER_PWM_PULSE_NEUTRAL (MOTORS_PWM_WIDTH_NEUTRAL)

/* Ultrasonic Sensor Constants */
#define ECHO_TIMER_CLOCK_HZ (20000)
#define ECHO_TIMER_PERIOD_SECONDS (0.5f)
#define ECHO_TIMER_PERIOD_HZ (ECHO_TIMER_PERIOD_SECONDS * ECHO_TIMER_CLOCK_HZ)
#define SPEED_OF_SOUND_CM_PER_US (0.034f)

/* Gyroscope Constants */
#define GYRO_NUM_OF_TESTS (100)

/* Constants for robot searching */
#define SEARCH_DISTANCE_LENGTH (1.45f) // in [m]
#define DISTANCE_PER_REV (WHEEL_RADIUS * 6.2832f)
#define SEARCH_DISTANCE_MAGNET_COUNT (SEARCH_DISTANCE_LENGTH / DISTANCE_PER_REV) * MAGNETS_PER_REV;
#define REVERSE_DISTANCE_LENGTH (.82f) // in [m]
#define REVERSE_DISTANCE_MAGNET_COUNT (REVERSE_DISTANCE_LENGTH / DISTANCE_PER_REV) * MAGNETS_PER_REV;

/* LabVIEW Communication constants */
#define LABVIEW_TIMER_PERIOD_SECONDS (0.150f) // Encoder timer period values
#define LABVIEW_TIMER_PERIOD_HZ (LABVIEW_TIMER_PERIOD_SECONDS * MOTORS_TIMER_CLOCK_HZ)

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
static void motor_timers_init(void);
static void drive_timer_handler(void *handler_arg, cyhal_gpio_event_t event);
static void steer_timer_handler(void *handler_arg, cyhal_gpio_event_t event);
static void left_encoder_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
static void right_encoder_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
static void calibration_btn_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
static void ultra_trig_handler(void);
static void echo_timers_init(void);
static void left_echo_handler(void *handler_arg, cyhal_gpio_event_t event);
static void center_echo_handler(void *handler_arg, cyhal_gpio_event_t event);
static void right_echo_handler(void *handler_arg, cyhal_gpio_event_t event);
static void init_and_calibrate_gyro(void);
static void labview_timer_init(void);
static void labview_timer_handler(void *handler_arg, cyhal_gpio_event_t event);
static void send_measurements(void);

/*******************************************************************************
 * Global Variables
 ********************************************************************************/
/* For Encoders */
uint16_t left_magnet_count = 0;
uint16_t right_magnet_count = 0;
float target_magnet_count = (TARGET_SPEED / WHEEL_RADIUS) * DRIVE_TIMER_PERIOD_SECONDS * (1 / (6.28f)) * MAGNETS_PER_REV;

/* For Drive Motor */
bool drive_timer_flag = false;
cyhal_timer_t drive_timer;
uint16_t drive_motor_input = MOTORS_PWM_WIDTH_NEUTRAL;

/* For Steer Motor */
bool steer_timer_flag = false;
cyhal_timer_t steer_timer;
uint16_t steer_motor_input = MOTORS_PWM_WIDTH_NEUTRAL;

/* For ultrasonic sensors */
cyhal_timer_t left_echo_timer;
uint32_t left_echo_time;
bool left_echo_is_pulsing;
cyhal_timer_t center_echo_timer;
uint32_t center_echo_time;
bool center_echo_is_pulsing;
cyhal_timer_t right_echo_timer;
uint32_t right_echo_time;
bool right_echo_is_pulsing;

/* For ultrasonic trigger interrupt config */
cy_stc_sysint_t ISR_ultra_trig_config = {
    .intrSrc = (IRQn_Type)ultra_trig_pwm_IRQ,
    .intrPriority = TRIG_PRIORITY};

/* For gyroscope */
int16_t GX_meas, GY_meas, GZ_meas; // raw gyroscope values (sensor outputs integers!)
float GX_off, GY_off, GZ_off;      // gyroscope offset values
float GX, GY, GZ;                  // gyroscope floats
float pitch = 0;
float roll = 0;
float yaw = 0;            // current pitch, roll and yaw angles

/* For LabVIEW communications */
uint8_t uart_read_value;
cyhal_timer_t labview_timer;
bool labview_timer_flag = false;
float pixels[64];                 // declares pixel temperature grid
float ticks_per_measurement_loop;

/* Flag for setting drive motor as calibrated */
bool drive_is_calibrated = false;
bool should_stop = true;

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * Return: int
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);

    /*******************************************************************************
     * Initialize and calibrate gyroscope
     ********************************************************************************/
    init_and_calibrate_gyro();

    /*******************************************************************************
     * Initialize thermal sensor
     ********************************************************************************/
    amg8833_i2c_config_registers();

    /*******************************************************************************
     * Set up Encoder pins and their interrupts
     ********************************************************************************/
    /* ---- LEFT Encoder Pin Setup ---- */
    cyhal_gpio_init(LEFT_ENCODER_PIN, CYHAL_GPIO_DIR_INPUT,
                    CYHAL_GPIO_DRIVE_PULLUP, true);
    cyhal_gpio_register_callback(LEFT_ENCODER_PIN,
                                 left_encoder_interrupt_handler, NULL);
    cyhal_gpio_enable_event(LEFT_ENCODER_PIN, CYHAL_GPIO_IRQ_FALL,
                            LEFT_ENCODER_PRIORITY, true);

    /* ---- RIGHT Encoder Pin Setup ---- */
    cyhal_gpio_init(RIGHT_ENCODER_PIN, CYHAL_GPIO_DIR_INPUT,
                    CYHAL_GPIO_DRIVE_PULLUP, true);
    cyhal_gpio_register_callback(RIGHT_ENCODER_PIN,
                                 right_encoder_interrupt_handler, NULL);
    cyhal_gpio_enable_event(RIGHT_ENCODER_PIN, CYHAL_GPIO_IRQ_FALL,
                            RIGHT_ENCODER_PRIORITY, true);

    /*******************************************************************************
     * Set up SW2 button and interrupt for calibrating drive motor
     ********************************************************************************/
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
                    CYHAL_GPIO_DRIVE_PULLUP, true);
    cyhal_gpio_register_callback(CYBSP_USER_BTN,
                                 calibration_btn_interrupt_handler, NULL);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
                            CAL_BUTTON_PRIORITY, true);

    /*******************************************************************************
    * Set up Drive Motor PWM
    * !!!! Important !!!!
    * ---> This PWM is configured through the Device Congigurator, and that is
    *       why it uses the CY_TCPWM functions that are found in cy_pdl
    ********************************************************************************/
    Cy_TCPWM_PWM_Init(TCPWM0, drive_motor_pwm_NUM, &drive_motor_pwm_config);
    Cy_TCPWM_PWM_Enable(TCPWM0, drive_motor_pwm_NUM);
    Cy_TCPWM_PWM_SetCompare0(TCPWM0, drive_motor_pwm_NUM, MOTORS_PWM_WIDTH_NEUTRAL);
    Cy_TCPWM_TriggerStart_Single(TCPWM0, drive_motor_pwm_NUM);

    /*******************************************************************************
     * Set up Steering Motor PWM
     * !!!! Important !!!!
     * ---> This PWM is configured through the Device Congigurator, and that is
     *       why it uses the CY_TCPWM functions that are found in cy_pdl
     ********************************************************************************/
    Cy_TCPWM_PWM_Init(TCPWM0, steer_motor_pwm_NUM, &steer_motor_pwm_config);
    Cy_TCPWM_PWM_Enable(TCPWM0, steer_motor_pwm_NUM);
    Cy_TCPWM_PWM_SetCompare0(TCPWM0, steer_motor_pwm_NUM, MOTORS_PWM_WIDTH_NEUTRAL);
    Cy_TCPWM_TriggerStart_Single(TCPWM0, steer_motor_pwm_NUM);

    /*******************************************************************************
     * Set up pins for the ultrasonic echo measurements and their interrups
     ********************************************************************************/
    /* ---- LEFT Echo Pin Setup ---- */
    cyhal_gpio_init(LEFT_ECHO_PIN, CYHAL_GPIO_DIR_INPUT,
                    CYHAL_GPIO_DRIVE_NONE, CYBSP_BTN_OFF);
    cyhal_gpio_register_callback(LEFT_ECHO_PIN, left_echo_handler, NULL);
    cyhal_gpio_enable_event(LEFT_ECHO_PIN, CYHAL_GPIO_IRQ_BOTH,
                            ECHO_PRIORITY, true);

    /* ---- CENTER Echo Pin Setup ---- */
    cyhal_gpio_init(CENTER_ECHO_PIN, CYHAL_GPIO_DIR_INPUT,
                    CYHAL_GPIO_DRIVE_NONE, CYBSP_BTN_OFF);
    cyhal_gpio_register_callback(CENTER_ECHO_PIN, center_echo_handler, NULL);
    cyhal_gpio_enable_event(CENTER_ECHO_PIN, CYHAL_GPIO_IRQ_BOTH,
                            ECHO_PRIORITY, true);

    /* ---- RIGHT Echo Pin Setup ---- */
    cyhal_gpio_init(RIGHT_ECHO_PIN, CYHAL_GPIO_DIR_INPUT,
                    CYHAL_GPIO_DRIVE_NONE, CYBSP_BTN_OFF);
    cyhal_gpio_register_callback(RIGHT_ECHO_PIN, right_echo_handler, NULL);
    cyhal_gpio_enable_event(RIGHT_ECHO_PIN, CYHAL_GPIO_IRQ_BOTH,
                            ECHO_PRIORITY, true);

    /* Initialize ALL echo timers */
    echo_timers_init();

    /*******************************************************************************
     * Initialize PWM and interrupt for the trigger pulse for the ultrasonic sensors
     * !!!! Important !!!!
     * ---> This PWM is configured through the Device Congigurator, and that is
     *       why it uses the CY_TCPWM functions that are found in cy_pdl
     ********************************************************************************/
    Cy_TCPWM_PWM_Init(TCPWM0, ultra_trig_pwm_NUM, &ultra_trig_pwm_config);
    Cy_TCPWM_PWM_Enable(TCPWM0, ultra_trig_pwm_NUM);
    Cy_TCPWM_SetInterruptMask(TCPWM0, ultra_trig_pwm_NUM, CY_TCPWM_INT_ON_TC);
    Cy_SysInt_Init(&ISR_ultra_trig_config, ultra_trig_handler);
    NVIC_EnableIRQ((IRQn_Type)ISR_ultra_trig_config.intrSrc);
    Cy_TCPWM_TriggerStart_Single(TCPWM0, ultra_trig_pwm_NUM);

    /* Enable global interrupts */
    __enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    // printf("\x1b[2J\x1b[;H");
    // printf("**************** PSoC 6 MCU: PID Testing *****************\r\n");
    // printf("Target Magnet Count: %5.3f\n\r", target_magnet_count);

    /* Initialize timer for control loop */
    motor_timers_init();

    /* Initialize timer for labview */
    labview_timer_init();

    // Variables for drive motor controller
    float error_drive = 0.0f;
    float error_integral_drive = 0.0f;
    int16_t error_sum_drive = 0;

    float magnet_avg;
    float kp_drive = 5.0f;
    float ki_drive = 0.25f;

    // Variables for steering motor controller
    float error_steer = 0.0f;
    float error_integral_steer = 0.0f;
    int16_t error_sum_steer = 0;

    float kp_steer = 20.0f;
    float ki_steer = 5.0f;

    // Variables for ultrasonic sensor measurements
    float left_distance_cm = 0.0f;
    float center_distance_cm = 0.0f;
    float right_distance_cm = 0.0f;

    /* Variables for robot searching
        These conditions are set for the robot turning left first */
    bool is_next_turn_left = true;
    bool is_currently_turning_first_half = false;
    bool is_currently_turning_second_half = false;
    bool should_reverse = false;
    bool should_use_steer_PID = true;
    int travel_distance_mag_count = (int)SEARCH_DISTANCE_MAGNET_COUNT;
    int reverse_distance_mag_count = (int)REVERSE_DISTANCE_MAGNET_COUNT;

    // Variables for responding to ultrasonic sensors
    bool should_turn_right_ultra = false;
    bool should_turn_left_ultra = false;

    // Variables for responding to LabVIEW Commands
    // bool drive_is_calibrated = false;

    // Variable used by ultrasonic sensors and LabVIEW
    // bool should_stop = true;

    for (;;)
    {
        /* Check if 'Enter' key was pressed */
        if (cyhal_uart_getc(&cy_retarget_io_uart_obj, &uart_read_value, 1) 
             == CY_RSLT_SUCCESS)
        {
        	switch(uart_read_value)
				{
        		case 'A':
        			// printf("AReceived the Start Command \r\n");
                    should_stop = false;
                    drive_is_calibrated = true;
                    send_measurements();
        			break;
        		// case 'B':
        		// 	printf("BReceived the Calibrate Command\r\n");
                //     drive_is_calibrated = true;
        		// 	break;
        		// case 'C':
        		// 	printf("CReceived the Stop Command Command\r\n");
                //     should_stop = true;
        		// 	break;
        		// case 'I':
        		//     printf("IPSOC\r\n");
        		//   	break;
        		default:
        			break;
				}
		}

        if(labview_timer_flag && drive_is_calibrated) {
            send_measurements();
            ticks_per_measurement_loop = 0.0f;  // reset the number of encoder ticks to 0
            labview_timer_flag = false;
        }

        /* Check the interrupt status */
        if (drive_timer_flag && drive_is_calibrated)
        {
            /* Apply previously calculated drive motor input */
            Cy_TCPWM_PWM_SetCompare0(TCPWM0, drive_motor_pwm_NUM, drive_motor_input);

            /* Calculate error term from encoder measurements */
            magnet_avg = (left_magnet_count + right_magnet_count) / 2.0f;
            error_drive = target_magnet_count - magnet_avg;
            error_integral_drive = error_integral_drive + error_drive * DRIVE_TIMER_PERIOD_SECONDS;

            // Update travel distance magnet count with the average distance traveled in past time step
            if (!is_currently_turning_first_half && !is_currently_turning_second_half)
            {
                travel_distance_mag_count -= magnet_avg;
            }

            if (should_reverse && !is_currently_turning_first_half && !is_currently_turning_second_half)
            {
                reverse_distance_mag_count -= magnet_avg;
            }

            /* Calculate drive motor input for next time step */
            error_sum_drive = (int)(error_drive * kp_drive + error_integral_drive * ki_drive);
            drive_motor_input = !should_reverse ? DRIVE_PWM_PULSE_MIN_FORWARD + error_sum_drive : 
                DRIVE_PWM_PULSE_MIN_REVERSE - error_sum_drive;

            /* If something is very close in front of robot stop drive motor completely,
                or if the stop button has been pressed */
            if (should_stop)
            {
                drive_motor_input = MOTORS_PWM_WIDTH_NEUTRAL;
            } 

            // Perform Checks on the calculated input -> FORWARD
            else if (!should_reverse)
            {
                if (drive_motor_input > DRIVE_PWM_PULSE_MAX_FORWARD)
                {
                    drive_motor_input = DRIVE_PWM_PULSE_MAX_FORWARD;
                }
                else if (drive_motor_input < DRIVE_PWM_PULSE_MIN_FORWARD)
                {
                    drive_motor_input = DRIVE_PWM_PULSE_MIN_FORWARD;
                }
            }

            // Perform Checks on the calculated input -> REVERSE
            // Speed settings for reverse are confusing because a smaller drive motor input means faster reverse speed
            else
            {
                if (drive_motor_input > DRIVE_PWM_PULSE_MIN_REVERSE)
                {
                    drive_motor_input = DRIVE_PWM_PULSE_MIN_REVERSE;
                }
                else if (drive_motor_input < DRIVE_PWM_PULSE_MAX_REVERSE)
                {
                    drive_motor_input = DRIVE_PWM_PULSE_MAX_REVERSE;
                }
            }

            /* Reset magnet counts */
            left_magnet_count = 0;
            right_magnet_count = 0;

            /* Clear the flag */
            drive_timer_flag = false;
        }

        if (steer_timer_flag & drive_is_calibrated)
        {
            /* Apply previously calculated steering motor input */
            Cy_TCPWM_PWM_SetCompare0(TCPWM0, steer_motor_pwm_NUM, steer_motor_input);

            /* Get ultrasonics sensor measurements */
            left_distance_cm = (SPEED_OF_SOUND_CM_PER_US * left_echo_time * 25);
            center_distance_cm = (SPEED_OF_SOUND_CM_PER_US * center_echo_time * 25);
            right_distance_cm = (SPEED_OF_SOUND_CM_PER_US * right_echo_time * 25);

            /* Get yaw measurement from gyro */
            MPU6050_getRotation(&GX_meas, &GY_meas, &GZ_meas);
            GZ = ((float)GZ_meas - GZ_off) / 131.07f;    // 131.07 is just 32768/250 to get us our 1deg/sec value
            yaw = yaw + GZ * STEER_TIMER_PERIOD_SECONDS; // Rotation Angles in degrees
            // printf("Yaw: %f \n\r", yaw);

            // Set variables to start turning the first 90 degrees in either direction
            if (travel_distance_mag_count <= 0 && !should_reverse && !is_currently_turning_first_half 
                && !is_currently_turning_second_half)
            {
                is_currently_turning_first_half = true;
                steer_motor_input = is_next_turn_left ? STEER_PWM_PULSE_MAX_LEFT : STEER_PWM_PULSE_MAX_RIGHT;
                should_use_steer_PID = false;
            }

            /* CHECK CONDITIONS FOR FIRST HALF OF TURN */
            // For turning first 90 degrees left
            else if (is_currently_turning_first_half && !is_currently_turning_second_half 
                && is_next_turn_left && yaw <= 90)
            {
                steer_motor_input = STEER_PWM_PULSE_MAX_LEFT;
            }

            // For turning first 90 degrees right
            else if (is_currently_turning_first_half && !is_currently_turning_second_half 
                && !is_next_turn_left && yaw >= 90)
            {
                steer_motor_input = STEER_PWM_PULSE_MAX_RIGHT;
            }

            // Set variables to reverse when turn is halfway finished
            else if (is_currently_turning_first_half && !is_currently_turning_second_half 
                && ((is_next_turn_left && yaw >= 90) || (!is_next_turn_left && yaw <= 90)))
            {
                is_currently_turning_first_half = false;
                should_reverse = true;
                steer_motor_input = is_next_turn_left ? STEER_PWM_PULSE_MAX_RIGHT : STEER_PWM_PULSE_MAX_LEFT;
                error_integral_drive = 0.0f;
            }

            // Set variables to stop reversing
            else if (reverse_distance_mag_count <= 0 && should_reverse 
                && !is_currently_turning_first_half && !is_currently_turning_second_half)
            {
                is_currently_turning_second_half = true;
                should_reverse = false;
                steer_motor_input = is_next_turn_left ? STEER_PWM_PULSE_MAX_LEFT : STEER_PWM_PULSE_MAX_RIGHT;
                error_integral_drive = 0.0f;
            }

            /* CHECK CONDITIONS FOR SECOND HALF OF TURN */
            // For turning second 90 degrees left
            else if (!is_currently_turning_first_half && is_currently_turning_second_half
                 && is_next_turn_left && yaw <= 180)
            {
                steer_motor_input = STEER_PWM_PULSE_MAX_LEFT;
            }

            // For turning second 90 degrees right
            else if (!is_currently_turning_first_half && is_currently_turning_second_half 
                && !is_next_turn_left && yaw >= 0)
            {
                steer_motor_input = STEER_PWM_PULSE_MAX_RIGHT;
            }

            // Set variables to reset when full turn is finished
            else if (!is_currently_turning_first_half && is_currently_turning_second_half 
                && ((is_next_turn_left && yaw >= 180) || (!is_next_turn_left && yaw <= 0)))
            {
                is_currently_turning_first_half = false;
                is_currently_turning_second_half = false;
                should_reverse = false;
                travel_distance_mag_count = (int)SEARCH_DISTANCE_MAGNET_COUNT;
                reverse_distance_mag_count = (int)REVERSE_DISTANCE_MAGNET_COUNT;
                is_next_turn_left = !is_next_turn_left;
                should_use_steer_PID = true;
                error_integral_steer = 0.0f;
            }

            // Only use steer PID when going straight
            if (should_use_steer_PID && !should_turn_left_ultra && !should_stop && !should_turn_right_ultra)
            {
                float target_yaw = is_next_turn_left ? 0.0f : 180.0f;
                error_steer = target_yaw - yaw;
                error_integral_steer = error_integral_steer + error_steer * STEER_TIMER_PERIOD_SECONDS;

                /* Calculate drive motor input for next time step */
                error_sum_steer = (int)(error_steer * kp_steer + error_integral_steer * ki_steer);
                steer_motor_input = STEER_PWM_PULSE_NEUTRAL + error_sum_steer;

                if (steer_motor_input > STEER_PWM_PULSE_MAX_LEFT)
                {
                    steer_motor_input = STEER_PWM_PULSE_MAX_LEFT;
                }
                else if (steer_motor_input < STEER_PWM_PULSE_MAX_RIGHT)
                {
                    steer_motor_input = STEER_PWM_PULSE_MAX_RIGHT;
                }
            }

            // Section for responding to the ultrasonic sensors
            if ((center_distance_cm < 30.0 || left_distance_cm < 30.0 || right_distance_cm < 30.0) && !should_reverse)
            {
                steer_motor_input = MOTORS_PWM_WIDTH_NEUTRAL;
                // This flag gets used in the drive motor loop
                should_stop = true;
            }
            else if ((left_distance_cm < 90.0 || center_distance_cm < 90.0 || right_distance_cm < 90.0) && (is_next_turn_left) 
                    && !is_currently_turning_first_half && !is_currently_turning_second_half && !should_reverse)
            {
                steer_motor_input = STEER_PWM_PULSE_MAX_LEFT;
            }
            else if ((left_distance_cm < 90.0 || center_distance_cm < 90.0 || right_distance_cm < 90.0) && (!is_next_turn_left)
                    && !is_currently_turning_first_half && !is_currently_turning_second_half && !should_reverse)
            {
                steer_motor_input = STEER_PWM_PULSE_MAX_RIGHT;
            }

            /* Clear the flag */
            steer_timer_flag = false;
        }
    }
}

/* Increments count for left encoder whenever a magnet passes hall sensor */
static void left_encoder_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    (void)handler_arg;
    (void)event;
    left_magnet_count++;
    ticks_per_measurement_loop++;
}

/* Increments count for right encoder whenever a magnet passes hall sensor */
static void right_encoder_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    (void)handler_arg;
    (void)event;
    right_magnet_count++;
    ticks_per_measurement_loop++;
}

/* Initialize motor timers */
static void motor_timers_init(void)
{
    // Config for drive motor timer
    const cyhal_timer_cfg_t drive_timer_cfg =
        {
            .compare_value = 0,              /* Timer compare value, not used */
            .period = DRIVE_TIMER_PERIOD_HZ, /* Defines the timer period */
            .direction = CYHAL_TIMER_DIR_UP, /* Timer counts up */
            .is_compare = false,             /* Don't use compare mode */
            .is_continuous = true,           /* Run timer indefinitely */
            .value = 0                       /* Initial value of counter */
        };

    // Initialize drive timer at a
    cyhal_timer_init(&drive_timer, NC, NULL);
    cyhal_timer_configure(&drive_timer, &drive_timer_cfg);
    cyhal_timer_set_frequency(&drive_timer, MOTORS_TIMER_CLOCK_HZ);
    cyhal_timer_register_callback(&drive_timer, drive_timer_handler, NULL);
    cyhal_timer_enable_event(&drive_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                             7, true);
    cyhal_timer_start(&drive_timer);

    // Config for steer motor timer
    const cyhal_timer_cfg_t steer_timer_cfg =
        {
            .compare_value = 0,              /* Timer compare value, not used */
            .period = STEER_TIMER_PERIOD_HZ, /* Defines the timer period */
            .direction = CYHAL_TIMER_DIR_UP, /* Timer counts up */
            .is_compare = false,             /* Don't use compare mode */
            .is_continuous = true,           /* Run timer indefinitely */
            .value = 0                       /* Initial value of counter */
        };

    // Initialize drive timer at a
    cyhal_timer_init(&steer_timer, NC, NULL);
    cyhal_timer_configure(&steer_timer, &steer_timer_cfg);
    cyhal_timer_set_frequency(&steer_timer, MOTORS_TIMER_CLOCK_HZ);
    cyhal_timer_register_callback(&steer_timer, steer_timer_handler, NULL);
    cyhal_timer_enable_event(&steer_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                             7, true);
    cyhal_timer_start(&steer_timer);
}

/* Set drive motor timer flag to true when timer finishes count */
static void drive_timer_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    (void)handler_arg;
    (void)event;

    drive_timer_flag = true;
}

/* Set steer motor timer flag to true when timer finishes count */
static void steer_timer_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    (void)handler_arg;
    (void)event;

    steer_timer_flag = true;
}

/* Clears the trigger PWM interrupt and gets ready for the next one */
static void ultra_trig_handler(void)
{
    Cy_TCPWM_ClearInterrupt(ultra_trig_pwm_HW, ultra_trig_pwm_NUM, CY_TCPWM_INT_ON_TC);
    NVIC_ClearPendingIRQ(ISR_ultra_trig_config.intrSrc);
}

/* Initialize all of the Echo timers */
static void echo_timers_init(void)
{
    const cyhal_timer_cfg_t echo_timer_cfg =
        {
            .compare_value = 0,
            .period = ECHO_TIMER_PERIOD_HZ,
            .direction = CYHAL_TIMER_DIR_UP,
            .is_compare = false,
            .is_continuous = true,
            .value = 0};
    /* Initialize all Echo timers with the same config */

    /* LEFT Echo Init */
    cyhal_timer_init(&left_echo_timer, NC, NULL);
    cyhal_timer_configure(&left_echo_timer, &echo_timer_cfg);
    cyhal_timer_set_frequency(&left_echo_timer, ECHO_TIMER_CLOCK_HZ);

    /* CENTER Echo Init */
    cyhal_timer_init(&center_echo_timer, NC, NULL);
    cyhal_timer_configure(&center_echo_timer, &echo_timer_cfg);
    cyhal_timer_set_frequency(&center_echo_timer, ECHO_TIMER_CLOCK_HZ);

    /* RIGHT Echo Init */
    cyhal_timer_init(&right_echo_timer, NC, NULL);
    cyhal_timer_configure(&right_echo_timer, &echo_timer_cfg);
    cyhal_timer_set_frequency(&right_echo_timer, ECHO_TIMER_CLOCK_HZ);
}

/* Records time from the pin on the left ultrasonic sensor */
static void left_echo_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    (void)handler_arg;
    (void)event;
    if (left_echo_is_pulsing)
    {
        left_echo_time = cyhal_timer_read(&left_echo_timer);
        cyhal_timer_stop(&left_echo_timer);
        left_echo_is_pulsing = false;
    }
    else
    {
        cyhal_timer_reset(&left_echo_timer);
        cyhal_timer_start(&left_echo_timer);
        left_echo_is_pulsing = true;
    }
}

/* Records time from the pin on the center ultrasonic sensor */
static void center_echo_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    (void)handler_arg;
    (void)event;
    if (center_echo_is_pulsing)
    {
        center_echo_time = cyhal_timer_read(&center_echo_timer);
        cyhal_timer_stop(&center_echo_timer);
        center_echo_is_pulsing = false;
    }
    else
    {
        cyhal_timer_reset(&center_echo_timer);
        cyhal_timer_start(&center_echo_timer);
        center_echo_is_pulsing = true;
    }
}

/* Records time from the pin on the right ultrasonic sensor */
static void right_echo_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    (void)handler_arg;
    (void)event;
    if (right_echo_is_pulsing)
    {
        right_echo_time = cyhal_timer_read(&right_echo_timer);
        cyhal_timer_stop(&right_echo_timer);
        right_echo_is_pulsing = false;
    }
    else
    {
        cyhal_timer_reset(&right_echo_timer);
        cyhal_timer_start(&right_echo_timer);
        right_echo_is_pulsing = true;
    }
}

/* Initializes gyro sensor, and calibrates it with 100 tests in quick succession */
static void init_and_calibrate_gyro(void)
{
    /* Initizalize PSOC I2C Communication w/ MPU6050 */
    I2C_init();
    /* Initizalize MPU6050 */
    MPU6050_full_init();

    int i = 0; // for loop increment variable
    for (i = 0; i < GYRO_NUM_OF_TESTS; i++)
    {
        // printf("Test Number: %d \n\r", i);

        MPU6050_getRotation(&GX_meas, &GY_meas, &GZ_meas);
        GX_off += GX_meas;
        GY_off += GY_meas;
        GZ_off += GZ_meas;

        // printf(" GX:%d, GY:%d, GZ:%d,\t", GX_meas, GY_meas, GZ_meas);
        cyhal_system_delay_ms(25);
    }
    GX_off = GX_off / GYRO_NUM_OF_TESTS;
    GY_off = GY_off / GYRO_NUM_OF_TESTS;
    GZ_off = GZ_off / GYRO_NUM_OF_TESTS;

    // printf("\n\nTest finished, offset values are shown below: \n\n\r");
    // printf("GXoff:%d, GYoff:%d, GZoff:%d\t", (int)GX_off, (int)GY_off, (int)GZ_off);
}

/* Initialize motor timers */
static void labview_timer_init(void)
{
    // Config for drive motor timer
    const cyhal_timer_cfg_t labview_timer_cfg =
        {
            .compare_value = 0,              /* Timer compare value, not used */
            .period = LABVIEW_TIMER_PERIOD_HZ, /* Defines the timer period */
            .direction = CYHAL_TIMER_DIR_UP, /* Timer counts up */
            .is_compare = false,             /* Don't use compare mode */
            .is_continuous = true,           /* Run timer indefinitely */
            .value = 0                       /* Initial value of counter */
        };

    // Initialize
    cyhal_timer_init(&labview_timer, NC, NULL);
    cyhal_timer_configure(&labview_timer, &labview_timer_cfg);
    cyhal_timer_set_frequency(&labview_timer, MOTORS_TIMER_CLOCK_HZ);
    cyhal_timer_register_callback(&labview_timer, labview_timer_handler, NULL);
    cyhal_timer_enable_event(&labview_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                             7, true);
    cyhal_timer_start(&labview_timer);
}

/* Set labview timer flag to true when timer finishes count */
static void labview_timer_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    (void)handler_arg;
    (void)event;

    labview_timer_flag = true;
}

/* Sets calibration flag to true when SW2 button is pressed */
static void calibration_btn_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    (void)handler_arg;
    (void)event;
    drive_is_calibrated = true;
    should_stop = false;
}

/* Sets calibration flag to true when SW2 button is pressed */
static void send_measurements(void)
{              
            float steer_motor_input_float = (float) steer_motor_input;
            float drive_motor_input_float = (float) drive_motor_input;
            // Printing the pixel temperatures (raw) to serial
            amg8833_i2c_8x8_read(pixels);
			printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f ", pixels[0],  pixels[1],  pixels[2],  pixels[3],  pixels[4],  pixels[5],  pixels[6],  pixels[7]);
			printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f ", pixels[8],  pixels[9],  pixels[10], pixels[11], pixels[12], pixels[13], pixels[14], pixels[15]);
			printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f ", pixels[16], pixels[17], pixels[18], pixels[19], pixels[20], pixels[21], pixels[22], pixels[23]);
			printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f ", pixels[24], pixels[25], pixels[26], pixels[27], pixels[28], pixels[29], pixels[30], pixels[31]);
			printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f ", pixels[32], pixels[33], pixels[34], pixels[35], pixels[36], pixels[37], pixels[38], pixels[39]);
			printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f ", pixels[40], pixels[41], pixels[42], pixels[43], pixels[44], pixels[45], pixels[46], pixels[47]);
			printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f ", pixels[48], pixels[49], pixels[50], pixels[51], pixels[52], pixels[53], pixels[54], pixels[55]);
			printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f ", pixels[56], pixels[57], pixels[58], pixels[59], pixels[60], pixels[61], pixels[62], pixels[63]);  // prints out the individual pixel temperature readings

			printf("%.2f ", yaw); // transmits the yaw value to serial appended after the 64x temperature values
			printf("%.2f ", ticks_per_measurement_loop); // transmits the encoder data for speed value to serial appended after the 64x temperature values and gyro heading
			printf("%.2f ", steer_motor_input_float); // transmits the current steering motor input to serial appended after the 64x temps, gyro heading, and encoder data
			printf("%.2f\r\n\n", drive_motor_input_float);
}

/* [] END OF FILE */
