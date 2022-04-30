/******************************************************************************
 * File Name:   main.c
 *
 * Description: Test code for all of the functions for the AMG8833
 *
 * Related Document: See README.md */

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"


/******************************************************************************
 * Macros
 *****************************************************************************/
/* Timer clock value in Hz  */
#define TIMER_CLOCK_HZ (10000)

/* Timer period value */
#define TIMER_PERIOD_SECONDS (1.0f)
#define TIMER_PERIOD_HZ (TIMER_PERIOD_SECONDS * TIMER_CLOCK_HZ)
#define TIMER_PERIOD_INVERSE (1 / TIMER_PERIOD_SECONDS)

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
#define DRIVE_PWM_PULSE_MIN_FORWARD (1550u)
/* PWM Max and Min input for traveling in reverse (absolute range is 1480(min) - 1250(max) microseconds)
    Reverse is confusing but remember that these are the max and min values with respect to the reverse speed*/
#define DRIVE_PWM_PULSE_MAX_REVERSE (1400u)
#define DRIVE_PWM_PULSE_MIN_REVERSE (1440u)

/* ONLY FOR STEERING MOTOR!! */
#define STEER_TIMER_PERIOD_SECONDS (0.1f) // Encoder timer period values
#define STEER_TIMER_PERIOD_HZ (STEER_TIMER_PERIOD_SECONDS * MOTORS_TIMER_CLOCK_HZ)
// STEERING MOTOR PWM Values for turning left and right
#define STEER_PWM_PULSE_MAX_LEFT (1750u)
#define STEER_PWM_PULSE_MAX_RIGHT (1250u)
#define STEER_PWM_PULSE_NEUTRAL_OFFSET (100u)
#define STEER_PWM_PULSE_NEUTRAL (MOTORS_PWM_WIDTH_NEUTRAL + STEER_PWM_PULSE_NEUTRAL_OFFSET)

/* Ultrasonic Sensor Constants */
#define ECHO_TIMER_CLOCK_HZ (20000)
#define ECHO_TIMER_PERIOD_SECONDS (0.5f)
#define ECHO_TIMER_PERIOD_HZ (ECHO_TIMER_PERIOD_SECONDS * ECHO_TIMER_CLOCK_HZ)
#define SPEED_OF_SOUND_CM_PER_US (0.034f)

/* Gyroscope Constants */
#define GYRO_NUM_OF_TESTS (100)

/* Constants for robot taks */
#define SEARCH_DISTANCE_LENGTH (1.45f) // in [m]
#define DISTANCE_PER_REV (WHEEL_RADIUS * 6.2832f)
#define SEARCH_DISTANCE_MAGNET_COUNT (SEARCH_DISTANCE_LENGTH / DISTANCE_PER_REV) * MAGNETS_PER_REV;
#define REVERSE_DISTANCE_LENGTH (.62f) // in [m]
#define REVERSE_DISTANCE_MAGNET_COUNT (REVERSE_DISTANCE_LENGTH / DISTANCE_PER_REV) * MAGNETS_PER_REV;
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

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void timer_init(void);
static void isr_timer(void *callback_arg, cyhal_timer_event_t event);
// static void init_and_calibrate_gyro(void);

/*******************************************************************************
* Global Variables
********************************************************************************/
bool timer_interrupt_flag = false;
/* Timer object used for setting when speed is measured */
cyhal_timer_t timer;

/* For gyroscope */
// int16_t GX_meas, GY_meas, GZ_meas; // raw gyroscope values (sensor outputs integers!)
// float GX_off, GY_off, GZ_off;      // gyroscope offset values
// float GX, GY, GZ;                  // gyroscope floats
// float pitch, roll, yaw;            // current pitch, roll and yaw angles

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  Test code for all of the functions for the AMG8833
 *
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

    float pixels[64];

    /* Initializes I2C communicaiton for PSoC 6 -> from the i2cFunctions File*/
    I2C_init();

    // Set up pixel registers
    amg8833_i2c_config_registers();

    /* Code to test the thermistor reading function */
    printf("Thermistor Value (deg C): %.2f\r\n\n", amg8833_i2c_thermistor_read());

    /* Code to test the frame rate set and read function */
    printf("Frame Rate (Hz): %i\r\n\n", amg8833_i2c_frame_rate_set(0)); // accepts an 8-bit integer where the 1st bit determines the frame rate (1 = 1Hz; 0 = 10Hz)
    
    // init_and_calibrate_gyro();
    timer_init();

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
        /* Check the interrupt status */
        if (timer_interrupt_flag)
        {
            /* Code to test the 8x8 pixel temperature readings every 1 second */
            // amg8833_i2c_8x8_read(pixels);

            
            // MPU6050_getRotation(&GX_meas, &GY_meas, &GZ_meas);
            // /* 131.07 is just 32768/250 to get us our 1deg/sec value */
            // GX = ((float)GX_meas - GX_off) / 131.07f;
            // GY = ((float)GY_meas - GY_off) / 131.07f;
            // GZ = ((float)GZ_meas - GZ_off) / 131.07f;
            // /* Rotation Angles in degrees */
            // pitch = pitch + GX * TIMER_PERIOD_SECONDS;
            // roll = roll + GY * TIMER_PERIOD_SECONDS;
            // yaw = yaw + GZ * TIMER_PERIOD_SECONDS;

            // printf("Pitch: %5.2f, Roll: %5.2f, Yaw: %5.2f\n\r", pitch, roll, yaw);

            amg8833_i2c_8x8_read(pixels);

            /* Clear the flag */
            drive_timer_flag = false;
        }

        if (steer_timer_flag)
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
            if (should_use_steer_PID && !should_turn_left_ultra && !should_stop_ultra && !should_turn_right_ultra)
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
            if ((center_distance_cm < 30.0 || left_distance_cm < 15.0 || right_distance_cm < 15.0) && !should_reverse)
            {
                steer_motor_input = MOTORS_PWM_WIDTH_NEUTRAL;
                // This flag gets used in the drive motor loop
                should_stop_ultra = true;
            }
            else if ((center_distance_cm < 50.0 || right_distance_cm < 50.0) && (is_next_turn_left && left_distance_cm > 50.0) 
                    && !is_currently_turning_first_half && !is_currently_turning_second_half && !should_reverse)
            {
                steer_motor_input = STEER_PWM_PULSE_MAX_LEFT;
            }
            else if ((center_distance_cm < 50.0 || left_distance_cm < 50.0) && (!is_next_turn_left && right_distance_cm > 50.0)
                    && !is_currently_turning_first_half && !is_currently_turning_second_half && !should_reverse)
            {
                steer_motor_input = STEER_PWM_PULSE_MAX_RIGHT;
            }

            /* Clear the flag */
            steer_timer_flag = false;
        }
    }
}

void timer_init(void)
{
    cy_rslt_t result;

    const cyhal_timer_cfg_t timer_cfg =
        {
            .compare_value = 0,              /* Timer compare value, not used */
            .period = TIMER_PERIOD_HZ,       /* Defines the timer period */
            .direction = CYHAL_TIMER_DIR_UP, /* Timer counts up */
            .is_compare = false,             /* Don't use compare mode */
            .is_continuous = true,           /* Run timer indefinitely */
            .value = 0                       /* Initial value of counter */
        };

    /* Initialize the timer object. Does not use input pin ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    result = cyhal_timer_init(&timer, NC, NULL);

    /* timer init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure timer period and operation mode such as count direction,
       duration */
    cyhal_timer_configure(&timer, &timer_cfg);

    /* Set the frequency of timer's clock source */
    cyhal_timer_set_frequency(&timer, TIMER_CLOCK_HZ);

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&timer, isr_timer, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                             7, true);

    /* Start the timer with the configured settings */
    cyhal_timer_start(&timer);
}

static void isr_timer(void *callback_arg, cyhal_timer_event_t event)
{
    (void)callback_arg;
    (void)event;

    /* Set the interrupt flag and process it from the main while(1) loop */
    timer_interrupt_flag = true;
}

// static void init_and_calibrate_gyro(void)
// {
//     /* Initizalize MPU6050 */
//     MPU6050_full_init();

//     for (uint8_t i = 0; i < 100; i++)
//     {
//         printf("Test Number: %d \n\r", i);

//         MPU6050_getRotation(&GX_meas, &GY_meas, &GZ_meas);
//         GX_off += GX_meas;
//         GY_off += GY_meas;
//         GZ_off += GZ_meas;

//         printf(" GX:%d, GY:%d, GZ:%d,\t", GX_meas, GY_meas, GZ_meas);
//         cyhal_system_delay_ms(25);
//     }
//     GX_off = GX_off / 100;
//     GY_off = GY_off / 100;
//     GZ_off = GZ_off / 100;

//     printf("\n\nTest finished, offset values are shown below: \n\n\r");
//     printf("GXoff:%d, GYoff:%d, GZoff:%d\t", (int)GX_off, (int)GY_off, (int)GZ_off);
// }