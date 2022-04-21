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
#define DRIVE_MOTOR_PIN (P9_0)
#define CENTER_ECHO_PIN (P10_2)

/* Interrupt Priorities  */
#define LEFT_ENCODER_PRIORITY (7u)
#define RIGHT_ENCODER_PRIORITY (7u)
#define CAL_BUTTON_PRIORITY (1u)
#define TRIG_PRIORITY (5u)
#define ECHO_PRIORITY (6u)

/* Encoder Constants  */
#define MAGNETS_PER_REV (5.0f)
#define TARGET_SPEED (0.5f)            // in [m/s]
#define WHEEL_RADIUS (0.033f)          // in [m]
#define ENCODER_TIMER_CLOCK_HZ (50000) // in [Hz]

/* Encoder timer period values */
#define ENCODER_TIMER_PERIOD_SECONDS (0.25f)
#define ENCODER_TIMER_PERIOD_HZ (ENCODER_TIMER_PERIOD_SECONDS * ENCODER_TIMER_CLOCK_HZ)

/* PWM Period in microseconds = 20,000 us = 20 ms */
#define PWM_PERIOD (20000u)
#define PWM_PULSE_WIDTH_ZERO_SPEED (1500u)  // 1,500 us = 1.5 ms ---> drive motor does not run at this input

/* PWM Max and Min input for traveling forward (absolute range is 1550(min) - 1795(max) microseconds)*/
#define PWM_PULSE_WIDTH_MAX_FORWARD (1620u)
#define PWM_PULSE_WIDTH_MIN_FORWARD (1550u)

/* PWM Max and Min input for traveling in reverse (absolute range is 1480(min) - 1250(max) microseconds)*/
#define PWM_PULSE_WIDTH_MAX_REVERSE (1420u)
#define PWM_PULSE_WIDTH_MIN_REVERSE (1490u)

#define ECHO_TIMER_CLOCK_HZ (20000)
#define ECHO_TIMER_PERIOD_SECONDS (0.5f)
#define ECHO_TIMER_PERIOD_HZ (ECHO_TIMER_PERIOD_SECONDS * ECHO_TIMER_CLOCK_HZ)

#define SPEED_OF_SOUND_CM_PER_US (0.034f)

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
static void pid_timer_init(void);
static void pid_timer_handler(void *handler_arg, cyhal_gpio_event_t event);
static void left_encoder_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
static void right_encoder_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
static void calibration_btn_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
static void ultra_trig_handler(void);
static void echo_timers_init(void);
static void center_echo_handler(void *handler_arg, cyhal_gpio_event_t event);
/*******************************************************************************
 * Global Variables
 ********************************************************************************/
bool pid_timer_flag = false;
bool drive_is_calibrated = false;
uint16_t left_magnet_count = 0;
uint16_t right_magnet_count = 0;
float target_magnet_count = (TARGET_SPEED / WHEEL_RADIUS) * ENCODER_TIMER_PERIOD_SECONDS * (1 / (6.28f)) * MAGNETS_PER_REV;

/* Drive Motor Input command, defined in microseconds */
uint16_t drive_motor_input = PWM_PULSE_WIDTH_ZERO_SPEED;

/* Timer object used for setting when motor input is applied */
cyhal_timer_t pid_timer;

/* PWM object for drive motor */
cyhal_pwm_t drive_motor_pwm;

cy_stc_sysint_t ISR_ultra_trig_config = {
    .intrSrc = (IRQn_Type) ultra_trig_pwm_IRQ,
    .intrPriority = TRIG_PRIORITY
};

/* Timer object used for recording Echo pulse time */
cyhal_timer_t center_echo_timer;
uint32_t center_echo_time;
bool center_echo_is_pulsing;


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

    /* Initialize pin P0_5 for LEFT encoder*/
    cyhal_gpio_init(LEFT_ENCODER_PIN, CYHAL_GPIO_DIR_INPUT,
                             CYHAL_GPIO_DRIVE_PULLUP, true);
    /* Initialize pin P0_6 for RIGHT encoder*/
    cyhal_gpio_init(RIGHT_ENCODER_PIN, CYHAL_GPIO_DIR_INPUT,
                             CYHAL_GPIO_DRIVE_PULLUP, true);

    /* Configure LEFT Encoder GPIO interrupt */
    cyhal_gpio_register_callback(LEFT_ENCODER_PIN,
                                 left_encoder_interrupt_handler, NULL);

    /* Configure RIGHT Encoder GPIO interrupt */
    cyhal_gpio_register_callback(RIGHT_ENCODER_PIN,
                                 right_encoder_interrupt_handler, NULL);

    /* Enable LEFT Encoder GPIO interrupt event when signal goes from HIGH to LOW*/
    cyhal_gpio_enable_event(LEFT_ENCODER_PIN, CYHAL_GPIO_IRQ_FALL,
                            LEFT_ENCODER_PRIORITY, true);

    /* Enable RIGHT Encoder GPIO interrupt event when signal goes from HIGH to LOW*/
    cyhal_gpio_enable_event(RIGHT_ENCODER_PIN, CYHAL_GPIO_IRQ_FALL,
                            RIGHT_ENCODER_PRIORITY, true);

    /* Configure SW2 Button for calibration */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
                              CYHAL_GPIO_DRIVE_PULLUP, true);

    /* Configure SW2 Button interrupt */
    cyhal_gpio_register_callback(CYBSP_USER_BTN,
                                 calibration_btn_interrupt_handler, NULL);

    /* Enable SW2 Button interrupt event when signal goes from LOW to HIGH*/
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
    						CAL_BUTTON_PRIORITY, true);

    /* Initialize the Drive Motor PWM */
    result = cyhal_pwm_init(&drive_motor_pwm, DRIVE_MOTOR_PIN, NULL);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("API cyhal_pwm_init failed with error code: %lu\r\n", (unsigned long)result);
        CY_ASSERT(false);
    }
    /* Set the PWM period and pulsewidth */
    result = cyhal_pwm_set_period(&drive_motor_pwm, PWM_PERIOD, drive_motor_input);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("API cyhal_pwm_set_duty_cycle failed with error code: %lu\r\n", (unsigned long)result);
        CY_ASSERT(false);
    }
    result = cyhal_pwm_start(&drive_motor_pwm);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("API cyhal_pwm_start failed with error code: %lu\r\n", (unsigned long)result);
        CY_ASSERT(false);
    }

    /* ----ECHO PINs SETUP---- */
    /* Set up echo pin with interrupts */
    result = cyhal_gpio_init(CENTER_ECHO_PIN, CYHAL_GPIO_DIR_INPUT,
                        CYHAL_GPIO_DRIVE_NONE, CYBSP_BTN_OFF);

    /* Set up inerrupt handler for the echo signal */
    cyhal_gpio_register_callback(CENTER_ECHO_PIN, center_echo_handler, NULL);

    /* Set up interrupt for both rising and falling of the echo signal */
    cyhal_gpio_enable_event(CENTER_ECHO_PIN, CYHAL_GPIO_IRQ_BOTH,
                            ECHO_PRIORITY, true);

    /* Initialize echo timer */
    echo_timers_init();

    Cy_TCPWM_PWM_Init(TCPWM0, ultra_trig_pwm_NUM, &ultra_trig_pwm_config);
    Cy_TCPWM_PWM_Enable(TCPWM0, ultra_trig_pwm_NUM);
    Cy_TCPWM_SetInterruptMask(TCPWM0, ultra_trig_pwm_NUM, CY_TCPWM_INT_ON_TC);
    Cy_SysInt_Init(&ISR_ultra_trig_config, ultra_trig_handler);
    NVIC_EnableIRQ((IRQn_Type)ISR_ultra_trig_config.intrSrc);
    Cy_TCPWM_TriggerStart_Single(TCPWM0, ultra_trig_pwm_NUM);


    /* Enable global interrupts */
    __enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("**************** PSoC 6 MCU: PID Testing *****************\r\n");
    printf("Target: %5.3f\n\r", target_magnet_count);

    /* Initialize timer for control loop */
    pid_timer_init();

    float error = 0.0f;
    float error_integral = 0.0f;
    int16_t error_sum = 0;

    float magnet_avg;
    float kp = 1.0f;
    float ki = 0.05f;

    float distance_cm = 0.0f;

    for (;;)
    {
        /* Check the interrupt status */
        if (pid_timer_flag && drive_is_calibrated)
        {
            /* Apply previously calculated drive motor input */
             result = cyhal_pwm_set_period(&drive_motor_pwm, PWM_PERIOD, drive_motor_input);

            /* Calculate error term from encoder measurements */
            magnet_avg = (left_magnet_count + right_magnet_count) / 2.0f;
            error = target_magnet_count - magnet_avg;
            error_integral = error_integral + error * ENCODER_TIMER_PERIOD_SECONDS;

            /* Calculate drive motor input for next time step */
            error_sum = (int)(error * kp + error_integral * ki);
            drive_motor_input = PWM_PULSE_WIDTH_MIN_FORWARD + error_sum;

            /*Perform Checks on the calculated input */
            if (drive_motor_input > PWM_PULSE_WIDTH_MAX_FORWARD)
            {
                drive_motor_input = PWM_PULSE_WIDTH_MAX_FORWARD;
            }

            else if (drive_motor_input < PWM_PULSE_WIDTH_MIN_FORWARD)
            {
                drive_motor_input = PWM_PULSE_WIDTH_MIN_FORWARD;
            }

            printf("Left: %5u\n\r", left_magnet_count);
        	printf("Right: %5u\n\r", right_magnet_count);
            printf("Error: %5i\n\r", error_sum);
        	printf("Motor Input: %5u\n\r", drive_motor_input);

            /* Reset magnet counts */
            left_magnet_count = 0;
            right_magnet_count = 0;

            /* Clear the flag */
            pid_timer_flag = false;
        }
            
        distance_cm = (SPEED_OF_SOUND_CM_PER_US * center_echo_time * 100);
        if (distance_cm < 40.0) {
            printf("Distance in: %3.1f cm \r\n", distance_cm);
        }
    }
}

/* Increments count for left encoder whenever a magnet passes hall sensor */
static void left_encoder_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    (void)handler_arg;
    (void)event;
    left_magnet_count++;
}

/* Increments count for right encoder whenever a magnet passes hall sensor */
static void right_encoder_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    (void)handler_arg;
    (void)event;
    right_magnet_count++;
}

/* Sets calibration flag to true when SW2 button is pressed */
static void calibration_btn_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    (void)handler_arg;
    (void)event;
	drive_is_calibrated = true;
}

/*******************************************************************************
 * Function Name: pid_timer_init
 *******************************************************************************/
static void pid_timer_init(void)
{
    cy_rslt_t result;

    const cyhal_timer_cfg_t pid_timer_cfg =
        {
            .compare_value = 0,                /* Timer compare value, not used */
            .period = ENCODER_TIMER_PERIOD_HZ, /* Defines the timer period */
            .direction = CYHAL_TIMER_DIR_UP,   /* Timer counts up */
            .is_compare = false,               /* Don't use compare mode */
            .is_continuous = true,             /* Run timer indefinitely */
            .value = 0                         /* Initial value of counter */
        };

    /* Initialize the timer object. Does not use input pin ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    result = cyhal_timer_init(&pid_timer, NC, NULL);

    /* timer init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure timer period and operation mode such as count direction,
       duration */
    cyhal_timer_configure(&pid_timer, &pid_timer_cfg);

    /* Set the frequency of timer's clock source */
    cyhal_timer_set_frequency(&pid_timer, ENCODER_TIMER_CLOCK_HZ);

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&pid_timer, pid_timer_handler, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&pid_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                             7, true);

    /* Start the timer with the configured settings */
    cyhal_timer_start(&pid_timer);
}

/*******************************************************************************
 * Function Name: pid_timer_handler
 *******************************************************************************/
static void pid_timer_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    (void)handler_arg;
    (void)event;

    /* Set the interrupt flag and process it from the main while(1) loop */
    pid_timer_flag = true;
}

/*******************************************************************************
 * Function Name: ultra_trig_handler
 *******************************************************************************/
static void ultra_trig_handler(void)
{
    Cy_TCPWM_ClearInterrupt(ultra_trig_pwm_HW, ultra_trig_pwm_NUM, CY_TCPWM_INT_ON_TC);
    NVIC_ClearPendingIRQ(ISR_ultra_trig_config.intrSrc);
}

/*******************************************************************************
 * Function Name: echo_timers_init
 *******************************************************************************/
static void echo_timers_init(void)
{
    cy_rslt_t result;

    const cyhal_timer_cfg_t echo_timer_cfg =
        {
            .compare_value = 0,              
            .period = ECHO_TIMER_PERIOD_HZ,  
            .direction = CYHAL_TIMER_DIR_UP, 
            .is_compare = false,             
            .is_continuous = true,          
            .value = 0                       
        };

    result = cyhal_timer_init(&center_echo_timer, NC, NULL);

    /* timer init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    cyhal_timer_configure(&center_echo_timer, &echo_timer_cfg);
    cyhal_timer_set_frequency(&center_echo_timer, ECHO_TIMER_CLOCK_HZ);
}

/*******************************************************************************
 * Function Name: center_echo_handler
 *******************************************************************************/
static void center_echo_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    (void)handler_arg;
    (void)event;
    if(center_echo_is_pulsing) {
        center_echo_time = cyhal_timer_read(&center_echo_timer);
        cyhal_timer_stop(&center_echo_timer);
        center_echo_is_pulsing = false;
    } else {
        cyhal_timer_reset(&center_echo_timer);
        cyhal_timer_start(&center_echo_timer);
        center_echo_is_pulsing = true;
    }
}

/* [] END OF FILE */
