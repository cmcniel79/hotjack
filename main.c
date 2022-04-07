/*******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PWM Square Wave code example
 *              for ModusToolbox.
 *
 * Related Document: See README.md
 *******************************************************************************/

#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"

/*******************************************************************************
 * Macros
 *******************************************************************************/
#define NUM_OF_TESTS 100

/* Gyroscope timer values */
#define GYRO_TIMER_CLOCK_HZ          (10000)
#define GYRO_TIMER_PERIOD_SECONDS    (0.1f)
#define GYRO_TIMER_PERIOD_HZ         (GYRO_TIMER_PERIOD_SECONDS * GYRO_TIMER_CLOCK_HZ)
#define GYRO_TIMER_PERIOD_INVERSE    (1 / GYRO_TIMER_PERIOD_SECONDS)

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
void gyro_timer_init(void);
static void gyro_isr_timer(void *callback_arg, cyhal_timer_event_t event);

/*******************************************************************************
 * Global Variables
 *******************************************************************************/
int16_t GX_meas, GY_meas, GZ_meas; // raw gyroscope values (sensor outputs integers!)
float GX_off, GY_off, GZ_off;      // gyroscope offset values
float GX, GY, GZ;                  // gyroscope floats
float pitch, roll, yaw;            // current pitch, roll and yaw angles

/* Timer flag and object used for setting when rotation is measured */
bool gyro_timer_interrupt_flag = false;
cyhal_timer_t gyro_timer;

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * This is the main function for the CPU. It configures the PWM and puts the CPU
 * in Sleep mode to save power.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    /* API return code */
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (CY_RSLT_SUCCESS != result)
    {
        /* Halt the CPU while debugging */
        CY_ASSERT(false);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    if (CY_RSLT_SUCCESS != result)
    {
        /* Halt the CPU while debugging */
        CY_ASSERT(false);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("****************** HotJACK MPU6050 Testing ******************\r\n\n");

    /* Initizalize PSOC I2C Communication w/ MPU6050 */
    I2C_MPU6050_init();
    /* Initizalize MPU6050 */
    MPU6050_full_init();

    int i = 0; // for loop increment variable
    for (i = 0; i < NUM_OF_TESTS; i++)
    {
        printf("Test Number: %d \n\r", i);

        MPU6050_getRotation(&GX_meas, &GY_meas, &GZ_meas);
        GX_off += GX_meas;
        GY_off += GY_meas;
        GZ_off += GZ_meas;

        printf(" GX:%d, GY:%d, GZ:%d,\t", GX, GY, GZ);
        cyhal_system_delay_ms(25);
    }
    GX_off = GX_off / NUM_OF_TESTS;
    GY_off = GY_off / NUM_OF_TESTS;
    GZ_off = GZ_off / NUM_OF_TESTS;

    printf("\n\nTest finished, offset values are shown below: \n\n\r");
    printf("GXoff:%d, GYoff:%d, GZoff:%d\t", (int)GX_off, (int)GY_off, (int)GZ_off);

    /* Initialize gyro timer */
    gyro_timer_init();

    /* Loop infinitely */
    for (;;)
    {
        /* Check the interrupt status */
        if (gyro_timer_interrupt_flag)
        {
            MPU6050_getRotation(&GX_meas, &GY_meas, &GZ_meas);
            /* 131.07 is just 32768/250 to get us our 1deg/sec value */
            GX = ((float)GX_meas - GX_off) / 131.07f;
            GY = ((float)GY_meas - GY_off) / 131.07f;
            GZ = ((float)GZ_meas - GZ_off) / 131.07f;
            /* Rotation Angles in degrees */
            pitch = pitch + GX * GYRO_TIMER_PERIOD_SECONDS;
            roll = roll + GY * GYRO_TIMER_PERIOD_SECONDS;
            yaw = yaw + GZ * GYRO_TIMER_PERIOD_SECONDS;

            printf("Roll: %5.2f, Pitch: %5.2f, Yaw: %5.2f\n\r", pitch, roll, yaw);

            /* Clear the flag */
            gyro_timer_interrupt_flag = false;
        }
    }
}

/*******************************************************************************
 * Function Name: gyro_timer_init
 ********************************************************************************
 * Summary:
 * This function creates and configures a Timer object. The timer ticks
 * continuously and produces a periodic interrupt on every terminal count
 * event. The period is defined by the 'period' and 'compare_value' of the
 * timer configuration structure 'led_blink_timer_cfg'. Without any changes,
 * this application is designed to produce an interrupt every 1 second.
 *
 * Parameters:
 *  none
 *
 *******************************************************************************/
void gyro_timer_init(void)
{
    cy_rslt_t result;

    const cyhal_timer_cfg_t gyro_timer_cfg =
        {
            .compare_value = 0,              /* Timer compare value, not used */
            .period = GYRO_TIMER_PERIOD_HZ,  /* Defines the timer period */
            .direction = CYHAL_TIMER_DIR_UP, /* Timer counts up */
            .is_compare = false,             /* Don't use compare mode */
            .is_continuous = true,           /* Run timer indefinitely */
            .value = 0                       /* Initial value of counter */
        };

    /* Initialize the timer object. Does not use input pin ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    result = cyhal_timer_init(&gyro_timer, NC, NULL);

    /* timer init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure timer period and operation mode such as count direction,
       duration */
    cyhal_timer_configure(&gyro_timer, &gyro_timer_cfg);

    /* Set the frequency of timer's clock source */
    cyhal_timer_set_frequency(&gyro_timer, GYRO_TIMER_CLOCK_HZ);

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&gyro_timer, gyro_isr_timer, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&gyro_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 7, true);

    /* Start the timer with the configured settings */
    cyhal_timer_start(&gyro_timer);
}

/*******************************************************************************
 * Function Name: gyro_isr_timer
 ********************************************************************************
 * Summary:
 * This is the interrupt handler function for the timer interrupt.
 *
 * Parameters:
 *    callback_arg    Arguments passed to the interrupt callback
 *    event            Timer/counter interrupt triggers
 *
 *******************************************************************************/
static void gyro_isr_timer(void *callback_arg, cyhal_timer_event_t event)
{
    (void)callback_arg;
    (void)event;

    /* Set the interrupt flag and process it from the main while(1) loop */
    gyro_timer_interrupt_flag = true;
}

/* [] END OF FILE */
