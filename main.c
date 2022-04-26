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
            timer_interrupt_flag = false;
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