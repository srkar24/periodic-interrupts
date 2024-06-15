/**
 * @file main.c
 * @brief Main source code for the Periodic Interrupts program.
 *
 * This file contains the main entry point and function definitions for the Periodic Interrupts program.
 * It interfaces with the following:
 *  - User LEDs of the TI MSP432 LaunchPad
 *  - 8-Channel QTRX Sensor Array module
 *
 *  It uses the following timer modules to demonstrate periodic interrupts.
 *  Each timer is configured to generate an interrupt every 1 ms.
 *  - SysTick
 *  - Timer A0
 *  - Timer A1
 *  - Timer A2
 *
 * To verify the pinout of the user buttons and LEDs, refer to the MSP432P401R SimpleLink Microcontroller LaunchPad Development Kit User's Guide
 * Link: https://docs.rs-online.com/3934/A700000006811369.pdf
 *
 * @author Aaron Nanas, Srushti Wadekar, Arshia P
 */

#include <stdint.h>
#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/GPIO.h"
#include "inc/Reflectance_Sensor.h"
#include "inc/Print_Binary.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Timer_A0_Interrupt.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/Timer_A2_Interrupt.h"

// Initialize a global variable for SysTick and Timer A to keep track of elapsed time in milliseconds
uint32_t SysTick_ms_elapsed = 0;
uint32_t Timer_A0_ms_elapsed = 0;
uint32_t Timer_A1_ms_elapsed = 0;
uint32_t Timer_A2_ms_elapsed = 0;

/**
 * @brief Toggles P8.0 every 500ms
 *
 * @param None
 *
 * @return None
 */
void SysTick_Handler(void)
{
    SysTick_ms_elapsed++;
    if (SysTick_ms_elapsed >= 500)
    {
        P8->OUT ^= 0x01;
        SysTick_ms_elapsed = 0;
    }
}

/**
 * @brief Toggles P8.5 every second
 *
 * @param None
 *
 * @return None
 */
void Timer_A0_Periodic_Task(void)
{
    Timer_A0_ms_elapsed++;
    if (Timer_A0_ms_elapsed >= 1000)
    {
        P8->OUT ^= 0x20;
        Timer_A0_ms_elapsed = 0;
    }
}

/* Timer_A1_Periodic_Task
 * @brief Samples the Reflectance Sensor Array at every periodic interrupt.
 *
 * This function performs the following steps:
 * 1. Increments Timer_A1_ms_elapsed by 1.
 * 2. Executes Reflectance_Sensor_Start every 10ms.
 * 3. Stores Reflectance_Sensor_End() in Reflectance_Sensor_Data 1ms after Reflectance_Sensor_Start is executed.
 * 4. Calls Detect_Line_Position with Reflectance_Sensor_Data as argument 1ms after executing Reflectance_Sensor_Start.
 * 5. Calls Print_Binary with Reflectance_Sensor_Data as argument 1ms after executing Reflectance_Sensor_Start.
 *
 * @param None
 *
 * @return None
 */
void Timer_A1_Periodic_Task(void)
{
    // Your implementation for the tasks will go here
    Timer_A1_ms_elapsed++;
        /*if (Timer_A1_ms_elapsed >= 200)
        {
            P8->OUT ^= 0x40;
            Timer_A1_ms_elapsed = 0;
        }*/
        if (Timer_A1_ms_elapsed % 10 == 0){
            Reflectance_Sensor_Start();
        }
        if (Timer_A1_ms_elapsed % 10 == 1){
            uint8_t Reflectance_Sensor_Data = Reflectance_Sensor_End();
            Detect_Line_Position(Reflectance_Sensor_Data);
            Print_Binary(Reflectance_Sensor_Data);
        }
}

/**
 * @brief Toggles P8.7 every 400ms
 *
 * @param None
 *
 * @return None
 */
void Timer_A2_Periodic_Task(void)
{
    // Your implementation for the tasks will go here
    Timer_A2_ms_elapsed++;
            if (Timer_A2_ms_elapsed >= 400)
            {
                P8->OUT ^= 0x80;
                Timer_A2_ms_elapsed = 0;
            }

}


void Detect_Line_Position(uint8_t reflectance_sensor_data) {
    switch(reflectance_sensor_data){
        case 0x18:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_GREEN);
            break;

        case 0x1C:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_YELLOW);
            break;

        case 0x38:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_PINK);
            break;

        case 0x01:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_WHITE);
            break;

        case 0x80:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_SKY_BLUE);
            break;

        case 0xF8:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_WHITE);
            break;

        case 0x1F:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_SKY_BLUE);
            break;

        case 0xFF:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_BLUE);
            break;

        case 0x00:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_RED);
            break;

        default:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_OFF);
    }
}

int main(void)
{
    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    // Initialize the built-in red LED and the RGB LEDs
    LED1_Init();
    LED2_Init();

    // Initialize the user buttons
    Buttons_Init();

    // Initialize the front and back LEDs on the chassis board
    Chassis_Board_LEDs_Init();

    // Initialize EUSCI_A0_UART
    EUSCI_A0_UART_Init_Printf();

    // Initialize the reflectance sensor
    Reflectance_Sensor_Init();

    // Initialize the SysTick timer to generate periodic interrupts every 1 ms
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    // Initialize Timer A0 periodic interrupts every 1 ms
    Timer_A0_Interrupt_Init(&Timer_A0_Periodic_Task, TIMER_A0_INT_CCR0_VALUE);

    // Initialize Timer A1 periodic interrupts every 1 ms
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    // Initialize Timer A2 periodic interrupts every 1 ms
    Timer_A2_Interrupt_Init(&Timer_A2_Periodic_Task, TIMER_A2_INT_CCR0_VALUE);

    // Enable the interrupts used by the SysTick and Timer A1 timers
    EnableInterrupts();

    while(1)
    {
        /*Timer_A1_ms_elapsed++;
        if (Timer_A1_ms_elapsed % 10 == 0){
            Reflectance_Sensor_Start();
        }
        if ((Timer_A1_ms_elapsed - 1) % 10 == 0){
            uint8_t Reflectance_Sensor_Data = Reflectance_Sensor_End();
            Detect_Line_Position(Reflectance_Sensor_Data);
            Print_Binary(Reflectance_Sensor_Data);
        }*/
    }
}
