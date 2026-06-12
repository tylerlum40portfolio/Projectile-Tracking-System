/**
 * @file Laser.c
 * @brief Implementation of the laser GPIO driver.
 *
 * The laser module is controlled using the CubeMX-generated LASER_SIGNAL GPIO
 * output. This file keeps laser control separate from the main tracking logic so
 * the system can safely enable, disable, toggle, and test the laser output.
 */

#include "Laser.h"
#include "main.h" // STM32 HAL functions are brought through main.h

/**
 * @brief Initializes the laser driver and forces the laser off.
 *
 * Starting with the laser disabled is a safety measure. The tracking loop should
 * only enable the laser after the camera target is centered and the LiDAR range
 * confirms that the object is within the accepted distance.
 */
void Laser_Init(void)
{
    Laser_Off(); // Start with the laser turned off
}

/**
 * @brief Turns the laser on by driving LASER_SIGNAL high.
 */
void Laser_On(void)
{
	// Set the LASER_SIGNAL pin HIGH, which turns the laser on
    HAL_GPIO_WritePin(LASER_SIGNAL_GPIO_Port, LASER_SIGNAL_Pin, GPIO_PIN_SET);
}

/**
 * @brief Turns the laser off by driving LASER_SIGNAL low.
 */
void Laser_Off(void)
{
	// Set the LASER_SIGNAL pin LOW, which turns the laser off
    HAL_GPIO_WritePin(LASER_SIGNAL_GPIO_Port, LASER_SIGNAL_Pin, GPIO_PIN_RESET);
}


/**
 * @brief Toggles the laser GPIO output.
 *
 * This function is useful for quick hardware checks during bring-up.
 */
void Laser_Toggle(void)
{
	// Switch the laser pin to the opposite state
    HAL_GPIO_TogglePin(LASER_SIGNAL_GPIO_Port, LASER_SIGNAL_Pin);
}

/**
 * @brief Blinks the laser using blocking delays.
 *
 * @param delay_ms Delay time in milliseconds for both the on and off portions of
 * the blink.
 */
void Laser_Blink(uint32_t delay_ms) // Testing only
{
	// Turn laser on, wait, then turn laser off and wait again
    Laser_On();
    HAL_Delay(delay_ms);

    Laser_Off();
    HAL_Delay(delay_ms);
}
