/**
 * @file Laser.h
 * @brief GPIO-based laser module driver for the projectile tracking system.
 *
 * This header exposes a small hardware-abstraction layer for the laser output.
 * The laser is controlled through the CubeMX-generated LASER_SIGNAL GPIO pin.
 * Keeping this logic in its own driver makes the main tracking loop easier to
 * read and allows the laser hardware to be tested independently from the camera,
 * LiDAR, and motor subsystems.
 */

#ifndef LASER_H_
#define LASER_H_

#include <stdint.h>

/**
 * @brief Initializes the laser subsystem.
 *
 * The initialization routine forces the laser into a known safe state by turning
 * it off during startup. This prevents the laser from accidentally enabling when
 * the STM32 first powers on or resets.
 */
void Laser_Init(void);

/**
 * @brief Turns the laser module on.
 *
 * Drives the LASER_SIGNAL output pin high. In the final tracking behavior this
 * should only be called when the target is centered and confirmed to be within
 * the accepted LiDAR distance range.
 */
void Laser_On(void);

/**
 * @brief Turns the laser module off.
 *
 * Drives the LASER_SIGNAL output pin low. This is the default and safe state when
 * no target is detected, the target is not centered, or the measured distance is
 * outside the valid firing range.
 */
void Laser_Off(void);

/**
 * @brief Toggles the laser output state.
 *
 * This helper is mainly useful during bench testing because it quickly verifies
 * that the laser GPIO pin, PCB trace, and laser module connection are working.
 */
void Laser_Toggle(void);

/**
 * @brief Blinks the laser for a visible test pattern.
 *
 * This function blocks using HAL_Delay(), so it should only be used during
 * startup checks or manual testing. It is not intended for the real-time tracking
 * loop.
 *
 * @param delay_ms On-time and off-time delay in milliseconds.
 */
void Laser_Blink(uint32_t delay_ms);

#endif /* LASER_H_ */
