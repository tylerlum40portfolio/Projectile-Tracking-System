/**
 * @file motor_driver.h
 * @brief Pan and tilt DC motor driver interface.
 *
 * This driver abstracts the GPIO and PWM details used to command the projectile
 * tracking mechanism. The pan motor is driven from TIM4 channel 3 and the tilt
 * motor is driven from TIM4 channel 4. Direction is controlled with INA/INB GPIO
 * pins and speed is controlled by PWM duty cycle.
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "main.h"
#include <stdint.h>

/**
 * @brief Initializes the motor driver subsystem.
 *
 * Starts the PWM outputs, configures motor-driver select pins, and stops both
 * motors so the mechanism begins in a safe state.
 */
void MotorDriver_Init(void);

/**
 * @brief Commands the pan-axis motor.
 *
 * Positive speed values drive the motor in one direction and negative values
 * drive it in the opposite direction. The command is clamped to the PWM timer
 * range before being applied.
 *
 * @param speed Signed speed command from -4999 to +4999.
 */
void PanMotor_Set(int16_t speed);

/**
 * @brief Commands the tilt-axis motor.
 *
 * Positive speed values drive the motor in one direction and negative values
 * drive it in the opposite direction. The command is clamped to the PWM timer
 * range before being applied.
 *
 * @param speed Signed speed command from -4999 to +4999.
 */
void TiltMotor_Set(int16_t speed);

/**
 * @brief Stops both pan and tilt motors.
 *
 * Sets both PWM duty cycles to zero and clears the motor direction pins. This is
 * used at startup, when no camera target is detected, and during fault or test
 * conditions.
 */
void Motors_Stop(void);

#endif /* MOTOR_DRIVER_H */
