/**
 * @file motor_driver.c
 * @brief Implementation of the pan and tilt motor driver.
 *
 * The driver converts signed speed commands into motor direction GPIO states and
 * PWM duty-cycle values. The pan axis uses TIM4 channel 3 and the tilt axis uses
 * TIM4 channel 4. Keeping this code in a driver prevents the main control loop
 * from directly manipulating PWM registers and GPIO pins.
 */

#include "motor_driver.h"

extern TIM_HandleTypeDef htim4;

/**
 * @brief Starts PWM generation and places both motors in a stopped state.
 *
 * The SEL0 pins are asserted to configure or enable the motor-driver channels.
 * Both PWM outputs are started before Motors_Stop() clears the duty cycles and
 * direction pins.
 */
void MotorDriver_Init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    HAL_GPIO_WritePin(GPIOB, PAN_SEL0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, TILT_SEL0_Pin, GPIO_PIN_SET);

    Motors_Stop();
}

/**
 * @brief Sets the pan motor speed and direction.
 *
 * Positive values select one direction, negative values select the opposite
 * direction, and the magnitude becomes the PWM compare value. The command is
 * clamped to the timer range of +/-4999.
 *
 * @param speed Signed pan command from -4999 to +4999.
 */
void PanMotor_Set(int16_t speed)
{
    if(speed > 4999) speed = 4999;
    if(speed < -4999) speed = -4999;

    if(speed >= 0)
    {
        HAL_GPIO_WritePin(GPIOB, PAN_INA_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, PAN_INB_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, PAN_INA_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, PAN_INB_Pin, GPIO_PIN_SET);
        speed = -speed;
    }

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, speed);
}

/**
 * @brief Sets the tilt motor speed and direction.
 *
 * Positive values select one direction, negative values select the opposite
 * direction, and the magnitude becomes the PWM compare value. The command is
 * clamped to the timer range of +/-4999.
 *
 * @param speed Signed tilt command from -4999 to +4999.
 */
void TiltMotor_Set(int16_t speed)
{
    if(speed > 4999) speed = 4999;
    if(speed < -4999) speed = -4999;

    if(speed >= 0)
    {
        HAL_GPIO_WritePin(GPIOB, TILT_INA_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, TILT_INB_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, TILT_INA_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, TILT_INB_Pin, GPIO_PIN_SET);
        speed = -speed;
    }

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, speed);
}

/**
 * @brief Stops both motor channels and clears all direction pins.
 *
 * This function is used during initialization, when the camera target is lost,
 * and whenever the mechanism needs to be placed into a safe non-moving state.
 */
void Motors_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

    HAL_GPIO_WritePin(GPIOB, PAN_INA_Pin|PAN_INB_Pin|TILT_INA_Pin|TILT_INB_Pin, GPIO_PIN_RESET);
}
