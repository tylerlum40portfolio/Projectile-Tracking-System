/**
 * @file controller.h
 * @brief High-level target tracking controller interface.
 *
 * This module coordinates the Pixy2 camera, TF-Luna LiDAR,
 * pan/tilt motor drivers, and laser module to implement
 * closed-loop target tracking.
 *
 * @authors Tyler Lum, Colby Cordoba, Sam Nunes
 * @date June 2026
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

// prototypes here

#endif
#include "controller.h"
#include "motor_driver.h"
#include "Laser.h"

/* PID gains:
 * Kp: raise until it tracks without lagging, lower if it oscillates
 * Ki: small value to remove steady state error, keep near 0 to start
 * Kd: raise until it stops overshooting targets
 */

#define PAN_KP  20.0f
#define PAN_KI  0.0f
#define PAN_KD  5.0f

#define TILT_KP 20.0f
#define TILT_KI 0.0f
#define TILT_KD 5.0f

#define PID_INTEGRAL_MAX  500.0f
#define PID_OUTPUT_MAX    4999.0f


#define LASER_MAX_RANGE_CM 300u

typedef struct{
	float kp;
	float ki;
	float kd;
	float integral;
	float prev_error;
	float integral_max;
	float output_max;
} PID_t;

static PID_t pan_pid;
static PID_t tilt_pid;
extern ADC_HandleTypeDef hadc1;

static int16_t pan_search_direction = 1;

static void PID_Init(PID_t *pid, float kp, float ki, float kd,
                     float integral_max, float output_max);

static float PID_Update(PID_t *pid, float error);

static void PID_Reset(PID_t *pid);

static uint16_t Controller_ReadADC(uint32_t channel);

// INIT / RESET

void Controller_Init(void)
{
    PID_Init(&pan_pid, PAN_KP, PAN_KI, PAN_KD,
             PID_INTEGRAL_MAX, PID_OUTPUT_MAX);

    PID_Init(&tilt_pid, TILT_KP, TILT_KI, TILT_KD,
             PID_INTEGRAL_MAX, PID_OUTPUT_MAX);
}

void Controller_Reset(void)
{
    PID_Reset(&pan_pid);
    PID_Reset(&tilt_pid);
}

// ADC FEEDBACK

static uint16_t Controller_ReadADC(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint16_t adc_value = 0;

    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        return 0;
    }

    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        adc_value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);

    return adc_value;
}

uint16_t Controller_ReadPanADC(void)
{
    return Controller_ReadADC(ADC_CHANNEL_0);   // PA0 = PAN_FB
}

uint16_t Controller_ReadTiltADC(void)
{
    return Controller_ReadADC(ADC_CHANNEL_1);   // PA1 = TILT_FB
}

// TILT HOMING

uint8_t Controller_TiltIsHome(void)
{
    uint16_t tilt_adc = Controller_ReadTiltADC();

    // Your measured vertical center was about 3856.
    // This accepts 3820 to 3890 as close enough to vertical.
    if (tilt_adc >= 3820 && tilt_adc <= 3890)
    {
        return 1u;
    }

    return 0u;
}

void Controller_HomeTiltStep(void)
{
    uint16_t tilt_adc = Controller_ReadTiltADC();

    // If tilt ADC is above vertical center, move back toward center.
    if (tilt_adc > 3890)
    {
        TiltMotor_Set(600);
    }

    // If tilt ADC is below vertical center, move the opposite direction.
    else if (tilt_adc < 3820)
    {
        TiltMotor_Set(-600);
    }

    // If close to vertical, stop.
    else
    {
        TiltMotor_Set(0);
    }
}

void Controller_HomeTiltBlocking(uint32_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick();

    while (Controller_TiltIsHome() == 0u)
    {
        Controller_HomeTiltStep();

        // Important: pan should NOT move while tilt is homing.
        PanMotor_Set(0);
        Laser_Off();

        if ((HAL_GetTick() - start_time) > timeout_ms)
        {
            break;
        }

        HAL_Delay(20);
    }

    Motors_Stop();
}

// ADC LIMIT PROTECTION

int16_t Controller_ApplyTiltLimits(int16_t tilt_cmd)
{
    uint16_t tilt_adc = Controller_ReadTiltADC();

    if (tilt_adc >= 4090 && tilt_cmd < 0)
    {
        tilt_cmd = 0;
    }

    if (tilt_adc <= 1600 && tilt_cmd > 0)
    {
        tilt_cmd = 0;
    }

    return tilt_cmd;
}

int16_t Controller_ApplyPanLimits(int16_t pan_cmd)
{
    uint16_t pan_adc = Controller_ReadPanADC();

    // Left-side limit
    if (pan_adc >= 2550 && pan_adc <= 2750 && pan_cmd < 0)
    {
        pan_cmd = 0;
    }

    // Right-side limit
    if (pan_adc >= 3150 && pan_adc <= 3220 && pan_cmd > 0)
    {
        pan_cmd = 0;
    }

    return pan_cmd;
}
// SEARCH MODE

void Controller_SearchMode(void)
{
    uint16_t pan_adc = Controller_ReadPanADC();

    Laser_Off();

    // Do NOT pan until tilt is back at vertical home.
    if (Controller_TiltIsHome() == 0u)
    {
        PanMotor_Set(0);
        Controller_HomeTiltStep();
        return;
    }

    // Once tilt is vertical, then pan can search.
    if (pan_adc >= 3150 && pan_adc <= 3220)
    {
        pan_search_direction = -1;
    }
    else if (pan_adc >= 2550 && pan_adc <= 2750)
    {
        pan_search_direction = 1;
    }

    PanMotor_Set(pan_search_direction * 700);
}

// OPTIONAL FULL CONTROLLER UPDATE
void Controller_Update(PixyTarget_t *camera_target, Lidar_t *lidar)
{
    bool target_found = Camera_FindTarget(camera_target);

    if (!target_found)
    {
        PanMotor_Set(0);
        Laser_Off();
        Controller_SearchMode();
        Controller_Reset();
        return;
    }

    float pan_speed = PID_Update(&pan_pid, (float)camera_target->x_error);
    float tilt_speed = PID_Update(&tilt_pid, (float)camera_target->y_error);

    int16_t pan_cmd = Controller_ApplyPanLimits((int16_t)pan_speed);
    int16_t tilt_cmd = Controller_ApplyTiltLimits((int16_t)tilt_speed);

    PanMotor_Set(pan_cmd);
    TiltMotor_Set(tilt_cmd);

    LidarStatus_t lidar_status = Lidar_ReadFrame(lidar, 10u);

    bool centered = Camera_TargetCentered(camera_target);
    bool in_range = (lidar_status == LIDAR_OK) &&
                    Lidar_ObjectDetected(lidar, LASER_MAX_RANGE_CM);

    if (centered && in_range)
    {
        Laser_On();
    }
    else
    {
        Laser_Off();
    }
}

// PID FUNCTIONS

static void PID_Init(PID_t *pid, float kp, float ki, float kd,
                     float integral_max, float output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_max = integral_max;
    pid->output_max = output_max;
}

static float PID_Update(PID_t *pid, float error)
{
    pid->integral += error;

    if (pid->integral > pid->integral_max)
    {
        pid->integral = pid->integral_max;
    }
    else if (pid->integral < -pid->integral_max)
    {
        pid->integral = -pid->integral_max;
    }

    float derivative = error - pid->prev_error;
    pid->prev_error = error;

    float output = (pid->kp * error) +
                   (pid->ki * pid->integral) +
                   (pid->kd * derivative);

    if (output > pid->output_max)
    {
        output = pid->output_max;
    }
    else if (output < -pid->output_max)
    {
        output = -pid->output_max;
    }

    return output;
}

static void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
