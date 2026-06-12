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
#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "main.h"
#include "LiDAR.h"
#include "pixy2_camera.h"
#include <stdbool.h>
#include <stdint.h>

void Controller_Init();
void Controller_Update(PixyTarget_t *camera_target, Lidar_t *lidar);
void Controller_Reset();

uint16_t Controller_ReadPanADC(void);
uint16_t Controller_ReadTiltADC(void);

uint8_t Controller_TiltIsHome(void);
void Controller_HomeTiltStep(void);
void Controller_HomeTiltBlocking(uint32_t timeout_ms);

int16_t Controller_ApplyTiltLimits(int16_t tilt_cmd);
int16_t Controller_ApplyPanLimits(int16_t pan_cmd);

void Controller_SearchMode(void);

#endif /* INC_CONTROLLER_H_ */
