/**
 * @file pixy2_camera.h
 * @brief Pixy2 SPI camera interface for target detection.
 *
 * This module communicates with the Pixy2 camera over SPI, requests color-code
 * connected-component blocks, selects the best target matching the configured
 * signature, and reports pixel error relative to the image center. The main
 * tracking loop uses these pixel errors as the input to the pan and tilt PID
 * controllers.
 */

#ifndef INC_PIXY2_CAMERA_H_
#define INC_PIXY2_CAMERA_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/** @brief Pixy2 frame width in pixels. */
#define PIXY2_FRAME_WIDTH       316

/** @brief Pixy2 frame height in pixels. */
#define PIXY2_FRAME_HEIGHT      208

/** @brief Horizontal center pixel of the Pixy2 frame. */
#define PIXY2_CENTER_X          158

/** @brief Vertical center pixel of the Pixy2 frame. */
#define PIXY2_CENTER_Y          104

/** @brief Minimum target area in pixels before a block is accepted. */
#define PIXY2_MIN_TARGET_AREA   30

/** @brief Allowed horizontal error for the target to be considered centered. */
#define PIXY2_X_DEADBAND        18

/** @brief Allowed vertical error for the target to be considered centered. */
#define PIXY2_Y_DEADBAND        18

/** @brief Pixy2 color signature used for the projectile target. */
#define PIXY2_TARGET_SIGNATURE  1

/** @brief Maximum number of Pixy2 blocks read per request. */
#define PIXY2_MAX_BLOCKS        10

/**
 * @brief Raw object block reported by the Pixy2 camera.
 *
 * Each block contains the signature, location, size, angle, tracking index, and
 * age reported by the Pixy2 connected-component algorithm.
 */
typedef struct
{
    uint16_t signature; /**< Pixy color signature number. */
    uint16_t x;         /**< X-coordinate of the block center in pixels. */
    uint16_t y;         /**< Y-coordinate of the block center in pixels. */
    uint16_t width;     /**< Block width in pixels. */
    uint16_t height;    /**< Block height in pixels. */
    int16_t angle;      /**< Block angle reported by Pixy2. */
    uint8_t index;      /**< Pixy2 tracking index. */
    uint8_t age;        /**< Number of frames the object has been tracked. */
} PixyBlock_t;

/**
 * @brief Processed target data used by the tracking controller.
 *
 * This structure stores the selected target and its pixel error relative to the
 * center of the camera image. The PID controllers use x_error and y_error to
 * generate pan and tilt motor commands.
 */
typedef struct
{
    bool target_detected; /**< True when a valid target was found. */
    uint16_t x;           /**< Target center X coordinate in pixels. */
    uint16_t y;           /**< Target center Y coordinate in pixels. */
    uint16_t width;       /**< Target width in pixels. */
    uint16_t height;      /**< Target height in pixels. */
    int16_t x_error;      /**< Horizontal error from image center in pixels. */
    int16_t y_error;      /**< Vertical error from image center in pixels. */
} PixyTarget_t;

/**
 * @brief Initializes Pixy2 communication.
 *
 * Deasserts the chip-select line and allows the camera time to become ready.
 */
void Pixy2_Init(void);

/**
 * @brief Requests object blocks from the Pixy2 camera.
 *
 * @param blocks Output array for decoded Pixy blocks.
 * @param max_blocks Maximum number of blocks to copy into the array.
 * @return Number of valid blocks copied into blocks[].
 */
uint8_t getCameraBlocks(PixyBlock_t *blocks, uint8_t max_blocks);

/**
 * @brief Finds the best matching target in the Pixy2 frame.
 *
 * The function requests blocks from the camera, filters by target signature and
 * minimum area, chooses the largest matching block, and computes image-center
 * errors.
 *
 * @param target Output target structure.
 * @return true if a valid target is detected, otherwise false.
 */
bool Camera_FindTarget(PixyTarget_t *target);

/**
 * @brief Checks whether the detected target is centered in the camera frame.
 *
 * @param target Pointer to the processed target structure.
 * @return true when the target is detected and both errors are within deadband.
 */
bool Camera_TargetCentered(PixyTarget_t *target);

/**
 * @brief Gets the latest horizontal target error.
 *
 * @param target Pointer to the processed target structure.
 * @return X-axis pixel error, or 0 if target is NULL.
 */
int16_t Camera_GetXError(PixyTarget_t *target);

/**
 * @brief Gets the latest vertical target error.
 *
 * @param target Pointer to the processed target structure.
 * @return Y-axis pixel error, or 0 if target is NULL.
 */
int16_t Camera_GetYError(PixyTarget_t *target);

#endif /* INC_PIXY2_CAMERA_H_ */
