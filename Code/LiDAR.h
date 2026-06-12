/**
 * @file LiDAR.h
 * @brief TF-Luna LiDAR UART driver for projectile range measurement.
 *
 * This driver receives and validates distance frames from the TF-Luna LiDAR
 * sensor. The validated range measurement is used by the projectile tracking
 * firmware to confirm whether the detected camera target is close enough for the
 * laser to be enabled. The driver stores the latest distance, signal strength,
 * temperature, and validity flag in a Lidar_t object.
 */

#ifndef LIDAR_H_
#define LIDAR_H_

#include "main.h"
#include <stdint.h>

/** @brief Minimum accepted LiDAR distance in centimeters. */
#define LIDAR_MIN_VALID_DISTANCE_CM     20u

/** @brief Maximum accepted LiDAR distance in centimeters. */
#define LIDAR_MAX_VALID_DISTANCE_CM     800u

/** @brief Minimum signal strength required for a measurement to be considered usable. */
#define LIDAR_MIN_VALID_STRENGTH        1u

/**
 * @brief Status codes returned by the LiDAR driver.
 *
 * These values allow the main application to distinguish between a successful
 * read, a timeout, a bad checksum, a UART failure, or an invalid argument.
 */
typedef enum
{
    LIDAR_OK = 0,      /**< A valid frame was received and parsed. */
    LIDAR_TIMEOUT,     /**< No complete frame was received before the timeout. */
    LIDAR_BAD_CHECKSUM,/**< A frame was received but failed checksum validation. */
    LIDAR_UART_ERROR,  /**< HAL UART returned an error other than timeout. */
    LIDAR_NULL_ARG     /**< A NULL device pointer or UART handle was provided. */
} LidarStatus_t;

/**
 * @brief LiDAR device object and latest measurement state.
 *
 * The structure stores the UART handle used to communicate with the sensor and
 * the most recent decoded measurement fields. The valid flag is set after each
 * read based on the configured distance and signal-strength limits.
 */
typedef struct
{
    UART_HandleTypeDef *huart;  /**< UART peripheral connected to the LiDAR sensor. */

    uint16_t distance_cm;       /**< Latest measured distance in centimeters. */
    uint16_t strength;          /**< Latest signal strength reported by the sensor. */
    uint16_t temperature_raw;   /**< Raw sensor temperature field. */
    float temperature_c;        /**< Converted sensor temperature in degrees Celsius. */

    uint8_t valid;              /**< Nonzero when the latest measurement passed validation. */
} Lidar_t;

/**
 * @brief Initializes a LiDAR device object.
 *
 * Associates the UART peripheral with the LiDAR object and clears all stored
 * measurement values.
 *
 * @param dev Pointer to the LiDAR device object.
 * @param huart Pointer to the STM32 HAL UART handle used by the sensor.
 */
void Lidar_Init(Lidar_t *dev, UART_HandleTypeDef *huart);

/**
 * @brief Reads and parses one complete TF-Luna measurement frame.
 *
 * The function searches for the two frame header bytes, receives the remaining
 * frame bytes, verifies the checksum, updates the device measurement fields, and
 * sets the validity flag.
 *
 * @param dev Pointer to the LiDAR device object.
 * @param timeout_ms Maximum time to wait for a complete frame.
 * @return LIDAR_OK on a successfully parsed frame, otherwise an error status.
 */
LidarStatus_t Lidar_ReadFrame(Lidar_t *dev, uint32_t timeout_ms);

/**
 * @brief Checks whether the latest LiDAR measurement is valid.
 *
 * A measurement is valid only when the signal strength is high enough and the
 * distance falls within the accepted sensor range.
 *
 * @param dev Pointer to the LiDAR device object.
 * @return 1 if the latest measurement is valid, otherwise 0.
 */
uint8_t Lidar_IsMeasurementValid(const Lidar_t *dev);

/**
 * @brief Determines whether an object is detected within a selected range.
 *
 * This helper is used by the main tracking loop to decide whether the target is
 * close enough to allow the laser to turn on.
 *
 * @param dev Pointer to the LiDAR device object.
 * @param max_distance_cm Detection threshold in centimeters.
 * @return 1 if a valid object is within max_distance_cm, otherwise 0.
 */
uint8_t Lidar_ObjectDetected(const Lidar_t *dev, uint16_t max_distance_cm);

/**
 * @brief Converts a LiDAR status code into a readable string.
 *
 * @param status LiDAR driver status code.
 * @return Constant string describing the status.
 */
const char *Lidar_StatusString(LidarStatus_t status);

#endif /* LIDAR_H_ */
