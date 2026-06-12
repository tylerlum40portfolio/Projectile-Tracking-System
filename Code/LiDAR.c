/**
 * @file LiDAR.c
 * @brief Implementation of the TF-Luna LiDAR UART driver.
 *
 * The driver searches the incoming UART byte stream for the TF-Luna frame
 * header, validates the checksum, decodes distance, strength, and temperature,
 * and marks the measurement as valid only if it passes the configured range and
 * signal-strength checks.
 */

#include "LiDAR.h"

#define LIDAR_HEADER        0x59u
#define LIDAR_FRAME_LEN     9u

/**
 * @brief Computes elapsed time from a HAL tick timestamp.
 *
 * @param start Starting tick count from HAL_GetTick().
 * @return Elapsed time in milliseconds.
 */
static uint32_t Lidar_ElapsedMs(uint32_t start)
{
    return HAL_GetTick() - start;
}

/**
 * @brief Initializes a LiDAR device object.
 *
 * @param dev Pointer to the LiDAR object to initialize.
 * @param huart UART handle connected to the TF-Luna sensor.
 */
void Lidar_Init(Lidar_t *dev, UART_HandleTypeDef *huart)
{
    if (dev == NULL)
    {
        return;
    }

    dev->huart = huart;
    dev->distance_cm = 0u;
    dev->strength = 0u;
    dev->temperature_raw = 0u;
    dev->temperature_c = 0.0f;
    dev->valid = 0u;
}

/**
 * @brief Validates the most recent LiDAR measurement.
 *
 * @param dev Pointer to the LiDAR object containing the latest reading.
 * @return 1 if strength and distance are within valid limits, otherwise 0.
 */
uint8_t Lidar_IsMeasurementValid(const Lidar_t *dev)
{
    if (dev == NULL)
    {
        return 0u;
    }

    if (dev->strength < LIDAR_MIN_VALID_STRENGTH)
    {
        return 0u;
    }

    if (dev->distance_cm < LIDAR_MIN_VALID_DISTANCE_CM)
    {
        return 0u;
    }

    if (dev->distance_cm > LIDAR_MAX_VALID_DISTANCE_CM)
    {
        return 0u;
    }

    return 1u;
}

/**
 * @brief Checks if a valid object is within a selected distance threshold.
 *
 * @param dev Pointer to the LiDAR object containing the latest reading.
 * @param max_distance_cm Maximum allowed object distance in centimeters.
 * @return 1 if an object is valid and within range, otherwise 0.
 */
uint8_t Lidar_ObjectDetected(const Lidar_t *dev, uint16_t max_distance_cm)
{
    if (dev == NULL)
    {
        return 0u;
    }

    if (dev->valid == 0u)
    {
        return 0u;
    }

    if (dev->distance_cm <= max_distance_cm)
    {
        return 1u;
    }

    return 0u;
}

/**
 * @brief Reads, validates, and decodes one TF-Luna data frame.
 *
 * The function scans for two 0x59 header bytes, collects the full 9-byte frame,
 * checks the checksum, updates the measurement fields, and returns a status code
 * describing the result.
 *
 * @param dev Pointer to the LiDAR device object.
 * @param timeout_ms Maximum time to wait for a complete frame.
 * @return LIDAR_OK when a frame is decoded, otherwise an error status.
 */
LidarStatus_t Lidar_ReadFrame(Lidar_t *dev, uint32_t timeout_ms)
{
    if ((dev == NULL) || (dev->huart == NULL))
    {
        return LIDAR_NULL_ARG;
    }

    dev->valid = 0u;

    uint8_t byte = 0u;
    uint8_t frame[LIDAR_FRAME_LEN] = {0u};
    uint8_t index = 0u;
    uint8_t state = 0u;
    uint8_t saw_bad_checksum = 0u;

    uint32_t start = HAL_GetTick();

    while (Lidar_ElapsedMs(start) < timeout_ms)
    {
        HAL_StatusTypeDef hal_status;

        hal_status = HAL_UART_Receive(dev->huart, &byte, 1u, 1u);

        if (hal_status == HAL_TIMEOUT)
        {
            continue;
        }

        if (hal_status != HAL_OK)
        {
            return LIDAR_UART_ERROR;
        }

        switch (state)
        {
            case 0u:
                if (byte == LIDAR_HEADER)
                {
                    frame[0] = byte;
                    state = 1u;
                }
                break;

            case 1u:
                if (byte == LIDAR_HEADER)
                {
                    frame[1] = byte;
                    index = 2u;
                    state = 2u;
                }
                else
                {
                    state = 0u;
                }
                break;

            case 2u:
                frame[index] = byte;
                index++;

                if (index >= LIDAR_FRAME_LEN)
                {
                    uint8_t checksum = 0u;

                    for (uint8_t i = 0u; i < LIDAR_FRAME_LEN - 1u; i++)
                    {
                        checksum = (uint8_t)(checksum + frame[i]);
                    }

                    if (checksum != frame[8])
                    {
                        saw_bad_checksum = 1u;
                        state = 0u;
                        index = 0u;
                        break;
                    }

                    dev->distance_cm = (uint16_t)frame[2] |
                                       ((uint16_t)frame[3] << 8);

                    dev->strength = (uint16_t)frame[4] |
                                    ((uint16_t)frame[5] << 8);

                    dev->temperature_raw = (uint16_t)frame[6] |
                                           ((uint16_t)frame[7] << 8);

                    dev->temperature_c = ((float)dev->temperature_raw / 8.0f) - 256.0f;

                    dev->valid = Lidar_IsMeasurementValid(dev);

                    return LIDAR_OK;
                }
                break;

            default:
                state = 0u;
                index = 0u;
                break;
        }
    }

    if (saw_bad_checksum)
    {
        return LIDAR_BAD_CHECKSUM;
    }

    return LIDAR_TIMEOUT;
}

/**
 * @brief Converts a LiDAR status value to text for debugging output.
 *
 * @param status Driver status code.
 * @return Constant string describing the status.
 */
const char *Lidar_StatusString(LidarStatus_t status)
{
    switch (status)
    {
        case LIDAR_OK:
            return "OK";

        case LIDAR_TIMEOUT:
            return "TIMEOUT";

        case LIDAR_BAD_CHECKSUM:
            return "BAD_CHECKSUM";

        case LIDAR_UART_ERROR:
            return "UART_ERROR";

        case LIDAR_NULL_ARG:
            return "NULL_ARG";

        default:
            return "UNKNOWN";
    }
}
