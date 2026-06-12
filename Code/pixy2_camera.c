/**
 * @file pixy2_camera.c
 * @brief Pixy2 SPI target-detection driver implementation.
 *
 * This file sends getBlocks requests to the Pixy2 camera, validates the response
 * packet, decodes color-connected-component blocks, and selects the largest block
 * matching the configured target signature. The selected block is converted into
 * horizontal and vertical pixel error for the tracking controller.
 */

#include "pixy2_camera.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

// Pixy2 packet sync bytes
#define PIXY2_REQ_SYNC_1              0xAE
#define PIXY2_REQ_SYNC_2              0xC1

#define PIXY2_RESP_SYNC_1             0xAF
#define PIXY2_RESP_SYNC_2             0xC1

/* Pixy2 packet types */
#define PIXY2_TYPE_GET_BLOCKS_REQ     0x20  // decimal 32
#define PIXY2_TYPE_GET_BLOCKS_RESP    0x21  // decimal 33

/* One normal Pixy2 CCC block is 14 bytes */
#define PIXY2_BLOCK_SIZE              14

#define PIXY2_SPI_TIMEOUT_MS          100
#define PIXY2_RESPONSE_BUFFER_SIZE    160

/** @brief Asserts the Pixy2 chip-select line for SPI communication. */
static void Pixy2_Select(void)
{
    HAL_GPIO_WritePin(PIXY_CS_GPIO_Port, PIXY_CS_Pin, GPIO_PIN_RESET);
}

/** @brief Deasserts the Pixy2 chip-select line after SPI communication. */
static void Pixy2_Deselect(void)
{
    HAL_GPIO_WritePin(PIXY_CS_GPIO_Port, PIXY_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Reads a little-endian unsigned 16-bit value from a byte buffer.
 * @param data Pointer to the first byte of the value.
 * @return Decoded unsigned 16-bit value.
 */
static uint16_t Pixy2_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

/**
 * @brief Reads a little-endian signed 16-bit value from a byte buffer.
 * @param data Pointer to the first byte of the value.
 * @return Decoded signed 16-bit value.
 */
static int16_t Pixy2_ReadS16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

/**
 * @brief Computes the Pixy2 payload checksum.
 * @param payload Pointer to payload bytes.
 * @param payload_len Number of payload bytes.
 * @return 16-bit additive checksum.
 */
static uint16_t Pixy2_Checksum(const uint8_t *payload, uint8_t payload_len)
{
    uint16_t sum = 0;

    for (uint8_t i = 0; i < payload_len; i++)
    {
        sum += payload[i];
    }

    return sum;
}

/*
 * Pixy2 SPI can return a few dummy/idle bytes before the real response.
 * This searches for the response sync: 0xAF 0xC1.
 */
/**
 * @brief Finds the Pixy2 response sync bytes inside a received buffer.
 *
 * Pixy2 may output dummy bytes before a real response, so the driver searches
 * for the response sync sequence instead of assuming the packet starts at byte 0.
 *
 * @param buffer Received SPI byte buffer.
 * @param length Number of bytes in the buffer.
 * @return Start index of the response packet, or -1 if not found.
 */
static int16_t Pixy2_FindResponseStart(const uint8_t *buffer, uint16_t length)
{
    for (uint16_t i = 0; i < length - 1; i++)
    {
        if (buffer[i] == PIXY2_RESP_SYNC_1 && buffer[i + 1] == PIXY2_RESP_SYNC_2)
        {
            return (int16_t)i;
        }
    }

    return -1;
}

/*
 * Low-level Pixy2 request.
 * Sends request packet, then clocks in response bytes using dummy bytes.
 */
/**
 * @brief Sends a Pixy2 request and reads back the response bytes.
 *
 * The function transmits the request packet, waits briefly, then clocks response
 * data out of the Pixy2 using dummy transmit bytes.
 *
 * @param request Pointer to request packet bytes.
 * @param request_len Number of bytes in the request packet.
 * @param response Buffer used to store received response bytes.
 * @param response_len Number of response bytes to clock in.
 * @return HAL SPI status.
 */
static HAL_StatusTypeDef Pixy2_SendRequest(const uint8_t *request,
                                           uint16_t request_len,
                                           uint8_t *response,
                                           uint16_t response_len)
{
    HAL_StatusTypeDef status;
    uint8_t dummy_tx[PIXY2_RESPONSE_BUFFER_SIZE];

    if (response_len > PIXY2_RESPONSE_BUFFER_SIZE)
    {
        response_len = PIXY2_RESPONSE_BUFFER_SIZE;
    }

    memset(dummy_tx, 0x00, sizeof(dummy_tx));
    memset(response, 0x00, response_len);

    Pixy2_Select();

    status = HAL_SPI_Transmit(&hspi1,
                              (uint8_t *)request,
                              request_len,
                              PIXY2_SPI_TIMEOUT_MS);

    if (status == HAL_OK)
    {
        /*
         * Pixy2 sometimes needs a very small delay before response bytes.
         */
        HAL_Delay(1);

        status = HAL_SPI_TransmitReceive(&hspi1,
                                          dummy_tx,
                                          response,
                                          response_len,
                                          PIXY2_SPI_TIMEOUT_MS);
    }

    Pixy2_Deselect();

    return status;
}

/**
 * @brief Initializes the Pixy2 SPI interface state.
 *
 * The function deasserts chip select and waits briefly for the camera to be
 * ready before normal requests are attempted.
 */
void Pixy2_Init()
{
    Pixy2_Deselect();
    HAL_Delay(100);
}

/*
 * Reads Pixy2 CCC blocks.
 *
 * Returns:
 * number of valid blocks copied into blocks[]
 *
 * For your test:
 * PIXY2_TARGET_SIGNATURE = 1
 * That means sigmap = 0x01, so Pixy2 only returns signature 1 blocks.
 */
/**
 * @brief Requests and decodes color blocks from the Pixy2 camera.
 *
 * The request is configured for PIXY2_TARGET_SIGNATURE. The response packet is
 * checked for sync bytes, response type, payload length, and checksum before any
 * blocks are copied to the output array.
 *
 * @param blocks Output array for decoded Pixy2 blocks.
 * @param max_blocks Maximum number of blocks to copy.
 * @return Number of valid blocks decoded.
 */
uint8_t getCameraBlocks(PixyBlock_t *blocks, uint8_t max_blocks)
{
    uint8_t request[6];
    uint8_t response_raw[PIXY2_RESPONSE_BUFFER_SIZE];

    if (blocks == NULL || max_blocks == 0)
    {
        return 0;
    }

    if (max_blocks > PIXY2_MAX_BLOCKS)
    {
        max_blocks = PIXY2_MAX_BLOCKS;
    }

    /*
     * Sigmap controls which signatures Pixy2 returns.
     *
     * Signature 1 -> bit 0 -> 0x01
     * Signature 2 -> bit 1 -> 0x02
     * Signature 3 -> bit 2 -> 0x04
     *
     * Since your red target is Signature 1, sigmap = 0x01.
     */
    uint8_t sigmap = (uint8_t)(1 << (PIXY2_TARGET_SIGNATURE - 1));

    /*
     * getBlocks(sigmap, maxBlocks)
     *
     * Byte 0: 0xAE
     * Byte 1: 0xC1
     * Byte 2: 0x20, getBlocks request
     * Byte 3: 0x02, payload length
     * Byte 4: sigmap
     * Byte 5: max blocks
     */
    request[0] = PIXY2_REQ_SYNC_1;
    request[1] = PIXY2_REQ_SYNC_2;
    request[2] = PIXY2_TYPE_GET_BLOCKS_REQ;
    request[3] = 0x02;
    request[4] = sigmap;
    request[5] = max_blocks;

    HAL_StatusTypeDef status = Pixy2_SendRequest(request,
                                                 sizeof(request),
                                                 response_raw,
                                                 sizeof(response_raw));

    if (status != HAL_OK)
    {
        return 0;
    }

    int16_t start = Pixy2_FindResponseStart(response_raw, sizeof(response_raw));

    if (start < 0)
    {
        return 0;
    }

    uint8_t *response = &response_raw[start];

    /*
     * Response format:
     * [0] = 0xAF
     * [1] = 0xC1
     * [2] = packet type, should be 0x21
     * [3] = payload length
     * [4] = checksum low
     * [5] = checksum high
     * [6...] = block data
     */
    if ((start + 6) >= PIXY2_RESPONSE_BUFFER_SIZE)
    {
        return 0;
    }

    if (response[0] != PIXY2_RESP_SYNC_1 ||
        response[1] != PIXY2_RESP_SYNC_2)
    {
        return 0;
    }

    if (response[2] != PIXY2_TYPE_GET_BLOCKS_RESP)
    {
        return 0;
    }

    uint8_t payload_len = response[3];

    if (payload_len < PIXY2_BLOCK_SIZE)
    {
        return 0;
    }

    if ((start + 6 + payload_len) > PIXY2_RESPONSE_BUFFER_SIZE)
    {
        return 0;
    }

    uint16_t received_checksum = Pixy2_ReadU16(&response[4]);
    uint16_t calculated_checksum = Pixy2_Checksum(&response[6], payload_len);

    if (received_checksum != calculated_checksum)
    {
        return 0;
    }

    uint8_t available_blocks = payload_len / PIXY2_BLOCK_SIZE;

    if (available_blocks > max_blocks)
    {
        available_blocks = max_blocks;
    }

    for (uint8_t i = 0; i < available_blocks; i++)
    {
        uint8_t *block = &response[6 + (i * PIXY2_BLOCK_SIZE)];

        blocks[i].signature = Pixy2_ReadU16(&block[0]);
        blocks[i].x         = Pixy2_ReadU16(&block[2]);
        blocks[i].y         = Pixy2_ReadU16(&block[4]);
        blocks[i].width     = Pixy2_ReadU16(&block[6]);
        blocks[i].height    = Pixy2_ReadU16(&block[8]);
        blocks[i].angle     = Pixy2_ReadS16(&block[10]);
        blocks[i].index     = block[12];
        blocks[i].age       = block[13];
    }

    return available_blocks;
}

/* Finds the red target from Pixy2 blocks, since red object is Signature 1,
this searches for: block.signature == PIXY2_TARGET_SIGNATURE
 If multiple red blocks are visible, it chooses the largest one by area.*/
/**
 * @brief Finds the best target block and computes camera pixel error.
 *
 * The function filters blocks by target signature and minimum area, chooses the
 * largest valid target, and calculates x/y error relative to the image center.
 *
 * @param target Output structure containing the selected target and errors.
 * @return true if a valid target is found, otherwise false.
 */
bool Camera_FindTarget(PixyTarget_t *target)
{
    PixyBlock_t blocks[PIXY2_MAX_BLOCKS];
    uint8_t block_count;
    uint8_t best_index = 0;
    uint32_t best_area = 0;
    bool found = false;

    if (target == NULL)
    {
        return false;
    }

    target->target_detected = false;
    target->x = 0;
    target->y = 0;
    target->width = 0;
    target->height = 0;
    target->x_error = 0;
    target->y_error = 0;

    block_count = getCameraBlocks(blocks, PIXY2_MAX_BLOCKS);

    if (block_count == 0)
    {
        return false;
    }

    for (uint8_t i = 0; i < block_count; i++)
    {
        if (blocks[i].signature == PIXY2_TARGET_SIGNATURE)
        {
            uint32_t area = (uint32_t)blocks[i].width * (uint32_t)blocks[i].height;

            if (area < PIXY2_MIN_TARGET_AREA) {
            	continue;
            }

            if (!found || area > best_area)
            {
                found = true;
                best_area = area;
                best_index = i;
            }
        }
    }

    if (!found)
    {
        return false;
    }

    target->target_detected = true;
    target->x = blocks[best_index].x;
    target->y = blocks[best_index].y;
    target->width = blocks[best_index].width;
    target->height = blocks[best_index].height;

    target->x_error = (int16_t)target->x - PIXY2_CENTER_X;
    target->y_error = (int16_t)target->y - PIXY2_CENTER_Y;

    return true;
}

/**
 * @brief Determines whether the target is centered in the Pixy2 image.
 *
 * @param target Pointer to the processed target structure.
 * @return true when the target is detected and both axis errors are within the configured deadband.
 */
bool Camera_TargetCentered(PixyTarget_t *target)
{
    if (target == NULL)
    {
        return false;
    }

    if (!target->target_detected)
    {
        return false;
    }

    if ((target->x_error < PIXY2_X_DEADBAND) &&
        (target->x_error > -PIXY2_X_DEADBAND) &&
        (target->y_error < PIXY2_Y_DEADBAND) &&
        (target->y_error > -PIXY2_Y_DEADBAND))
    {
        return true;
    }

    return false;
}

/**
 * @brief Returns the latest horizontal camera error.
 * @param target Pointer to the processed target structure.
 * @return X-axis error in pixels, or 0 for NULL input.
 */
int16_t Camera_GetXError(PixyTarget_t *target)
{
    if (target == NULL)
    {
        return 0;
    }

    return target->x_error;
}

/**
 * @brief Returns the latest vertical camera error.
 * @param target Pointer to the processed target structure.
 * @return Y-axis error in pixels, or 0 for NULL input.
 */
int16_t Camera_GetYError(PixyTarget_t *target)
{
    if (target == NULL)
    {
        return 0;
    }

    return target->y_error;
}
