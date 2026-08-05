#include "robot_protocol.h"

#include <string.h>

#define FRAME_HEADER_0       (0xA5U)
#define FRAME_HEADER_1       (0x5AU)
#define FRAME_FIXED_SIZE     (10U)
#define FRAME_CRC_INPUT_POS  (2U)
#define FRAME_PAYLOAD_POS    (8U)

void robot_protocol_parser_init (robot_parser_t * parser)
{
    parser->count = 0U;
}

uint16_t robot_protocol_crc16 (uint8_t const * data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0U; i < length; i++)
    {
        crc ^= (uint16_t) data[i] << 8;
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            crc = (0U != (crc & 0x8000U)) ? (uint16_t) ((crc << 1) ^ 0x1021U) : (uint16_t) (crc << 1);
        }
    }
    return crc;
}

size_t robot_protocol_encode (uint8_t type, uint16_t sequence, uint8_t const * payload, uint8_t payload_length,
                              uint8_t * output, size_t output_capacity)
{
    size_t const total = FRAME_FIXED_SIZE + payload_length;
    if ((payload_length > ROBOT_PROTOCOL_MAX_PAYLOAD) || (output_capacity < total))
    {
        return 0U;
    }

    output[0] = FRAME_HEADER_0;
    output[1] = FRAME_HEADER_1;
    output[2] = ROBOT_PROTOCOL_VERSION;
    output[3] = type;
    robot_protocol_put_u16(&output[4], sequence);
    output[6] = payload_length;
    output[7] = 0U;
    if ((payload_length > 0U) && (NULL != payload))
    {
        memcpy(&output[FRAME_PAYLOAD_POS], payload, payload_length);
    }
    uint16_t const crc = robot_protocol_crc16(&output[FRAME_CRC_INPUT_POS], 6U + payload_length);
    robot_protocol_put_u16(&output[FRAME_PAYLOAD_POS + payload_length], crc);
    return total;
}

bool robot_protocol_parse_byte (robot_parser_t * parser, uint8_t byte, robot_frame_t * frame)
{
    if ((0U == parser->count) && (FRAME_HEADER_0 != byte))
    {
        return false;
    }
    if ((1U == parser->count) && (FRAME_HEADER_1 != byte))
    {
        parser->count = (FRAME_HEADER_0 == byte) ? 1U : 0U;
        parser->buffer[0] = byte;
        return false;
    }

    if (parser->count >= sizeof(parser->buffer))
    {
        parser->count = 0U;
        return false;
    }
    parser->buffer[parser->count++] = byte;

    if (parser->count >= FRAME_PAYLOAD_POS)
    {
        uint8_t const length = parser->buffer[6];
        if ((parser->buffer[2] != ROBOT_PROTOCOL_VERSION) || (length > ROBOT_PROTOCOL_MAX_PAYLOAD))
        {
            parser->count = 0U;
            return false;
        }
        size_t const expected = FRAME_FIXED_SIZE + length;
        if (parser->count == expected)
        {
            uint16_t const received_crc = robot_protocol_get_u16(&parser->buffer[FRAME_PAYLOAD_POS + length]);
            uint16_t const calculated_crc = robot_protocol_crc16(&parser->buffer[FRAME_CRC_INPUT_POS], 6U + length);
            if (received_crc == calculated_crc)
            {
                frame->type = parser->buffer[3];
                frame->sequence = robot_protocol_get_u16(&parser->buffer[4]);
                frame->length = length;
                if (length > 0U)
                {
                    memcpy(frame->payload, &parser->buffer[FRAME_PAYLOAD_POS], length);
                }
                parser->count = 0U;
                return true;
            }
            parser->count = 0U;
        }
    }
    return false;
}

uint16_t robot_protocol_get_u16 (uint8_t const * data)
{
    return (uint16_t) data[0] | ((uint16_t) data[1] << 8);
}

int16_t robot_protocol_get_i16 (uint8_t const * data)
{
    return (int16_t) robot_protocol_get_u16(data);
}

int32_t robot_protocol_get_i32 (uint8_t const * data)
{
    uint32_t const value = (uint32_t) data[0] | ((uint32_t) data[1] << 8) | ((uint32_t) data[2] << 16) |
                           ((uint32_t) data[3] << 24);
    return (int32_t) value;
}

void robot_protocol_put_u16 (uint8_t * data, uint16_t value)
{
    data[0] = (uint8_t) value;
    data[1] = (uint8_t) (value >> 8);
}

void robot_protocol_put_i16 (uint8_t * data, int16_t value)
{
    robot_protocol_put_u16(data, (uint16_t) value);
}

void robot_protocol_put_i32 (uint8_t * data, int32_t value)
{
    uint32_t const encoded = (uint32_t) value;
    data[0] = (uint8_t) encoded;
    data[1] = (uint8_t) (encoded >> 8);
    data[2] = (uint8_t) (encoded >> 16);
    data[3] = (uint8_t) (encoded >> 24);
}
