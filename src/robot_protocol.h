#ifndef ROBOT_PROTOCOL_H
#define ROBOT_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ROBOT_PROTOCOL_VERSION       (3U)
#define ROBOT_PROTOCOL_MAX_PAYLOAD   (96U)
#define ROBOT_PROTOCOL_MAX_FRAME     (106U)

typedef enum e_robot_message_type
{
    ROBOT_MSG_CONTROL    = 0x01,
    ROBOT_MSG_CONFIG     = 0x02, /* Reserved by protocol v3; firmware ignores it. */
    ROBOT_MSG_FACE_EVENT = 0x03,
    ROBOT_MSG_TELEMETRY  = 0x81
} robot_message_type_t;

enum
{
    ROBOT_FLAG_ARM         = (1U << 0),
    ROBOT_FLAG_DISARM      = (1U << 1),
    ROBOT_FLAG_HOME        = (1U << 2), /* Reserved; no operation. */
    ROBOT_FLAG_CLEAR_FAULT = (1U << 3),
    ROBOT_FLAG_CALIBRATE   = (1U << 4)  /* Reserved; no operation. */
};

typedef struct st_robot_frame
{
    uint8_t  type;
    uint16_t sequence;
    uint8_t  length;
    uint8_t  payload[ROBOT_PROTOCOL_MAX_PAYLOAD];
} robot_frame_t;

typedef struct st_robot_parser
{
    uint8_t buffer[ROBOT_PROTOCOL_MAX_FRAME];
    size_t  count;
} robot_parser_t;

void     robot_protocol_parser_init(robot_parser_t * parser);
bool     robot_protocol_parse_byte(robot_parser_t * parser, uint8_t byte, robot_frame_t * frame);
uint16_t robot_protocol_crc16(uint8_t const * data, size_t length);
size_t   robot_protocol_encode(uint8_t type, uint16_t sequence, uint8_t const * payload, uint8_t payload_length,
                               uint8_t * output, size_t output_capacity);
uint16_t robot_protocol_get_u16(uint8_t const * data);
int16_t  robot_protocol_get_i16(uint8_t const * data);
int32_t  robot_protocol_get_i32(uint8_t const * data);
void     robot_protocol_put_u16(uint8_t * data, uint16_t value);
void     robot_protocol_put_i16(uint8_t * data, int16_t value);
void     robot_protocol_put_i32(uint8_t * data, int32_t value);

#endif
