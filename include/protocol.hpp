#pragma once

#include <cstdint>

// messages should be of structure: [HEADER][TYPE][LEN][PAYLOAD][CRC]
namespace protocol
{

    static constexpr uint8_t HEADER     = 0xAA;
    static constexpr uint8_t MSG_ODOM   = 0x01; // pico -> host
    static constexpr uint8_t MSG_CMD    = 0x02; // host -> pico

    // __attribute__((packed)) ensures no padding bytes are inserted.
    struct __attribute__((packed)) OdomPayload
    {
        float left_pos; // encoder ticks or radians
        float left_vel; // RPM
        float right_pos;
        float right_vel;
    };

    struct __attribute__((packed)) CmdPayload
    {
        float left_rpm; // desired speed in RPM, signed
        float right_rpm;
    };

    // Covers msg_type + payload_len + payload bytes.
    inline uint8_t crc8(const uint8_t* data, size_t len)
    {
        uint8_t crc = 0x00;
        for (size_t i = 0; i < len; i++)
        {
            crc ^= data[i];
            for (int j = 0; j < 8; j++)
            {
                crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
            }
        }
        return crc;
    }

}   // namespace protocol