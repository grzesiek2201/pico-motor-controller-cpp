#pragma once

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "protocol.hpp"
#include <optional>
#include <vector>

namespace picomotorcontroller
{
    class UARTCom
    {
    public:
        UARTCom(uart_inst_t* uart_id, uint pin_tx, uint pin_rx, uint32_t baudrate = 115200);
        ~UARTCom();

        // Send encoder data to host
        void send_odom(const protocol::OdomPayload& odom);

        // Returns a command if complete, valid frame was received
        std::optional<protocol::CmdPayload> receive_cmd();

    private:
        uart_inst_t* uart_;
        std::vector<uint8_t> rx_buf_;
        uint8_t buf_len_ = 64;

        // Frames and payload struct into [HEADER][TYPE][LEN][PAYLOAD][CRC]
        template<typename T>
        void send_frame_(uint8_t msg_type, const T& payload);

        // Tries to parse one complete valid frame from rx_buf_
        // Returns msg_type + payload bytes if successful
        struct RawFrame { uint8_t type; std::vector<uint8_t> payload; };
        std::optional<RawFrame> parse_frame_();

        void drain_uart_();
    };
    
}   // namespace picomotorcontroller