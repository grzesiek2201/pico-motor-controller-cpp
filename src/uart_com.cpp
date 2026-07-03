#include "uart_com.hpp"
#include <cstring>
#include <algorithm>

namespace picomotorcontroller
{
    UARTCom::UARTCom(uart_inst_t* uart_id, uint pin_tx, uint pin_rx, uint32_t baudrate)
        : uart_(uart_id)
    {
        uart_init(uart_, baudrate);
        gpio_set_function(pin_tx, GPIO_FUNC_UART);
        gpio_set_function(pin_rx, GPIO_FUNC_UART);
        uart_set_format(uart_, 8, 1, UART_PARITY_NONE);
        uart_set_fifo_enabled(uart_, true);
        rx_buf_.reserve(buf_len_);
    }

    UARTCom::~UARTCom()
    {
        uart_deinit(uart_);
    }

    // Builds: [HEADER][TYPE][LEN][PAYLOAD...][CRC8]
    // CRC covers TYPE + LEN + PAYLOAD — not the header byte.
    template<typename T>
    void UARTCom::send_frame_(uint8_t msg_type, const T& payload)
    {
        constexpr uint8_t len = sizeof(T);

        uint8_t frame[3 + len + 1]; // header + type + len + payload + crc
        frame[0] = protocol::HEADER;
        frame[1] = msg_type;
        frame[2] = len;
        std::memcpy(&frame[3], &payload, len);

        // CRC covers everything after the header
        frame[3 + len] = protocol::crc8(&frame[1], 2 + len);

        uart_write_blocking(uart_, frame, sizeof(frame));
    }

    void UARTCom::send_odom(const protocol::OdomPayload& odom)
    {
        send_frame_(protocol::MSG_ODOM, odom);
    }

    void UARTCom::drain_uart_()
    {
        while (uart_is_readable(uart_))
        {
            rx_buf_.push_back(uart_getc(uart_));
        }
    }

    std::optional<UARTCom::RawFrame> UARTCom::parse_frame_()
    {
        while (true)
        {
            // Find the header byte
            auto it = std::find(rx_buf_.begin(), rx_buf_.end(), protocol::HEADER);
            if (it == rx_buf_.end()) { rx_buf_.clear(); return std::nullopt; }

            // Discard garbage before header
            if (it != rx_buf_.begin()) rx_buf_.erase(rx_buf_.begin(), it);

            // Need at least: HEADER(1) + TYPE(1) + LEN(1) + CRC(1) = 4 bytes minimum
            if (rx_buf_.size() < 4) return std::nullopt;

            uint8_t msg_type    = rx_buf_[1];
            uint8_t payload_len = rx_buf_[2];

            // Sanity check payload length
            if (payload_len > buf_len_ - 4)  // payload can't be larger than buffer minus header/type/len/crc
            {
                rx_buf_.erase(rx_buf_.begin()); // Skip bad header, try again
                continue;
            }

            // Wait for full frame
            size_t frame_size = 1 + 1 + 1 + payload_len + 1;
            if (rx_buf_.size() < frame_size) return std::nullopt;

            // Validate CRC
            uint8_t expected_crc    = protocol::crc8(&rx_buf_[1], 2 + payload_len);
            uint8_t actual_crc      = rx_buf_[3 + payload_len];

            if (actual_crc != expected_crc)
            {
                // Bad CRC - discard this header byte and try next
                rx_buf_.erase(rx_buf_.begin());
                continue;
            }

            // Valid frame - extract payload
            RawFrame frame;
            frame.type = msg_type;
            frame.payload.assign(rx_buf_.begin() + 3, rx_buf_.begin() + 3 + payload_len);
            rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + frame_size);  // Remove this frame from buffer

            return frame;
        }
    }

    std::optional<protocol::CmdPayload> UARTCom::receive_cmd()
    {
        drain_uart_();

        auto frame = parse_frame_();
        if (!frame.has_value()) return std::nullopt;

        // Ignore frames that aren't CMD type or wrong size
        if (frame->type != protocol::MSG_CMD ||
            frame->payload.size() != sizeof(protocol::CmdPayload))
        {
            return std::nullopt;
        }

        protocol::CmdPayload cmd;
        std::memcpy(&cmd, frame->payload.data(), sizeof(cmd));
        return cmd;
    }

}   // namespace picomotorcontroller