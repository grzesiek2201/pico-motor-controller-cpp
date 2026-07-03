#include "pico/stdlib.h"
#include "encoder.hpp"
#include "motor.hpp"
#include "pid.hpp"
#include "uart_com.hpp"
#include <cstdio>

using picomotorcontroller::Encoder;
using picomotorcontroller::Motor;
using picomotorcontroller::PID;
using picomotorcontroller::UARTCom;

int main()
{
    stdio_init_all();

    Encoder enc_left(4, 5);
    Encoder enc_right(2, 3);
    
    Motor motor_left(enc_left, 12, 11, 10, 1000);
    Motor motor_right(enc_right, 8, 7, 6, 1000);

    PID pid_left(motor_left, {
        .kp = 3.0f,
        .ki = 1.5f,
        .kd = 0.3f,
        .kff = 3.54f
    });
    PID pid_right(motor_right, {
        .kp = 3.0f,
        .ki = 1.5f,
        .kd = 0.3f,
        .kff = 3.54f
    });

    UARTCom uart(uart0, 0, 1, 115200);

    // Send odom at 50Hz
    repeating_timer_t odom_timer;
    add_repeating_timer_ms(-20, [](repeating_timer_t* rt) -> bool {
            // Retrieve the tuple from the void pointer from user_data
            auto* ctx = static_cast<std::tuple<UARTCom*, Encoder*, Encoder*>*>(rt->user_data);
            // Unpack the tuple into named pointers
            auto& [comm, left, right] = *ctx;
            // Build the OdomPayload inline and send it
            comm->send_odom(protocol::OdomPayload{
                .left_pos       = (float)left->get_pos(),
                .left_vel     = left->get_speed(),
                .right_pos      = (float)right->get_pos(),
                .right_vel    = right->get_speed()
            });

            return true;    // Return true to keep the timer repeating
        },
        new std::tuple<UARTCom*, Encoder*, Encoder*>{&uart, &enc_left, &enc_right}, // Bundle passed as void* user_data
        &odom_timer
    );

    // motor_left.set_speed(200);
    pid_left.set_target(30.0f);
    pid_left.start(100);

    // motor_right.set_speed(200);
    pid_right.set_target(-30.0f);
    pid_right.start(100);

    while (true)
    {
        sleep_ms(100);
        // printf("L pos: %ld  speed: %.1f | R pos: %ld  speed: %.1f\n",
        //        (long)enc_left.get_pos(),  enc_left.get_speed(),
        //        (long)enc_right.get_pos(), enc_right.get_speed());
        printf("rpm left: %.1f\nrpm right: %.1f", enc_left.last_calculated_speed, enc_right.last_calculated_speed);
    }

}