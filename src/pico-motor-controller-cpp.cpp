#include "pico/stdlib.h"
#include "encoder.hpp"
#include "motor.hpp"
#include "pid.hpp"
#include <cstdio>

using picomotorcontroller::Encoder;
using picomotorcontroller::Motor;
using picomotorcontroller::PID;

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