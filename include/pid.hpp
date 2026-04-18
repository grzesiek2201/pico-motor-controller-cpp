#pragma once

#include "pico/stdlib.h"
#include "pico/time.h"
#include "motor.hpp"


namespace picomotorcontroller
{
    class PID
    {
    public:
        struct Gains {
            float kp = 1.0f;
            float ki = 0.0f;
            float kd = 0.0f;
            float kff = 0.0f;
        };
        // output_limit is the max absolute value sent to Motor::set_speed()
        // integral_limit is the anti-windup clamp on the integral term
        PID(Motor& motor, Gains gains, float output_limit = 800.0f, float intergral_limit = 300.0f);

        ~PID();

        // set the RPM target. Negative = reverse.
        void set_target(float rpm);

        // Start/stop the periodic control loop
        // freq_hz - how many times per second the controller runs (default to 20Hz)
        void start(uint32_t freq_hz = 20);
        void stop();

        // Manually stop the controller once - useful for manual control of the control loop
        // from a separate main loop instead of the internal timer
        void update();

    private:
        Motor& motor_;
        Gains gains_;

        float output_limit_;
        float integral_limit_;
        float target_rpm_;
    
        // PID state
        float e_prev_;
        float e_integral_;

        // Timing
        uint64_t last_update_us_;

        repeating_timer_t timer_;

        // Compute PID output given current value, target, and elapsed time
        float compute(float current, float target, float dt_seconds);

        // Timer dispatch - uses rt->user_data
        static bool timer_dispatch_(repeating_timer_t* rt);
    };

}    // namespace picomotorcontroller