#include "pid.hpp"
#include <algorithm>
#include <cmath>


namespace picomotorcontroller
{
    // Timer dispatch
    // user_data was set to 'tihs' in start() - cast it back and call update()
    bool PID::timer_dispatch_(repeating_timer_t* rt)
    {
        static_cast<PID*>(rt->user_data)->update();
        return true;
    }

    PID::PID(Motor& motor, Gains gains, float output_limit, float integral_limit)
        : motor_(motor),
          gains_(gains),
          output_limit_(output_limit),
          integral_limit_(integral_limit),
          target_rpm_(0.0f),
          e_prev_(0.0f),
          e_integral_(0.0f),
          last_update_us_(to_us_since_boot(get_absolute_time()))
    {}

    PID::~PID()
    {
        stop();
    }

    void PID::set_target(float rpm)
    {
        target_rpm_ = rpm;
    }

    void PID::start(uint32_t freq_hz)
    {
        last_update_us_ = to_us_since_boot(get_absolute_time());

        // If the delay is > 0 then this is the delay between the previous callback ending and the next starting.
        // If the delay is negative (see below) then the next call to the callback will be exactly period_ms after the
        // start of the call to the last callback
        int32_t period_ms = -(int32_t)(1000 / freq_hz); // negative = end-to-start
        add_repeating_timer_ms(
            period_ms,
            timer_dispatch_,
            this,
            &timer_
        );
    }

    void PID::stop()
    {
        cancel_repeating_timer(&timer_);
        motor_.stop();
    }

    void PID::update()
    {
        uint64_t now_us = to_us_since_boot(get_absolute_time());
        float dt = (float)(now_us - last_update_us_) / 1'000'000.0f;    // seconds
        last_update_us_ = now_us;

        if (dt <= 0.0f) return;

        float current_rpm   = motor_.encoder().get_speed();
        float output        = compute(current_rpm, target_rpm_, dt);

        motor_.set_speed((int)output);
    }

    float PID::compute(float current, float target, float dt_seconds)
    {
        float e     = target - current; 

        // proportional
        float p     = gains_.kp * e;

        // derivative
        float dedt  = (e - e_prev_) / dt_seconds;
        float d     = gains_.kd * dedt;

        // integral
        e_integral_ += e * dt_seconds;
        e_integral_ = std::clamp(e_integral_, -integral_limit_, integral_limit_);
        float i     = gains_.ki * e_integral_;

        // feedforward
        // kff * target gives a baseline proportional to desired RPM,
        // so the PID only has to correct the remaining error
        float ff    = gains_.kff * target;

        // sum + output clamp
        float output = std::clamp(ff + p + i + d, -output_limit_, output_limit_);

        e_prev_ = e;

        return output;
    }

}    // namespace picomotorcontroller