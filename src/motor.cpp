#include "motor.hpp"
#include <algorithm>


using picomotorcontroller::Motor;

Motor::Motor(Encoder& enc, uint p_in1, uint p_in2, uint p_en, uint32_t freq)
    : enc_(enc), 
      pin_in1_(p_in1),
      pin_in2_(p_in2),
      pin_en_(p_en),
      freq_(freq)
{
    // Direction pins
    gpio_init(pin_in1_);    
    gpio_set_dir(pin_in1_, GPIO_OUT);
    gpio_put(pin_in1_, 0);

    gpio_init(pin_in2_);
    gpio_set_dir(pin_in2_, GPIO_OUT);
    gpio_put(pin_in2_, 0);

    // PWM pin setup
    // Tell the GPIO mux to route this pin to the PWM hardware
    gpio_set_function(pin_en_, GPIO_FUNC_PWM);

    // Every GPIO maps to one slice + channel
    pwm_slice_ = pwm_gpio_to_slice_num(pin_en_);    // 0-7
    pwm_channel_ = pwm_gpio_to_channel(pin_en_);    // CHAN_A - CHAN_B

    // "wrap" - the counter top value - counts 0...wrap, then resets
    // This sets the PWM period = (wrap + 1) / f_sys
    // f_sys default = 125MHz. For freq=1000Hz, wrap = 125000000 / 1000 - 1 = 124999
    uint32_t wrap = (125000000 / freq_) - 1;
    pwm_set_wrap(pwm_slice_, wrap);

    // Start at 0 duty (motor off)
    pwm_set_chan_level(pwm_slice_, pwm_channel_, 0);

    // Enable the PWM slice
    pwm_set_enabled(pwm_slice_, true);
}

Motor::~Motor()
{
    stop();
    pwm_set_enabled(pwm_slice_, false);
    gpio_put(pin_in1_, 0);
    gpio_put(pin_in2_, 0);
}

// Equivalent to Arduino map()
int Motor::map_range(int x, int i_m, int i_M, int o_m, int o_M)
{
    int result = (x - i_m) * (o_M - o_m) / (i_M - i_m) + o_m;
    return std::clamp(result, o_m, o_M);
}

void Motor::set_speed(int M)
{
    M = std::clamp(M, -1000, 1000);

    // set the direction
    if (M > 0)
    {
        gpio_put(pin_in1_, 1);
        gpio_put(pin_in2_, 0);
    } 
    else if (M < 0)
    {
        gpio_put(pin_in1_, 0);
        gpio_put(pin_in2_, 1);
    }
    else
    {
        gpio_put(pin_in1_, 0);
        gpio_put(pin_in2_, 0);
    }

    // PWM duty, map |M| from [0, 1000] to [0, wrap]
    uint32_t wrap = (125000000 / freq_) - 1;
    uint32_t level = map_range(std::abs(M), 0, 1000, 0, (int)wrap);
    pwm_set_chan_level(pwm_slice_, pwm_channel_, level);
}

void Motor::stop()
{
    set_speed(0);
}