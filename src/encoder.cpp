#include "encoder.hpp"

using picomotorcontroller::Encoder;

const float Encoder::ALPHA = 0.7f;
Encoder* Encoder::registry_[30] = {};

// ISR dispatcher
void Encoder::dispatch_(uint gpio, uint32_t events)
{
    if (gpio < 30 && registry_[gpio])
    {
        registry_[gpio]->isr_tick();
    }
}

// Constructor
Encoder::Encoder(uint pin_a, uint pin_b, float tick_per_rev, float reduction_ratio)
    : pin_a(pin_a), pin_b(pin_b),
        tick_per_rev(tick_per_rev), reduction_ratio(reduction_ratio),
        smoothed_speed_(0.0f), pos_(0),
        last_speed_pos_(0),
        last_speed_time_ms_(to_ms_since_boot(get_absolute_time())),
        last_calculated_speed(0.0f)
{
    registry_[pin_b] = this;

    gpio_init(pin_a);   gpio_set_dir(pin_a, GPIO_IN);
    gpio_init(pin_b);   gpio_set_dir(pin_b, GPIO_IN);
    
    gpio_set_irq_enabled_with_callback(
        pin_b, GPIO_IRQ_EDGE_RISE, true, &dispatch_
    );
}

Encoder::~Encoder()
{
    gpio_set_irq_enabled(pin_b, GPIO_IRQ_EDGE_RISE, false);
    registry_[pin_b] = nullptr;
}

// ISR
void Encoder::isr_tick()
{
    if (gpio_get(pin_a)) pos_++; else pos_--;
}

// Public API
int32_t Encoder::get_pos() const
{
    return pos_;
}

float Encoder::get_speed()
{
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    int32_t now_pos = pos_;

    uint32_t dt = now_ms - last_speed_time_ms_;
    if (dt == 0) return smoothed_speed_;

    float ds        = (float)(now_pos - last_speed_pos_) / tick_per_rev / reduction_ratio;
    float new_speed = (ds / (float)dt) * 1000.0f * 60.0f;

    smoothed_speed_     = ALPHA * smoothed_speed_ + (1.0f - ALPHA) * new_speed;
    last_speed_time_ms_ = now_ms;
    last_speed_pos_     = now_pos;
    
    last_calculated_speed = new_speed;

    return smoothed_speed_;
}

void Encoder::reset()
{
    pos_            = 0;
    last_speed_pos_ = 0;
    smoothed_speed_ = 0.0f;
}
