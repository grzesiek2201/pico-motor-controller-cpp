#pragma once

#include "pico/stdlib.h"      // gpio_init, gpio_set_dir, sleep_ms
#include "hardware/gpio.h"    // gpio_set_irq_enabled_with_callback


namespace picomotorcontroller
{
        
    class Encoder
    {
    public:
        Encoder(uint pin_a, uint pin_b,
                float tick_per_rev = 11.0f,
                float reduction_ratio = 35.0f);
        
        ~Encoder();

        int32_t get_pos() const;
        float   get_speed();    // non-const: updates internal smoothed speed

        void reset();

        float last_calculated_speed;

    private:
        uint pin_a;
        uint pin_b;
        float tick_per_rev;
        float reduction_ratio;
        float smoothed_speed_;

        // written only by ISR
        volatile int32_t pos_;
        // snapshot of last time get_speed() was called
        int32_t last_speed_pos_;
        uint32_t last_speed_time_ms_;
        
        void isr_tick();    // called by dispatcher

        static const float ALPHA;   // smoothing factor

        static Encoder* registry_[30];
        // Single global dispatcher for all encoder instances - the gpio is assigned to one encoder and is used to index the registry
        static void dispatch_(uint gpio, uint32_t events);
    };

}    // namespace picomotorcontroller