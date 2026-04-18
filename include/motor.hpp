#pragma once

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "encoder.hpp"
#include <memory>

namespace picomotorcontroller
{
    class Motor
    {
    public:
        Motor(Encoder& enc, uint pin_in1, uint pin_in2, uint p_en, uint32_t freq);
        ~Motor();
        void set_speed(int M);
        void stop();
        Encoder& encoder() { return enc_; }

    private:
        Encoder& enc_;
        uint pin_in1_;
        uint pin_in2_;
        uint pin_en_;
        uint32_t freq_;
        
        uint pwm_slice_;
        uint pwm_channel_;
    
        static int map_range(int x, int i_m, int i_M, int o_m, int o_M);
    };
}    // namespace picomotorcontroller
