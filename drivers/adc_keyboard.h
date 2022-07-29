/*
 * Copyright (c) 2017 Dmytro Shestakov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include "adc.h"

namespace Mcudrv {
namespace AdcKeys {
using namespace Adcs;

template<uint8_t _SampleCount,
         uint32_t Rcom,
         uint32_t R1,
         uint32_t R2,
         uint32_t R3,
         uint32_t R4,
         uint32_t R5,
         uint32_t R6,
         uint32_t R7>
struct KeyboardTraits
{
    static const uint8_t max_value = 0xFF;
    static const uint8_t SampleCount = _SampleCount;
    static const uint16_t tolerance = max_value / 20; // 5%
    static const uint16_t WatchdogThreshold = max_value - 4; //влияет на срабатывание последней кнопки
    static const uint16_t B1value = max_value * R1 / (Rcom + R1);
    static const uint16_t B2value = max_value * (R1 + R2) / (Rcom + R1 + R2);
    static const uint16_t B3value = max_value * (R1 + R2 + R3) / (Rcom + R1 + R2 + R3);
    static const uint16_t B4value = max_value * (R1 + R2 + R3 + R4) / (Rcom + R1 + R2 + R3 + R4);
    static const uint16_t B5value = max_value * (R1 + R2 + R3 + R4 + R5) / (Rcom + R1 + R2 + R3 + R4 + R5);
    static const uint16_t B6value = max_value * (R1 + R2 + R3 + R4 + R5 + R6) / (Rcom + R1 + R2 + R3 + R4 + R5 + R6);
    static const uint16_t B7value =
      max_value * (R1 + R2 + R3 + R4 + R5 + R6 + R7) / (Rcom + R1 + R2 + R3 + R4 + R5 + R6 + R7);
    static const uint16_t B1value_max = B1value + tolerance;
    static const uint16_t B2value_max = B2value + tolerance;
    static const uint16_t B3value_max = B3value + tolerance;
    static const uint16_t B4value_max = B4value + tolerance;
    static const uint16_t B5value_max = B5value + tolerance;
    static const uint16_t B6value_max = B6value + tolerance;
    static const uint16_t B7value_max = B7value + tolerance;
    static const uint16_t B1value_min = B1value - tolerance;
    static const uint16_t B2value_min = B2value - tolerance;
    static const uint16_t B3value_min = B3value - tolerance;
    static const uint16_t B4value_min = B4value - tolerance;
    static const uint16_t B5value_min = B5value - tolerance;
    static const uint16_t B6value_min = B6value - tolerance;
    static const uint16_t B7value_min = B7value - tolerance;
};

typedef KeyboardTraits<0xFF, 10000UL, 2400UL, 3300UL, 4300UL, 5100UL, 8200UL, 16000UL, 56000UL> DefaultCfg;

template<typename Handler, typename Traits = DefaultCfg>
struct Buttons : Adcs::Adc<Mode8Bit>
{
#pragma inline = forced
    template<Channel ch, Adcs::Div clockdiv>
    static void Init(/*callback_t cb*/)
    {
        Adc::Init<ContMode, clockdiv>();
        SelectChannel(ch);
        WatchdogInit<0, Traits::WatchdogThreshold>();
        EnableInterrupt(AnalogWatchdog);
        Enable();
        StartConversion();
    }
private:
    //			static callback_t cb_;
    _Pragma(VECTOR_ID(ADC1_AWDG_vector)) __interrupt static void AnalogWatchdogISR()
    {
        static uint8_t scount;
        if(IsEvent<AnalogWatchdog>()) {
            ClearEvent<AnalogWatchdog>();
            static uint8_t prev_key;
            uint8_t key;
            uint8_t sample = Buttons::ReadSample();
            if(sample > Traits::B1value_min && sample < Traits::B1value_max)
                key = 0;
            if(sample > Traits::B2value_min && sample < Traits::B2value_max)
                key = 1;
            if(sample > Traits::B3value_min && sample < Traits::B3value_max)
                key = 2;
            if(sample > Traits::B4value_min && sample < Traits::B4value_max)
                key = 3;
            if(sample > Traits::B5value_min && sample < Traits::B5value_max)
                key = 4;
            if(sample > Traits::B6value_min && sample < Traits::B6value_max)
                key = 5;
            if(sample > Traits::B7value_min && sample < Traits::B7value_max)
                key = 6;
            if(key == prev_key) {
                if(scount != Traits::SampleCount)
                    scount++;
            }
            else {
                prev_key = key;
                scount = 0;
            }
            if(scount == Traits::SampleCount - 1)
                Handler::KeyboardHandler(key);
            // cb_(key);
        }
    }
};

//		template<typename Traits>
//		callback_t Buttons<Traits>::cb_;

} // AdcKeys

} // Mcudrv
