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
#ifndef ENCODER_H
#define ENCODER_H

#include "gpio.h"

namespace Mcudrv {
namespace Encoders {
enum Direction
{
    noAction,
    increment,
    decrement
};

template<typename T, T min, T max>
class Value
{
private:
    T value_;
public:
    typedef T value_type;
    enum
    {
        min_value = min,
        max_value = max
    };
    Value(T initial = min) : value_(initial)
    { }
    Value& operator++()
    {
        if(value_ < max)
            ++value_;
        return *this;
    }
    Value& operator--()
    {
        if(value_ > min)
            --value_;
        return *this;
    }
    operator T()
    {
        return value_;
    }
    T GetValue()
    {
        return value_;
    }
};

// ---=== Encoder types ===---

// 18 steps per revolution
struct Bldc
{
#pragma inline = forced
    static Direction Process(uint8_t state)
    {
        static uint8_t prev_state;
        Direction dir;
        if(state != prev_state) {
            switch(prev_state) {
                case 1: {
                    if(state == 5)
                        dir = increment;
                    if(state == 3)
                        dir = decrement;
                } break;
                case 5: {
                    if(state == 4)
                        dir = increment;
                    if(state == 1)
                        dir = decrement;
                } break;
                case 4: {
                    if(state == 6)
                        dir = increment;
                    if(state == 5)
                        dir = decrement;
                } break;
                case 6: {
                    if(state == 2)
                        dir = increment;
                    if(state == 4)
                        dir = decrement;
                } break;
                case 2: {
                    if(state == 3)
                        dir = increment;
                    if(state == 6)
                        dir = decrement;
                } break;
                case 3: {
                    if(state == 1)
                        dir = increment;
                    if(state == 2)
                        dir = decrement;
                } break;
                default:
                    dir = noAction;
            }
            prev_state = state;
            return dir;
        }
        return noAction;
    }
};

// 9 steps per revolution
struct BldcHalf
{
#pragma inline = forced
    static Direction Process(uint8_t state)
    {
        static uint8_t prev_state;
        Direction dir;
        if(state != prev_state) {
            switch(prev_state) {
                case 1: {
                    if(state == 5)
                        dir = increment;
                    if(state == 3)
                        dir = decrement;
                } break;
                case 4: {
                    if(state == 6)
                        dir = increment;
                    if(state == 5)
                        dir = decrement;
                } break;
                case 2: {
                    if(state == 3)
                        dir = increment;
                    if(state == 6)
                        dir = decrement;
                } break;
                default:
                    dir = noAction;
            }
            prev_state = state;
            return dir;
        }
        return noAction;
    }
};

// 12 Detents(steps) / 24 Pulses per revolution
// TODO: Not checked yet
struct Mechanical
{
#pragma inline = forced
    static Direction Process(uint8_t state)
    {
        static uint8_t prev_state;
        Direction dir;
        if(state != prev_state) {
            switch(prev_state) {
                    //	case 2:{
                    //		if(state == 3) dir = decrement;
                    //		if(state == 0) dir = increment;
                    //	}
                    //		break;
                    //	case 0:{
                    //		if(state == 2) dir = decrement;
                    //		if(state == 1) dir = increment;
                    //	}
                    //		break;
                case 1: {
                    if(state == 0)
                        dir = decrement;
                    if(state == 3)
                        dir = increment;
                } break;
                    //	case 3:{
                    //		if(state == 1) dir = decrement;
                    //		if(state == 2) dir = increment;
                    //	}
                    //		break;
                default:
                    dir = noAction;
            }
            prev_state = state;
            return dir;
        }
        return noAction;
    }
};

// ---=== Main class ===---

template<typename Pins, typename EncType = Mechanical>
class Encoder
{
public:
    static void InitGpio()
    {
        Pins::template SetConfig<GpioBase::In_Pullup>();
    }
#pragma inline = forced
    template<typename T>
    static bool IsChanged(T& value)
    {
        Direction dir = EncType::Process(Pins::Read());
        if(dir) {
            if(dir == increment)
                ++value;
            else
                --value;
            return true;
        }
        else
            return false;
    }
};

} // Encoder
} // Mcudrv

#endif // ENCODER_H
