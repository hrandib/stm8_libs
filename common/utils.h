/*
 * Copyright (c) 2021 Dmytro Shestakov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef UTILS_H
#define UTILS_H

#include "gpio.h"
#include "type_traits.h"

namespace utils {

template<typename VAL_TYPE, VAL_TYPE MIN, VAL_TYPE MAX, VAL_TYPE STEP>
class RangeLinear
{
    VAL_TYPE value_;
public:
    enum {
        MAXVAL = MAX,
        MINVAL = MIN,
        STEPVAL = STEP,
    };
    RangeLinear(VAL_TYPE init = MIN)
      : value_(init)
    { }

    operator VAL_TYPE() const
    {
        return value_;
    }

    VAL_TYPE operator++()
    {
        if(value_ > (MAX - STEP)) {
            value_ = MAX;
        }
        else {
            value_ += STEP;
        }
        return value_;
    }

    VAL_TYPE operator--()
    {
        if(value_ < (MIN + STEP)) {
            value_ = MIN;
        }
        else {
            value_ -= STEP;
        }
        return value_;
    }

    VAL_TYPE operator=(VAL_TYPE value)
    {
        if(value > MAX) {
            value_ = MAX;
        }
        else if(value < MIN) {
            value_ = MIN;
        }
        else {
            value_ = value;
        }
        return value_;
    }

    void SetMax()
    {
        value_ = MAXVAL;
    }
    void SetMin()
    {
        value_ = MINVAL;
    }
};

template<uint8_t BUTTONS_COUNT, uint16_t TPOLL_MS, uint16_t TLONGPRESS_MS, bool SAFETY_CHECK = false>
class ButtonsLongPress
{
    typedef void (*Callback_t)(bool isLongPress);
    typedef typename stdx::SelectSizeForLength<(TLONGPRESS_MS / TPOLL_MS) + 1>::type Counter_t;
    Callback_t callbacks_[BUTTONS_COUNT];
    Counter_t pollCounters_[BUTTONS_COUNT];
    enum {
        DEBOUNCE_DELAY_MS = 100,
        COUNTER_MAXVAL = stdx::TypeMaxvalue<Counter_t>::value,
    };
public:
    ButtonsLongPress()
      : callbacks_()
      , pollCounters_()
    { }
    // Active level = LOW, the button with pullup
    void UpdateState(uint8_t buttonsMask)
    {
        for(size_t i = 0; i < BUTTONS_COUNT; ++i) {
            if(buttonsMask & ~(1 << i)) {
                if((pollCounters_[i] * TPOLL_MS) >= DEBOUNCE_DELAY_MS && pollCounters_[i] != COUNTER_MAXVAL) {
                    ProcessCallback(i, false);
                }
                pollCounters_[i] = 0;
            }
            else if(pollCounters_[i] != COUNTER_MAXVAL) {
                ++pollCounters_[i];
                if((pollCounters_[i] * TPOLL_MS) >= TLONGPRESS_MS) {
                    ProcessCallback(i, true);
                    pollCounters_[i] = COUNTER_MAXVAL;
                }
            }
        }
    }

    void RegisterCallback(size_t buttonIndex, Callback_t cb)
    {
        if(SAFETY_CHECK && buttonIndex >= BUTTONS_COUNT) {
            return;
        }
        callbacks_[buttonIndex] = cb;
    }

    Callback_t& operator[](size_t buttonIndex)
    {
        return callbacks_[buttonIndex];
    }
private:
    void ProcessCallback(size_t index, bool isLongPress)
    {
        if(SAFETY_CHECK && !callbacks_[index]) {
            return;
        }
        callbacks_[index](isLongPress);
    }
};

} // utils

#endif // UTILS_H
