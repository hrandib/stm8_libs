/*
 * Copyright (c) 2022 Dmytro Shestakov
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

#ifndef I2C_LCD_BACKPACK_H
#define I2C_LCD_BACKPACK_H

#include "gpio.h"

namespace Mcudrv {

// HD44780 helper for using via i2c IO extender
template<typename Twi, uint8_t ADDR>
class LcdBackpack
{
    enum LcdConnScheme
    {
        POS_RS,
        POS_RW,
        POS_E,
        POS_BL,
        POS_DATA
    };
    static uint8_t state;
public:
    struct Databus
    {
        template<GpioBase::Cfg>
        static void SetConfig()
        { }
        static void Write(uint8_t data)
        {
            state = (state & 0x0F) | (data << 4);
            Twi::Write(ADDR, state);
        }
    };
    struct Rs
    {
        template<GpioBase::Cfg>
        static void SetConfig()
        { }

        static void Clear()
        {
            state &= ~(1U << POS_RS);
        }
        static void Set()
        {
            state |= (1U << POS_RS);
        }
    };
    struct E
    {
        template<GpioBase::Cfg>
        static void SetConfig()
        { }
        static void Clear()
        {
            Twi::Write(ADDR, state);
        }
        static void Set()
        {
            Twi::Write(ADDR, (1U << POS_E) | state);
        }
    };
    static void SetBacklight(bool isOn)
    {
        if(isOn) {
            state &= ~(1U << POS_BL);
        }
        else {
            state |= (1U << POS_BL);
        }
        Twi::Write(ADDR, state);
    }
};

template<typename Twi, uint8_t ADDR>
uint8_t LcdBackpack<Twi, ADDR>::state;

} // Mcudrv

#endif // I2C_LCD_BACKPACK_H
