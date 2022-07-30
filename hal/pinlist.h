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
#ifndef PINLIST_H
#define PINLIST_H

#include "gpio.h"

namespace Mcudrv {

template<typename S0 = Nullpin,
         typename S1 = Nullpin,
         typename S2 = Nullpin,
         typename S3 = Nullpin,
         typename S4 = Nullpin,
         typename S5 = Nullpin,
         typename S6 = Nullpin,
         typename S7 = Nullpin>
struct Pinlist : GpioBase
{
    enum
    {
        size = S0::mask == 0   ? 0
               : S1::mask == 0 ? 1
               : S2::mask == 0 ? 2
               : S3::mask == 0 ? 3
               : S4::mask == 0 ? 4
               : S5::mask == 0 ? 5
               : S6::mask == 0 ? 6
               : S7::mask == 0 ? 7
                               : 8
    };
    static uint8_t ReadODR()
    {
        uint8_t value = 0;
        if(S0::IsODRSet())
            value |= 0x01;
        if(S1::IsODRSet())
            value |= 0x02;
        if(S2::IsODRSet())
            value |= 0x04;
        if(S3::IsODRSet())
            value |= 0x08;
        if(S4::IsODRSet())
            value |= 0x10;
        if(S5::IsODRSet())
            value |= 0x20;
        if(S6::IsODRSet())
            value |= 0x40;
        if(S7::IsODRSet())
            value |= 0x80;
        return value;
    }
    static uint8_t Read()
    {
        uint8_t value = 0;
        if(S0::IsSet())
            value |= 0x01;
        if(S1::IsSet())
            value |= 0x02;
        if(S2::IsSet())
            value |= 0x04;
        if(S3::IsSet())
            value |= 0x08;
        if(S4::IsSet())
            value |= 0x10;
        if(S5::IsSet())
            value |= 0x20;
        if(S6::IsSet())
            value |= 0x40;
        if(S7::IsSet())
            value |= 0x80;
        return value;
    }
    static void Toggle(uint8_t mask)
    {
        if(mask & 0x01)
            S0::Toggle();
        if(mask & 0x02)
            S1::Toggle();
        if(mask & 0x04)
            S2::Toggle();
        if(mask & 0x08)
            S3::Toggle();
        if(mask & 0x10)
            S4::Toggle();
        if(mask & 0x20)
            S5::Toggle();
        if(mask & 0x40)
            S6::Toggle();
        if(mask & 0x80)
            S7::Toggle();
    }
    static void Set(uint8_t mask)
    {
        if(mask & 0x01)
            S0::Set();
        if(mask & 0x02)
            S1::Set();
        if(mask & 0x04)
            S2::Set();
        if(mask & 0x08)
            S3::Set();
        if(mask & 0x10)
            S4::Set();
        if(mask & 0x20)
            S5::Set();
        if(mask & 0x40)
            S6::Set();
        if(mask & 0x80)
            S7::Set();
    }
    static void Clear(uint8_t mask)
    {
        if(mask & 0x01)
            S0::Clear();
        if(mask & 0x02)
            S1::Clear();
        if(mask & 0x04)
            S2::Clear();
        if(mask & 0x08)
            S3::Clear();
        if(mask & 0x10)
            S4::Clear();
        if(mask & 0x20)
            S5::Clear();
        if(mask & 0x40)
            S6::Clear();
        if(mask & 0x80)
            S7::Clear();
    }
    static void Write(uint8_t value)
    {
        if(value & 0x01)
            S0::Set();
        else
            S0::Clear();
        if(value & 0x02)
            S1::Set();
        else
            S1::Clear();
        if(value & 0x04)
            S2::Set();
        else
            S2::Clear();
        if(value & 0x08)
            S3::Set();
        else
            S3::Clear();
        if(value & 0x10)
            S4::Set();
        else
            S4::Clear();
        if(value & 0x20)
            S5::Set();
        else
            S5::Clear();
        if(value & 0x40)
            S6::Set();
        else
            S6::Clear();
        if(value & 0x80)
            S7::Set();
        else
            S7::Clear();
    }
#pragma inline = forced
    template<Cfg cfg>
    static void SetConfig()
    {
        S0::template SetConfig<cfg>();
        S1::template SetConfig<cfg>();
        S2::template SetConfig<cfg>();
        S3::template SetConfig<cfg>();
        S4::template SetConfig<cfg>();
        S5::template SetConfig<cfg>();
        S6::template SetConfig<cfg>();
        S7::template SetConfig<cfg>();
    }
#pragma inline = forced
    template<uint8_t mask_, Cfg cfg>
    static void SetConfig()
    {
        if(mask_ & 0x01)
            S0::template SetConfig<cfg>();
        if(mask_ & 0x02)
            S1::template SetConfig<cfg>();
        if(mask_ & 0x04)
            S2::template SetConfig<cfg>();
        if(mask_ & 0x08)
            S3::template SetConfig<cfg>();
        if(mask_ & 0x10)
            S4::template SetConfig<cfg>();
        if(mask_ & 0x20)
            S5::template SetConfig<cfg>();
        if(mask_ & 0x40)
            S6::template SetConfig<cfg>();
        if(mask_ & 0x80)
            S7::template SetConfig<cfg>();
    }
};

template<uint8_t seq>
struct SequenceOf
{
    enum
    {
        value = seq
    };
};

template<typename First, uint8_t Seq>
struct Pinlist<First, SequenceOf<Seq> > : GpioBase
{
    typedef typename First::Port Port;
    enum
    {
        OFFSET = First::position,
        MASK = stdx::NumberToMask<Seq>::value << OFFSET,
        SIZE = Seq
    };
#pragma inline = forced
    static uint16_t ReadODR()
    {
        return (Port::ReadODR() & MASK) >> OFFSET;
    }
#pragma inline = forced
    static uint16_t Read()
    {
        return (Port::Read() & MASK) >> OFFSET;
    }
#pragma inline = forced
    static void Write(uint16_t value)
    {
        value = (value << OFFSET) & MASK;
        Port::ClearAndSet(~value & MASK, value);
    }
#pragma inline = forced
    template<uint8_t mask, Cfg cfg>
    static void SetConfig()
    {
        Port::template SetConfig<(uint8_t)(mask << OFFSET), cfg>();
    }
#pragma inline = forced
    template<Cfg cfg>
    static void SetConfig()
    {
        Port::template SetConfig<MASK, cfg>();
    }
#pragma inline = forced
    static void Toggle(uint8_t mask)
    {
        Port::Toggle(mask << OFFSET);
    }
#pragma inline = forced
    static void Set(uint8_t mask)
    {
        Port::Set(mask << OFFSET);
    }
#pragma inline = forced
    static void Clear(uint8_t mask)
    {
        Port::Clear(mask << OFFSET);
    }
};

// Combines up to 4 linear pin ranges to the single virtual port,
// e.g (Pc3, Pc4, Pc5, Pd0, Pd1, Pd2) sequence could be defined as PinSequence<Pc3, 3, Pd0, 3>
template<typename T1,
         uint8_t Seq1,
         typename T2 = Nullpin,
         uint8_t Seq2 = 0,
         typename T3 = Nullpin,
         uint8_t Seq3 = 0,
         typename T4 = Nullpin,
         uint8_t Seq4 = 0>
class PinSequence : GpioBase
{
private:
    typedef Pinlist<T1, SequenceOf<Seq1> > Pinlist1;
    typedef Pinlist<T2, SequenceOf<Seq2> > Pinlist2;
    typedef Pinlist<T3, SequenceOf<Seq3> > Pinlist3;
    typedef Pinlist<T4, SequenceOf<Seq4> > Pinlist4;
public:
#pragma inline = forced
    static uint16_t ReadODR()
    {
        return Pinlist1::ReadODR() | Pinlist2::ReadODR() << Seq1 | Pinlist3::ReadODR() << Seq2 |
               Pinlist4::ReadODR() << Seq3;
    }
#pragma inline = forced
    static uint16_t Read()
    {
        return Pinlist1::Read() | Pinlist2::Read() << Seq1 | Pinlist3::Read() << Seq2 | Pinlist4::Read() << Seq3;
    }
#pragma inline = forced
    static void Write(uint16_t value)
    {
        Pinlist1::Write(value);
        Pinlist2::Write(value >> Seq1);
        Pinlist3::Write(value >> Seq2);
        Pinlist4::Write(value >> Seq3);
    }
#pragma inline = forced
    static void Clear(uint16_t value)
    {
        Pinlist1::Clear(value);
        Pinlist2::Clear(value >> Seq1);
        Pinlist3::Clear(value >> Seq2);
        Pinlist4::Clear(value >> Seq3);
    }
#pragma inline = forced
    static void Set(uint16_t value)
    {
        Pinlist1::Set(value);
        Pinlist2::Set(value >> Seq1);
        Pinlist3::Set(value >> Seq2);
        Pinlist4::Set(value >> Seq3);
    }
#pragma inline = forced
    static void Toggle(uint16_t value)
    {
        Pinlist1::Toggle(value);
        Pinlist2::Toggle(value >> Seq1);
        Pinlist3::Toggle(value >> Seq2);
        Pinlist4::Toggle(value >> Seq3);
    }
#pragma inline = forced
    template<uint8_t mask, Cfg cfg>
    static void SetConfig()
    {
        Pinlist1::template SetConfig<static_cast<uint8_t>(mask), cfg>();
        Pinlist2::template SetConfig<static_cast<uint8_t>(mask >> Seq1), cfg>();
        Pinlist3::template SetConfig<static_cast<uint8_t>(mask >> Seq2), cfg>();
        Pinlist4::template SetConfig<static_cast<uint8_t>(mask >> Seq3), cfg>();
    }
#pragma inline = forced
    template<Cfg cfg>
    static void SetConfig()
    {
        Pinlist1::template SetConfig<cfg>();
        Pinlist2::template SetConfig<cfg>();
        Pinlist3::template SetConfig<cfg>();
        Pinlist4::template SetConfig<cfg>();
    }
};

} // Mcudrv
#endif // PINLIST_H
