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
#ifndef SSD1306_H
#define SSD1306_H

#include "fonts.h"
#include "i2c.h"
#include "icons.h"

namespace Mcudrv {
using Resources::Bitmap;
using Resources::Color;
using Resources::Font;

// SSD1306 driver
template<typename Twi = Twis::SoftTwi<>>
class ssd1306 : Twi
{
private:
    enum
    {
        BaseAddr = 0x3C
    };
    enum
    {
        Max_X = 127,
        Max_Y = 63
    };
    enum ControlByte
    {
        CtrlCmdSingle = 0x80,
        CtrlCmdStream = 0x00,
        CtrlDataStream = 0x40
    };
    enum InstructionSet
    {
        CmdSetContrast = 0x81, // with followed by value 0xCF
        CmdDisplayRam = 0xA4,
        CmdDisplayAllOn = 0xA5,
        CmdDisplayNormal = 0xA6,
        CmdDisplayInverted = 0xA7,
        CmdDisplayOff = 0xAE,
        CmdDisplayOn = 0xAF,

        CmdMemAddrMode =
          0x20, // with followed by: 0x00 - Horizontal, 0x01 - Vertical, 0x02 - Page(Default), 0x03 - Invalid
        ModeHorizontal = 0x00,
        ModeVertical = 0x01,
        ModePage = 0x02,

        CmdSetColRange = 0x21,  // with followed by column range (0x00 0x7F)
        CmdSetPageRange = 0x22, // with followed by page range (0x00 0x07)

        CmdDisplayStartLine = 0x40,  // | 0..63 - Only in Page mode, default: 0x40
        CmdSetPageStart = 0xB0,      // | 0..7 - Only in Page mode, default: 0xB0
        CmdSetColStartLower = 0x00,  // | 0..15 - Only in Page mode, Lower nibble of column
        CmdSetColStartHigher = 0x10, // | 0..15  - Only in Page mode, Higher nibble of column
        CmdSegmentRemap = 0xA1,
        CmdMuxRatio = 0xA8, // with followed by ratio (default: 0x3F = 64 MUX)
        CmdComScanMode = 0xC8,
        CmdDisplayOffset = 0xD3, // with followed by 0x00
        CmdComPinMap = 0xDA,     // with followed by 0x12
        CmdNop = 0xE3,

        CmdClkDiv = 0xD5,        // with followed by value
        CmdPrecharge = 0xD9,     // with followed by 0xF1
        CmdVComHDeselect = 0xDB, // with followed by 0x30

        CmdChargePump = 0x8D // with followed by 0x14(internal gen) or 0x10(external gen)
    };
    static const uint8_t initSequence[24];
    static uint8_t x_, y_;
public:
    static void Init()
    {
        Twi::Write(BaseAddr, initSequence, sizeof(initSequence));
    }
    static void SetContrast(uint8_t con)
    {
        uint8_t seq[3] = { CtrlCmdStream, CmdSetContrast, con & 0x7F };
        Twi::Write(BaseAddr, seq, sizeof(seq));
    }
    static void SetX(uint8_t x)
    {
        const uint8_t seq[] = { CtrlCmdStream, CmdSetColStartHigher | (x >> 4), CmdSetColStartLower | (x & 0x0F) };
        Twi::Write(BaseAddr, seq, sizeof(seq));
        x_ = x;
    }
    static void SetY(uint8_t y)
    {
        const uint8_t seq[] = { CtrlCmdSingle, CmdSetPageStart | (y & 0x07) };
        Twi::Write(BaseAddr, seq, sizeof(seq));
        y_ = y;
    }
    static void SetXY(uint8_t x, uint8_t y)
    {
        SetX(x);
        SetY(y);
    }
    static void Fill(const Color color = Resources::Clear)
    {
        for(uint8_t y = 0; y < 8; ++y) {
            SetXY(0, y);
            Twi::Write(BaseAddr, CtrlDataStream, Twis::NoStop);
            for(uint8_t x = 0; x < 128; ++x) {
                Twi::WriteByte(color);
            }
            Twi::Stop();
        }
        SetXY(0, 0);
    }
    static void Fill(uint8_t x, uint8_t xRange, uint8_t y, uint8_t yRange, Color color = Resources::Clear)
    {
        for(uint8_t ypos = 0; ypos < yRange; ++ypos) {
            SetXY(x, y + ypos);
            Twi::Write(BaseAddr, CtrlDataStream, Twis::NoStop);
            for(uint8_t xpos = 0; xpos < xRange; ++xpos) {
                Twi::WriteByte(color);
            }
            Twi::Stop();
        }
        SetXY(x + xRange, y);
    }
    static void Draw(const Bitmap& bmap, uint8_t x = x_, uint8_t y = y_)
    {
        for(uint8_t ypos = 0; ypos < (bmap.Height() >> 3); ++ypos) {
            SetXY(x, y + ypos);
            Twi::Write(BaseAddr, CtrlDataStream, Twis::NoStop);
            for(uint8_t x = 0; x < bmap.Width(); ++x) {
                Twi::WriteByte(bmap[x + bmap.Width() * ypos]);
            }
            Twi::Stop();
        }
        SetXY(x + bmap.Width(), y);
    }
    static void Putch(uint8_t ch, const Font& font = Resources::font5x8)
    {
        if(ch == '\r') {
            SetX(0);
            return;
        }
        const uint8_t heightInBytes = font.Height() >> 3;
        if(ch == '\n') {
            SetY(y_ + heightInBytes);
            return;
        }
        if(ch == '\b') {
            SetX(x_ - font.Width() - (font.Width() >> 2));
            return;
        }
        if(ch == ' ') {
            Fill(x_, font.Width() + 1, y_, heightInBytes);
            return;
        }
        // adjust height offset
        static uint8_t prevHeight = Resources::font5x8.Height() >> 3;
        int8_t diff = prevHeight - heightInBytes;
        if(diff) {
            uint8_t ypos = y_;
            if(diff < 0) // current font bigger than previous
            {
                diff = -diff;
                if(ypos >= diff)
                    ypos -= diff; // there is a place on top
            }
            else if(diff > 0) // current font smaller than previous
            {
                ypos += diff;
            }
            SetY(ypos);
            prevHeight = heightInBytes;
        }
        // end of line
        if((Max_X - x_) < font.Width()) {
            SetXY(0, (y_ + heightInBytes) & 0x07);
        }
        Draw(Bitmap(font[ch], font.Width(), font.Height()));
        // Space between chars
        Fill(x_, font.Width() >> 2, y_, heightInBytes);
    }
    static void Puts(const uint8_t* str, const Font& font = Resources::font5x8)
    {
        while(*str)
            Putch(*str++, font);
    }
};
template<typename Twi>
uint8_t ssd1306<Twi>::x_;
template<typename Twi>
uint8_t ssd1306<Twi>::y_;
template<typename Twi>
const uint8_t ssd1306<Twi>::initSequence[] = { CtrlCmdStream,
                                               CmdSetColRange,
                                               0x00,
                                               0x7F,
                                               CmdSetPageRange,
                                               0x00,
                                               0x07,
                                               CmdDisplayOff, // default
                                               //			CmdClkDiv, 0x80,		//default = 0x80
                                               //			CmdMuxRatio, 0x3F,		//default = 0x3F
                                               //			CmdDisplayOffset, 0x00,	//default = 0
                                               CmdChargePump,
                                               0x14,
                                               CmdMemAddrMode,
                                               ModePage, // default = 0x02 (Page)
                                               CmdSegmentRemap,
                                               CmdComScanMode,
                                               CmdComPinMap,
                                               0x12,
                                               CmdSetContrast,
                                               0xCF, // default = 0x7F
                                               CmdPrecharge,
                                               0xF1,
                                               CmdVComHDeselect,
                                               0x40, // default = 0x20
                                               CmdDisplayRam,
                                               CmdDisplayOn };

} // Mcudrv

#endif // SSD1306_H
