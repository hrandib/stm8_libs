#pragma once
#ifndef S1D157XX_H
#define S1D157XX_H

#include "stdint.h"
#include "clock.h"
#include "pinlist.h"
#include "fonts.h"
#include "icons.h"

namespace Mcudrv {

enum DisplayType
{
	S1D15705,		//162x64 (0...161; 0...63) 65-й пиксел относится к 5 неиспользуемым строкам не дисплее: 3 сверху, 2 снизу
	S1D15710		//219x60
};

template<DisplayType Disptype, typename Databus, typename Cs, typename Rs, typename Wr,
								typename Res = Nullpin, typename Rd = Nullpin>
class S1d157xx
{
private:
	enum
	{
		Res_X = Disptype == S1D15705 ? 162 : 219,		//TODO: Need offset(2)?
		Res_Y = Disptype == S1D15705 ? 64 : 60
	};
	enum InstructionSet
	{						//S1D15705	S1D15710
		CmdBias1 = 0xA2,   //1/9		1/6
		CmdBias2 = 0xA3,   //1/7		1/5
		CmdDisplayAllOff = 0xA4,
		CmdDisplayAllOn = 0xA5,
		CmdDisplayNormal = 0xA6,
		CmdDisplayInverted = 0xA7,
		CmdDisplayOff = 0xAE,
		CmdDisplayOn = 0xAF,
		CmdDisplayStartLine = 0x40, // | 0..63, default: 0x40
		CmdSetPageStart = 0xB0, // | 0..8, default: 0xB0
		CmdSetColStartHigher = 0x10, // | 0..15  - Only in Page mode, Higher nibble of column
		CmdSetColStartLower = 0x00, // | 0..15 - Only in Page mode, Lower nibble of column
		CmdSegmentRemapNormal = 0xA0, // | 0 - CW, | 1 - CCW
		CmdSegmentRemapCCW = 0xA1,
		CmdRmw = 0xE0,
		CmdRmwEnd = 0xEE,
		CmdReset = 0xE2,
		CmdComScanModeNormal = 0xC8, // | 0 - Normal, | 1 - Reversal
		CmdComScanModeReversal = 0xC9,
		CmdOscOn = 0xAB,

		CmdPowerSaveStandby = 0xA8,
		CmdPowerSaveSleep = 0xA9,
		CmdPowerSaveReset = 0xE1,

		CmdPowerControl = 0x28, // | Boost << 2 | Vadj << 1 | V/F
		PowerBoostOn = 1U << 2,
		PowerVadjOn = 1U << 1,
		PowerVFOn = 0x01,		//Voltage follower

		CmdV5AdjResistance = 0x20, // | 0..7 (Small->Large)
		CmdSetContrast = 0x81,	//with followed by 0..63, default set to 32

		CmdNop = 0xE3
	};
	enum StatusFlags
	{
		StatusReset = 1U << 4,
		StatusOff = 1U << 5,
		StatusAdc = 1U << 6,
		StatusBusy = 1U << 7
	};
	static const uint8_t initSequence[14];
	static uint8_t x_, y_;

/*	#pragma inline=forced
	static void Delay()
	{
		if(F_CPU > 8000000UL) __no_operation();
	}

	static void Reset()
	{
		Rd::Set();
		Wr::Clear();
		Databus::Write();
	}
	*/
	static void WriteCmd(uint8_t cmd)
	{
		Rs::Clear();
		Rd::Set();
		Wr::Clear();
		Databus::Write(cmd);
		Cs::Clear();
		Cs::Set();
		Wr::Set();
		Databus::Write(0xFF);
	}
	static void WriteData(uint8_t data)
	{
		Rs::Set();
		Rd::Set();
		Wr::Clear();
		Databus::Write(data);
		Cs::Clear();
		Cs::Set();
		Wr::Set();
		Databus::Write(0xFF);
	}

public:
	static void Init()
	{
		Cs::template SetConfig<GpioBase::Out_PushPull_fast>();
		Rs::template SetConfig<GpioBase::Out_PushPull_fast>();	//A0
		Rd::template SetConfig<GpioBase::Out_PushPull_fast>();
		Wr::template SetConfig<GpioBase::Out_PushPull_fast>();
		Databus::template SetConfig<0xFF, GpioBase::Out_PushPull_fast>();
		Res::template SetConfig<GpioBase::Out_PushPull>();
		Res::Set();
		Rs::Set();
		Cs::Set();
		Rd::Set();
		Wr::Set();
		for(uint8_t i = 0; i < sizeof(initSequence); ++i)
		{
			WriteCmd(initSequence[i]);
		}
		Fill();
	}
	static void SetContrast(uint8_t val) //0..63
	{
		WriteCmd(CmdSetContrast);
		WriteCmd(val);
	}
	static void Fill(Resources::Color col = Resources::Clear)
	{
		for(uint8_t y = 0; y < 0x09; ++y)
		  {
			SetXY(0, y);
			for(uint8_t x = 0; x < Res_X; ++x)
				WriteData(col);
		  }
	}
	static void Fill(uint8_t x, uint8_t xRange, uint8_t y, uint8_t yRange, Resources::Color color = Resources::Clear)
	{
		for(uint8_t ypos = 0; ypos < yRange; ++ypos)
		{
			SetXY(x, y + ypos);
			for(uint8_t xpos = 0; xpos < xRange; ++xpos)
			{
				WriteData(color);
			}
		}
		SetXY(x + xRange, y);
	}
	static void SetX(uint8_t x)
	{
		WriteCmd(CmdSetColStartHigher | (x >> 4));
		WriteCmd(CmdSetColStartLower | (x & 0x0f + 3));
		x_ = x;
	}
	static void SetY(uint8_t y)
	{
		WriteCmd(CmdSetPageStart | y);
		y_ = y;
	}
	static void SetXY(uint8_t x, uint8_t y)
	{
		SetX(x);
		SetY(y);
	}
	static void Draw(const Resources::Bitmap& bmap, uint8_t x = x_, uint8_t y = y_)
	{
		for(uint8_t ypos = 0; ypos < (bmap.Height() >> 3); ++ypos)
		{
			SetXY(x, y + ypos);
			uint8_t bitmapOffset = bmap.Width() * ypos;
			for(uint8_t x = 0; x < bmap.Width(); ++x)
			{
				WriteData(bmap[x + bitmapOffset]);
			}
		}
		SetXY(x + bmap.Width(), y);
	}
	static void Putch(uint8_t ch, const Resources::Font& font = Resources::font5x8)
	{
		if(ch == '\r')
		{
			SetX(0);
			return;
		}
		const uint8_t heightInBytes = font.Height() >> 3;
		if(ch == '\n')
		{
			SetY(y_ + heightInBytes);
			return;
		}
		if(ch == '\b')
		{
			SetX(x_ - font.Width() - (font.Width() >> 2));
			return;
		}
		if(ch == ' ')
		{
			Fill(x_, font.Width() + 1, y_, heightInBytes);
			return;
		}
	//adjust height offset
		static uint8_t prevHeight = Resources::font5x8.Height() >> 3;
		int8_t diff = prevHeight - heightInBytes;
		if(diff)
		{
			uint8_t ypos = y_;
			if(diff < 0) //current font bigger than previous
			{
				diff = -diff;
				if(ypos >= diff) ypos -= diff;	//there is a place on top
			}
			else if(diff > 0) //current font smaller than previous
			{
				ypos += diff;
			}
			SetY(ypos);
			prevHeight = heightInBytes;
		}
	//end of line
		if((Res_X - 1 - x_) < font.Width())
		{
			SetXY(0, (y_ + heightInBytes) & 0x07);
		}
		Draw(Resources::Bitmap(font[ch], font.Width(), font.Height()));
		//Space between chars
		Fill(x_, font.Width() >> 2, y_, heightInBytes);
	}
	static void Puts(const uint8_t* str, const Resources::Font& font = Resources::font5x8)
	{
		while(*str)
			Putch(*str++, font);
	}

};

template<DisplayType disptype, typename Databus, typename Cs, typename A0, typename Wr, typename Res, typename Rd>
uint8_t S1d157xx<disptype, Databus, Cs, A0, Wr, Res, Rd>::x_;
template<DisplayType disptype, typename Databus, typename Cs, typename A0, typename Wr, typename Res, typename Rd>
uint8_t S1d157xx<disptype, Databus, Cs, A0, Wr, Res, Rd>::y_;

template<DisplayType disptype, typename Databus, typename Cs, typename A0, typename Wr, typename Res, typename Rd>
const uint8_t S1d157xx<disptype, Databus, Cs, A0, Wr, Res, Rd>::initSequence[] = { CmdReset,
																		CmdOscOn,
																		CmdBias2,
																		CmdSegmentRemapNormal, //Default after reset
																		CmdComScanModeNormal,  //Default after reset
																		CmdV5AdjResistance | 6,
																		CmdSetContrast, 0x13,	//Default: 0x20
																		CmdPowerControl | PowerBoostOn | PowerVadjOn | PowerVFOn,
																		CmdDisplayNormal,
																		CmdDisplayStartLine,	//Default after reset
																		CmdDisplayOn,
																		CmdDisplayAllOn,
																		CmdDisplayAllOff };

}


#endif // S1D157XX_H

