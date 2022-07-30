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

#include "wake_base.h"
#include "gpio.h"
#include "timers.h"

//		MBI6651
//	First Channel,	Fan Control
//  PC3(TIM1_CH3)	PD4(TIM2_CH1)
//		NCP3066
//	First Channel,				Fan Control
//	PA3(TIM2_CH3) - On/Off		PD4(TIM2_CH1)
//  PD3(TIM2_CH2) - driver NFB

namespace Mcudrv
{
	namespace Wk
  {

	struct LedDriverDefaultFeatures
	{
		enum {
			TwoChannels = false,
			FanControl = false
		};
	};

	static const uint8_t linTable[101] = {
				0,
				3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
				15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
				26, 27, 28, 29,
				30, 32, 34, 36, 38, 40, 42, 44,
        46, 48, 50, 52, 54, 56, 58, 60, 62, 64,
        66, 68, 70, 72, 74, 76, 78, 80, 82, 84,
        86, 89, 92, 95, 98, 101, 104, 107, 110,	113,
        116, 119, 122, 125, 128, 131, 134, 137, 140, 143,
        147, 151, 155, 159, 163, 167, 171, 175, 179, 183,
        187, 191, 195, 199, 204, 209, 214, 219, 224, 229,
				234, 239, 244, 249, 255
    };

	template<typename Features = LedDriverDefaultFeatures>
	class LedDriver : WakeData
	{
	private:
		union state_t {
			uint8_t ch[2];
			uint16_t Data;
		};
		enum Ch {
			Ch1,
			Ch2 = Features::TwoChannels
		};
		enum InstructionSet {
			C_GetState = 16,
			C_GetBright = 16,
			C_GetFan = 16,
			C_SetBright = 17,
			C_IncBright = 18,
			C_DecBright = 19,
			C_SetFan = 20,
			C_SetGfgFanAuto = 21
		};
		#pragma data_alignment=4
		#pragma location=".eeprom.noinit"
		static state_t state_nv;// @ ".eeprom.noinit";
		static state_t curState, onState;
		static const state_t DefaultState;
		FORCEINLINE
		static uint8_t ReadSpeed()
		{
			return T2::Timer2::ReadCompareByte<T2::Ch1>();
		}
		FORCEINLINE
		static void WriteSpeed(uint8_t val)
		{
			T2::Timer2::WriteCompareByte<T2::Ch1>(val);
		}
		static uint8_t GetFanSpeed()
		{
			uint8_t tmp = ReadSpeed();
			return tmp > 20 ? (tmp - 20) / 2 : 0;
		}
		static void SetFanSpeed(uint8_t speed)
		{
			if(speed > 100) speed = 255;
			else if (speed > 0)
			{
				speed = speed * 2 + 20;
			}
			WriteSpeed(speed);
		}
		static void UpdateChannel1()
		{
			using namespace T1;
			static uint8_t br;
			if(br < curState.ch[Ch1])
				++br;
			else if(br > curState.ch[Ch1])
				--br;
			Timer1::WriteCompareByte<Ch3>(linTable[br]);
		}
		static void UpdateChannel2(stdx::Int2Type<true>)
		{
			using namespace T2;
			static uint8_t br;
			if(br < curState.ch[Ch2])
				++br;
			else if(br > curState.ch[Ch2])
				--br;
			Timer2::WriteCompareByte<T2::Ch2>(linTable[br]);
		}
		static void UpdateChannel2(stdx::Int2Type<false>) { }
	public:
		enum
		{
			deviceMask = DevLedDriver,
			features = Features::TwoChannels | Features::FanControl << 1UL
		};
		static void Init()
		{
			{
				using namespace T1;
				Pc3::SetConfig<GpioBase::Out_PushPull_fast>();
				Timer1::Init(8, Cfg(ARPE | CEN));
				Timer1::WriteAutoReload(255);			//Fcpu/8/256 ~= 980Hz for 2 MHz
				Timer1::SetChannelCfg<Ch3, Output, ChannelCfgOut(Out_PWM_Mode1 | Out_PreloadEnable)>();
				Timer1::ChannelEnable<Ch3>();
			}
			if(Features::FanControl)
			{
				using namespace T2;
				Pd4::SetConfig<GpioBase::Out_PushPull_fast>();
				Timer2::Init(Div_1, Cfg(ARPE | CEN));
				Timer2::WriteAutoReload(0xFF);			//Fcpu/256 ~= 7800Hz for 2 MHz
				Timer2::SetChannelCfg<T2::Ch1, Output, ChannelCfgOut(Out_PWM_Mode1 | Out_PreloadEnable)>();
				Timer2::ChannelEnable<T2::Ch1>();
			}
		}
		static void Process()
		{
			switch(cmd) {
			case C_GetState:
				//Common query - all channels brightness
				if(!pdata.n) {
					pdata.buf[0] = ERR_NO;
					pdata.buf[1] = GetBrightness(Ch1);
					if(Ch2) {
						pdata.buf[2] = GetBrightness(Ch2);
						pdata.n = 3;
					}
					else {
						pdata.n = 2;
					}
				}
				else if(pdata.n == 1) {
					// Ch1 brightness
					if(!pdata.buf[0]) {
						pdata.buf[0] = ERR_NO;
						pdata.buf[1] = GetBrightness(Ch1);
						pdata.n = 2;
					}
					//Ch2 brightness
					else if(Ch2 && pdata.buf[0] == 0x80) {
						pdata.buf[0] = ERR_NO;
						pdata.buf[1] = GetBrightness(Ch2);
						pdata.n = 2;
					}
					//fan speed
					else if(Features::FanControl && pdata.buf[0] == 0x01) {
						pdata.buf[0] = ERR_NO;
						pdata.buf[1] = GetFanSpeed();
						pdata.n = 2;
					}
					else {
						pdata.buf[0] = ERR_NI;
						pdata.n = 1;
					}
				}
				//param error
				else {
					pdata.buf[0] = ERR_PA;
					pdata.n = 1;
				}
				break;
			case C_SetBright:
				if(pdata.n == 1) {
					if(!(pdata.buf[0] & 0x80)) {
						SetBrightness(pdata.buf[0], Ch1);
					}
					else if(Ch2) {
						SetBrightness(pdata.buf[0] & 0x7F, Ch2);
					}
					else {
						pdata.buf[0] = ERR_NI;
						break;
					}
					pdata.buf[0] = ERR_NO;
				}
				else {
					pdata.buf[0] = ERR_PA;
					pdata.n = 1;
				}
				break;
			case C_IncBright:
				if(pdata.n == 1) {
					//1st channel selected
					if(!(pdata.buf[0] & 0x80)) {
						pdata.buf[1] = IncBrightness(pdata.buf[0], Ch1);
					}
					else if(Ch2) {
						pdata.buf[1] = IncBrightness(pdata.buf[0] & 0x7F, Ch2);
					}
					else {
						pdata.buf[0] = ERR_NI;
						break;
					}
					pdata.buf[0] = ERR_NO;
					pdata.n = 2;
				}
				else {
					pdata.buf[0] = ERR_PA;
					pdata.n = 1;
				}
			break;
			case C_DecBright:
				if(pdata.n == 1) {
					//1st channel selected
					if(!(pdata.buf[0] & 0x80)) {
						pdata.buf[1] = DecBrightness(pdata.buf[0], Ch1);
					}
					else if(Ch2) {
						pdata.buf[1] = DecBrightness(pdata.buf[0] & 0x7F, Ch2);
					}
					else {
						pdata.buf[0] = ERR_NI;
						break;
					}
					pdata.buf[0] = ERR_NO;
					pdata.n = 2;
				}
				else {
					pdata.buf[0] = ERR_PA;
					pdata.n = 1;
				}
			break;
			case C_SetFan:
				if(pdata.n == 1) {
					if(Features::FanControl) {
						SetFanSpeed(pdata.buf[0]);
						pdata.buf[0] = ERR_NO;
						pdata.buf[1] = GetFanSpeed();
						pdata.n = 2;
					}
					else pdata.buf[0] = ERR_NI;
				}
				else {
					pdata.buf[0] = ERR_PA;
					pdata.n = 1;
				}
				break;
			case C_SetGfgFanAuto:
				if(pdata.n == 3) {
					pdata.buf[0] = ERR_NI;
				}
				else {
					pdata.buf[0] = ERR_PA;
				}
				break;
			default:
				//if command not processed by this module
				processedMask |= deviceMask;
			}//switch
		}
		static void SetBrightness(uint8_t br, const Ch ch = Ch1)
		{
			if(!br) {
				onState.ch[ch] = DefaultState.ch[ch];
			}
			if(br > 100) {
				br = 100;
			}
			curState.ch[ch] = br;
		}
		static uint8_t GetBrightness(Ch ch = Ch1)
		{
			return curState.ch[ch];
		}
		static uint8_t IncBrightness(uint8_t step, Ch ch = Ch1)
		{
			uint8_t cur = GetBrightness(ch);
			if(cur < 100)
			{
				step = step < 100 ? step : 100;
				cur += step;
				SetBrightness(cur, ch);
			}
			return cur;
		}
		static uint8_t DecBrightness(uint8_t step, Ch ch = Ch1)
		{
			uint8_t cur = GetBrightness(ch);
			if(cur <= step) cur = 0;
			else cur -= step;
			SetBrightness(cur, ch);
			return cur;
		}
		static void SaveState()
		{
			using namespace Mem;
			if(state_nv.Data != curState.Data) //Only if changed
			{
				Unlock<Eeprom>();
				if(IsUnlocked<Eeprom>())
				{
					SetWordProgramming();
					state_nv = curState;
			//this dummy write need to complete word programming
			//simple two bytes write cause to greater wear due to physical mem organization
					(&state_nv)[1].Data = 0;
					Lock<Eeprom>();
				}
			}
		}
		static void On()
		{
			if(!onState.Data) {
				onState = DefaultState;
			}
			curState = onState;
		}
		static void Off()
		{
			onState.ch[Ch1] = curState.ch[Ch1] ? curState.ch[Ch1] : DefaultState.ch[Ch1];
			if(Features::TwoChannels) onState.ch[Ch2] = curState.ch[Ch2] ? curState.ch[Ch2] : DefaultState.ch[Ch2];
			if(Features::FanControl) WriteSpeed(0);
			curState.Data = 0;
		}
		static void ToggleOnOff()
		{
			if(!curState.Data) On();
			else Off();
		}
		static uint8_t GetDeviceFeatures(const uint8_t)
		{
			return features;
		}
		static void UpdIRQ()	//Soft Dimming
		{
			UpdateChannel1();
			UpdateChannel2(stdx::Int2Type<Features::TwoChannels>());
		}
	};

	template<typename Features>
    typename LedDriver<Features>::state_t LedDriver<Features>::state_nv;
	template<typename Features>
    typename LedDriver<Features>::state_t LedDriver<Features>::curState = state_nv;
	template<typename Features>
    typename LedDriver<Features>::state_t LedDriver<Features>::onState;
	template<typename Features>
	const typename LedDriver<Features>::state_t LedDriver<Features>::DefaultState = {100, 100};


  } //Ldrv
} //Mcudrv

