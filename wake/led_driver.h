#pragma once
#include <cstring>

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
		enum
		{
			TwoChannels = false,
			FanControl = true
		};
	};

	static const uint8_t ledLinear[75] = {
		26, 28, 30, 32, 34, 36, 38, 40, 42, 44,
		46, 48, 50, 52, 54, 56, 58, 60, 62, 64,
		66, 68, 70, 72, 74, 76, 78, 80, 82, 84,
		86, 89, 92, 95, 98, 101, 104, 107, 110,	113,
		116, 119, 122, 125, 128, 131, 134, 137, 140, 143,
		147, 151, 155, 159, 163, 167, 171, 175, 179, 183,
		187, 191, 195, 199, 204, 209, 214, 219, 224, 229,
		234, 239, 244, 249, 255
	};

//	{	46, 48,	50, 52, 54, 56, 58, 60, 62, 64,
//		66, 68, 70, 72, 74,	76, 78, 80, 83, 86,
//		89, 92, 95, 98, 101, 104, 107, 110, 113, 117,
//		121, 125, 129, 133, 137, 141, 145, 150, 155, 160,
//		165, 170, 175, 180,	186, 192, 198, 204, 210, 217,
//		224, 231, 238, 245, 255
//	};

	template<typename Features = LedDriverDefaultFeatures>
	class LedDriver : WakeData
	{
	private:
		union state_t
		{
			struct
			{
				uint8_t ch[2];
				uint8_t fanSpeed;
			};
			uint32_t Data;
		};
		enum Ch { Ch1, Ch2 = Features::TwoChannels };
		enum InstructionSet
		{
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
		static uint8_t GetFanSpeed()
		{
			uint8_t tmp = curState.fanSpeed;
			return tmp > 20 ? (tmp - 20) / 2 : 0;
		}
		static void SetFanSpeed(uint8_t speed)
		{
			if(speed > 100) speed = 255;
			else if (speed > 0)
			{
				speed = speed * 2 + 20;
			}
			curState.fanSpeed = speed;
		}
		#pragma inline=forced
		static void SetCh1()
		{
			using namespace T2;
			static uint8_t br = state_nv.ch[Ch1];
			if(br < curState.ch[Ch1])
				++br;
			else if(br > curState.ch[Ch1])
				--br;
			Timer2::WriteCompareByte<T2::Ch3>(br < ledLinear[0] ? br : ledLinear[br - ledLinear[0]]);
		}
		#pragma inline=forced
		static void SetCh2(stdx::Int2Type<true>)
		{
			using namespace T2;
			static uint8_t br = state_nv.ch[Ch2];
			if(br < curState.ch[Ch2])
				++br;
			else if(br > curState.ch[Ch2])
				--br;
			Timer2::WriteCompareByte<T2::Ch2>(br < ledLinear[0] ? br : ledLinear[br - ledLinear[0]]);
		}
		#pragma inline=forced
		static void SetCh2(stdx::Int2Type<false>) { }
		#pragma inline=forced
		static void UpdIRQ()	//Soft Dimming
		{
			using namespace T2;
			if(Timer2::CheckIntStatus(IRQ_Update))
			{
				Timer2::ClearIntFlag(IRQ_Update);
				SetCh1();
				SetCh2(stdx::Int2Type<Features::TwoChannels>());
				if(Features::FanControl)
				{
					if(Timer2::ReadCompareByte<T2::Ch1>() < curState.fanSpeed)
					{
						Timer2::GetCompareByte<T2::Ch1>()++;
					}
					else if(Timer2::ReadCompareByte<T2::Ch1>() > curState.fanSpeed)
					{
						Timer2::GetCompareByte<T2::Ch1>()--;
					}
				}
			}
		}
	public:
		enum
		{
			deviceMask = devLedDriver,
			features = Features::TwoChannels | Features::FanControl << 1UL
		};
		#pragma inline=forced
		static void Init()
		{
			using namespace T2;
			static const ChannelCfgOut channelConfig = ChannelCfgOut(Out_PWM_Mode1 | Out_PreloadEnable);
			Pa3::SetConfig<GpioBase::Out_PushPull_fast>();
			Timer2::Init(Div_16, Cfg(ARPE | CEN));
			Timer2::WriteAutoReload(255);			//Fcpu/16/256 ~= 490Hz for 2 MHz
			Timer2::SetChannelCfg<T2::Ch3, Output, channelConfig>();
			Timer2::ChannelEnable<T2::Ch3>();
			Pd3::SetConfig<GpioBase::Out_PushPull_fast>();
			if(Features::TwoChannels)
			{
				Timer2::SetChannelCfg<T2::Ch2, Output, channelConfig>();
				Timer2::ChannelEnable<T2::Ch2>();
			}
			if(Features::FanControl)
			{
				Pd4::SetConfig<GpioBase::Out_PushPull_fast>();
				Timer2::SetChannelCfg<T2::Ch1, Output, channelConfig>();
				Timer2::ChannelEnable<T2::Ch1>();
			}
			OpTime::SetTimerCallback(UpdIRQ);
		}
		#pragma inline=forced
		static void Process()
		{
			switch(cmd)
			{
				case C_GetState:
				{
					if(!pdata.n)	//Common query - all channels brightness
					{
						pdata.buf[0] = ERR_NO;
						pdata.buf[1] = GetBrightness(Ch1);
						if(Ch2)
						{
							pdata.buf[2] = GetBrightness(Ch2);
							pdata.n = 3;
						}
						else pdata.n = 2;
					}
					else if(pdata.n == 1)
					{
						if(!pdata.buf[0]) // Ch1 brightness
						{
							pdata.buf[0] = ERR_NO;
							pdata.buf[1] = GetBrightness(Ch1);
							pdata.n = 2;
						}
						else if(Ch2 && pdata.buf[0] == 0x80) //Ch2 brightness
						{
							pdata.buf[0] = ERR_NO;
							pdata.buf[1] = GetBrightness(Ch2);
							pdata.n = 2;
						}
						else if(Features::FanControl && pdata.buf[0] == 0x01) //fan speed
						{
							pdata.buf[0] = ERR_NO;
							pdata.buf[1] = GetFanSpeed();
							pdata.n = 2;
						}
						else
						{
							pdata.buf[0] = ERR_NI;
							pdata.n = 1;
						}
					}
					else // param error
					{
						pdata.buf[0] = ERR_PA;
						pdata.n = 1;
					}
				}
					break;
				case C_SetBright:
				{
					if(pdata.n == 1)
					{
						if(!(pdata.buf[0] & 0x80))
						{
							SetBrightness(pdata.buf[0], Ch1);
						}
						else if(Ch2)
						{
							SetBrightness(pdata.buf[0] & 0x7F, Ch2);
						}
						else
						{
							pdata.buf[0] = ERR_NI;
							break;
						}
						pdata.buf[0] = ERR_NO;
					}
					else
					{
						pdata.buf[0] = ERR_PA;
						pdata.n = 1;
					}
				}
					break;
				case C_IncBright:
				{
					if(pdata.n == 1)
					{
						if(!(pdata.buf[0] & 0x80)) //1st channel selected
						{
							pdata.buf[1] = IncBrightness(pdata.buf[0], Ch1);
						}
						else if(Ch2)
						{
							pdata.buf[1] = IncBrightness(pdata.buf[0] & 0x7F, Ch2);
						}
						else
						{
							pdata.buf[0] = ERR_NI;
							break;
						}
						pdata.buf[0] = ERR_NO;
						pdata.n = 2;
					}
					else
					{
						pdata.buf[0] = ERR_PA;
						pdata.n = 1;
					}
				}
					break;
				case C_DecBright:
				{
					if(pdata.n == 1)
					{
						if(!(pdata.buf[0] & 0x80)) //1st channel selected
						{
							pdata.buf[1] = DecBrightness(pdata.buf[0], Ch1);
						}
						else if(Ch2)
						{
							pdata.buf[1] = DecBrightness(pdata.buf[0] & 0x7F, Ch2);
						}
						else
						{
							pdata.buf[0] = ERR_NI;
							break;
						}
						pdata.buf[0] = ERR_NO;
						pdata.n = 2;
					}
					else
					{
						pdata.buf[0] = ERR_PA;
						pdata.n = 1;
					}
				}
					break;
				case C_SetFan:
				{
					if(pdata.n == 1)
					{
						if(Features::FanControl)
						{
							SetFanSpeed(pdata.buf[0]);
							pdata.buf[0] = ERR_NO;
							pdata.buf[1] = GetFanSpeed();
							pdata.n = 2;
						}
						else pdata.buf[0] = ERR_NI;
					}
					else
					{
						pdata.buf[0] = ERR_PA;
						pdata.n = 1;
					}
				}
					break;
				case C_SetGfgFanAuto:
				{
					if(pdata.n == 3) pdata.buf[0] = ERR_NI;
					else pdata.buf[0] = ERR_PA;
				}
					break;
			}
		}

		#pragma inline=forced
		static void SetBrightness(uint8_t br, const Ch ch = Ch1)
		{
			if(br > 100) br = 100;
			curState.ch[ch] = br;
		}
		#pragma inline=forced
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
			if (state_nv.Data != curState.Data) //Only if changed
			{
				Unlock<Eeprom>();
				if(IsUnlocked<Eeprom>())
				{
					SetWordProgramming();
					state_nv = curState;
				}
				Lock<Eeprom>();
			}
		}

		#pragma inline=forced
		static void On()
		{
			curState = onState;
		}
		static void Off()
		{
			onState.ch[Ch1] = curState.ch[Ch1] ? curState.ch[Ch1] : 30;
			if(Features::TwoChannels) onState.ch[Ch2] = curState.ch[Ch2] ? curState.ch[Ch2] : 30;
			if(Features::FanControl) onState.fanSpeed = curState.fanSpeed;
			curState.Data = 0;
		}
		#pragma inline=forced
		static void ToggleOnOff()
		{
			if(!curState.Data) On();
			else Off();
		}

		#pragma inline=forced
		static uint8_t GetDeviceFeatures(const uint8_t)
		{
			return features;
		}
	};

	template<typename Features>
	LedDriver<Features>::state_t LedDriver<Features>::state_nv;
	template<typename Features>
	LedDriver<Features>::state_t LedDriver<Features>::curState = state_nv;
	template<typename Features>
	LedDriver<Features>::state_t LedDriver<Features>::onState;

  } //Ldrv
} //Mcudrv

