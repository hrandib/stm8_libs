#pragma once
#include <string.h>
#define DEVICEINFO "LED Strip V100 100"  //размер буфера приема-передачи равен размеру этой строки
#include "stdint.h"
#include "wake.h"
#include "gpio.h"
#include "timers.h"


namespace Mcudrv
{
	struct StripBase
	{
		enum Cmd
		{
			C_GetState = WakeBase::C_ON + 1,
			C_SetBright,
			C_GetBright,
			C_SetFan,
			C_GetFan,
		};
	};

	class Strip : Wake1, StripBase
	{
	private:
		struct nvstate_t
		{
			uint8_t brightness;
			uint8_t fanSpeed;
		};
		enum
		{
			MAXLED = 0x7F,
			MINLED = 0,
			MAXFAN = 0x7F,
			MINFAN = 0x25
		};
		typedef Pa3 PowerSwitch;

		static const uint8_t Info[];

		#pragma data_alignment=4
		static nvstate_t NVstate @ ".eeprom.noinit";
		static nvstate_t curState;
	public:
	
		#pragma inline=forced
		static void Init()
		{
			using namespace T2;
			PowerSwitch::SetConfig<GpioBase::Out_PushPull>();
			GpioD::SetConfig<P3 | P4, GpioBase::Out_PushPull_fast>();
			Wake1::Init();
			Timer2::Init<Div1, static_cast<Cfg>(ARPE | CEN)>();
			Timer2::WriteAutoReload(0x7F);
			Timer2::SetChannelCfg<Ch2, Output, static_cast<ChannelCfgOut>(Out_PWM_Mode1 | Out_PreloadEnable)>();
			Timer2::SetChannelCfg<Ch1, Output, static_cast<ChannelCfgOut>(Out_PWM_Mode1 | Out_PreloadEnable)>();
			Timer2::ChannelEnable<Ch2>();
			Timer2::ChannelEnable<Ch1>();
			Timer2::EnableInterrupt<UpdInt>();
		}
		#pragma inline=forced
		static void Process();

		static void SetBrightness(uint8_t br)
		{	
			if (br >= 100) br = 127;
			else if (br > 76) br = (br - 38) * 2;
			curState.brightness = br;
		}

		static uint8_t ReadBrightness()
		{
			uint8_t x = curState.brightness;
			if (x >= MAXLED) x = 100;
			else if (x > 76) x = (x / 2) + 38;
			return x;
		}

		static void SetFanSpeed(uint8_t sp)
		{
			using namespace T2;
			if (!sp) curState.fanSpeed = 0;
			else if (sp < 100) curState.fanSpeed = sp + (MAXFAN - 100);
			else curState.fanSpeed = MAXFAN;
			Timer2::WriteCompareLSB<Ch1>(curState.fanSpeed);
		}
		
		static uint8_t ReadFanSpeed()
		{
			if (!curState.fanSpeed) return 0;
			else return curState.fanSpeed - (MAXFAN - 100);
		}

		#pragma inline=forced
		static void SaveSettings()
		{
			using namespace Mem;
			using namespace T1;
			Unlock<Eeprom>();
			if (IsUnlocked<Eeprom>())
			{
				NVstate = curState;
				pdata.buf[0] = WakeBase::ERR_NO;
			}
			else pdata.buf[0] = WakeBase::ERR_EEPROMUNLOCK;
			Lock<Eeprom>();
		}
		
		#pragma inline=forced
		static bool IsActive()
		{
			return activity;
		}
		
		#pragma inline=forced
		static void UpdIRQ()
		{
			using namespace T2;
			static uint8_t delay;
			if (++delay == 128)
			{
				delay = 0;
				if (Timer2::GetCompareLSB<Ch2>() < curState.brightness)
				{
					if (++Timer2::GetCompareLSB<Ch2>() == MAXLED) PowerSwitch::Set();
				}
				else if (Timer2::GetCompareLSB<Ch2>() > curState.brightness)
				{
					--Timer2::GetCompareLSB<Ch2>();
					PowerSwitch::Clear();
				}
			}
		}
		
		#pragma inline=forced
		static void SaveStateIfChanged()
		{
			using namespace Mem;
			static uint8_t prevState = NVstate.brightness;
			static bool changed = false;
			uint8_t state = curState.brightness;
			if (prevState != state)
			{
				changed = true;
				prevState = state;
			}
			else if (changed)
			{
				changed = false;
				Unlock<Eeprom>();
				if (IsUnlocked<Eeprom>())
				{
					NVstate.brightness = state;
				}
				Lock<Eeprom>();
			}
		}

		#pragma inline=forced
		static void SetPrevBrightness()
		{
			curState.brightness = NVstate.brightness;
		}
	};

	Strip::nvstate_t Strip::NVstate;
	Strip::nvstate_t Strip::curState = NVstate;
	const uint8_t Strip::Info[] = DEVICEINFO;

	INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13)
	{
		using namespace T2;
		if (Timer2::CheckIntStatus<UpdInt>())
		{
			Timer2::ClearIntFlag<UpdInt>();
			Strip::UpdIRQ();
		}
	}

} //Mcudrv

