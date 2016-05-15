#pragma once

#include "wake_base.h"
#include "gpio.h"
#include "timers.h"

/*
 * PA3 - Bypass power switch
 * PD3 - Brightness PWM (TIM2 CH2)
 * PD4 - Fan control PWM (TIM2 CH1)
 */

namespace Mcudrv
{
	namespace Wk
  {

	struct LedDriverDefaultFeatures
	{
		enum {
			TwoChannels = false,
			FanControl = true
		};
	};

	template<typename Features = LedDriverDefaultFeatures>
	class LedDriver : WakeData
	{
	private:
		enum {
			PWM_MAX = 0x7F,
			FAN_START_VALUE = 15,
		};
		typedef Pa3 PowerSwitch;
		union state_t {
			struct {
				uint8_t ch[2];
				uint8_t fanSpeed;
			};
			uint32_t Data;
		};
		enum Ch { Ch1, Ch2 = Features::TwoChannels };
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
		static uint8_t GetFanSpeed()
		{
			uint8_t speed = curState.fanSpeed;
			if(speed == PWM_MAX) {
				speed = 100;
			}
			else if(speed > FAN_START_VALUE) {
				speed -= FAN_START_VALUE;
			}
			else {
				speed = 0;
			}
			return speed;
		}
		static void SetFanSpeed(uint8_t speed)
		{
			if(speed >= 100) {
				speed = PWM_MAX;
			}
			else if (speed > 0)	{
				speed = speed + FAN_START_VALUE;
			}
			curState.fanSpeed = speed;
		}
		FORCEINLINE
		static void SetCh1()
		{
			using namespace T2;
			static uint8_t br = state_nv.ch[Ch1];
			if(br < curState.ch[Ch1]) {
				++br;
			}
			else if(br > curState.ch[Ch1]) {
				--br;
				PowerSwitch::Clear();
			}
			uint8_t linBr;
			if(br >= 100) {
				//linBr = PWM_MAX;
				PowerSwitch::Set();
			}
			else if(br > 76) {
				linBr = (br - 38) * 2;
			}
			else {
				linBr = br;
			}
			Timer2::WriteCompareByte<T2::Ch2>(linBr);
		}
		FORCEINLINE
		static void SetCh2(stdx::Int2Type<true>)
		{
			using namespace T2;
			static uint8_t br = state_nv.ch[Ch2];
			if(br < curState.ch[Ch2])
				++br;
			else if(br > curState.ch[Ch2])
				--br;
			Timer2::WriteCompareByte<T2::Ch2>(br);
		}
		FORCEINLINE
		static void SetCh2(stdx::Int2Type<false>) { }
		FORCEINLINE
		static void UpdIRQ()	//Soft Dimming
		{
			SetCh1();
			SetCh2(stdx::Int2Type<Features::TwoChannels>());
			using namespace T2;
			if(Features::FanControl) {
				if(Timer2::ReadCompareByte<T2::Ch1>() < curState.fanSpeed) {
					Timer2::GetCompareByte<T2::Ch1>()++;
				}
				else if(Timer2::ReadCompareByte<T2::Ch1>() > curState.fanSpeed) {
					Timer2::GetCompareByte<T2::Ch1>()--;
				}
			}
		}
		FORCEINLINE
		static void SetBrightness(uint8_t br, const Ch ch = Ch1)
		{
			if(br > 100) br = 100;
			curState.ch[ch] = br;
		}
		FORCEINLINE
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

	public:
		enum
		{
			deviceMask = DevLedDriver,
			features = Features::TwoChannels | Features::FanControl << 1UL
		};
		FORCEINLINE
		static void Init()
		{
			using namespace T2;
			PowerSwitch::SetConfig<GpioBase::Out_PushPull>();
			Timer2::Init(Div_1, Cfg(ARPE | CEN));
			Timer2::WriteAutoReload(0x7F);
			Pd3::SetConfig<GpioBase::Out_PushPull_fast>();
			Timer2::SetChannelCfg<T2::Ch2, Output, ChannelCfgOut(Out_PWM_Mode1 | Out_PreloadEnable)>();
			Timer2::ChannelEnable<T2::Ch2>();
			if(Features::FanControl) {
				Pd4::SetConfig<GpioBase::Out_PushPull_fast>();
				Timer2::SetChannelCfg<T2::Ch1, Output, ChannelCfgOut(Out_PWM_Mode1 | Out_PreloadEnable)>();
				Timer2::ChannelEnable<T2::Ch1>();
			}
			OpTime::SetTimerCallback(UpdIRQ);
		}
		FORCEINLINE
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
		FORCEINLINE
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
				}
				Lock<Eeprom>();
			}
		}
		FORCEINLINE
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
		FORCEINLINE
		static void ToggleOnOff()
		{
			if(!curState.Data) On();
			else Off();
		}
		FORCEINLINE
		static uint8_t GetDeviceFeatures(const uint8_t)
		{
			return features;
		}
	};

	template<typename Features>
	typename LedDriver<Features>::state_t LedDriver<Features>::state_nv;
	template<typename Features>
	typename LedDriver<Features>::state_t LedDriver<Features>::curState = state_nv;
	template<typename Features>
	typename LedDriver<Features>::state_t LedDriver<Features>::onState;

  } //Ldrv
} //Mcudrv

