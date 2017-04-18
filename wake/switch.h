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
#include "adc_keyboard.h"
#include "Relays.h"

namespace Mcudrv
{
	namespace Wk
	{

	struct SwitchDefaultFeatures
	{
		enum {
			ChannelsNumber = 6
		};
	};

	template<typename Features = SwitchDefaultFeatures>
	class Switch : WakeData
	{
	private:
		typedef Switch<Features> Self;
		enum InstructionSet {
			C_GetState = 24,
			C_SetState,
			C_ClearState,
			C_WriteState,
			C_SetChannel,
			C_ClearChannel,
			C_ToggleChannel
		};
		typedef Pd3 R0;
		typedef Pd2 R1;
		typedef Pc7 R2;
		typedef Pc6 R3;
		typedef Pc5 R4;
		typedef Pc3 R5;

		typedef Pinlist<R0, R1, R2, R3, R4, R5> Vport;

		typedef Relays<Vport> SwitchRelays;
		typedef AdcKeys::Buttons<Self> Keyboard;

		#pragma data_alignment=4
		#pragma location=".eeprom.noinit"
		static uint8_t nv_state;// @ ".eeprom.noinit";
		FORCEINLINE
		static void FormResponse(void(*cb)(uint8_t))
		{
			if(pdata.n == 1) {
				cb(pdata.buf[0]);
				pdata.buf[0] = ERR_NO;
				pdata.buf[1] = SwitchRelays::ReadODR();
				pdata.n = 2;
			}
			else {
				pdata.buf[0] = ERR_PA;
				pdata.n = 1;
			}
		}
		FORCEINLINE
		static void FormResponseMask(void(*cb)(uint8_t))
		{
			if(pdata.n == 1 && pdata.buf[0] < Features::ChannelsNumber) {
				cb(1U << pdata.buf[0]);
				pdata.buf[0] = ERR_NO;
				pdata.buf[1] = SwitchRelays::ReadODR();
				pdata.n = 2;
			}
			else {
				pdata.buf[0] = ERR_PA;
				pdata.n = 1;
			}
		}
	public:
		enum {
			deviceMask = DevSwitch,
			features = Features::ChannelsNumber
		};
		FORCEINLINE
		static void Init()
		{
			Vport::SetConfig<GpioBase::Out_PushPull>();
			Keyboard::template Init<Adcs::Ch2, Adcs::Div12>();
			SwitchRelays::Write(nv_state);
		}
		FORCEINLINE
		static void Process()
		{
			switch(cmd) {
			case C_GetState:
				if(!pdata.n) {
					pdata.buf[0] = ERR_NO;
					pdata.buf[1] = SwitchRelays::ReadODR();
					pdata.buf[2] = SwitchRelays::ReadPrevState();
					pdata.n = 3;
				}
				else {
					pdata.buf[0] = ERR_PA;
					pdata.n = 1;
				}
				break;
			case C_SetState:
				FormResponse(SwitchRelays::Set);
				break;
			case C_ClearState:
				FormResponse(SwitchRelays::Clear);
				break;
			case C_WriteState:
				FormResponse(SwitchRelays::Write);
				break;
			case C_SetChannel:
				FormResponseMask(SwitchRelays::Set);
				break;
			case C_ClearChannel:
				FormResponseMask(SwitchRelays::Clear);
				break;
			case C_ToggleChannel:
				FormResponseMask(SwitchRelays::Toggle);
				break;
			default:
				processedMask |= deviceMask;
			}
		}
		FORCEINLINE
		static void On()
		{
			SwitchRelays::Restore();
		}
		FORCEINLINE
		static void Off()
		{
			SwitchRelays::Clear(0xFF);
		}
		FORCEINLINE
		static void ToggleOnOff()
		{
			SwitchRelays::Toggle(0xFF);
		}
		FORCEINLINE
		static void SaveState()
		{
			using namespace Mem;
			uint8_t currentState = SwitchRelays::ReadODR();
			if(nv_state == currentState) {
				return;
			}
			Unlock<Eeprom>();
			if(IsUnlocked<Eeprom>()) {
				nv_state = currentState;
				Lock<Eeprom>();
			}
		}
		FORCEINLINE
		static void KeyboardHandler(uint8_t key)
		{
			if(key < 6) {
				SwitchRelays::Toggle(1U << key);
			}
			else {
				SwitchRelays::Toggle(0xFF);
			}
		}
	};

	template<typename Features>
	uint8_t Switch<Features>::nv_state;

	}//Wk
}//Mcudrv
