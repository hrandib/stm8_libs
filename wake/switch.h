#pragma once
#include <string.h>

#include "wake_base.h"
#include "pinlist.h"
#include "Relays.h"
#include "adc_keyboard.h"

namespace Mcudrv
{
	namespace Wk
	{

	typedef Pd3 R0;
	typedef Pd2 R1;
	typedef Pc7 R2;
	typedef Pc6 R3;
	typedef Pc5 R4;
	typedef Pc3 R5;


	typedef Pinlist<R0, R1, R2, R3, R4, R5> Vport;
	typedef Wk::Relays<Vport> Relays;
  
	enum Cmd
	{
		C_GetState = 24,
		C_SetState,
		C_ClearState, 
		C_WriteState,
		C_SetChannel,
		C_ClearChannel
	};
	template<typename Features = SwitchDefaultFeatures>
	class Switch
	{
	private:
		static void ProcessKeyboard(uint8_t key)
		{
			switch (key)
			{
			case 0: R0::Toggle();
				break;
			case 1: R1::Toggle();
				break;
			case 2: R2::Toggle();
				break;
			case 3: R3::Toggle();
				break;
			case 4: R4::Toggle();
				break;
			case 5: R5::Toggle();
				break;
			case 6: Relays::Toggle(0xff);
					break;
			default:
					;
			}
		}

	public:
		#pragma data_alignment=4
		#pragma location=".eeprom.noinit"
		static uint8_t nv_state;// @ ".eeprom.noinit";
		#pragma inline=forced
		static void Init()
		{
			Vport::SetConfig<GpioBase::Out_PushPull>();
			if(nv_state < 0x40) {
				Relays::Write(nv_state);
			}
		}

		#pragma inline=forced
		static void Process();

		#pragma inline=forced
		static void SaveSettings()
		{
			using namespace Mem;
			Unlock<Eeprom>();
			if (IsUnlocked<Eeprom>())
			{
				nv_state = Relays::ReadODR();
				pdata.buf[0] = Wk::ERR_NO;
			}
			else pdata.buf[0] = Wk::ERR_EEPROMUNLOCK;
			Lock<Eeprom>();
		}
		
		#pragma inline=forced
		static bool IsActive()
		{
			return activity;
		}
	};

	}//Wk
}//Mcudrv
