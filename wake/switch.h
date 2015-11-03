#pragma once
#include <string.h>
#define DEVICEINFO "Switch, V100, 600"  //размер буфера приема-передаче равен размеру этой строки

#include "wake.h"
#include "gpio.h"
#include "Relays.h"



namespace Mcudrv
{
	typedef Pd3 R0;
	typedef Pd2 R1;
	typedef Pc7 R2;
	typedef Pc6 R3;
	typedef Pc5 R4;
	typedef Pc3 R5;


	typedef Pinlist<R0, R1, R2, R3, R4, R5> Vport;
	typedef cRelays<Vport> Relays;
  
	namespace Sw
  {
	enum Cmd
	{
		C_GetState = Wk::C_ON + 1,
		C_SetState,
		C_ClearState, 
		C_WriteState	
	};

	class Switch: Wake1
	{
	private:
		static const uint8_t Info[];
		
	public:
		#pragma data_alignment=4
		static uint8_t NVstate @ ".eeprom.noinit";
		#pragma inline=forced
		static void Init()
		{
			Wake1::Init();
			if (NVstate < 0x40) Relays::Write(NVstate);
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
				NVstate = Relays::ReadODR();
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

	uint8_t Switch::NVstate;
	const uint8_t Switch::Info[] = DEVICEINFO;

	
  }
}
