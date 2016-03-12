/*
 *	Bootloader implementation, Wake protocol used:
 *  2016, Shestakov Dmitry
 */

#pragma once

#define TOSTRING(s) str(s)
#define str(s) #s

#define WAKEDATABUFSIZE 256
#define UBC_END 9000		//In HEX

#include "wake_base.h"

namespace Mcudrv {
namespace Wk {

enum McuId
{
	ID_STM8S003F3,
	ID_STM8S103F3,
	ID_STM8S105C6
};

enum
{
	BOOTLOADER_KEY = 0x34B8126E,	// Host should use big endian format
	WakeAddress = 112
};
template<Uarts::BaudRate baud = 9600UL,
				 typename DriverEnable = Pd6>
class Bootloader : WakeData
{
private:
	typedef Uarts::Uart Uart;
	enum { SingleWireMode = (Uart::BaseAddr == UART1_BaseAddress ? Uarts::SingleWireMode : 0) };
	static volatile uint8_t prev_byte;
	static volatile State state;				//Current tranfer mode
	static volatile uint8_t ptr;				//data pointer in Rx buffer
	static Crc::Crc8 crc;

	static void Deinit()
	{
		/* Lock program memory */
		FLASH->IAPSR = ~0x02;
		/* Lock data memory */
		FLASH->IAPSR = ~0x08;
	}

	static void Go()
	{
		Deinit();
		//reset stack pointer (lower byte - because compiler decreases SP with some bytes)
		asm("LDW X,  SP ");
		asm("LD  A,  $FF");
		asm("LD  XL, A  ");
		asm("LDW SP, X  ");
		asm("JPF $" TOSTRING(UBC_END));
	}

public:
	static void Init();
	static void Process();
};
}//Wk
}//Mcudrv
