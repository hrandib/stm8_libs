/*
 *	Bootloader implementation, Wake protocol used:
 *  2016, Shestakov Dmitry
 */
#pragma once

#define WAKEDATABUFSIZE 128
#include "wake_base.h"

#define UBC_END 0x9000
#define VECTOR(N) (UBC_END + N * 4 )

#define TOSTRING(s) str(s)
#define str(s) #s

typedef void (*interrupt_handler_t)(void);

struct interrupt_vector {
		uint16_t interrupt_instruction;
		interrupt_handler_t interrupt_handler;
};

extern "C" {
void __iar_program_start();
}

#pragma location=".intvec"
extern "C" const interrupt_vector ISR_TABLE[32] = {
		{0x8200, __iar_program_start},/* Reset */
		{0x8200, (interrupt_handler_t)VECTOR(1)}, /* trap  */
		{0x8200, (interrupt_handler_t)VECTOR(2)}, /* irq0  */
		{0x8200, (interrupt_handler_t)VECTOR(3)}, /* irq1  */
		{0x8200, (interrupt_handler_t)VECTOR(4)}, /* irq2  */
		{0x8200, (interrupt_handler_t)VECTOR(5)}, /* irq3  */
		{0x8200, (interrupt_handler_t)VECTOR(6)}, /* irq4  */
		{0x8200, (interrupt_handler_t)VECTOR(7)}, /* irq5  */
		{0x8200, (interrupt_handler_t)VECTOR(8)}, /* irq6  */
		{0x8200, (interrupt_handler_t)VECTOR(9)}, /* irq7  */
		{0x8200, (interrupt_handler_t)VECTOR(10)}, /* irq8  */
		{0x8200, (interrupt_handler_t)VECTOR(11)}, /* irq9  */
		{0x8200, (interrupt_handler_t)VECTOR(12)}, /* irq10 */
		{0x8200, (interrupt_handler_t)VECTOR(13)}, /* irq11 */
		{0x8200, (interrupt_handler_t)VECTOR(14)}, /* irq12 */
		{0x8200, (interrupt_handler_t)VECTOR(15)}, /* irq13 */
		{0x8200, (interrupt_handler_t)VECTOR(16)}, /* irq14 */
		{0x8200, (interrupt_handler_t)VECTOR(17)}, /* irq15 */
		{0x8200, (interrupt_handler_t)VECTOR(18)}, /* irq16 */
		{0x8200, (interrupt_handler_t)VECTOR(19)}, /* irq17 */
		{0x8200, (interrupt_handler_t)VECTOR(20)}, /* irq18 */
		{0x8200, (interrupt_handler_t)VECTOR(21)}, /* irq19 */
		{0x8200, (interrupt_handler_t)VECTOR(22)}, /* irq20 */
		{0x8200, (interrupt_handler_t)VECTOR(23)}, /* irq21 */
		{0x8200, (interrupt_handler_t)VECTOR(24)}, /* irq22 */
		{0x8200, (interrupt_handler_t)VECTOR(25)}, /* irq23 */
		{0x8200, (interrupt_handler_t)VECTOR(26)}, /* irq24 */
		{0x8200, (interrupt_handler_t)VECTOR(27)}, /* irq25 */
		{0x8200, (interrupt_handler_t)VECTOR(28)}, /* irq26 */
		{0x8200, (interrupt_handler_t)VECTOR(29)}, /* irq27 */
		{0x8200, (interrupt_handler_t)VECTOR(30)}, /* irq28 */
		{0x8200, (interrupt_handler_t)VECTOR(31)}, /* irq29 */
};

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
	FORCEINLINE
	static void Init()
	{

	}

	FORCEINLINE
	static void Process()
	{

	}
};
}//Wk
}//Mcudrv
