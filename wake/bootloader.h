/*
 *  Bootloader implementation, Wake protocol used
 *  2016, Shestakov Dmitry
 */
#pragma once

#define WAKEDATABUFSIZE 128
#include "wake_base.h"
#include "flash.h"

#define UBC_END 0x8400
#define UBC_END_ASM "$8400"

//#define TOSTRING(s) str(s)
//#define str(s) #s
namespace Mcudrv {
namespace Wk {

typedef void (*interrupt_handler_t)(void);

struct interrupt_vector {
		uint16_t instruction;
		interrupt_handler_t handler;
};

#define VECTOR(N) ((interrupt_handler_t)(UBC_END + N * 4))

extern "C" {
void __iar_program_start();
}

#pragma location=".intvec"
extern "C" const interrupt_vector ISR_TABLE[32] = {
		{0x8200, __iar_program_start},/* Reset */
		{0x8200, VECTOR(1)}, /* trap  */
		{0x8200, VECTOR(2)}, /* irq0  */
		{0x8200, VECTOR(3)}, /* irq1  */
		{0x8200, VECTOR(4)}, /* irq2  */
		{0x8200, VECTOR(5)}, /* irq3  */
		{0x8200, VECTOR(6)}, /* irq4  */
		{0x8200, VECTOR(7)}, /* irq5  */
		{0x8200, VECTOR(8)}, /* irq6  */
		{0x8200, VECTOR(9)}, /* irq7  */
		{0x8200, VECTOR(10)}, /* irq8  */
		{0x8200, VECTOR(11)}, /* irq9  */
		{0x8200, VECTOR(12)}, /* irq10 */
		{0x8200, VECTOR(13)}, /* irq11 */
		{0x8200, VECTOR(14)}, /* irq12 */
		{0x8200, VECTOR(15)}, /* irq13 */
		{0x8200, VECTOR(16)}, /* irq14 */
		{0x8200, VECTOR(17)}, /* irq15 */
		{0x8200, VECTOR(18)}, /* irq16 */
		{0x8200, VECTOR(19)}, /* irq17 */
		{0x8200, VECTOR(20)}, /* irq18 */
		{0x8200, VECTOR(21)}, /* irq19 */
		{0x8200, VECTOR(22)}, /* irq20 */
		{0x8200, VECTOR(23)}, /* irq21 */
		{0x8200, VECTOR(24)}, /* irq22 */
		{0x8200, VECTOR(25)}, /* irq23 */
		{0x8200, VECTOR(26)}, /* irq24 */
		{0x8200, VECTOR(27)}, /* irq25 */
		{0x8200, VECTOR(28)}, /* irq26 */
		{0x8200, VECTOR(29)}, /* irq27 */
		{0x8200, VECTOR(30)}, /* irq28 */
		{0x8200, VECTOR(31)}, /* irq29 */
};

enum McuId
{
	ID_STM8S003F3,
	ID_STM8S103F3,
	ID_STM8S105C6 = 0x10
};

enum
{
	BOOTLOADER_KEY = 0x34B8126E,	// Host should use big endian format
	WakeAddress = 112
};

template<McuId Device, Uarts::BaudRate baud = 9600UL,
				 typename DriverEnable = Pd6>
class Bootloader
{
private:
	typedef Uarts::Uart Uart;
	enum { SingleWireMode = (Uart::BaseAddr == UART1_BaseAddress ? Uarts::SingleWireMode : 0) };
	enum { BlockSize = (Device >= ID_STM8S105C6 ? 128 : 64) };
	enum InstructionSet {
		C_Err = 1,
		C_Echo = 2,
		C_GetInfo = 3,
		C_BootStart = 12,
		C_SetPosition,
		C_Read,
		C_Write
	};
	static WakeData::Packet packet_;
	static Crc::Crc8 crc_;
	static uint8_t prevByte_;
	static State state_;				//Current tranfer mode
	static uint8_t ptr_;				//data pointer in Rx buffer
	static uint8_t cmd_;

	__ramfunc
	static void Write()
	{ }

	FORCEINLINE
	static void Deinit()
	{
		using namespace Mem;
		Lock<Flash>();
		Lock<Eeprom>();
	}

	static void Receive()
	{
		using namespace Uarts;
		while(true) {
			uint8_t rxData = Uart::Getch();              //чтение данных
			//Check for comm errors
			bool error = Uart::IsEvent(static_cast<Events>(EvParityErr | EvFrameErr | EvNoiseErr | EvOverrunErr)); //чтение флагов ошибок
			if(error)	{
				state_ = WAIT_FEND;		//ожидание нового пакета
				cmd_ = C_ERR;					//рапортуем об ошибке
				return;
			}
			//Frame Begin
			if(rxData == FEND) {		//если обнаружено начало фрейма,
				prevByte_ = rxData;		//то сохранение пре-байта,
				crc_.Reset(CRC_INIT).Eval(rxData);	//инициализация CRC,
				state_ = ADDR;				//сброс указателя данных,
				continue;
			}
			if(state_ == WAIT_FEND) {          //-----> если ожидание FEND,
				continue;
			}
			//Byte stuffing cleanup
			char prev = prevByte_;               //сохранение старого пре-байта
			prevByte_ = rxData;              //обновление пре-байта
			if(prev == FESC) {                    //если пре-байт равен FESC,
				if(rxData == TFESC) {            //а байт данных равен TFESC,
					rxData = FESC;               //то заменить его на FESC
				}
				else if(rxData == TFEND) {       //если байт данных равен TFEND,
					rxData = FEND;          //то заменить его на FEND
				}
				else {
					state_ = WAIT_FEND;     //для всех других значений байта данных,
					cmd_ = C_ERR;         //следующего за FESC, ошибка
					return;
				}
			}
			else if(rxData == FESC) {             //если байт данных равен FESC, он просто
				continue;                         //запоминается в пре-байте
			}
			//Main processing
			switch(state_) {
			case ADDR:                     //-----> ожидание приема адреса
				if(rxData == WakeAddress | 0x80) { //если бит 7 данных не равен нулю, то это адрес
					crc_(rxData); //то обновление CRC и
					packet_.addr = rxData;
					state_ = CMD;       //переходим к приему команды
					break;
				}
				else {
					state_ = WAIT_FEND;        //адрес не совпал, ожидание нового пакета
					break;
				}
			case CMD:                      //-----> ожидание приема команды
				if(rxData & 0x80) {            //проверка бита 7 данных
					state_ = WAIT_FEND;        //если бит 7 не равен нулю,
					cmd_ = C_ERR;            //то ошибка
					return;
				}
				packet_.cmd = rxData;          //сохранение команды
				crc_(rxData);				//обновление CRC
				state_ = NBT;           //переходим к приему количества байт
				break;
			case NBT:					//-----> ожидание приема количества байт
				if(rxData > WAKEDATABUFSIZE) {	//если количество байт > bufsize,
					state_ = WAIT_FEND;
					cmd_ = C_ERR;		//то ошибка
					return;
				}
				packet_.n = rxData;
				crc_(rxData);		//обновление CRC
				ptr_ = 0;			//обнуляем указатель данных
				state_ = DATA;		//переходим к приему данных
				break;
			case DATA:                     //-----> ожидание приема данных
				if(ptr_ < packet_.n) {      //если не все данные приняты,
					packet_.buf[ptr_++] = rxData; //то сохранение байта данных,
					crc_(rxData);  //обновление CRC
					break;
				}
				if(rxData != crc_.Get()) {  //если приняты все данные, то проверка CRC
					state_ = WAIT_FEND;				//если CRC не совпадает,
					cmd_ = C_ERR;							//то ошибка
					return;
				}
				state_ = WAIT_FEND;		//прием пакета завершен,
				cmd_ = packet_.cmd;		//загрузка команды на выполнение
				return;
			}
		}
	}
	static void Transmit()
	{
		using namespace Uarts;
		DriverEnable::Set(); //Switch to TX
		char txData = FEND;
		crc_.Reset(CRC_INIT); //инициализация CRC,
		crc_(txData);		//обновление CRC
		Uart::Putch(txData);
		state_ = ADDR;
		prevByte_ = TFEND;
		while(true) {
			char dataByte;
			if(prevByte_ == FEND) {               //если производится стаффинг,
				prevByte_ = TFEND;									//передача TFEND вместо FEND
				Uart::Putch(prevByte_);
				continue;
			}
			if(prevByte_ == FESC) {               //если производится стаффинг,
				prevByte_ = TFESC;
				Uart::Putch(prevByte_);						//передача TFESC вместо FESC
				continue;
			}
			switch(state_) {
			case ADDR:                     //-----> передача адреса
				state_ = CMD;
				//FIXME: MSB ignored in CRC calculation on PC side (fix on pc)
				dataByte = WakeAddress | 0x80; //то он передается (бит 7 равен единице)
				break;
			case CMD:                      //-----> передача команды
				dataByte = packet_.cmd & 0x7F;
				state_ = NBT;
				break;
			case NBT:                      //-----> передача количества байт
				dataByte = packet_.n;
				state_ = DATA;
				ptr_ = 0;                  //обнуление указателя данных для передачи
				break;
			case DATA:                     //-----> передача данных
				if(ptr_ < packet_.n) {
					dataByte = packet_.buf[ptr_++];
				}
				else {
					dataByte = crc_.Get();        //передача CRC
					state_ = CRC;
				}
				break;
			default:
				state_ = SEND_IDLE;          //передача пакета завершена
				while(!Uart::IsEvent(EvTxComplete))
					;
				DriverEnable::Clear();		//Switch to RX
				return;
			}
			crc_(dataByte);     //обновление CRC
			prevByte_ = dataByte;              //сохранение пре-байта
			if(dataByte == FEND)// || data_byte == FESC)
				dataByte = FESC;                 //передача FESC, если нужен стаффинг
			Uart::Putch(dataByte);
		}
	}

public:
	static void Go()
	{
		Deinit();
		//reset stack pointer (lower byte - because compiler decreases SP with some bytes)
		asm("LDW X,  SP ");
		asm("LD  A,  $FF");
		asm("LD  XL, A  ");
		asm("LDW SP, X  ");
		asm("JPF " UBC_END_ASM);
	}

	FORCEINLINE
	static void Init()
	{
		using namespace Uarts;
		//Single Wire mode is default for UART1
		Uart::template Init<Cfg(Uarts::DefaultCfg | (Cfg)SingleWireMode), 9600UL>();
		DriverEnable::Clear();
		DriverEnable::template SetConfig<GpioBase::Out_PushPull_fast>();
	}

	FORCEINLINE
	static void Process()
	{
		while(true) {
			Receive();
			if(cmd_ == C_Err)
				continue;
			switch (cmd_) {
			case C_Echo:
				break;
			case C_BootStart:
				break;
			case C_SetPosition:
				break;
			case C_Read:
				break;
			case C_Write:
				break;
			default:
				packet_.buf[0] = ERR_NI;
				packet_.cmd = C_Err;
				packet_.n = 1;
			}
			Transmit();
		}
//		Write();
//		Go();
	}
};

template<McuId Device, Uarts::BaudRate baud, typename DriverEnable>
WakeData::Packet Bootloader<Device, baud, DriverEnable>::packet_;
template<McuId Device, Uarts::BaudRate baud, typename DriverEnable>
Crc::Crc8 Bootloader<Device, baud, DriverEnable>::crc_;
template<McuId Device, Uarts::BaudRate baud, typename DriverEnable>
uint8_t Bootloader<Device, baud, DriverEnable>::prevByte_;
template<McuId Device, Uarts::BaudRate baud, typename DriverEnable>
State Bootloader<Device, baud, DriverEnable>::state_;
template<McuId Device, Uarts::BaudRate baud, typename DriverEnable>
uint8_t Bootloader<Device, baud, DriverEnable>::ptr_;
template<McuId Device, Uarts::BaudRate baud, typename DriverEnable>
uint8_t Bootloader<Device, baud, DriverEnable>::cmd_;


}//Wk
}//Mcudrv
