/*
 *  Bootloader implementation, Wake protocol used
 *  2016, Shestakov Dmitry
 */
#pragma once

#include "uart.h"
#include "crc.h"
#include "flash.h"
#include "bootloaderDefines.h"

#define WAKEDATABUFSIZE 140

#define UBC_END 0x8600UL
#define UBC_END_ASM "$8600"

namespace Mcudrv {
  namespace Wk {
	using namespace WkBoot;

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

		enum {
			BOOTLOADER_VER = 0x01
		};

		enum {
			CRC_INIT = 0xDE,
      FEND = 0xC0,    //Frame END
      FESC = 0xDB,    //Frame ESCape
      TFEND = 0xDC,    //Transposed Frame END
      TFESC = 0xDD    //Transposed Frame ESCape
    };

    template<McuId Id>
    struct BootTraits;
    template<>
    struct BootTraits<ID_STM8S003F3>
    {
      enum {
        FlashStart = UBC_END,
        FlashEnd = 0xA000UL,
        FlashSize = FlashEnd - UBC_END,
        EepromStart = 0x4000UL,
        //	EepromEnd = 0x4080UL,
        EepromEnd = 0x4280UL,		//not documented, but exist
        EepromSize = EepromEnd - EepromStart
      };
    };
    template<>
    struct BootTraits<ID_STM8S103F3>
    {
      enum {
        FlashStart = UBC_END,
        FlashEnd = 0xA000UL,
        FlashSize = FlashEnd - UBC_END,
        EepromStart = 0x4000UL,
        EepromEnd = 0x4280UL,
        EepromSize = EepromEnd - EepromStart
      };
    };
    template<>
    struct BootTraits<ID_STM8L051F3>
    {
      enum {
        FlashStart = UBC_END,
        FlashEnd = 0xA000UL,
        FlashSize = FlashEnd - UBC_END,
        EepromStart = 0x1000UL,
        EepromEnd = 0x1100UL,
        EepromSize = EepromEnd - EepromStart
      };
    };
    template<>
    struct BootTraits<ID_STM8S105C6>
    {
      enum {
        FlashStart = UBC_END,
        FlashEnd = 0x10000UL,
        FlashSize = FlashEnd - UBC_END,
        EepromStart = 0x4000UL,
        EepromEnd = 0x4400UL,
        EepromSize = EepromEnd - EepromStart
      };
    };

    struct Packet
    {
//        uint8_t addr;
        uint8_t cmd;
        uint8_t n;
        uint8_t buf[WAKEDATABUFSIZE];
    };

    template<McuId DeviceID, Uarts::BaudRate baud = 9600UL,
             typename DriverEnable = Pd6>
		class Bootloader
		{
		private:
			typedef BootTraits<DeviceID> Traits;
			typedef Uarts::Uart Uart;
			enum {
				SingleWireMode = Uart::BaseAddr == UART1_BaseAddress ? Uarts::SingleWireMode : 0,
				BLOCK_SIZE = DeviceID >= ID_STM8S105C6 ? 128 : 64,
				BLOCK_BYTES = BLOCK_SIZE,
				FLASH_START = Traits::FlashStart,
				EEPROM_START = Traits::EepromStart
			};
			enum FLASH_MemType {
				MEMTYPE_PROG,
				MEMTYPE_DATA
			};
			enum State
			{
				WAIT_FEND = 0,     //ожидание приема FEND
				SEND_IDLE = 0,											//состояние бездействия
				ADDR,     //ожидание приема адреса						//передача адреса
				CMD,      //ожидание приема команды						//передача команды
				NBT,      //ожидание приема количества байт в пакете	//передача количества байт в пакете
				DATA,     //прием данных								//передача данных
				CRC,      //ожидание окончания приема CRC				//передача CRC
				CARR	   //ожидание несущей							//окончание передачи пакета
			};
			enum Err
			{
				ERR_NO,	//no error
				ERR_TX,	//Rx/Tx error
				ERR_BU,	//device busy error
				ERR_RE,	//device not ready error
				ERR_PA,	//parameters value error
				ERR_NI,	//Command not impl
				ERR_NR,	//no replay
				ERR_NC,	//no carrier
				ERR_ADDRFMT,	//new address is wrong
				ERR_EEPROMUNLOCK //EEPROM wasn't unlocked
			};
			static Packet packet_;
			static Crc::Crc8_NoLUT crc_;
			static uint8_t prevByte_;
			static State state_;            //Current tranfer mode
			static uint8_t rxBufPtr_;				//data pointer in Rx buffer
			static uint8_t cmd_;
			static uint8_t* memPtr_;
			__ramfunc static void WriteFlashBlock(u8** data)
			{
				/* Standard block programming mode */
				FLASH->CR2 |= FLASH_CR2_PRG;
				FLASH->NCR2 &= ~FLASH_CR2_PRG;

				/* Copy data bytes from RAM to FLASH memory */
				for(u16 Count = 0; Count < BLOCK_SIZE; ++Count) {
					*memPtr_++ = *((*data)++);
				}
#if defined(STM8S105)
				if (MemType == FLASH_MEMTYPE_DATA)
				{
					u16 timeout = (u16)0x6000;
					/* Waiting until High voltage flag is cleared*/
					while ((FLASH->IAPSR & 0x40) != 0x00 || (timeout == 0x00))
					{
						timeout--;
					}
				}
#endif /* STM8S105 */
			}
			FORCEINLINE static void GetInfo()
			{
				//check if key valid
				if(BOOTLOADER_KEY == packet_.buf[0]) {
					//generate response
					packet_.buf[0] = ERR_NO;
					packet_.buf[1] = DeviceID << 4 | BOOTLOADER_VER;
					packet_.buf[2] = (FLASH_START - 0x8000) / 64;
					packet_.n = 3;
				}
				//key is not valid
				else {
					packet_.buf[0] = ERR_PA;
					packet_.n = 1;
				}
			}
			FORCEINLINE static void WriteFlash()
			{
				u8 DataCount = packet_.n;
				u8* DataPointer = packet_.buf;
				//program beginning bytes before words
				while(((uint16_t)memPtr_ % 4) && (DataCount))
				{
					*memPtr_++ = *DataPointer++;
					while((FLASH->IAPSR & (FLASH_IAPSR_EOP | FLASH_IAPSR_WR_PG_DIS)) == 0)
						;
					DataCount--;
				}
				//program beginning words before blocks
				while(((uint16_t)memPtr_ % BLOCK_BYTES) && (DataCount >= 4))
				{
					FLASH->CR2 |= (u8)0x40;
					FLASH->NCR2 &= (u8)~0x40;
					for(uint8_t i = 0; i < 4; ++i) {
						*memPtr_++ = *DataPointer++;
					}
					while((FLASH->IAPSR & (FLASH_IAPSR_EOP | FLASH_IAPSR_WR_PG_DIS)) == 0)
						;
					DataCount -= 4;
				}
				//program blocks
				while(DataCount >= BLOCK_BYTES)
				{
					WriteFlashBlock(&DataPointer);
					DataCount -= BLOCK_BYTES;
				}
				//program remaining words (after blocks)
				while(DataCount >= 4)
				{
					FLASH->CR2 |= (u8)0x40;
					FLASH->NCR2 &= (u8)~0x40;
					for(uint8_t i = 0; i < 4; ++i) {
						*memPtr_++ = *DataPointer++;
					}
					while( (FLASH->IAPSR & (FLASH_IAPSR_EOP | FLASH_IAPSR_WR_PG_DIS)) == 0)
						;
					DataCount -= 4;
				}
				//program remaining bytes (after words)
				while(DataCount)
				{
					*memPtr_++ = *DataPointer++;
					while( (FLASH->IAPSR & (FLASH_IAPSR_EOP | FLASH_IAPSR_WR_PG_DIS)) == 0)
						;
					DataCount--;
				}
				packet_.buf[0] = ERR_NO;
				packet_.n = 1;
			}
			FORCEINLINE static void SetPosition()
			{
				//packet size validation
				if(packet_.n != 2) {
					packet_.buf[0] = ERR_PA;
					packet_.n = 1;
					return;
				}
				bool eepromFlag = packet_.buf[0] & 0x80;
				//set flash address
				if(!eepromFlag) {
					uint16_t addr = *(uint16_t*)packet_.buf + FLASH_START;
					//address is valid
					if(addr < Traits::FlashEnd) {
						memPtr_ = (uint8_t*)addr;
						packet_.buf[0] = ERR_NO;
						*(uint16_t*)&packet_.buf[1] = addr;
						packet_.n = 3;
						return;
					}
				}
				//set eeprom address
				else {
					uint16_t addr = (*(uint16_t*)packet_.buf & ~0x8000U) + EEPROM_START;
					//address is valid
					if(addr < Traits::EepromEnd) {
						memPtr_ = (uint8_t*)addr;
						packet_.buf[0] = ERR_NO;
						*(uint16_t*)&packet_.buf[1] = addr;
						packet_.n = 3;
						return;
					}
				}
				packet_.buf[0] = ERR_ADDRFMT;
				packet_.n = 1;
			}
			FORCEINLINE static void ReadFlash()
			{
				enum { BUF_OFFSET = 3 };
				//Check packet consistency
				if(packet_.n != 1 || packet_.buf[0] > 128) {
					packet_.buf[0] = ERR_PA;
					packet_.n = 1;
					return;
				}
				//length of data to read
				uint8_t length = packet_.buf[0];
				//Get End position of selected memory type
				const uint16_t memEnd = (uint16_t)memPtr_ & 0x8000U ? Traits::FlashEnd : Traits::EepromEnd;
				//If requested more than remained, read only a remnant
				if((uint16_t)memPtr_ + length > memEnd) {
					length = memEnd - (uint16_t)memPtr_;
				}
				//Fill buffer
				for(uint8_t i = 0; i < length; ++i) {
					packet_.buf[i + BUF_OFFSET] = *memPtr_++;
				}
				packet_.buf[0] = ERR_NO;
				*(uint16_t*)&packet_.buf[1] = (uint16_t)memPtr_ -
																			(memEnd == Traits::FlashEnd ? Traits::FlashStart : Traits::EepromStart);
				packet_.n = length + BUF_OFFSET;
			}
			FORCEINLINE static void Receive()
			{
				using namespace Uarts;
				while(true) {
					uint8_t rxData = Uart::Getch();              //чтение данных
					//Check for comm errors
					bool error = Uart::IsEvent(static_cast<Events>(EvParityErr | EvFrameErr | EvNoiseErr | EvOverrunErr)); //чтение флагов ошибок
					if(error)	{
						state_ = WAIT_FEND;		//ожидание нового пакета
						//   cmd_ = C_ERR;					//рапортуем об ошибке
						continue;
					}
					//Frame Begin
					if(rxData == FEND) {		//если обнаружено начало фрейма,
						prevByte_ = rxData;		//то сохранение пре-байта,
						crc_.Reset(CRC_INIT)(rxData);	//инициализация CRC,
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
							//        cmd_ = C_ERR;         //следующего за FESC, ошибка
							continue;
						}
					}
					else if(rxData == FESC) {             //если байт данных равен FESC, он просто
						continue;                         //запоминается в пре-байте
					}
					//Main processing
					switch(state_) {
					case ADDR:                     //-----> ожидание приема адреса
						if(rxData == (BOOTADDRESS | 0x80)) { //если бит 7 данных не равен нулю, то это адрес
							crc_(rxData); //то обновление CRC и
							state_ = CMD;       //переходим к приему команды
							break;
						}
						else {
							state_ = WAIT_FEND;        //адрес не совпал, ожидание нового пакета
							//							cmd_ = C_NOP;
							break;
						}
					case CMD:                      //-----> ожидание приема команды
						if(rxData & 0x80) {            //проверка бита 7 данных
							state_ = WAIT_FEND;        //если бит 7 не равен нулю,
							//              cmd_ = C_ERR;            //то ошибка
							break;
						}
						packet_.cmd = rxData;          //сохранение команды
						crc_(rxData);				//обновление CRC
						state_ = NBT;           //переходим к приему количества байт
						break;
					case NBT:					//-----> ожидание приема количества байт
						if(rxData > WAKEDATABUFSIZE) {	//если количество байт > bufsize,
							state_ = WAIT_FEND;
							//              cmd_ = C_ERR;		//то ошибка
							break;
						}
						packet_.n = rxData;
						crc_(rxData);		//обновление CRC
						rxBufPtr_ = 0;			//обнуляем указатель данных
						state_ = DATA;		//переходим к приему данных
						break;
					case DATA:                     //-----> ожидание приема данных
						if(rxBufPtr_ < packet_.n) {      //если не все данные приняты,
							packet_.buf[rxBufPtr_++] = rxData; //то сохранение байта данных,
							crc_(rxData);  //обновление CRC
							break;
						}
						if(rxData != crc_.GetResult()) {  //если приняты все данные, то проверка CRC
							state_ = WAIT_FEND;				//если CRC не совпадает,
							//              cmd_ = C_ERR;							//то ошибка
							break;
						}
						state_ = WAIT_FEND;		//прием пакета завершен,
						cmd_ = packet_.cmd;		//загрузка команды на выполнение
						return;
					default:
						;
					}
				}
			}
			FORCEINLINE static void Transmit()
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
						dataByte = BOOTADDRESS | 0x80; //то он передается (бит 7 равен единице)
						break;
					case CMD:                      //-----> передача команды
						dataByte = packet_.cmd & 0x7F;
						state_ = NBT;
						break;
					case NBT:                      //-----> передача количества байт
						dataByte = packet_.n;
						state_ = DATA;
						rxBufPtr_ = 0;                  //обнуление указателя данных для передачи
						break;
					case DATA:                     //-----> передача данных
						if(rxBufPtr_ < packet_.n) {
							dataByte = packet_.buf[rxBufPtr_++];
						}
						else {
							dataByte = crc_.GetResult();        //передача CRC
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
			FORCEINLINE static void Deinit()
			{
				using namespace Mem;
				Lock<Flash>();
				Lock<Eeprom>();
			}
		public:
			FORCEINLINE static bool ProcessHandshake()
			{
				if(Uart::IsEvent(Uarts::EvRxne) && Uart::Getch() == BOOTSTART_KEY) {
					DriverEnable::Set();
					Uart::Putch(BOOTRESPONSE);
					while(!Uart::IsEvent(Uarts::EvTxComplete))
						;
					DriverEnable::Clear();
					return true;
				}
				return false;
			}
			FORCEINLINE static void Go()
			{
				//simple check if user firmware exist
				if((*((u8*)UBC_END)==0x82) || (*((u8*)UBC_END)==0xAC)) {
					Deinit();
					//reset stack pointer (lower byte - because compiler decreases SP with some bytes)
					asm("LDW X,  SP ");
					asm("LD  A,  $FF");
					asm("LD  XL, A  ");
					asm("LDW SP, X  ");
					asm("JPF " UBC_END_ASM);
				}
			}
			FORCEINLINE static void Init()
			{
				using namespace Uarts;
				using namespace Mem;
				Unlock<Flash>();
				Unlock<Eeprom>();
				//Single Wire mode is default for UART1
				Uart::template Init<Cfg(Uarts::DefaultCfg | (Cfg)SingleWireMode), 9600UL>();
				DriverEnable::Clear();
				DriverEnable::template SetConfig<GpioBase::Out_PushPull_fast>();
			}
			FORCEINLINE static void Process()
			{
				while(true) {
					Receive();
					switch (cmd_) {
					case C_NOP: case C_ERR:
						cmd_ = C_NOP;
						continue;
					case C_ECHO:
						break;
					case C_GETINFO:
						GetInfo();
						break;
					case C_SETPOSITION:
						SetPosition();
						break;
					case C_READ:
						ReadFlash();
						break;
					case C_WRITE:
						WriteFlash();
						break;
					case C_GO:
						if(BOOTLOADER_KEY == packet_.buf[0]) {
							Go();
							//if user firmware not found
							packet_.buf[0] = ERR_RE; //Not ready
							packet_.n = 1;
						}
						else {
							packet_.buf[0] = ERR_PA;
							packet_.n = 1;
						}
						break;
					default:
						packet_.buf[0] = ERR_NI;
						packet_.cmd = C_ERR;
						packet_.n = 1;
					}
					Transmit();
				}
			}
		};

    template<McuId DeviceID, Uarts::BaudRate baud, typename DriverEnable>
    Packet Bootloader<DeviceID, baud, DriverEnable>::packet_;
    template<McuId DeviceID, Uarts::BaudRate baud, typename DriverEnable>
    Crc::Crc8_NoLUT Bootloader<DeviceID, baud, DriverEnable>::crc_;
    template<McuId DeviceID, Uarts::BaudRate baud, typename DriverEnable>
    uint8_t Bootloader<DeviceID, baud, DriverEnable>::prevByte_;
    template<McuId DeviceID, Uarts::BaudRate baud, typename DriverEnable>
    Bootloader<DeviceID, baud, DriverEnable>::State Bootloader<DeviceID, baud, DriverEnable>::state_;
    template<McuId DeviceID, Uarts::BaudRate baud, typename DriverEnable>
    uint8_t Bootloader<DeviceID, baud, DriverEnable>::rxBufPtr_;
    template<McuId DeviceID, Uarts::BaudRate baud, typename DriverEnable>
    uint8_t Bootloader<DeviceID, baud, DriverEnable>::cmd_;
    template<McuId DeviceID, Uarts::BaudRate baud, typename DriverEnable>
    uint8_t* Bootloader<DeviceID, baud, DriverEnable>::memPtr_ = (uint8_t*)UBC_END;

  }//Wk
}//Mcudrv
