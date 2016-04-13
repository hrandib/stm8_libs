/*
 *  Bootloader implementation, Wake protocol used
 *  2016, Shestakov Dmitry
 */
#pragma once

#define WAKEDATABUFSIZE 128
#include "wake_base.h"
#include "flash.h"
#include <string.h>

#define UBC_END 0x8600UL
#define UBC_END_ASM "$8600"

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
      ID_STM8S105C6 = 0x08	//Devices with 128 flash block size start from 0x08
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
        EepromSize = EepromEnd - EepromStart,
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
        EepromSize = EepromEnd - EepromStart,
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
        EepromSize = EepromEnd - EepromStart,
      };
    };

    enum
    {
      BOOTLOADER_KEY = 0x34B8126EUL,	// Host should use big endian format
      WakeAddress = 112,
      BOOTLOADER_VER = 0x01
    };

    template<McuId DeviceID = ID_STM8S003F3, Uarts::BaudRate baud = 9600UL,
             typename DriverEnable = Pd6>
    class Bootloader
    {
    private:
      typedef BootTraits<DeviceID> Traits;
      typedef Uarts::Uart Uart;
      enum {
        BlockSize = DeviceID >= ID_STM8S105C6 ? 128 : 64,
        SingleWireMode = Uart::BaseAddr == UART1_BaseAddress ? Uarts::SingleWireMode : 0,
        BLOCK_BYTES = BlockSize,
        FLASH_START = Traits::FlashStart,
        EEPROM_START = Traits::EepromStart,
        BLOCK_SIZE = BlockSize
      };
      enum FLASH_MemType {
        MEMTYPE_PROG,
        MEMTYPE_DATA
      };
      enum InstructionSet {
        C_Err = 1,
        C_Echo = 2,
        C_GetInfo = 3,
        C_SetPosition = 12,
        C_Read,
        C_Write,
        C_Go
      };
      static WakeData::Packet packet_;
      static Crc::Crc8_NoLUT crc_;
      static uint8_t prevByte_;
      static State state_;                  //Current tranfer mode
      static uint8_t rxBufPtr_;				//data pointer in Rx buffer
      static uint8_t cmd_;
      static uint8_t* memPtr_;

      /*struct Uart : Uarts::Uart
      {
        static uint8_t Getch()
        {
          uint8_t count = 0;
          while(true) {
            if(IsEvent(Uarts::EvRxne)) {
              return Regs()->DR;
            }
            if(!--count) {
              state_ = WAIT_FEND;
              return 0;
            }
            delay_us<1000>();
          }
        }
      };*/

      __ramfunc
      static void WriteFlashBlock(u8** data)
      {
        /* Standard block programming mode */
        FLASH->CR2 |= (u8)0x01;
        FLASH->NCR2 &= (u8)~0x01;

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
        if(BOOTLOADER_KEY == *(uint32_t*)packet_.buf) {
          //set position at application flash start
          memPtr_ = (uint8_t*)UBC_END;
          //generate response
          packet_.buf[0] = ERR_NO;
          packet_.buf[1] = DeviceID << 4 | BOOTLOADER_VER;
          packet_.buf[2] = (Traits::FlashStart - 0x8000) / 64;
          packet_.n = 3;
        }
        //key not valid
        else {
          packet_.buf[0] = ERR_PA;
          packet_.n = 1;
        }
      }
      static void WriteFlash()
      {
        u8 DataCount = packet_.n;
        u8* DataPointer = packet_.buf;
        //program beginning bytes before words
        while(((uint16_t)memPtr_ % 4) && (DataCount))
        {
          *memPtr_++ = *DataPointer++;
          while( (FLASH->IAPSR & (FLASH_IAPSR_EOP | FLASH_IAPSR_WR_PG_DIS)) == 0)
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
          //			memPtr_[0] = DataPointer[0]; /* Write one byte - from lowest memptr_*/
          //			memPtr_[1] = DataPointer[1]; /* Write one byte*/
          //			memPtr_[2] = DataPointer[2]; /* Write one byte*/
          //			memPtr_[3] = DataPointer[3]; /* Write one byte - from higher address*/
          while((FLASH->IAPSR & (FLASH_IAPSR_EOP | FLASH_IAPSR_WR_PG_DIS)) == 0)
            ;
          //			memPtr_ += 4;
          //			DataPointer += 4;
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
          //			memPtr_[0] = DataPointer[0]; /* Write one byte - from lowest memptr_*/
          //			memPtr_[1] = DataPointer[1]; /* Write one byte*/
          //			memPtr_[2] = DataPointer[2]; /* Write one byte*/
          //			memPtr_[3] = DataPointer[3]; /* Write one byte - from higher address*/
          while( (FLASH->IAPSR & (FLASH_IAPSR_EOP | FLASH_IAPSR_WR_PG_DIS)) == 0)
            ;
          //			memPtr_ += 4;
          //			DataPointer += 4;
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
          uint16_t addr = *(uint16_t*)packet_.buf + Traits::FlashStart;
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
          uint16_t addr = (*(uint16_t*)packet_.buf & ~0x8000U) + Traits::EepromStart;
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
      static void Read()
      {
        enum { BUF_OFFSET = 3 };
        if(packet_.n != 1 || packet_.buf[0] > 128) {
          packet_.buf[0] = ERR_PA;
          packet_.n = 1;
          return;
        }
        uint8_t length = packet_.buf[0];
        const uint16_t memEnd = (uint16_t)memPtr_ & 0x8000 ? Traits::FlashEnd : Traits::EepromEnd;
        if(memEnd < (uint16_t)memPtr_ + length) {
          length = memEnd - (uint16_t)memPtr_;
        }
        for(uint8_t i = 0; i < length; ++i) {
          packet_.buf[i + BUF_OFFSET] = *memPtr_++;
        }
        packet_.buf[0] = ERR_NO;
        *(uint16_t*)&packet_.buf[1] = (uint16_t)memPtr_ - (memEnd == Traits::FlashEnd ? Traits::FlashStart : Traits::EepromStart);
        packet_.n = length + BUF_OFFSET;
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
      FORCEINLINE static void Go()
      {
        Deinit();
        //TODO: Need RAM cleanup
        //reset stack pointer (lower byte - because compiler decreases SP with some bytes)
        asm("LDW X,  SP ");
        asm("LD  A,  $FF");
        asm("LD  XL, A  ");
        asm("LDW SP, X  ");
        asm("JPF " UBC_END_ASM);
      }
      FORCEINLINE static void Init()
      {
        using namespace Uarts;
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
          case C_NOP: case C_Err:
            cmd_ = C_NOP;
            continue;
          case C_Echo:
            break;
          case C_GetInfo:
            GetInfo();
            break;
          case C_SetPosition:
            SetPosition();
            break;
          case C_Read:
            Read();
            break;
          case C_Write:
            WriteFlash();
            break;
          case C_Go:
            Go();
            break;
          default:
            packet_.buf[0] = ERR_NI;
            packet_.cmd = C_Err;
            packet_.n = 1;
          }
          Transmit();
        }
      }
    };

    template<McuId DeviceID, Uarts::BaudRate baud, typename DriverEnable>
    WakeData::Packet Bootloader<DeviceID, baud, DriverEnable>::packet_;
    template<McuId DeviceID, Uarts::BaudRate baud, typename DriverEnable>
    Crc::Crc8_NoLUT Bootloader<DeviceID, baud, DriverEnable>::crc_;
    template<McuId DeviceID, Uarts::BaudRate baud, typename DriverEnable>
    uint8_t Bootloader<DeviceID, baud, DriverEnable>::prevByte_;
    template<McuId DeviceID, Uarts::BaudRate baud, typename DriverEnable>
    State Bootloader<DeviceID, baud, DriverEnable>::state_;
    template<McuId DeviceID, Uarts::BaudRate baud, typename DriverEnable>
    uint8_t Bootloader<DeviceID, baud, DriverEnable>::rxBufPtr_;
    template<McuId DeviceID, Uarts::BaudRate baud, typename DriverEnable>
    uint8_t Bootloader<DeviceID, baud, DriverEnable>::cmd_;
    template<McuId DeviceID, Uarts::BaudRate baud, typename DriverEnable>
    uint8_t* Bootloader<DeviceID, baud, DriverEnable>::memPtr_ = (uint8_t*)UBC_END;


  }//Wk
}//Mcudrv
