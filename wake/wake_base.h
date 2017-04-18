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

// Peripherals used: Uart, TIM4 

#pragma once

#include "uart.h"
#include "timers.h"
#include "flash.h"
#include "itc.h"
#include "crc.h"
#include "gpio.h"

#ifndef WAKEDATABUFSIZE
#define WAKEDATABUFSIZE 64
#endif

#ifndef BOOTLOADER_EXIST
#define BOOTLOADER_EXIST 1
#endif

namespace Mcudrv
{
	namespace Wk
	{
//	---=== Operation time counter ===---
		template<typename TCallback>
		class OpTime
		{
		private:
			struct EepromBuf_t
			{
				uint16_t Dummy1;
				uint8_t Dummy2;
				uint8_t lvalue;
			};

			#pragma data_alignment=4
			#pragma location=".eeprom.noinit"
			static EepromBuf_t eebuf[16];// @ ".eeprom.noinit";
			#pragma data_alignment=4
			#pragma location=".eeprom.noinit"
			static uint16_t hvalue;// @ ".eeprom.noinit";
			volatile static bool tenMinPassed;

			_Pragma(VECTOR_ID(TIM4_OVR_UIF_vector))
			__interrupt static void UpdIRQ()	//Fcpu/256/128 ~= 61 Hz for 2 MHz HSI
			{
				static uint16_t tempCounter;
				T4::Timer4::ClearIntFlag();
				if(++tempCounter == (600U * 61))
				{
					tempCounter = 0;
					SetTenMinutesFlag();
				}
				TCallback::UpdIRQ();
			}

		public:
			#pragma inline=forced
			static void Init()
			{
				using namespace T4;
				Itc::SetPriority(TIM4_OVR_UIF_vector, Itc::prioLevel_2_middle);
				Timer4::Init(Div_128, CEN);
				Timer4::EnableInterrupt();
			}
			#pragma inline=forced
			static bool GetTenMinitesFlag()
			{
				return tenMinPassed;
			}
			#pragma inline=forced
			static void SetTenMinutesFlag()
			{
				tenMinPassed = true;
			}
			#pragma inline=forced
			static void ClearTenMinutesFlag()
			{
				tenMinPassed = false;
			}
			static uint8_t GetIndex()
			{
				uint8_t i;
				for (i = 0; i < 15; i++)
				{
					if (eebuf[i + 1].lvalue != eebuf[i].lvalue + 1) break;
				}
				return i;
			}
			static void Get(volatile uint8_t *arr)
			{
				uint16_t temp = hvalue;
				arr[0] = eebuf[GetIndex()].lvalue;
				arr[1] = temp & 0xFF;
				arr[2] = temp >> 8UL;
			}

			#pragma inline=forced
			static void CountInc()
			{
				using namespace Mem;
				uint8_t i = GetIndex();
				uint8_t tmp = eebuf[i].lvalue + 1;
				Unlock<Eeprom>();
				if(IsUnlocked<Eeprom>())
				{
					if (i != 15) eebuf[i + 1].lvalue = tmp;
					else eebuf[0].lvalue = tmp;
					if (tmp == 0) ++hvalue;
				}
				Lock<Eeprom>();
			}
		};

		template<typename TCallback>
		OpTime<TCallback>::EepromBuf_t OpTime<TCallback>::eebuf[16];
		template<typename TCallback>
		uint16_t OpTime<TCallback>::hvalue;
		template<typename TCallback>
		volatile bool OpTime<TCallback>::tenMinPassed;

	//			---=== Wake main definitions ===---

		enum
		{
			DefaultADDR = 127,
			DefaultGroupADDR = 95,
			REBOOT_KEY = 0xCB47ED91U,		// Host should use big endian format
			CRC_INIT = 0xDE,
			FEND = 0xC0,    //Frame END
			FESC = 0xDB,    //Frame ESCape
			TFEND = 0xDC,    //Transposed Frame END
			TFESC = 0xDD    //Transposed Frame ESCape
		};

		enum Mode
		{
			Master,
			Slave
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

		enum Cmd
		{
			C_NOP,    //нет операции
			C_ERR,    //ошибка приема пакета
			C_ECHO,    //передать эхо
			C_GETINFO,
			C_SETNODEADDRESS,
			C_GETGROUPADDRESS,
			C_SETGROUPADDRESS = C_GETGROUPADDRESS,
			C_GETOPTIME,
			C_OFF,
			C_ON,
			C_ToggleOnOff,
			C_SAVESETTINGS,
#if BOOTLOADER_EXIST
			C_REBOOT,
#endif
			C_BASE_END
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

		enum DeviceType
		{
			DevNull,
			DevLedDriver = 0x01,
			DevSwitch = 0x02,
			DevRgbDriver = 0x04,
			DevGenericIO = 0x08,
			DevSensor = 0x10,
			DevPowerSupply = 0x20,
//		Reserved = 0x40,
			DevCustom = 0x80
		};

		enum CustomDeviceID {
			CustomID_Themostat = 0x01
		};

		enum AddrType
		{
			addrGroup,
			addrNode
		};

		//Every module should implement the same interface
		struct NullModule
		{
			enum { deviceMask = DevNull, features = 0 };
			static void Init() { }
			static void Process() { }
			static void SaveState() { }
			static void On() { }
			static void Off() { }
			static uint8_t GetDeviceFeatures(uint8_t) {return 0;}
			static void ToggleOnOff() { }
			static void UpdIRQ() { }
		};

		template<typename Module1, typename Module2 = NullModule, typename Module3 = NullModule,
				 typename Module4 = NullModule, typename Module5 = NullModule, typename Module6 = NullModule>
		struct ModuleList
		{
			enum { deviceMask = (uint8_t)Module1::deviceMask | Module2::deviceMask | Module3::deviceMask |
								Module4::deviceMask | Module5::deviceMask | Module6::deviceMask };
			static void Init()
			{
				Module1::Init();
				Module2::Init();
				Module3::Init();
				Module4::Init();
				Module5::Init();
				Module6::Init();
			}
			static void Process()
			{
				Module1::Process();
				Module2::Process();
				Module3::Process();
				Module4::Process();
				Module5::Process();
				Module6::Process();
			}
			static uint8_t GetDeviceFeatures(uint8_t deviceMask)
			{
				return deviceMask & Module1::deviceMask ? Module1::GetDeviceFeatures(deviceMask) :
						deviceMask & Module2::deviceMask ? Module2::GetDeviceFeatures(deviceMask) :
						deviceMask & Module3::deviceMask ? Module3::GetDeviceFeatures(deviceMask) :
						deviceMask & Module4::deviceMask ? Module4::GetDeviceFeatures(deviceMask) :
						deviceMask & Module5::deviceMask ? Module5::GetDeviceFeatures(deviceMask) :
						deviceMask & Module6::deviceMask ? Module6::GetDeviceFeatures(deviceMask) : 0;
			}
			static void SaveState()		//module should be save only if settings changed
			{
				Module1::SaveState();
				Module2::SaveState();
				Module3::SaveState();
				Module4::SaveState();
				Module5::SaveState();
				Module6::SaveState();
			}
			static void On()
			{
				Module1::On();
				Module2::On();
				Module3::On();
				Module4::On();
				Module5::On();
				Module6::On();
			}
			static void Off()
			{
				Module1::Off();
				Module2::Off();
				Module3::Off();
				Module4::Off();
				Module5::Off();
				Module6::Off();
			}
			static void ToggleOnOff()
			{
				Module1::ToggleOnOff();
				Module2::ToggleOnOff();
				Module3::ToggleOnOff();
				Module4::ToggleOnOff();
				Module5::ToggleOnOff();
				Module6::ToggleOnOff();
			}
			static void UpdIRQ()
			{
				Module1::UpdIRQ();
				Module2::UpdIRQ();
				Module3::UpdIRQ();
				Module4::UpdIRQ();
				Module5::UpdIRQ();
				Module6::UpdIRQ();
			}
		};

		class WakeData
		{
		public:
			struct Packet
			{
				uint8_t addr;
				uint8_t cmd;
				uint8_t n;
				uint8_t buf[WAKEDATABUFSIZE];
			};
        protected:
			static volatile Packet pdata;
			static volatile uint8_t cmd;
			static uint8_t processedMask;		//used to check if command processed in modules
		};
		volatile WakeData::Packet WakeData::pdata;
		volatile uint8_t WakeData::cmd;
		uint8_t WakeData::processedMask;

		template<typename moduleList = ModuleList<NullModule>,
				 Uarts::BaudRate baud = 9600UL,
				 typename DriverEnable = Pd6,
				 Mode mode = Slave>	//TODO: Master mode
		class Wake : WakeData
		{
		private:
			typedef Uarts::Uart Uart;
			typedef OpTime<moduleList> OpTime;
			#pragma location=".eeprom.noinit"
			static volatile uint8_t nodeAddr_nv;// @ ".eeprom.data";
			#pragma location=".eeprom.noinit"
			static volatile uint8_t groupAddr_nv;// @ ".eeprom.data";
			static volatile uint8_t prev_byte;
			static volatile State state;				//Current tranfer mode
			static volatile uint8_t ptr;				//data pointer in Rx buffer
			static Crc::Crc8 crc;

			static void SetAddress(const AddrType nodeOrGroup) //and get address
			{
				if(pdata.n == 2 && pdata.addr)	//data length correct and no broadcast
				{
					if(nodeOrGroup ? CheckNodeAddress() : CheckGroupAddress())
					{
						using namespace Mem;
						uint8_t tempAddr = pdata.buf[0];
						pdata.buf[0] = ERR_NO;
						pdata.buf[1] = tempAddr;
						if(tempAddr != (nodeOrGroup ? nodeAddr_nv : groupAddr_nv)) //no write if address equal
						{
							Unlock(Eeprom);
							if (IsUnlocked(Eeprom))
							{
								if(nodeOrGroup) nodeAddr_nv = tempAddr;
								else groupAddr_nv = tempAddr;
							}
							else pdata.buf[0] = ERR_EEPROMUNLOCK;
							Lock(Eeprom);
						}
					}
					else
					{
						pdata.buf[0] = ERR_ADDRFMT;
					}
				}
				else if(!pdata.n) //Get address
				{
					pdata.buf[0] = groupAddr_nv;
					pdata.n = 1;
				}
				else
				{
					pdata.buf[0] = ERR_PA;
					pdata.n = 2;
				}
				if(pdata.buf[0]) pdata.buf[1] = 0;
			}
			#pragma inline=forced
			static bool CheckNodeAddress()
			{
				uint8_t taddr = pdata.buf[0];
				return taddr == (~pdata.buf[1] & 0xFF)
						&& ((taddr && taddr < 80) || (taddr > 112 && taddr < 128));
			}
			#pragma inline=forced
			static bool CheckGroupAddress()
			{
				uint8_t taddr = pdata.buf[0];
				return taddr == (~pdata.buf[1] & 0xFF)
						&& taddr > 79 && taddr < 96;
			}
		public:
			#pragma inline=forced
			static void Init()
			{
				using namespace Uarts;
		//Single Wire mode is default for UART1
				enum { SingleWireMode = (Uart::BaseAddr == UART1_BaseAddress ? Uarts::SingleWireMode : 0) };
				Uart::template Init<Cfg(Uarts::DefaultCfg | (Cfg)SingleWireMode), baud>();
				DriverEnable::template SetConfig<GpioBase::Out_PushPull_fast>();
				DriverEnable::Clear();
		//validate node address saved in eeprom
				if(!nodeAddr_nv || nodeAddr_nv > 127) {
					Mem::Unlock(Mem::Eeprom);
					if(Mem::IsUnlocked(Mem::Eeprom)) {
						nodeAddr_nv = 127;
						Mem::Lock(Mem::Eeprom);
					}
				}
				moduleList::Init();
				OpTime::Init();
				Wdg::Iwdg::Enable(Wdg::P_1s);
				Uart::EnableInterrupt(IrqDefault);
			}
			#pragma inline=forced
			static void Process()
			{
				Wdg::Iwdg::Refresh();
				if(OpTime::GetTenMinitesFlag() && !IsActive()) {
					OpTime::ClearTenMinutesFlag();
					OpTime::CountInc();			//Refresh optime counter every 10 mins
					moduleList::SaveState();		//Save to EEPROM
				}
				if(cmd) {
					switch(cmd) {
					case C_NOP: case C_ECHO:
						break;
					case C_ERR:
						cmd = Wk::C_NOP;
						return;
					case C_GETINFO:
						//Common device info
						if (!pdata.n)	{
							pdata.buf[0] = ERR_NO;
							pdata.buf[1] = moduleList::deviceMask;
							pdata.buf[2] = INSTRUCTION_SET_VER_MAJOR << 4 | INSTRUCTION_SET_VER_MINOR;
							pdata.n = 3;
						}
						//Info about single logical device
						else if(pdata.n == 1)	{
							if(pdata.buf[0] < 7) {
								const uint8_t deviceMask = 1 << pdata.buf[0];
								//Device is available
								if(moduleList::deviceMask & deviceMask) {
									pdata.buf[0] = ERR_NO;
									pdata.buf[1] = moduleList::GetDeviceFeatures(deviceMask);
									pdata.n = 2;
								}
								//device not available
								else {
									pdata.buf[0] = ERR_NI;
								}
							}
							//else if(pdata.buf[0] == 7) //custom device
						}
						else {
							pdata.buf[0] = ERR_PA;
							pdata.n = 1;
						}
						break;
					case C_SETNODEADDRESS:
						SetAddress(addrNode);
						break;
					case C_SETGROUPADDRESS:
						if(!pdata.n) {
							pdata.n = 2;
							pdata.buf[0] = ERR_NO;
							pdata.buf[1] = groupAddr_nv;
						}
						else {
							SetAddress(addrGroup);
						}
						break;
					case C_GETOPTIME:
						if(!pdata.n) {
							pdata.buf[0] = Wk::ERR_NO;
							OpTime::Get(&pdata.buf[1]);
							pdata.n = 4;
						}
						else {
							pdata.buf[0] = Wk::ERR_PA;
							pdata.n = 1;
						}
						break;
					case C_OFF:
						if(!pdata.n) {
							pdata.buf[0] = Wk::ERR_NO;
							moduleList::Off();
						}
						else {
							pdata.buf[0] = Wk::ERR_PA;
						}
						pdata.n = 1;
						break;
					case C_ON:
						if(!pdata.n) {
							pdata.buf[0] = Wk::ERR_NO;
							moduleList::On();
						}
						else {
							pdata.buf[0] = Wk::ERR_PA;
						}
						pdata.n = 1;
						break;
					case C_ToggleOnOff:
						if(!pdata.n) {
							pdata.buf[0] = Wk::ERR_NO;
							moduleList::ToggleOnOff();
						}
						else {
							pdata.buf[0] = Wk::ERR_PA;
						}
						pdata.n = 1;
						break;
					case C_SAVESETTINGS:
						if(!pdata.n) {
							pdata.buf[0] = ERR_NO;
							moduleList::SaveState();
						}
						else pdata.buf[0] = ERR_PA;
						pdata.n = 1;
						break;
#if BOOTLOADER_EXIST
					case C_REBOOT:
						if(pdata.n == 4 && *(uint32_t*)pdata.buf == REBOOT_KEY) {
							System::Reset();
						}
						else {
							pdata.buf[0] = Wk::ERR_PA;
						}
						pdata.n = 1;
						break;
#else
					case C_REBOOT:
						pdata.buf[0] = Wk::ERR_NI;
						pdata.n = 1;
						break;
#endif
					default:
						moduleList::Process();
						//Check if command not processed in modules
						if(processedMask == moduleList::deviceMask) {
							processedMask = 0;
							pdata.buf[0] = Wk::ERR_NI;
							pdata.n = 1;
						}
					} //Switch
					uint8_t tempAddr = pdata.addr;
					if(tempAddr == nodeAddr_nv || tempAddr && cmd == C_SETNODEADDRESS) {
						Send();
					}
					cmd = Wk::C_NOP;
				}
			}

			#pragma inline=forced
			static bool IsActive()
			{
				return state != WAIT_FEND;
			}

			static void Send()
			{
				using namespace Uarts;
				DriverEnable::Set(); //Switch to TX
				char data_byte = FEND;
				crc.Reset(CRC_INIT); //инициализация CRC,
				crc(data_byte);		//обновление CRC
				Uart::Regs()->DR = data_byte;
				state = ADDR;
				prev_byte = TFEND;
				Uart::DisableInterrupt(IrqRxne);
				Uart::EnableInterrupt(IrqTxEmpty);
			}

#if defined (STM8S103) || defined (STM8S003)
			_Pragma(VECTOR_ID(UART1_T_TXE_vector))
#elif defined (STM8S105)
			_Pragma(VECTOR_ID(UART2_T_TXE_vector))
#endif
			__interrupt static void TxISR()
			{
				using namespace Uarts;
				if(Uart::IsEvent(EvTxComplete)) {
					Uart::ClearEvent(EvTxComplete);
					Uart::ClearEvent(EvRxne);
					Uart::EnableInterrupt(IrqRxne);
					DriverEnable::Clear();		//Switch to RX
				}
				else { //if(Uart::IsEvent(Uarts::TxEmpty))
					uint8_t data_byte;
					if(prev_byte == FEND) {
						data_byte = TFEND;                //передача TFEND вместо FEND
						prev_byte = data_byte;
						Uart::Regs()->DR = data_byte;
						return;
					}
					if(prev_byte == FESC) {
						data_byte = TFESC;                //передача TFESC вместо FESC
						prev_byte = data_byte;
						Uart::Regs()->DR = data_byte;
						return;
					}
					switch(state) {
					case ADDR:                     //-----> передача адреса	
						state = CMD;
						if(pdata.addr) {
							data_byte = nodeAddr_nv | 0x80; //то он передается (бит 7 равен единице)
							break;
						}
						//иначе сразу передаем команду
					case CMD:
						data_byte = pdata.cmd & 0x7F;
						state = NBT;
						break;
					case NBT:                      //-----> передача количества байт
						data_byte = pdata.n;
						state = DATA;
						ptr = 0;                  //обнуление указателя данных для передачи
						break;
					case DATA: {                     //-----> передача данных
						uint8_t ptr_ = ptr;
						if(ptr_ < pdata.n) {
							data_byte = pdata.buf[ptr++];
						}
						else {
							data_byte = crc.GetResult();        //передача CRC
							state = CRC;
						}
						break;
					}
					default:
							Uart::DisableInterrupt(IrqTxEmpty);
							state = SEND_IDLE;          //передача пакета завершена
							return;
					}
					crc(data_byte);     //обновление CRC
					prev_byte = data_byte;              //сохранение пре-байта
					if(data_byte == FEND)// || data_byte == FESC)
						data_byte = FESC;                 //передача FESC, если нужен стаффинг
					Uart::Regs()->DR = data_byte;
				}
			}

#if defined (STM8S103) || defined (STM8S003)
			_Pragma(VECTOR_ID(UART1_R_RXNE_vector))
#elif defined (STM8S105)
			_Pragma(VECTOR_ID(UART2_R_RXNE_vector))
#endif
			__interrupt static void RxISR()
			{
				using namespace Uarts;
				bool error = Uart::IsEvent(static_cast<Events>(EvParityErr | EvFrameErr | EvNoiseErr | EvOverrunErr));
				uint8_t data_byte = Uart::Regs()->DR;
				if(error) {
					state = WAIT_FEND;		//ожидание нового пакета
					cmd = C_ERR;			//рапортуем об ошибке
					return;
				}
				if(data_byte == FEND) {
					prev_byte = data_byte;
					crc.Reset(CRC_INIT);
					state = ADDR;
					crc(data_byte);
					return;
				}
				if(state == WAIT_FEND) {
					return;
				}
				char Pre = prev_byte;               //сохранение старого пре-байта
				prev_byte = data_byte;              //обновление пре-байта
				if(Pre == FESC) {
					if(data_byte == TFESC)            //а байт данных равен TFESC,
						data_byte = FESC;               //то заменить его на FESC
					else if(data_byte == TFEND)       //если байт данных равен TFEND,
						data_byte = FEND;          //то заменить его на FEND
					else {
						state = WAIT_FEND;     //для всех других значений байта данных,
						cmd = C_ERR;         //следующего за FESC, ошибка
						return;
					}
				}
				else {
					if(data_byte == FESC)             //если байт данных равен FESC, он просто
						return;                         //запоминается в пре-байте
				}
				switch(state) {
				case ADDR:                     //-----> ожидание приема адреса
					if(data_byte & 0x80) {
						crc(data_byte); //то обновление CRC и
						data_byte &= 0x7F; //обнуляем бит 7, получаем истинный адрес
						if(data_byte == 0 || data_byte == nodeAddr_nv || data_byte == groupAddr_nv) {
							pdata.addr = data_byte;
							state = CMD;       //переходим к приему команды
							break;
						}
						state = WAIT_FEND;        //адрес не совпал, ожидание нового пакета
						break;
					}
					else pdata.addr = 0;	//если бит 7 данных равен нулю, то
					state = CMD;					//сразу переходим к приему команды
				case CMD:                      //-----> ожидание приема команды
					if(data_byte & 0x80) {
						state = WAIT_FEND;        //если бит 7 не равен нулю,
						cmd = C_ERR;            //то ошибка
						break;
					}
					pdata.cmd = data_byte;          //сохранение команды
					crc(data_byte);				//обновление CRC
					state = NBT;           //переходим к приему количества байт
					break;
				case NBT:
					if(data_byte > WAKEDATABUFSIZE) {
						state = WAIT_FEND;
						cmd = C_ERR;		//то ошибка
						break;
					}
					pdata.n = data_byte;
					crc(data_byte);		//обновление CRC
					ptr = 0;			//обнуляем указатель данных
					state = DATA;		//переходим к приему данных
					break;
				case DATA:
					uint8_t ptr_ = ptr;
					if(ptr_ < pdata.n) {
						pdata.buf[ptr++] = data_byte; //то сохранение байта данных,
						crc(data_byte);  //обновление CRC
						break;
					}
					if(data_byte != crc.GetResult()) {
						state = WAIT_FEND;		//если CRC не совпадает,
						cmd = C_ERR;			//то ошибка
						break;
					}
					state = WAIT_FEND;		//прием пакета завершен,
					cmd = pdata.cmd;		//загрузка команды на выполнение
					break;
				}
			}
		};

		template<typename moduleList,
				 Uarts::BaudRate baud,
				 typename DEpin,
				 Mode mode>
		volatile uint8_t Wake<moduleList, baud, DEpin, mode>::nodeAddr_nv;// = 127;
		template<typename moduleList,
				 Uarts::BaudRate baud,
				 typename DEpin,
				 Mode mode>
		volatile uint8_t Wake<moduleList, baud, DEpin, mode>::groupAddr_nv;// = 95;
		template<typename moduleList,
				 Uarts::BaudRate baud,
				 typename DEpin,
				 Mode mode>
		volatile uint8_t Wake<moduleList, baud, DEpin, mode>::prev_byte;
		template<typename moduleList,
				 Uarts::BaudRate baud,
				 typename DEpin,
				 Mode mode>
		volatile State Wake<moduleList, baud, DEpin, mode>::state;
		template<typename moduleList,
				 Uarts::BaudRate baud,
				 typename DEpin,
				 Mode mode>
		volatile uint8_t Wake<moduleList, baud, DEpin, mode>::ptr;
		template<typename moduleList,
				 Uarts::BaudRate baud,
				 typename DEpin,
				 Mode mode>
		Crc::Crc8 Wake<moduleList, baud, DEpin, mode>::crc;
    }//Wk
}//Mcudrv
