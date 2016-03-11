//
//	Uart and TIM4 peripherals used
//
#pragma once

#include "timers.h"
#include "uart.h"
#include "flash.h"
#include "itc.h"
#include "crc.h"
#include "gpio.h"

#define INSTRUCTION_SET_VERSION 2

#ifndef WAKEDATABUFSIZE
#define WAKEDATABUFSIZE 64
#endif

namespace Mcudrv
{
	namespace Wk
	{

//	---=== Operation time counter ===---
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
				if(Timer_cb) Timer_cb();
			}

		public:
			static void (*volatile Timer_cb)();
			#pragma inline=forced
			static void Init()
			{
				using namespace T4;
				Itc::SetPriority(TIM4_OVR_UIF_vector, Itc::prioLevel_2_middle);
				Timer4::Init(Div_128, CEN);
				Timer4::EnableInterrupt();
			}
			#pragma inline=forced
			static void SetTimerCallback(void (*t_cb)())
			{
				Timer_cb = t_cb;
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

		OpTime::EepromBuf_t OpTime::eebuf[16];
		uint16_t OpTime::hvalue;
		volatile bool OpTime::tenMinPassed;
		void (*volatile OpTime::Timer_cb)();

	//			---=== Wake main definitions ===---

		enum
		{
			DefaultADDR = 127,
			DefaultGroupADDR = 95,
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
			C_SETGROUPADDRESS,
			C_SAVESETTINGS,
			C_GETOPTIME,
			C_OFF,
			C_ON,
			C_ToggleOnOff
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
			ERR_EEPROMUNLOCK //EEPROM hasn't unlocked
		};

		enum DeviceType
		{
			devNull,
			devLedDriver = 0x01,
			devSwitch = 0x02,
			devRgbDriver = 0x04,
			devGenericIO = 0x08,
			devSensor = 0x10,
			devPowerSupply = 0x20,
//			Reserved = 0x40,
			devCustom = 0x80
		};

		enum AddrType
		{
			addrGroup,
			addrNode
		};

		struct NullModule
		{
			enum { deviceMask = devNull, features = 0 };
			static void Init() { }
			static void Process() { }
			static void SaveState() { }
			static void On() { }
			static void Off() { }
			static uint8_t GetDeviceFeatures(uint8_t) {return 0;}
			static void ToggleOnOff() { }
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
			static uint8_t GetDeviceFeatures(const uint8_t deviceMask)
			{
				return deviceMask == Module1::deviceMask ? Module1::features :
						deviceMask == Module2::deviceMask ? Module2::features :
						deviceMask == Module3::deviceMask ? Module3::features :
						deviceMask == Module4::deviceMask ? Module4::features :
						deviceMask == Module5::deviceMask ? Module5::features :
						deviceMask == Module6::deviceMask ? Module6::features : 0;
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
		};

		class WakeData
		{
		protected:
			struct Packet
			{
				uint8_t addr;
				uint8_t cmd;
				uint8_t n;
				uint8_t buf[WAKEDATABUFSIZE];
			};
			static volatile Packet pdata;
			static volatile uint8_t cmd;
		};
		volatile WakeData::Packet WakeData::pdata;
		volatile uint8_t WakeData::cmd;

		template<typename moduleList = ModuleList<NullModule>,
				 Uarts::BaudRate baud = 9600UL,
				 typename DriverEnable = Pd6,
				 Mode mode = Slave>	//TODO: Master mode
		class Wake : WakeData
		{
		private:
			typedef Uarts::Uart Uart;
			enum { SingleWireMode = (Uart::BaseAddr == UART1_BaseAddress ? Uarts::SingleWireMode : 0) };
			#pragma location=".eeprom.data"
			static volatile uint8_t nodeAddr_nv;// @ ".eeprom.data";
			#pragma location=".eeprom.data"
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
							Unlock<Eeprom>();
							if (IsUnlocked<Eeprom>())
							{
								if(nodeOrGroup) nodeAddr_nv = tempAddr;
								else groupAddr_nv = tempAddr;
							}
							else pdata.buf[0] = ERR_EEPROMUNLOCK;
							Lock<Eeprom>();
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
						&& ((taddr && taddr < 80) || (taddr > 111 && taddr < 128));
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
				Uart::template Init<Cfg(Uarts::DefaultCfg | (Cfg)SingleWireMode), baud>();
				DriverEnable::template SetConfig<GpioBase::Out_PushPull_fast>();
				DriverEnable::Clear();
				moduleList::Init();
				OpTime::Init();
				Wdg::Iwdg::Enable(Wdg::P_1s);
				Uart::EnableInterrupt(IrqDefault);
			}
			#pragma inline=forced
			static void Process()
			{
				Wdg::Iwdg::Refresh();
				if(OpTime::GetTenMinitesFlag() && !IsActive())
				{
					OpTime::ClearTenMinutesFlag();
					OpTime::CountInc();			//Refresh Uptime counter every 10 mins
					moduleList::SaveState();		//Save to EEPROM
				}
				if(cmd)
				{
				switch(cmd)
				{
					case C_NOP: case C_ERR: case C_ECHO:
						break;
					case C_GETINFO:
					{
						if (!pdata.n)	//Common device info
						{
							pdata.buf[0] = moduleList::deviceMask;
							pdata.buf[1] = INSTRUCTION_SET_VERSION;
							pdata.n = 2;
						}
						else if(pdata.n == 1)	//Info for each logical device
						{
							if(pdata.buf[0] < 7)
							{
								const uint8_t deviceMask = 1 << pdata.buf[0];
								if(moduleList::deviceMask & deviceMask) //device available
								{
									pdata.buf[0] = ERR_NO;
									pdata.buf[1] = moduleList::GetDeviceFeatures(deviceMask);
									pdata.n = 2;
								}
								else //device not available
								{
									pdata.buf[0] = ERR_NI;
								}
							}
							//else if(pdata.buf[0] == 7) //custom device
						}
						else
						{
							pdata.buf[0] = ERR_PA;
							pdata.n = 1;
							break;
						}
					}
						break;
					case C_SETNODEADDRESS: SetAddress(addrNode);
						break;
					case C_SETGROUPADDRESS: SetAddress(addrGroup);
						break;
					case C_SAVESETTINGS:
					{
						if(!pdata.n)
						{
							moduleList::SaveState();
							pdata.buf[0] = ERR_NO;
						}
						else pdata.buf[0] = ERR_PA;
						pdata.n = 1;
					}
						break;
					case C_GETOPTIME:
					{
						if(!pdata.n)
						{
							pdata.buf[0] = Wk::ERR_NO;
							OpTime::Get(&pdata.buf[1]);
							pdata.n = 4;
						}
						else
						{
							pdata.buf[0] = Wk::ERR_PA;
							pdata.n = 1;
						}
					}
						break;
					case C_OFF:
					{
						if (!pdata.n)
						{
							pdata.buf[0] = Wk::ERR_NO;
							moduleList::Off();
						}
						else
						{
							pdata.buf[0] = Wk::ERR_PA;
						}
						pdata.n = 1;
					}
						break;
					case C_ON:
					{
						if (!pdata.n)
						{
							pdata.buf[0] = Wk::ERR_NO;
							moduleList::On();
						}
						else
						{
							pdata.buf[0] = Wk::ERR_PA;
						}
						pdata.n = 1;
					}
						break;
					case C_ToggleOnOff:
					{
						if(!pdata.n)
						{
							pdata.buf[0] = Wk::ERR_NO;
							moduleList::ToggleOnOff();
						}
						else
						{
							pdata.buf[0] = Wk::ERR_PA;
						}
						pdata.n = 1;
					}
						break;
					default: moduleList::Process();
				}
					uint8_t taddr = pdata.addr;
					if(taddr && cmd == C_SETNODEADDRESS || taddr == nodeAddr_nv)
					{
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
				if(Uart::IsEvent(EvTxComplete))
				{

					Uart::ClearEvent(EvTxComplete);
					Uart::ClearEvent(EvRxne);
					Uart::EnableInterrupt(IrqRxne);
					DriverEnable::Clear();		//Switch to RX
				}
				else //if(Uart::IsEvent(Uarts::TxEmpty))
				{
					char data_byte;
					if(prev_byte == FEND)               //если производится стаффинг,
					{
						data_byte = TFEND;                //передача TFEND вместо FEND
						prev_byte = data_byte;
						Uart::Regs()->DR = data_byte;
						return;
					}
					if(prev_byte == FESC)               //если производится стаффинг,
					{
						data_byte = TFESC;                //передача TFESC вместо FESC
						prev_byte = data_byte;
						Uart::Regs()->DR = data_byte;
						return;
					}
					switch(state)
					{
					case ADDR:                     //-----> передача адреса	
						{
							state = CMD;
							if(pdata.addr)                   //если адрес не равен нулю,
							{
								data_byte = nodeAddr_nv; //то он передается (бит 7 равен единице)
								break;
							}
								//иначе сразу передаем команду
						}
					case CMD:                      //-----> передача команды
						{
							data_byte = pdata.cmd & 0x7F;
							state = NBT;
							break;
						}
					case NBT:                      //-----> передача количества байт
						{
							data_byte = pdata.n;
							state = DATA;
							ptr = 0;                  //обнуление указателя данных для передачи
							break;
						}
					case DATA:                     //-----> передача данных
						{
							uint8_t ptr_ = ptr;
							if(ptr_ < pdata.n)
								data_byte = pdata.buf[ptr++];
							else
							{
								data_byte = crc.Get();        //передача CRC
								state = CRC;
							}
							break;
						}
					default:
						{
							Uart::DisableInterrupt(IrqTxEmpty);
							state = SEND_IDLE;          //передача пакета завершена
							return;
						}
					}
					crc(data_byte);     //обновление CRC
					if (state == CMD)			//Метка адреса
						data_byte |= 0x80;
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
				bool error = Uart::IsEvent(static_cast<Events>(EvParityErr | EvFrameErr | EvNoiseErr | EvOverrunErr)); //чтение флагов ошибок
				uint8_t data_byte = Uart::Regs()->DR;              //чтение данных

				if(error)					//если обнаружены ошибки при приеме байта
				{	
					state = WAIT_FEND;		//ожидание нового пакета
					cmd = C_ERR;			//рапортуем об ошибке
					return;
				}

				if(data_byte == FEND)		//если обнаружено начало фрейма,
				{
					prev_byte = data_byte;	//то сохранение пре-байта,
					crc.Reset(CRC_INIT);	//инициализация CRC,
					state = ADDR;			//сброс указателя данных,
					crc(data_byte);			//обновление CRC,
					return;
				}

				if(state == WAIT_FEND)          //-----> если ожидание FEND,
				{
					return;				//то выход
				}

				char Pre = prev_byte;               //сохранение старого пре-байта
				prev_byte = data_byte;              //обновление пре-байта
				
				if(Pre == FESC)                     //если пре-байт равен FESC,
				{
					if(data_byte == TFESC)            //а байт данных равен TFESC,
						data_byte = FESC;               //то заменить его на FESC
					else if(data_byte == TFEND)       //если байт данных равен TFEND,
						data_byte = FEND;          //то заменить его на FEND
					else
					{
						state = WAIT_FEND;     //для всех других значений байта данных,
						cmd = C_ERR;         //следующего за FESC, ошибка
						return;
					}
				}
				else
				{
					if(data_byte == FESC)             //если байт данных равен FESC, он просто
						return;                         //запоминается в пре-байте
				}

				switch(state)
				{
				case ADDR:                     //-----> ожидание приема адреса
					{
						if(data_byte & 0x80)            //если бит 7 данных не равен нулю, то это адрес
						{
							data_byte = data_byte & 0x7F; //обнуляем бит 7, получаем истинный адрес
							if(data_byte == 0 || data_byte == nodeAddr_nv || data_byte == groupAddr_nv) //если нулевой или верный адрес,
							{
								crc(data_byte); //то обновление CRC и
								pdata.addr = data_byte;
								state = CMD;       //переходим к приему команды
								break;
							}
							state = WAIT_FEND;        //адрес не совпал, ожидание нового пакета
							break;
						}
						else pdata.addr = 0;	//если бит 7 данных равен нулю, то
						state = CMD;					//сразу переходим к приему команды
					}
				case CMD:                      //-----> ожидание приема команды
					{
						if(data_byte & 0x80)            //проверка бита 7 данных
						{
							state = WAIT_FEND;        //если бит 7 не равен нулю,
							cmd = C_ERR;            //то ошибка
							break;
						}
						pdata.cmd = data_byte;          //сохранение команды
						crc(data_byte);				//обновление CRC
						state = NBT;           //переходим к приему количества байт
						break;
					}
				case NBT:					//-----> ожидание приема количества байт
					{
						if(data_byte >= WAKEDATABUFSIZE)	//если количество байт > bufsize,
						{
							state = WAIT_FEND;
							cmd = C_ERR;		//то ошибка
							break;
						}
						pdata.n = data_byte;
						crc(data_byte);		//обновление CRC
						ptr = 0;			//обнуляем указатель данных
						state = DATA;		//переходим к приему данных
						break;
					}
				case DATA:                     //-----> ожидание приема данных
					{
						uint8_t ptr_ = ptr;
						if(ptr_ < pdata.n)       //если не все данные приняты,
						{
							pdata.buf[ptr++] = data_byte; //то сохранение байта данных,
							crc(data_byte);  //обновление CRC
							break;
						}
						if(data_byte != crc.Get())      //если приняты все данные, то проверка CRC
						{
							state = WAIT_FEND;		//если CRC не совпадает,
							cmd = C_ERR;			//то ошибка
							break;
						}
						state = WAIT_FEND;		//прием пакета завершен,
						cmd = pdata.cmd;		//загрузка команды на выполнение
						break;
					}
				}

			}
		};

		template<typename moduleList,
				 Uarts::BaudRate baud,
				 typename DEpin,
				 Mode mode>
		volatile uint8_t Wake<moduleList, baud, DEpin, mode>::nodeAddr_nv = 127;
		template<typename moduleList,
				 Uarts::BaudRate baud,
				 typename DEpin,
				 Mode mode>
		volatile uint8_t Wake<moduleList, baud, DEpin, mode>::groupAddr_nv = 95;
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
	}
}
