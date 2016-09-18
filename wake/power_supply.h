#ifndef POWER_SUPPLY_H
#define POWER_SUPPLY_H

#include "wake_base.h"
#include "gpio.h"
#include "adc.h"
#include "hd44780.h"
#include "string_utils.h"
#include "sensors.h"

namespace Mcudrv {
	namespace Wk {

	typedef Pa2 Relay1;
	typedef Pa1 Relay2;

	struct PowersSupplyDefaultFeatures {
		enum {
			PowerRating = 36,
			MaxCurrent = 1580 //mA
		};
	};

	class PowerSupply : WakeData, public NullModule
	{
	private:
		enum InstructionSet {
			C_Voltage,
			C_Current,
			C_Power,
			C_Load,
			C_GetValue = 52,
			C_SetCurrentLim = 53
		};

		typedef Pd3 Isen; //AIN4
		typedef Pd2 Vsen; //AIN3
		typedef Pd4 Ilim; //TIM2 CH1

		typedef Adcs::Adc1 Adc;

		static const Adcs::Channel VsenChannel = (Adcs::Channel)Adcs::PinToCh<Vsen>::value;
		static const Adcs::Channel IsenChannel = (Adcs::Channel)Adcs::PinToCh<Isen>::value;

		//32 samples simple moving average
		enum { MA_SIZE = 1U << 5 };
		static uint16_t rawVoltage[MA_SIZE];
		static uint16_t rawCurrent[MA_SIZE];

		static uint16_t To_mA(uint16_t value)
		{
			value = ((value * 5) / 32);
			return  value > 9 ? value - 10 : 0;
		}
		static uint16_t ToTensOf_mV(uint16_t value)
		{
			return 1850U + ((value * 5) / 64);
		}
		_Pragma(VECTOR_ID(ADC1_EOC_vector))
		__interrupt static void AdcISR()
		{
			Adc::ClearEvent(Adcs::EndOfConv);
			Adc::Disable();
			static uint8_t maIndex;
			uint16_t result = 0;
			for(uint8_t i = 0; i < 10; ++i) {
				result += Adc::buffer[i];
			}
			if(Adc::GetSelectedChannel() == VsenChannel) {
				rawVoltage[maIndex] = result;
				Adc::SelectChannel(IsenChannel);
			}
			else {
				rawCurrent[maIndex] = result;
				Adc::SelectChannel(VsenChannel);
				if(++maIndex == MA_SIZE) {
					maIndex = 0;
				}
			}
		}
	public:
		enum
		{
			deviceMask = DevPowerSupply,
			features = PowersSupplyDefaultFeatures::PowerRating,
			MaxCurrent = PowersSupplyDefaultFeatures::MaxCurrent
		};
		static struct VI
		{
			uint16_t voltage;
			uint16_t current;
		} vi;

		FORCEINLINE
		static void Init()
		{
			{	using namespace Adcs;
				Isen::SetConfig<GpioBase::In_float>();
				Vsen::SetConfig<GpioBase::In_float>();
				Adc::DisableSchmittTrigger<IsenChannel | VsenChannel>();
				Adc::SelectChannel((Channel)VsenChannel);
				Adc::Init<Cfg(ADCEnable | ContMode | BufferEnable), Div2>();
				Adc::EnableInterrupt(EndOfConv);
			}
			{ using namespace T2;
				Ilim::SetConfig<GpioBase::Out_PushPull_fast>();
				Timer2::Init(Div_1, Cfg(ARPE | CEN));
				Timer2::WriteAutoReload(0xFF);			//Fcpu/256 ~= 7800Hz for 2 MHz
				Timer2::WriteCompareByte<Ch1>(0xFF);
				Timer2::SetChannelCfg<Ch1, Output, ChannelCfgOut(Out_PWM_Mode1 | Out_PreloadEnable)>();
				Timer2::ChannelEnable<Ch1>();
			}
		}
		static void Process()
		{
			uint16_t* const bufval = (uint16_t*)&pdata.buf[1];
			switch(cmd) {
			case C_GetValue:
				pdata.buf[0] = ERR_NO;
				if(!pdata.n) {
					pdata.n = 6;
					VIRefresh();
					bufval[0] = GetVoltage();
					bufval[1] = GetCurrent();
					*(uint8_t*)&bufval[2] = GetLoad();
				}
				else if(pdata.n == 1 && pdata.buf[0] < 4) {
					pdata.n = 3;
					VIRefresh();
					switch(pdata.buf[0]) {
					case C_Voltage:
						*bufval = GetVoltage();
						break;
					case C_Current:
						*bufval = GetCurrent();
						break;
					case C_Power:
						*bufval = GetPower();
						break;
					case C_Load:
						pdata.n = 2;
						pdata.buf[1] = GetLoad();
						break;
					}
				}
				else {
					pdata.buf[0] = ERR_PA;
					pdata.n = 1;
				}
				break;
			case C_SetCurrentLim: {
				if(pdata.n == 1) {
					SetCurrentLimit(pdata.buf[0]);
					pdata.buf[0] = ERR_NO;
				}
				else {
					pdata.buf[0] = ERR_PA;
					pdata.n = 1;
				}
			}
				break;
			default:
				processedMask |= deviceMask;
			}
		}
		static uint8_t GetDeviceFeatures(const uint8_t)
		{
			return features;
		}
		static void UpdIRQ()
		{
			Adc::Enable();
			Adc::StartConversion();
		}

		static void VIRefresh()
		{
			uint32_t voltage = 0, current = 0;
			for(uint8_t i = 0; i < MA_SIZE; ++i) {
				voltage += rawVoltage[i];
				current += rawCurrent[i];
			}
			vi.voltage = ToTensOf_mV(voltage >> 5);
			vi.current = To_mA(current >> 5);
		}
		static uint16_t GetVoltage()
		{
			return vi.voltage;
		}
		static uint16_t GetCurrent()
		{
			return vi.current;
		}
		static uint16_t GetPower()
		{
			uint32_t power = (uint32_t)GetVoltage() * GetCurrent();
			return uint16_t(power / 10000);
		}
		static uint8_t GetLoad()
		{
			uint16_t result = (uint32_t)GetCurrent() * 255 / MaxCurrent;
			return result < 0xFF ? result : 0xFF;
		}
		static void SetCurrentLimit(uint8_t limit)
		{
			T2::Timer2::WriteCompareByte<T2::Ch1>(limit);
		}
	};

	uint16_t PowerSupply::rawVoltage[PowerSupply::MA_SIZE];
	uint16_t PowerSupply::rawCurrent[PowerSupply::MA_SIZE];
	PowerSupply::VI PowerSupply::vi;

	class Display : public NullModule
	{
	private:
		typedef Pinlist<Pc3, SequenceOf<4> > LcdDataBus;
		typedef Pd1 LcdRs;
		typedef Pc7 LcdE;
		typedef Hd44780<LcdDataBus, LcdRs, LcdE> Lcd;
		enum LcdPatterns { SymDegree };
		volatile static bool updFlag;
	public:
		static void Init()
		{
			static const uint8_t degreePattern[8] = {0x1C, 0x14, 0x1C, 0};
			Lcd::Init();
			Lcd::BuildCustomChar(0, degreePattern);
		}
		static void UpdIRQ()
		{
			updFlag = true;
		}
		static void Refresh()
		{
			if(updFlag) {
				updFlag = false;
				static uint8_t counter;
				if(++counter & 0x20) { // div32 ~= 2Hz
					counter = 0;
					uint8_t buf[8];
					PowerSupply::VIRefresh();
					Lcd::Clear();
					Lcd::Puts(io::InsertDot(PowerSupply::GetVoltage(), 2, buf));
					Lcd::Putch('V');
					Lcd::SetPosition(10);
					Lcd::Puts(io::InsertDot(PowerSupply::GetCurrent(), 3, buf));
					Lcd::Putch('A');
					Lcd::SetPosition(0, 1);
					Lcd::Puts(io::InsertDot(PowerSupply::GetPower(), 1, buf));
					Lcd::Putch('W');
					Lcd::SetPosition(10, 1);
					Lcd::Puts(io::InsertDot(Tsensor_LM75::Read(), 1, buf));
					Lcd::Putch(SymDegree);
				}
			}
		}

	};

	volatile bool Display::updFlag;

	}//Wk
}//Mcudrv

#endif // POWER_SUPPLY_H
