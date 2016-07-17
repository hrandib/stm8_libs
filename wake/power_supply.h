#ifndef POWER_SUPPLY_H
#define POWER_SUPPLY_H

#include "wake_base.h"
#include "gpio.h"
#include "adc.h"
#include "hd44780.h"
#include "i2c.h"
#include "string_utils.h"

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

		typedef Pinlist<Pc3, SequenceOf<4> > LcdDataBus;
		typedef Pd1 LcdRs;
		typedef Pc7 LcdE;
		typedef Hd44780<LcdDataBus, LcdRs, LcdE> Lcd;

		typedef Adcs::Adc1 Adc;

		typedef Twis::SoftTwi<Twis::Standard, Pb4, Pb5> Twi;
		typedef Twis::Lm75<Twi> Tsense;

		static const Adcs::Channel VsenChannel = (Adcs::Channel)Adcs::PinToCh<Vsen>::value;
		static const Adcs::Channel IsenChannel = (Adcs::Channel)Adcs::PinToCh<Isen>::value;
		static uint16_t rawVoltage;
		static uint16_t rawCurrent;
		volatile static bool updFlag;
		FORCEINLINE
		static uint16_t To_mA(uint16_t value)
		{
			value = ((value * 5) / 32);
			return  value > 9 ? value - 10 : 0;
		}
		FORCEINLINE
		static uint16_t ToTensOf_mV(uint16_t value)
		{
			return 1850U + ((value * 5) / 64);
		}
		_Pragma(VECTOR_ID(ADC1_EOC_vector))
		__interrupt static void AdcISR()
		{
			Adc::ClearEvent(Adcs::EndOfConv);
			Adc::Disable();
			uint16_t result = 0;
			for(uint8_t i = 0; i < 10; ++i) {
				result += Adc::buffer[i];
			}
			if(Adc::GetSelectedChannel() == VsenChannel) {
				rawVoltage = result;
				Adc::SelectChannel(IsenChannel);
			}
			else {
				rawCurrent = result;
				Adc::SelectChannel(VsenChannel);
			}
		}
	public:
		enum
		{
			deviceMask = DevPowerSupply,
			features = PowersSupplyDefaultFeatures::PowerRating,
			MaxCurrent = PowersSupplyDefaultFeatures::MaxCurrent
		};
		FORCEINLINE
		static void Init()
		{
			{	using namespace Adcs;
				Isen::SetConfig<GpioBase::In_float>();
				Vsen::SetConfig<GpioBase::In_float>();
				Adc::DisableSchmittTrigger<IsenChannel | VsenChannel>();
				Adc::SelectChannel((Channel)IsenChannel);
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
			Twi::Init();
			Lcd::Init();
		}
		FORCEINLINE
		static void Process()
		{
			uint16_t* const bufval = (uint16_t*)&pdata.buf[1];
			switch(cmd) {
			case C_GetValue:
				pdata.buf[0] = ERR_NO;
				if(!pdata.n) {
					pdata.n = 6;
					bufval[0] = GetVoltage();
					bufval[1] = GetCurrent();
					*(uint8_t*)&bufval[2] = GetLoad();
				}
				else if(pdata.n == 1 && pdata.buf[0] < 4) {
					pdata.n = 3;
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
		FORCEINLINE
		static uint8_t GetDeviceFeatures(const uint8_t)
		{
			return features;
		}
		FORCEINLINE
		static void UpdIRQ()
		{
			Adc::Enable();
			Adc::StartConversion();
			updFlag = true;
		}
		FORCEINLINE
		static void DisplayRefresh()
		{
			if(updFlag) {
				updFlag = false;
				static uint8_t counter;
				if(++counter & 0x20) { // div32 ~= 2Hz
					counter = 0;
					uint8_t buf[16];
					Lcd::Clear();
					//Lcd::SetPosition(0);
					uint8_t* ptr = io::InsertDot(GetVoltage(), 2, buf);
					*ptr++ = 'v';
					*ptr++ = ' ';
					io::utoa16(GetCurrent(), ptr);
//					*ptr++ = 'm'; *ptr++ = 'A';
//					*ptr = '\0';
					Lcd::Puts(buf);
					Lcd::SetPosition(0, 1);
					uint16_t temperature = Tsense::Read();
					Lcd::Puts(io::utoa16(temperature/2, buf));
					Lcd::Putch('.');
					Lcd::Putch(temperature & 0x01 ? '5' : '0');
				}
			}
		}

		static uint16_t GetVoltage()
		{
			return ToTensOf_mV(rawVoltage);
		}
		static uint16_t GetCurrent()
		{
			return To_mA(rawCurrent);
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
	uint16_t PowerSupply::rawVoltage;
	uint16_t PowerSupply::rawCurrent;
	volatile bool PowerSupply::updFlag;

	}//Wk
}//Mcudrv

#endif // POWER_SUPPLY_H
