#pragma once

#include "adc.h"
#ifdef __VS12
#define __interrupt
#define __enable_interrupt()
#define __weak
#define __eeprom
#define __no_init
#endif

namespace Mcudrv
{
	namespace Btns
	{
		using namespace Adcs;
		
		#pragma inline=forced
		void ButtonsExec(uint8_t key);

		template<uint8_t _SampleCount, uint32_t Rcom, uint32_t R1, uint32_t R2, uint32_t R3,
				uint32_t R4,uint32_t R5, uint32_t R6, uint32_t R7>
		struct BTraits
		{
			static const uint8_t max_value = 0xFF;
			static const uint8_t SampleCount = _SampleCount;
			static const uint16_t tolerance = max_value / 20;		//5%
			static const uint16_t WatchdogThreshold = max_value - 6;			//влияет на срабатывание последней кнопки
			static const uint16_t B1value = max_value * R1 / (Rcom + R1);
			static const uint16_t B2value = max_value * (R1 + R2) / (Rcom + R1 + R2);
			static const uint16_t B3value = max_value * (R1 + R2 + R3) / (Rcom + R1 + R2 + R3);
			static const uint16_t B4value = max_value * (R1 + R2 + R3 + R4) / (Rcom + R1 + R2 + R3 + R4);
			static const uint16_t B5value = max_value * (R1 + R2 + R3 + R4 + R5) / (Rcom + R1 + R2 + R3 + R4 + R5);
			static const uint16_t B6value = max_value * (R1 + R2 + R3 + R4 + R5 + R6) / (Rcom + R1 + R2 + R3 + R4 + R5 + R6);
			static const uint16_t B7value = max_value * (R1 + R2 + R3 + R4 + R5 + R6 + R7) / (Rcom + R1 + R2 + R3 + R4 + R5 + R6 + R7);
			static const uint16_t B1value_max = B1value + tolerance;
			static const uint16_t B2value_max = B2value	+ tolerance;
			static const uint16_t B3value_max = B3value	+ tolerance;
			static const uint16_t B4value_max = B4value	+ tolerance;
			static const uint16_t B5value_max = B5value	+ tolerance;
			static const uint16_t B6value_max = B6value	+ tolerance;
			static const uint16_t B7value_max = B7value	+ tolerance;
			static const uint16_t B1value_min = B1value - tolerance;
			static const uint16_t B2value_min = B2value - tolerance;
			static const uint16_t B3value_min = B3value - tolerance;
			static const uint16_t B4value_min = B4value - tolerance;
			static const uint16_t B5value_min = B5value - tolerance;
			static const uint16_t B6value_min = B6value - tolerance;
			static const uint16_t B7value_min = B7value - tolerance;
		};

		class Buttons: public Adcs::Adc<Mode8Bit>
		{
		private:
		  
		public:
			#pragma inline=forced
			template<Channel ch, Adcs::Div clockdiv, uint16_t threshold>
			static void Init()
			{
				Adc::Init<ContMode, clockdiv>();
				ChannelSelect<ch>();
				WatchdogInit< 0, threshold>();
				EnableInterrupt<AnalogWatchdog>();
				Enable();
				StartConversion();
			}
		};
	}

	typedef Btns::BTraits<0xFF, 10000UL, 2400UL, 3300UL, 4300UL, 5100UL, 8200UL, 16000UL, 56000UL> BCfg;

	INTERRUPT_HANDLER(ADC1_IRQHandler, 22)
	{
		using namespace Mcudrv::Btns;
		
		static uint8_t scount;
		if (Buttons::IsEvent<AnalogWatchdog>())
		{	
			Buttons::ClearEvent<AnalogWatchdog>();
			static uint8_t prev_key;
			uint8_t key;
			uint8_t sample = Buttons::ReadSample();
			if (sample > BCfg::B1value_min && sample < BCfg::B1value_max) key = 0;
			if (sample > BCfg::B2value_min && sample < BCfg::B2value_max) key = 1;
			if (sample > BCfg::B3value_min && sample < BCfg::B3value_max) key = 2;
			if (sample > BCfg::B4value_min && sample < BCfg::B4value_max) key = 3;
			if (sample > BCfg::B5value_min && sample < BCfg::B5value_max) key = 4;
			if (sample > BCfg::B6value_min && sample < BCfg::B6value_max) key = 5;
			if (sample > BCfg::B7value_min && sample < BCfg::B7value_max) key = 6;
			if (key == prev_key)
			{
				if (scount != BCfg::SampleCount)
					scount++;
			}
			else 
			{
				prev_key = key;
				scount = 0;
			}
			if (scount == BCfg::SampleCount - 1)
				ButtonsExec(key);
		}
	}
}