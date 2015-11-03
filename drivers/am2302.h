//With Bugs. Refer to stm32 version.
#pragma once
#include "stm8s.h"
#include "delay.h"
#include "gpio.h"
#include "timers.h"
#include "static_assert.h"

namespace Mcudrv
{
namespace Dht
{
	enum
	{
		Start,
		GetResponse,
		Reading,
		Timeout
	} state;

	template<typename Pin = Pd4>
	class Dht22
	{
	private:
		enum {
			PollPeriod = 5000,	//ms
			TimerStep = 4,		//us
			ResponseTimeMin = 72 / TimerStep,
			ResponseTimeMax = 84 / TimerStep,
			Threshold = 50 / TimerStep,
		};  
		static uint8_t index, bitcount, value;
		#pragma inline=forced
		static void DetectResponse()
		{
			static uint8_t stagecount;
			uint8_t timer = HardTimer::ReadCounter();
			HardTimer::Clear();
			switch (stagecount)
			{
				case 0: case 1: ++stagecount;
					break;
				case 2: case 3: if (timer >= ResponseTimeMin && timer <= ResponseTimeMax)
				{
					if (Pin::IsSet()) ++stagecount;
					else if (stagecount == 3)
					{
						stagecount = 0;
						index = bitcount = value = 0;
						state = Reading;
					}
				}
				else stagecount = 0;
					break;
			}
		}
		#pragma inline=forced
		static void ReadingProcess()
		{
			uint8_t timer = HardTimer::ReadCounter();
			HardTimer::Clear();
 			if (!Pin::IsSet())
			{
				value <<= 1;
				if (timer > Threshold) value |= 0x01;
				bitcount++;
			}
			if (bitcount >= 8)
			{
				bitcount = 0;
				if (index == 4)
				{
					uint8_t sum = valueArray[0] + valueArray[1] + valueArray[2] + valueArray[3];
					if (sum != value)
						valueArray[0] = valueArray[1] = valueArray[2] = valueArray[3] = 0;
					else
						state = Timeout;
						index = 0;
				}
				else valueArray[index++] = value;
			}
		}
		static uint8_t valueArray[4];
	public:
		typedef T4::Timer4 HardTimer;
		#pragma inline=forced
		static void Init()
		{
			Exti::SetExtIntMode<Pin::Port, Exti::RisingFallingEdge>();
//			Pin::template SetConfig<GpioBase::In_Pullup_int>();
			HardTimer::Init<T4::Div8, T4::ARPE>();			//clock 4us
			HardTimer::EnableInterrupt();
			HardTimer::Enable();
			Pin::Clear();
			Pin::template SetConfig<GpioBase::Out_OpenDrain_fast>();
			state = Start;
		}
		
		#pragma inline=forced
		static const uint8_t* GetValues()
		{
			return valueArray;
		}

		#pragma inline=forced
		static void DeInit()
		{
			state = Start;
			HardTimer::Disable();
			HardTimer::DisableInterrupt();
			Pin::template SetConfig<GpioBase::In_Pullup>();
		}

		#pragma inline=forced
		static void ExtInt()
		{
			switch (state)
			{
			case GetResponse: DetectResponse();
				break;
			case Reading: ReadingProcess();
				break;
			default:
				break;
			}
		}

		#pragma inline=forced
		static void TimerInt()
		{
			HardTimer::ClearIntFlag();
			static uint16_t timeout;
			if (state == Start)
			{
				Pin::template SetConfig<GpioBase::In_Pullup_int>();
				state = GetResponse;
			}
			else if (timeout++ == PollPeriod)
			{
				Pin::template SetConfig<GpioBase::Out_OpenDrain_fast>();
				timeout = 0;
				state = Start;
 			}
 		}
	};

	template<typename Pin>
	uint8_t Dht22<Pin>::valueArray[];
	template<typename Pin>
	uint8_t Dht22<Pin>::bitcount;
	template<typename Pin>
	uint8_t Dht22<Pin>::index;
 	template<typename Pin>
  	uint8_t Dht22<Pin>::value;

	INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 6)
	{
		Dht22<>::ExtInt();
	}

	INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
	 {
		 Dht22<>::TimerInt();
	 }
}//Dht22
}//Mcudrv

