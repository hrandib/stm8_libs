#ifndef POWER_SUPPLY_H
#define POWER_SUPPLY_H

#include "wake_base.h"
#include "gpio.h"
#include "adc.h"

//		MBI6651
//	First Channel,	Fan Control
//  PC3(TIM1_CH3)	PD4(TIM2_CH1)
//		NCP3066
//	First Channel,				Fan Control
//	PA3(TIM2_CH3) - On/Off		PD4(TIM2_CH1)
//  PD3(TIM2_CH2) - driver NFB

namespace Mcudrv {
	namespace Wk {



	}//Wk
}//Mcudrv
#endif // POWER_SUPPLY_H
