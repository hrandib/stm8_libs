/*
 *	Bootloader implementation, Wake protocol used:
 *  2016, Shestakov Dmitry
 */

#pragma once

#define WAKEDATABUFSIZE 256

#include "wake_base.h"

namespace Mcudrv {
namespace Wk {

enum McuId
{
	ID_STM8S003F3,
	ID_STM8S103F3,
	ID_STM8S105C6
};

enum
{
	BOOTLOADER_KEY = 0x34B8126E
};

class Bootloader : WakeData
{


};
}//Wk
}//Mcudrv
