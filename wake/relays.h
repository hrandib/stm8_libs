#pragma once

#include "pinlist.h"

namespace Mcudrv {
namespace Wk {

template<typename Port>
struct Relays : Port
{
	static void Clear(uint8_t mask)
	{
		prevValue = Port::ReadODR();
		Port::Clear(mask);
	}
	static void Set(uint8_t mask)
	{
		prevValue = Port::ReadODR();
		Port::Set(mask);
	}
	static void Toggle(uint8_t mask)
	{
		uint8_t value = Port::ReadODR();
		if(0xFF == mask) {
			if(value) {
				prevValue = value;
				Port::Clear(mask);
			}
			else {
				Port::Set(prevValue);
			}
		}
		else {
			prevValue = value;
			Port::Toggle(mask);
		}
	}
	static void Restore()
	{
		if (!Port::ReadODR()) Port::Write(prevValue);
	}
	static uint8_t ReadPrevState()
	{
		return prevValue;
	}

private:
	static uint8_t prevValue;
};

template<typename Port>
uint8_t Relays<Port>::prevValue;

}//Wk
}//Mcudrv
