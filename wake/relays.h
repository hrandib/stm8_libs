#pragma once

#include "pinlist.h"

template<typename Port>
struct cRelays : Port
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
	static void Toggle()
	{
		uint8_t value = Port::ReadODR();
		if (value)
		{
			prevValue = value;
			Port::Clear(0xFF);
		}
		else Port::Set(prevValue);

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
uint8_t cRelays<Port>::prevValue;

