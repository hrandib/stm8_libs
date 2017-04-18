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
