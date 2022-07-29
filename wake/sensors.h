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

#ifndef SENSORS_H
#define SENSORS_H

#include "wake_base.h"
#include "i2c.h"
namespace Mcudrv {
	namespace Wk {

	enum SensorType {
		SenTemperature = 0x01,
		SenHumidity = 0x02,
		SenPressure = 0x04,
		SenLight = 0x08,
		SenCO2 = 0x10,
		SenPresence = 0x20,
		SenWaterLeak = 0x40
	};

	//Temperature sensors
	class Tsensor_LM75 : WakeData, public NullModule
	{
	private:
		typedef Twis::SoftTwi<Twis::Standard, Pb4, Pb5> Twi;
		typedef Twis::Lm75<Twi> Tsense;
		enum InstructionSet {
			C_GetValue = 48
		};

	public:
		enum { deviceMask = DevSensor, features = SenTemperature };
		static void Init()
		{
            Twi::Init();
		}
		static uint16_t Read()
		{
			return Tsense::Read() * 5;
		}
		static uint8_t GetDeviceFeatures(uint8_t)
		{
			return features;
		}
		static void Process()
		{
			uint16_t* const bufval = (uint16_t*)&pdata.buf[1];
			switch(cmd) {
			case C_GetValue:
				pdata.buf[0] = ERR_NO;
				if(!pdata.n) {
					pdata.n = 3;
					bufval[0] = Read();
				}
				else if(pdata.n == 1 && pdata.buf[0] == 0) {
				//change in case of multiple sensors
					pdata.n = 3;
					bufval[0] = Read();
				}
				else {
					pdata.n = 1;
					pdata.buf[0] = ERR_PA;
				}
				break;
			default:
				processedMask |= deviceMask;
			}
		}
	};

	}//Wk
}//Mcudrv

#endif // SENSORS_H
