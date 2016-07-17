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
