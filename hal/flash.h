#pragma once

#include "stm8s.h"

namespace Mcudrv
{
	namespace Mem
	{
		enum MemType
		{
			Flash,
			Eeprom
		};
		
		#pragma inline=forced
		template<MemType>		//For Flash
		void inline Lock()
		{
			FLASH->IAPSR &= ~FLASH_IAPSR_PUL;
		}

		#pragma inline=forced
		template<MemType>		//For Flash
		void inline Unlock()
		{
			FLASH->PUKR = 0x56;
			FLASH->PUKR = 0xAE;
		}
		
		template<>
		void inline Lock<Eeprom>()	
		{
			FLASH->IAPSR &= ~FLASH_IAPSR_DUL;
		}

		template<>
		void inline Unlock<Eeprom>()	
		{
			FLASH->DUKR = 0xAE;
			FLASH->DUKR = 0x56;
		}

		#pragma inline=forced 
		template<MemType>
		bool inline IsUnlocked()		//For Flash
		{
			__no_operation();
			__no_operation();
			__no_operation();
			return FLASH->IAPSR & FLASH_IAPSR_PUL;
		}

		template<>
		bool inline IsUnlocked<Eeprom>()
		{
			__no_operation();
			__no_operation();
			__no_operation();
			return FLASH->IAPSR & FLASH_IAPSR_DUL;
		}

		#pragma inline=forced
		void SetWordProgramming()
		{
			FLASH->CR2 = FLASH_CR2_WPRG;
			FLASH->NCR2 = ~FLASH_NCR2_NWPRG;
		}

	}
}
