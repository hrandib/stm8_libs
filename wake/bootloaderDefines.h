#pragma once

namespace WkBoot {

enum {
	BOOTSTART_KEY = 0x5A,
	BOOTLOADER_KEY = 0xA5,
	BOOTRESPONSE = 0xAB,
	BOOTADDRESS = 112
};

enum McuId {
	ID_STM8S003F3,
	ID_STM8S103F3,
	ID_STM8L051F3,
	ID_STM8S105C6 = 0x08	//Devices with 128 bytes flash block size start from 0x08
};

enum InstructionSet {
	C_NOP,
	C_ERR,
	C_ECHO,
	C_GETINFO,
	C_SETPOSITION = 12,
	C_READ,
	C_WRITE,
	C_GO
};

}//Wk
