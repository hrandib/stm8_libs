/*
 * Copyright (c) 2017-2022 Dmytro Shestakov
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

// Peripherals used: Uart, TIM4

#pragma once

#include "proto_version.h"
#include "wake_config.h"

#include "crc.h"
#include "flash.h"
#include "gpio.h"
#include "itc.h"
#include "timers.h"
#include "uart.h"

namespace Mcudrv {
namespace Wk {
//	---=== Operation time counter ===---
template<typename TCallback>
class OpTime
{
private:
    struct EepromBuf_t
    {
        uint16_t Dummy1;
        uint8_t Dummy2;
        uint8_t lvalue;
    };

#pragma data_alignment = 4
#pragma location = ".eeprom.noinit"
    static EepromBuf_t eebuf[16];
#pragma data_alignment = 4
#pragma location = ".eeprom.noinit"
    static uint16_t hvalue;
    volatile static bool tenMinPassed;

    // Fcpu/256/128 ~= 61 Hz for 2 MHz HSI
    _Pragma(VECTOR_ID(TIM4_OVR_UIF_vector)) __interrupt static void UpdIRQ()
    {
        static uint16_t tempCounter;
        T4::Timer4::ClearIntFlag();
        if(++tempCounter == (600U * 61)) {
            tempCounter = 0;
            SetTenMinutesFlag();
        }
        TCallback::UpdIRQ();
    }
public:
#pragma inline = forced
    static void Init()
    {
        using namespace T4;
        Itc::SetPriority(TIM4_OVR_UIF_vector, Itc::prioLevel_2_middle);
        Timer4::Init(Div_128, CEN);
        Timer4::EnableInterrupt();
    }
#pragma inline = forced
    static bool GetTenMinitesFlag()
    {
        return tenMinPassed;
    }
#pragma inline = forced
    static void SetTenMinutesFlag()
    {
        tenMinPassed = true;
    }
#pragma inline = forced
    static void ClearTenMinutesFlag()
    {
        tenMinPassed = false;
    }
    static uint8_t GetIndex()
    {
        uint8_t i;
        for(i = 0; i < 15; i++) {
            if(eebuf[i + 1].lvalue != eebuf[i].lvalue + 1)
                break;
        }
        return i;
    }
    static void Get(volatile uint8_t* arr)
    {
        uint16_t temp = hvalue;
        arr[0] = eebuf[GetIndex()].lvalue;
        arr[1] = temp & 0xFF;
        arr[2] = temp >> 8UL;
    }

#pragma inline = forced
    static void CountInc()
    {
        using namespace Mem;
        uint8_t i = GetIndex();
        uint8_t tmp = eebuf[i].lvalue + 1;
        Unlock<Eeprom>();
        if(IsUnlocked<Eeprom>()) {
            if(i != 15)
                eebuf[i + 1].lvalue = tmp;
            else
                eebuf[0].lvalue = tmp;
            if(tmp == 0)
                ++hvalue;
        }
        Lock<Eeprom>();
    }
};

template<typename TCallback>
typename OpTime<TCallback>::EepromBuf_t OpTime<TCallback>::eebuf[16];
template<typename TCallback>
uint16_t OpTime<TCallback>::hvalue;
template<typename TCallback>
volatile bool OpTime<TCallback>::tenMinPassed;

//			---=== Wake main definitions ===---

enum
{
    DefaultADDR = 127,
    DefaultGroupADDR = 95,
    REBOOT_KEY = 0xCB47ED91U, // Host should use big endian format
    CRC_INIT = 0xDE,
    FEND = 0xC0,  // Frame END
    FESC = 0xDB,  // Frame ESCape
    TFEND = 0xDC, // Transposed Frame END
    TFESC = 0xDD  // Transposed Frame ESCape
};

enum Mode
{
    Master,
    Slave
};

enum State
{
    WAIT_FEND = 0, // Wait for the FEND byte
    SEND_IDLE = 0, // Idle state
    ADDR,          // Waiting for the node address / send address
    CMD,           // Waiting for the commamd / send command
    NBT,           // Wait for the packet length / send packet length
    DATA,          // Data receiving / sending
    CRC,           // Wait for CRC / send CRC
    CARR           // Wait for the carrier / packet sending finish
};

enum Cmd
{
    C_NOP,                                 // No operation
    C_ERR,                                 // Packet recv error
    C_ECHO,                                // Echo response
    C_GETINFO,                             // Get device info
    C_SETNODEADDRESS,                      // Change node address
    C_GETGROUPADDRESS,                     // Get node group address (multicast)
    C_SETGROUPADDRESS = C_GETGROUPADDRESS, // Change node group address (multicast)
    C_GETOPTIME,                           // Get node operation time (non-volatile, whole period)
    C_OFF,                                 // Common Off command, can be handled by multiple modules
    C_ON,                                  // Common On command, can be handled by multiple modules
    C_ToggleOnOff,                         // Common Toggle On/Off command, can be handled by multiple modules
    C_SAVESETTINGS,                        // Save current state to the non-volatile memory
    C_REBOOT,                              // Reboot the node, useful for the bootloader interaction

    C_BASE_END
};

enum Err
{
    ERR_NO,          // no error
    ERR_TX,          // Rx/Tx error
    ERR_BU,          // device busy error
    ERR_RE,          // device not ready error
    ERR_PA,          // parameters value error
    ERR_NI,          // Command not impl
    ERR_NR,          // no replay
    ERR_NC,          // no carrier
    ERR_ADDRFMT,     // new address is wrong
    ERR_EEPROMUNLOCK // EEPROM wasn't unlocked
};

enum DeviceType
{
    DevNull,
    DevLedDriver = 0x01,
    DevSwitch = 0x02,
    DevRgbDriver = 0x04,
    DevGenericIO = 0x08,
    DevSensor = 0x10,
    DevPowerSupply = 0x20,
    DevReserved = 0x40,
    DevCustom = 0x80
};

enum AddrType
{
    addrGroup,
    addrNode
};

// Each module must implement this interface
struct NullModule
{
    enum
    {
        deviceMask = DevNull,
        features = 0
    };
    static void Init()
    { }
    static void Process()
    { }
    static void SaveState()
    { }
    static void On()
    { }
    static void Off()
    { }
    static uint8_t GetDeviceFeatures(uint8_t)
    {
        return 0;
    }
    static void ToggleOnOff()
    { }
    static void UpdIRQ()
    { }
};

template<typename Module1,
         typename Module2 = NullModule,
         typename Module3 = NullModule,
         typename Module4 = NullModule,
         typename Module5 = NullModule,
         typename Module6 = NullModule>
struct ModuleList
{
    enum
    {
        deviceMask = (uint8_t)Module1::deviceMask | Module2::deviceMask | Module3::deviceMask | Module4::deviceMask |
                     Module5::deviceMask | Module6::deviceMask
    };
    static void Init()
    {
        Module1::Init();
        Module2::Init();
        Module3::Init();
        Module4::Init();
        Module5::Init();
        Module6::Init();
    }
    static void Process()
    {
        Module1::Process();
        Module2::Process();
        Module3::Process();
        Module4::Process();
        Module5::Process();
        Module6::Process();
    }
    static uint8_t GetDeviceFeatures(uint8_t deviceMask)
    {
        return deviceMask & Module1::deviceMask   ? Module1::GetDeviceFeatures(deviceMask)
               : deviceMask & Module2::deviceMask ? Module2::GetDeviceFeatures(deviceMask)
               : deviceMask & Module3::deviceMask ? Module3::GetDeviceFeatures(deviceMask)
               : deviceMask & Module4::deviceMask ? Module4::GetDeviceFeatures(deviceMask)
               : deviceMask & Module5::deviceMask ? Module5::GetDeviceFeatures(deviceMask)
               : deviceMask & Module6::deviceMask ? Module6::GetDeviceFeatures(deviceMask)
                                                  : 0;
    }
    // The module should save its state only if it changed
    static void SaveState()
    {
        Module1::SaveState();
        Module2::SaveState();
        Module3::SaveState();
        Module4::SaveState();
        Module5::SaveState();
        Module6::SaveState();
    }
    static void On()
    {
        Module1::On();
        Module2::On();
        Module3::On();
        Module4::On();
        Module5::On();
        Module6::On();
    }
    static void Off()
    {
        Module1::Off();
        Module2::Off();
        Module3::Off();
        Module4::Off();
        Module5::Off();
        Module6::Off();
    }
    static void ToggleOnOff()
    {
        Module1::ToggleOnOff();
        Module2::ToggleOnOff();
        Module3::ToggleOnOff();
        Module4::ToggleOnOff();
        Module5::ToggleOnOff();
        Module6::ToggleOnOff();
    }
    static void UpdIRQ()
    {
        Module1::UpdIRQ();
        Module2::UpdIRQ();
        Module3::UpdIRQ();
        Module4::UpdIRQ();
        Module5::UpdIRQ();
        Module6::UpdIRQ();
    }
};

class WakeData
{
public:
    struct Packet
    {
        uint8_t addr;
        uint8_t cmd;
        uint8_t n;
        uint8_t buf[WAKEDATABUFSIZE];
    };
protected:
    static volatile Packet pdata;
    static volatile uint8_t cmd;
    // Each modules will set the respective flag if it NOT processed active command
    static volatile uint8_t processedMask;
};

template<typename moduleList = ModuleList<NullModule>,
         Uarts::BaudRate baud = 9600UL,
         typename DriverEnable = Pd6,
         Mode mode = Slave> // TODO: Master mode
class Wake : WakeData
{
private:
    typedef Uarts::Uart Uart;
    typedef OpTime<moduleList> OpTime;
#pragma location = ".eeprom.noinit"
    static volatile uint8_t nodeAddr_nv;
#pragma location = ".eeprom.noinit"
    static volatile uint8_t groupAddr_nv;
    static volatile uint8_t prev_byte;
    static volatile State state; // Current tranfer mode
    static volatile uint8_t ptr; // data pointer in Rx buffer
    static Crc::Crc8 crc;

    static void SetAddress(const AddrType nodeOrGroup) // and get address
    {
        if(pdata.n == 2 && pdata.addr) // data length correct and no broadcast
        {
            if(nodeOrGroup ? CheckNodeAddress() : CheckGroupAddress()) {
                using namespace Mem;
                uint8_t tempAddr = pdata.buf[0];
                pdata.buf[0] = ERR_NO;
                pdata.buf[1] = tempAddr;
                if(tempAddr != (nodeOrGroup ? nodeAddr_nv : groupAddr_nv)) // no write if address equal
                {
                    Unlock(Eeprom);
                    if(IsUnlocked(Eeprom)) {
                        if(nodeOrGroup)
                            nodeAddr_nv = tempAddr;
                        else
                            groupAddr_nv = tempAddr;
                    }
                    else
                        pdata.buf[0] = ERR_EEPROMUNLOCK;
                    Lock(Eeprom);
                }
            }
            else {
                pdata.buf[0] = ERR_ADDRFMT;
            }
        }
        else if(!pdata.n) // Get address
        {
            pdata.buf[0] = groupAddr_nv;
            pdata.n = 1;
        }
        else {
            pdata.buf[0] = ERR_PA;
            pdata.n = 2;
        }
        if(pdata.buf[0])
            pdata.buf[1] = 0;
    }
#pragma inline = forced
    static bool CheckNodeAddress()
    {
        uint8_t taddr = pdata.buf[0];
        return taddr == (~pdata.buf[1] & 0xFF) && ((taddr && taddr < 80) || (taddr > 112 && taddr < 128));
    }
#pragma inline = forced
    static bool CheckGroupAddress()
    {
        uint8_t taddr = pdata.buf[0];
        return taddr == (~pdata.buf[1] & 0xFF) && taddr > 79 && taddr < 96;
    }
public:
#pragma inline = forced

    static void Init()
    {
        using namespace Uarts;
        enum
        {
            SingleWireMode = (Uart::BaseAddr == UART1_BaseAddress && UART_SINGLEWIRE_MODE ? Uarts::SingleWireMode : 0)
        };
        Uart::template Init<Cfg(Uarts::DefaultCfg | Cfg(SingleWireMode)), baud>();
        DriverEnable::template SetConfig<GpioBase::Out_PushPull_fast>();
        DriverEnable::Clear();
        // validate node address saved in eeprom
        if(!nodeAddr_nv || nodeAddr_nv > 127) {
            Mem::Unlock(Mem::Eeprom);
            if(Mem::IsUnlocked(Mem::Eeprom)) {
                nodeAddr_nv = 127;
                Mem::Lock(Mem::Eeprom);
            }
        }
        moduleList::Init();
        OpTime::Init();
        Wdg::Iwdg::Enable(Wdg::P_1s);
        Uart::EnableInterrupt(IrqDefault);
    }
    static void Process()
    {
        using namespace Mem;
        Wdg::Iwdg::Refresh();
        if(OpTime::GetTenMinitesFlag() && !IsActive()) {
            OpTime::ClearTenMinutesFlag();
            OpTime::CountInc(); // Refresh optime counter every 10 mins
            Unlock<Eeprom>();
            if(IsUnlocked<Eeprom>()) {
                moduleList::SaveState(); // Save to EEPROM
                Lock<Eeprom>();
            }
        }
        if(cmd) {
            switch(cmd) {
                case C_NOP:
                case C_ECHO:
                    break;
                case C_ERR:
                    cmd = Wk::C_NOP;
                    return;
                case C_GETINFO:
                    // Common device info
                    if(!pdata.n) {
                        pdata.buf[0] = ERR_NO;
                        pdata.buf[1] = moduleList::deviceMask;
                        pdata.buf[2] = INSTRUCTION_SET_VER_MAJOR << 4 | INSTRUCTION_SET_VER_MINOR;
                        pdata.n = 3;
                    }
                    // Info about single logical device
                    else if(pdata.n == 1) {
                        if(pdata.buf[0] < 7) {
                            const uint8_t deviceMask = 1 << pdata.buf[0];
                            // Device is available
                            if(moduleList::deviceMask & deviceMask) {
                                pdata.buf[0] = ERR_NO;
                                pdata.buf[1] = moduleList::GetDeviceFeatures(deviceMask);
                                pdata.n = 2;
                            }
                            // device not available
                            else {
                                pdata.buf[0] = ERR_NI;
                            }
                        }
                        // TODO: Custom device extension support if(pdata.buf[0] == 7)
                    }
                    else {
                        pdata.buf[0] = ERR_PA;
                        pdata.n = 1;
                    }
                    break;
                case C_SETNODEADDRESS:
                    SetAddress(addrNode);
                    break;
                case C_SETGROUPADDRESS:
                    if(!pdata.n) {
                        pdata.n = 2;
                        pdata.buf[0] = ERR_NO;
                        pdata.buf[1] = groupAddr_nv;
                    }
                    else {
                        SetAddress(addrGroup);
                    }
                    break;
                case C_GETOPTIME:
                    if(!pdata.n) {
                        pdata.buf[0] = Wk::ERR_NO;
                        OpTime::Get(&pdata.buf[1]);
                        pdata.n = 4;
                    }
                    else {
                        pdata.buf[0] = Wk::ERR_PA;
                        pdata.n = 1;
                    }
                    break;
                case C_OFF:
                    if(!pdata.n) {
                        pdata.buf[0] = Wk::ERR_NO;
                        moduleList::Off();
                    }
                    else {
                        pdata.buf[0] = Wk::ERR_PA;
                    }
                    pdata.n = 1;
                    break;
                case C_ON:
                    if(!pdata.n) {
                        pdata.buf[0] = Wk::ERR_NO;
                        moduleList::On();
                    }
                    else {
                        pdata.buf[0] = Wk::ERR_PA;
                    }
                    pdata.n = 1;
                    break;
                case C_ToggleOnOff:
                    if(!pdata.n) {
                        pdata.buf[0] = Wk::ERR_NO;
                        moduleList::ToggleOnOff();
                    }
                    else {
                        pdata.buf[0] = Wk::ERR_PA;
                    }
                    pdata.n = 1;
                    break;
                case C_SAVESETTINGS:
                    if(!pdata.n) {
                        pdata.buf[0] = ERR_NO;
                        moduleList::SaveState();
                    }
                    else
                        pdata.buf[0] = ERR_PA;
                    pdata.n = 1;
                    break;
#if BOOTLOADER_EXIST
                case C_REBOOT:
                    if(pdata.n == 4 && *(uint32_t*)pdata.buf == REBOOT_KEY) {
                        System::Reset();
                    }
                    else {
                        pdata.buf[0] = Wk::ERR_PA;
                    }
                    pdata.n = 1;
                    break;
#else
                case C_REBOOT:
                    pdata.buf[0] = Wk::ERR_NI;
                    pdata.n = 1;
                    break;
#endif
                default:
                    moduleList::Process();
                    // Check if command not processed in modules
                    if(processedMask == moduleList::deviceMask) {
                        processedMask = 0;
                        pdata.buf[0] = Wk::ERR_NI;
                        pdata.n = 1;
                    }
            } // Switch
            uint8_t tempAddr = pdata.addr;
            if(tempAddr == nodeAddr_nv || tempAddr && cmd == C_SETNODEADDRESS) {
                Send();
            }
            cmd = Wk::C_NOP;
        }
    }

#pragma inline = forced
    static bool IsActive()
    {
        return state != WAIT_FEND;
    }

    static void Send()
    {
        using namespace Uarts;
        DriverEnable::Set(); // Switch to TX
        char data_byte = FEND;
        crc.Reset(CRC_INIT); //инициализация CRC,
        crc(data_byte);      //обновление CRC
        Uart::Regs()->DR = data_byte;
        state = ADDR;
        prev_byte = TFEND;
        Uart::DisableInterrupt(IrqRxne);
        Uart::EnableInterrupt(IrqTxEmpty);
    }

#if defined(STM8S103) || defined(STM8S003)
    _Pragma(VECTOR_ID(UART1_T_TXE_vector))
#elif defined(STM8S105)
    _Pragma(VECTOR_ID(UART2_T_TXE_vector))
#endif
      __interrupt static void TxISR()
    {
        using namespace Uarts;
        if(Uart::IsEvent(EvTxComplete)) {
            Uart::ClearEvent(EvTxComplete);
            Uart::ClearEvent(EvRxne);
            Uart::EnableInterrupt(IrqRxne);
            DriverEnable::Clear(); // Switch to RX
        }
        else { // if(Uart::IsEvent(Uarts::TxEmpty))
            uint8_t data_byte;
            if(prev_byte == FEND) {
                data_byte = TFEND; // send TFEND instead FEND
                prev_byte = data_byte;
                Uart::Regs()->DR = data_byte;
                return;
            }
            if(prev_byte == FESC) {
                data_byte = TFESC; // send TFESC instead FESC
                prev_byte = data_byte;
                Uart::Regs()->DR = data_byte;
                return;
            }
            switch(state) {
                case ADDR: // send address
                    state = CMD;
                    if(pdata.addr) {
                        // MSB is always set for the address byte, it lets to skip this field for broadcasts
                        data_byte = nodeAddr_nv | 0x80;
                        break;
                    }
                case CMD:
                    data_byte = pdata.cmd & 0x7F;
                    state = NBT;
                    break;
                case NBT: // send packet byte length
                    data_byte = pdata.n;
                    state = DATA;
                    ptr = 0; // reset data pointer
                    break;
                case DATA: { // send data
                    uint8_t ptr_ = ptr;
                    if(ptr_ < pdata.n) {
                        data_byte = pdata.buf[ptr++];
                    }
                    else {
                        data_byte = crc.GetResult(); // send CRC
                        state = CRC;
                    }
                    break;
                }
                default:
                    Uart::DisableInterrupt(IrqTxEmpty);
                    state = SEND_IDLE; // packet sending finished
                    return;
            }
            crc(data_byte);        // calculate CRC
            prev_byte = data_byte; // store pre-byte
            if(data_byte == FEND)
                data_byte = FESC; // send FESC if byte stuffing required
            Uart::Regs()->DR = data_byte;
        }
    }

#if defined(STM8S103) || defined(STM8S003)
    _Pragma(VECTOR_ID(UART1_R_RXNE_vector))
#elif defined(STM8S105)
    _Pragma(VECTOR_ID(UART2_R_RXNE_vector))
#endif
      __interrupt static void RxISR()
    {
        using namespace Uarts;
        bool error = Uart::IsEvent(static_cast<Events>(EvParityErr | EvFrameErr | EvNoiseErr | EvOverrunErr));
        uint8_t data_byte = Uart::Regs()->DR;
        if(error) {
            state = WAIT_FEND; // wait for new packet
            cmd = C_ERR;       // send error status
            return;
        }
        if(data_byte == FEND) {
            prev_byte = data_byte;
            crc.Reset(CRC_INIT);
            state = ADDR;
            crc(data_byte);
            return;
        }
        if(state == WAIT_FEND) {
            return;
        }
        char Pre = prev_byte;
        prev_byte = data_byte;
        if(Pre == FESC) {
            if(data_byte == TFESC) // byte de-stuffing routine
                data_byte = FESC;
            else if(data_byte == TFEND)
                data_byte = FEND;
            else {
                state = WAIT_FEND; // send error for the unexpected data
                cmd = C_ERR;
                return;
            }
        }
        else {
            if(data_byte == FESC)
                return;
        }
        switch(state) {
            case ADDR: // wait for address recv
                if(data_byte & 0x80) {
                    crc(data_byte);
                    data_byte &= 0x7F; // normalize address byte
                    if(data_byte == 0 || data_byte == nodeAddr_nv || data_byte == groupAddr_nv) {
                        pdata.addr = data_byte;
                        state = CMD; // next step - command recv
                        break;
                    }
                    state = WAIT_FEND; // adress not matched, wait for new packet
                    break;
                }
                else
                    pdata.addr = 0; // MSB = 0, skip address recv, next step - command recv
                state = CMD;
            case CMD: // wait for the command
                // command MSB must always equal zero
                if(data_byte & 0x80) {
                    state = WAIT_FEND;
                    cmd = C_ERR;
                    break;
                }
                pdata.cmd = data_byte; // store command
                crc(data_byte);
                state = NBT; // next step - recv packet length
                break;
            case NBT:
                if(data_byte > WAKEDATABUFSIZE) {
                    state = WAIT_FEND;
                    cmd = C_ERR;
                    break;
                }
                pdata.n = data_byte;
                crc(data_byte);
                ptr = 0;      // data pointer reset
                state = DATA; // wait for payload
                break;
            case DATA:
                uint8_t ptr_ = ptr;
                if(ptr_ < pdata.n) {
                    pdata.buf[ptr++] = data_byte;
                    crc(data_byte);
                    break;
                }
                // Check CRC
                if(data_byte != crc.GetResult()) {
                    state = WAIT_FEND;
                    cmd = C_ERR;
                    break;
                }
                state = WAIT_FEND;
                cmd = pdata.cmd; // Process received command
                break;
        }
    }
};

template<typename moduleList, Uarts::BaudRate baud, typename DEpin, Mode mode>
volatile uint8_t Wake<moduleList, baud, DEpin, mode>::nodeAddr_nv; // = 127;
template<typename moduleList, Uarts::BaudRate baud, typename DEpin, Mode mode>
volatile uint8_t Wake<moduleList, baud, DEpin, mode>::groupAddr_nv; // = 95;
template<typename moduleList, Uarts::BaudRate baud, typename DEpin, Mode mode>
volatile uint8_t Wake<moduleList, baud, DEpin, mode>::prev_byte;
template<typename moduleList, Uarts::BaudRate baud, typename DEpin, Mode mode>
volatile State Wake<moduleList, baud, DEpin, mode>::state;
template<typename moduleList, Uarts::BaudRate baud, typename DEpin, Mode mode>
volatile uint8_t Wake<moduleList, baud, DEpin, mode>::ptr;
template<typename moduleList, Uarts::BaudRate baud, typename DEpin, Mode mode>
Crc::Crc8 Wake<moduleList, baud, DEpin, mode>::crc;
} // Wk
} // Mcudrv
