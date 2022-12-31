#ifndef IWDG_H
#define IWDG_H

#include "stm8s.h"

namespace Mcudrv {

class Iwdg
{
private:
    enum
    {
        KEY_START = 0xCC,
        KEY_REFRESH = 0xAA,
        KEY_ACCESS = 0x55
    };
public:
    enum Period
    {
        P_16ms,
        P_32ms,
        P_64ms,
        P_128ms,
        P_256ms,
        P_512ms,
        P_1s
    };

    static void Refresh()
    {
        IWDG->KR = KEY_REFRESH;
    }

    static void SetPeriod(Period p)
    {
        IWDG->KR = KEY_ACCESS;
        IWDG->PR = p;
        IWDG->KR = KEY_REFRESH;
    }

    static void Enable()
    {
        IWDG->KR = KEY_START;
    }
};

} // Mcudrv

#endif // IWDG_H
