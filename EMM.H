#ifndef _EMM_H_
#define _EMM_H_ 1
#include "USBDDOS/PLATFORM.H"
#include "USBDDOS/DPMI/DPMI.H"

#if __cplusplus
extern "C"
{
#endif

typedef uint32_t (*EMM_IOTRAP_HANDLER)(uint32_t port, uint32_t val, uint32_t out);

//user interface, not actual struct
typedef struct EMM_IODispatchTable
{
    uintptr_t    port;          //don't care about pointer size
    EMM_IOTRAP_HANDLER handler;
}EMM_IODT;

//get EMM version
uint16_t EMM_GetVersion(void);

//I/O virtualize
BOOL EMM_IOPortTrap(uint16_t start, uint16_t end, EMM_IODT* iodt, uint16_t count);

#if __cplusplus
}
#endif

#endif