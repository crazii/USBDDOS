#ifndef _EMM_H_
#define _EMM_H_ 1
#include "USBDDOS/PLATFORM.H"
#include "USBDDOS/DPMI/DPMI.H"

#ifdef __cplusplus
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

typedef struct EMM_IOPorTrap
{
    uint32_t memory;
    uint32_t handle;
    uint16_t func;
}EMM_IOPT;

//get EMM version
uint16_t EMM_GetVersion(void);

//I/O virtualize
BOOL EMM_Install_IOPortTrap(uint16_t start, uint16_t end, EMM_IODT* inputp iodt, uint16_t count, EMM_IOPT* outputp iopt);

BOOL EMM_Uninstall_IOPortTrap(EMM_IOPT* inputp iopt);

#ifdef __cplusplus
}
#endif

#endif