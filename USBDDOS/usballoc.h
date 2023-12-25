#ifndef _USBALLOC_H_
#define _USBALLOC_H_
#include "USBDDOS/DPMI/dpmi.h"

#ifdef __cplusplus
extern "C" {
#endif

//note: this code need more test
#define USBALLOC_ENABLE 0

#if USBALLOC_ENABLE

void USBALLOC_Init(void);

void USBALLOC_Shutdown(void);

//allocate transient buffer
void* USBALLOC_TransientAlloc(uint16_t size, uint16_t alignment);

//free transient buffer
void USBALLOC_TransientFree(void* ptr);

//allocate 32 bytes transient memory, ususally used for Transfer Descriptors
void* USBALLOC_TransientAlloc32(uint16_t sizeverify);

//free 32 bytes transient memory, ususally used for Transfer Descriptors
void USBALLOC_TransientFree32(void* ptr);

#define USB_InitAlloc() USBALLOC_Init()    //init usb alloc
#define USB_ShutdownAlloc() USBALLOC_Shutdown() //shutdown usb alloc
#define USB_TAlloc(size, align) USBALLOC_TransientAlloc(size, align)
#define USB_TFree(ptr) USBALLOC_TransientFree(ptr)
#define USB_TAlloc32(size) USBALLOC_TransientAlloc32(size)
#define USB_TFree32(ptr) USBALLOC_TransientFree32(ptr)

#else

#define USB_InitAlloc() //init usb alloc
#define USB_ShutdownAlloc() //shutdown usb alloc
#define USB_TAlloc(size, align) DPMI_DMAMalloc(size, align)
#define USB_TFree(ptr) DPMI_DMAFree(ptr)
#define USB_TAlloc32(size) DPMI_DMAMalloc(size, 32) //alignment to 32 for EHCI
#define USB_TFree32(ptr) DPMI_DMAFree(ptr)

#endif


#ifdef __cplusplus
}
#endif

#endif//_USBALLOC_H_
