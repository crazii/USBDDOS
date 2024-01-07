#ifndef _DPMI_H_
#define _DPMI_H_
#include "USBDDOS/platform.h"

#ifdef __cplusplus
extern "C"
{
#endif

//a DPMI wrapper for different compilers. Borland C++, DJGPP, WatCom etc.
//the porpose of this file is to hide all __dpmi* or int 31h calls from outside
typedef union _DPMI_REG {
struct
{
    uint16_t di, hdi;
    uint16_t si, hsi;
    uint16_t bp, hbp;
    uint16_t _reservedl, _reservedh;
    uint16_t bx, hbx;
    uint16_t dx, hdx;
    uint16_t cx, hcx;
    uint16_t ax, hax;
    uint16_t flags;
    uint16_t es;
    uint16_t ds;
    uint16_t fs;
    uint16_t gs;
    uint16_t ip;
    uint16_t cs;
    uint16_t sp;
    uint16_t ss;
}w; //BC doesn't support unamed struct.

struct
{
    uint32_t edi;
    uint32_t esi;
    uint32_t ebp;
    uint32_t _reserved;
    uint32_t ebx;
    uint32_t edx;
    uint32_t ecx;
    uint32_t eax;
}d;

struct
{
    uint8_t edi4b[4];
    uint8_t esi4b[4];
    uint8_t ebp4b[4];
    uint8_t _reserved4b[4];
    uint8_t bl, bh, eb2, eb3;
    uint8_t dl, dh, ed2, ed3;
    uint8_t cl, ch, ec2, ec3;
    uint8_t al, ah, ea2, ea3;
}h;
}DPMI_REG;

//DPMI_SPACE && DPMI_ADDRESSING usage: get physical addr and setup paging/gdt, then set to DPMI wrapper to use.

typedef struct//TSR/callback usage
{
    uint32_t baseds;        //physical base for ds,es, ss
    uint32_t limitds;       //limit for ds, in bytes
    uint32_t basecs;        //physical base for cs
    uint32_t limitcs;       //limit for cs, in bytes
    uint32_t stackpointer;  //esp
}DPMI_SPACE;

typedef struct //TSR/callback usage. state for linear DPMI_Load*/DPMI_Store*/DPMI_Mask*/DPMI_CopyLinear functions
{
    uint16_t selector; //selector created from outside
    uint16_t physical; //1: physical addr instead of virutal. used when not called through DPMI/DPMI, and context unavailable, and no paging
}DPMI_ADDRESSING;

typedef struct //old interrupt handler info returned by install isr. 
{
    uint32_t extra;
    uint32_t offset;
    uint16_t cs;
    uint16_t rm_offset;
    uint16_t rm_cs;
    uint8_t n; //INTn
    uint8_t user; //user defined
}DPMI_ISR_HANDLE;

#if defined(__BC__) || defined(__WC__)
#include <stdlib.h>

//convert a linear (virtual) addr to physical addr (current segment)
//only working with memory allocated with DPMI_DMAMalloc, or DPMI_MapMemory
#define DPMI_L2P(addr) (addr)

//convert a physical addr to linear addr (current segment)
//only working with memory allocated with DPMI_DMAMalloc, or DPMI_MapMemory
#define DPMI_P2L(addr) (addr)

//convert a DATA ptr to linear (page mapped) addr
//ptr must in data segment, not code, i.e. function address
uint32_t DPMI_PTR2L(void* ptr);

void* DPMI_L2PTR(uint32_t addr);

int DPMI_Exit(int c);

#define exit(c) DPMI_Exit(c)

#elif defined(__DJ2__)

//note: DJGPP use paged flat mode linear address.

//convert a linear (virtual) addr to physical addr
//only working with memory allocated with DPMI_DMAMalloc, or DPMI_MapMemory
uint32_t DPMI_L2P(uint32_t vaddr);

//convert a physical addr to linear addr
//only working with memory allocated with DPMI_DMAMalloc, or DPMI_MapMemory
uint32_t DPMI_P2L(uint32_t paddr);

//convert a ptr to linear (page mapped) addr
uint32_t DPMI_PTR2L(void* ptr);

void* DPMI_L2PTR(uint32_t addr);

#define DPMI_Exit(c) (c)

#else //stub

//convert a linear (virtual) addr to physical addr
uint32_t DPMI_L2P(uint32_t addr);
//convert a physical addr to linear addr
uint32_t DPMI_P2L(uint32_t addr);
//convert a ptr to linear (page mapped) addr
uint32_t DPMI_PTR2L(void* ptr);
void* DPMI_L2PTR(uint32_t addr);
int DPMI_Exit(int);

#endif

//convert ptr to physical addr
#define DPMI_PTR2P(ptr) DPMI_L2P(DPMI_PTR2L(ptr))
//convert physical addr to ptr
#define DPMI_P2PTR(addr) DPMI_L2PTR(DPMI_P2L(addr))

//return virtual mapped memory in linear space, usually for device IO
//get the device base address in physical space and map it to the linear space to access it
//input physical addr must < 1M (0x100000)
//http://www.delorie.com/djgpp/v2faq/faq18_7.html
uint32_t DPMI_MapMemory(uint32_t physicaladdr, uint32_t size);
uint32_t DPMI_UnmapMemory(uint32_t linearaddr);

//the 'DMA' here doesn't involve DMA controller, but device directly accessing physical RAM, i.e. PCI bus master
//memory allocated with this function is guaranteed to work with DPMI_L2P/DPMI_P2L
//use this function if you want map between virtual addr and physical addr, meant for driver/device MMIO
//malloc is recommended for normal memory allocation.
void* DPMI_DMAMalloc(unsigned int size, unsigned int alignment);
void* DPMI_DMAMallocNCPB(unsigned int size, unsigned int alignment); //no crossing page boundary
void DPMI_DMAFree(void* ptr);

//allocate DOS conventional memory (below 640K)
//input: size in paragraphs (16 bytes)
//return: high 16: selector, low 16: segment base address.
//allocated memory is 1:1 mapped (physical==linear)
uint32_t DPMI_DOSMalloc(uint16_t size);
void DPMI_DOSFree(uint32_t segment);

//RM call. return 0 on succeed.
uint16_t DPMI_CallRealModeRETF(DPMI_REG* reg);
uint16_t DPMI_CallRealModeINT(uint8_t i, DPMI_REG* reg);
uint16_t DPMI_CallRealModeIRET(DPMI_REG* reg);
//install interrupt service routine. ISR use normal return, no need to do IRET.
//return 0 if succeed
uint16_t DPMI_InstallISR(uint8_t i, void(*ISR)(void), DPMI_ISR_HANDLE* outputp handle);
uint16_t DPMI_UninstallISR(DPMI_ISR_HANDLE* inputp handle);
//allocate realmode callback. return: hiword: segment, lowword: offset. return 0 if fail
//input: Fn is a common function with normal return. the RMCB will use RETF finally
uint32_t DPMI_AllocateRMCB_RETF(void(*Fn)(void), DPMI_REG* reg);
uint32_t DPMI_AllocateRMCB_IRET(void(*Fn)(void), DPMI_REG* reg);

//TSR/callback usage
void DPMI_GetPhysicalSpace(DPMI_SPACE* outputp spc);
BOOL DPMI_TSR(void);
//switch addressing mode for linear functions, should use it temporarily and interrupt disabled.
void DPMI_SetAddressing(DPMI_ADDRESSING* inputp newaddr, DPMI_ADDRESSING* outputp oldaddr);

void DPMI_Init(void);
//void DPMI_Shutdown(void); auto called atexit

//linear memory access
uint8_t __CDECL DPMI_LoadB(uint32_t addr);
void __CDECL DPMI_StoreB(uint32_t addr, uint8_t val);
void __CDECL DPMI_MaskB(uint32_t addr, uint8_t mand, uint8_t mor);

uint16_t __CDECL DPMI_LoadW(uint32_t addr);
void __CDECL DPMI_StoreW(uint32_t addr, uint16_t val);
void __CDECL DPMI_MaskW(uint32_t addr, uint16_t mand, uint16_t mor);

uint32_t __CDECL DPMI_LoadD(uint32_t addr);
void __CDECL DPMI_StoreD(uint32_t addr, uint32_t val);
void __CDECL DPMI_MaskD(uint32_t addr, uint32_t mand, uint32_t mor);

void __CDECL DPMI_CopyLinear(uint32_t dest, uint32_t src, uint32_t size);
void __CDECL DPMI_SetLinear(uint32_t dest, uint8_t val, uint32_t size);
int32_t __CDECL DPMI_CompareLinear(uint32_t addr1, uint32_t addr2, uint32_t size);

//topdown allocation of DPMI_DOSMalloc, from high address (640K/1024K downwards).
//input: size in paragraphs (16 bytes)
//UMB: use UMB memory (640K~1024K), memory may not be 1:1 mapped.
//output: segment address in low 16 bit.
//note: only conventional 640K is 1:1 mapped.
//UMB returned addr not essentially equals to physical addr
//do not use UMB if physical addr (DPMI_L2P) is needed.
uint32_t DPMI_HighMalloc(uint16_t size, BOOL UMB);
void DPMI_HighFree(uint32_t segment);

//convert real mode far pointer to linear addr
#define DPMI_FP2L(f32) ((((f32)>>12)&0xFFFF0)+((f32)&0xFFFF))
//convert far pointer (seg:off) to linear
#define DPMI_SEGOFF2L(seg, off) ((((uint32_t)((seg)&0xFFFF))<<4) + ((off)&0xFFFF))
//convert far pointer (seg:off) to 32bit far ptr
#define DPMI_MKFP(seg, off) ((((uint32_t)((seg)&0xFFFF))<<16) | ((off)&0xFFFF))

#ifdef __cplusplus
}
#endif

#endif
