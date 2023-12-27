
//NOTE: this file is deprecated because now 16-bit real mode is used for Wactom C.

#if defined(__WC__)
#include "USBDDOS/DPMI/dpmi.h"
#include "USBDDOS/DPMI/xms.h"
#include <conio.h>
#include <stdlib.h>
//#include <dos.h>
//#include <assert.h>

#define USE_DLMALLOC 1
#if USE_DLMALLOC

#define ONLY_MSPACES 1
#define NO_MALLOC_STATS 1
#define USE_LOCKS 0
#define HAVE_MMAP 0
#define LACKS_SYS_PARAM_H
#pragma warning * 9
#include "dlmalloc.h"

static const int32_t DOS_HEAP_SIZE = 1024*16;
static uint32_t DOS_HeapHandle;
static mspace DOS_Space;

#endif

int32_t __dpmi_physical_address_mapping(int32_t addr, int32_t size); //TODO: make interface compatible with DJGPP
//int 31h, 0800h, bx:cx(bx:cx, si:di)
#pragma aux __dpmi_physical_address_mapping = \
"mov ecx, ebx" \
"shr ebx, 16" \
"mov di, ax" \
"shr eax, 16" \
"mov si, ax" \
"mov ax, 0800h" \
"int 31h" \
"mov ax, cx" \
"mov cx, bx" \
"shl ecx, 16" \
"mov cx, ax" \
value [ecx] \
parm [ebx] [eax] \
modify [eax ecx ebx si di]

uint16_t _my_cs();
#pragma aux _my_cs = "mov ax, cs" value[ax] parm[] modify[]

uint16_t _my_ds();
#pragma aux _my_ds = "mov ax, ds" value[ax] parm[] modify[]

uint16_t _my_es();
#pragma aux _my_es = "mov ax, es" value[ax] parm[] modify[]

uint16_t _my_ss();
#pragma aux _my_ss = "mov ax, ss" value[ax] parm[] modify[]

int32_t __dpmi_get_segment_base_address(uint16_t);
//int 31h, 0006h, cx:dx(bx)
#pragma aux __dpmi_get_segment_base_address = \
"mov ax, 0006h" \
"int 31h" \
"shl ecx, 16" \
"mov cx, dx" \
value [ecx] \
parm [bx] \
modify [ax dx bx ecx]

int32_t __dpmi_allocate_dos_memory(uint16_t paragraphs); //TODO: make interface compatible with DJGPP
//int 31h, 0100h, dx:ax(bx) dx=pm selector, ax=rm addr
#pragma aux __dpmi_allocate_dos_memory = \
"mov ax, 0100h" \
"int 31h" \
"shl edx, 16" \
"mov dx, ax" \
value [edx] \
parm [bx] \
modify [ax edx]

uint16_t __dpmi_free_dos_memory(uint16_t selector);
//int 31h, 0101h, ax(dx) ax=error code
#pragma aux __dpmi_free_dos_memory = \
"mov ax, 0101h" \
"int 31h" \
value [ax] \
parm [dx] \
modify [ax]

//only supprt by DOS/4GW professional
uint16_t __dpmi_simulate_real_mode_procedure_retf(void* reg);
//int 31h, 0301h, ax(bh=0,cx,es:edi)
#pragma aux __dpmi_simulate_real_mode_procedure_retf = \
"push es" \
"xor bx, bx" \
"xor cx, cx" \
"mov ax, ds" \
"mov es, ax" \
"mov ax, 0301h" \
"int 31h" \
"pop es" \
"sbb ax, ax" \
value [ax] \
parm [edi] \
modify [ax cx bx]

uint16_t __dpmi_simulate_real_mode_interrupt(uint8_t _int, void* reg);
//int 31h, 0300h ax(bl=i,bh=0,cx,es:edi)
#pragma aux __dpmi_simulate_real_mode_interrupt = \
"push es" \
"xor bh, bh" \
"xor cx, cx" \
"mov ax, ds" \
"mov es, ax" \
"mov ax, 0300h" \
"int 31h" \
"pop es" \
"sbb ax, ax" \
value [ax] \
parm [bl] [edi] \
modify [ax cx bx]

uint16_t __dpmi_simulate_real_mode_procedure_iret(void* reg);
//int 31h, 0302h ax(bh=0,cx,es:edi)
#pragma aux __dpmi_simulate_real_mode_interrupt = \
"push es" \
"xor bh, bh" \
"xor cx, cx" \
"mov ax, ds" \
"mov es, ax" \
"mov ax, 0302h" \
"int 31h" \
"pop es" \
"sbb ax, ax" \
value [ax] \
parm [bl] [edi] \
modify [ax cx bx]

uint32_t DPMI_MapMemory(uint32_t physicaladdr, uint32_t size)
{
    //DPMI call to map physical memory
    //if DOS/4GW starts at real mode, the mapped linear addr
    //should be the same as physical addr below 1M.
    //if EMM exist, memory always mapped.
    return __dpmi_physical_address_mapping(physicaladdr, size);
}

#if !USE_DLMALLOC
void* DPMI_DMAMalloc(unsigned int size, unsigned int alignment)
{
    //don't do malloc as TSR
    //temporary: use dos alloc to get physial address which equal linear on DOS/4GW
    //http://www.delorie.com/djgpp/v2faq/faq18_13.html option 1
    //
    //TODO:
    //1. DOS/4GW don't have uncommitted memory support, we cannot map XMS to a specific address, but since DOS/4GW
    //is pure flat with 0 base, we don't need map it to a specific location as in DJGPP. we map it to any location and it is near pointer already.
    //2. if we use dos memory, we can do the data transaction in real(v86) mode TSR directly.
    //
    //new conclusion:
    //DOS/4GW and Causeway don't have "call real mode far ret" support, thus XMS not quite available.
    uint32_t sel_seg = __dpmi_allocate_dos_memory((uint16_t)((size + alignment + 2 + 15)>>4));
    uint16_t selector = (uint16_t)((sel_seg>>16)&0xFFFF);
    uint16_t physicalseg = (uint16_t)(sel_seg&0xFFFF);

    int8_t* ptr = (int8_t*)(physicalseg<<4) + 2;
    void* aligned;

    uint32_t addr = DPMI_PTR2L(ptr);
    uint16_t offset = (uint16_t)(align(addr, alignment) - addr);

    aligned = (int8_t*)ptr + offset;
    ((uint16_t*)aligned)[-1] = (uint16_t)selector;

    //assert(align((int32_t)aligned, alignment) == (int32_t)aligned);

    return aligned;
}

void DPMI_DMAFree(void* ptr)
{
    uint16_t selector = ((uint16_t*)ptr)[-1];
    __dpmi_free_dos_memory(selector);
}

#else

void* DPMI_DMAMalloc(unsigned int size, unsigned int alignment/* = 4*/)
{
    return mspace_memalign(DOS_Space, alignment, size);
}

void DPMI_DMAFree(void* ptr)
{
    return mspace_free(DOS_Space, ptr);
}

#endif

uint32_t DPMI_DOSMalloc(uint16_t size)
{
    return __dpmi_allocate_dos_memory(size);
}

void DPMI_DOSFree(uint32_t segment)
{
    __dpmi_free_dos_memory((uint16_t)(segment>>16));
}

uint16_t DPMI_CallRealModeRETF(DPMI_REG* reg) //not supported by DOS/4GW, supported by Causeway or DOS/4GW Professional
{
    reg->d._reserved = 0;
    return __dpmi_simulate_real_mode_procedure_retf(&reg);
}

uint16_t DPMI_CallRealModeINT(uint8_t i, DPMI_REG* reg)
{
    reg->d._reserved = 0;
    return __dpmi_simulate_real_mode_interrupt(i, reg);
}

uint16_t DPMI_CallRealModeIRET(DPMI_REG* reg)
{
    reg->d._reserved = 0;
    return (uint16_t)__dpmi_simulate_real_mode_procedure_iret(i, (__dpmi_regs*)reg);
}

uint16_t DPMI_InstallISR(int i, void(*ISR)(void), DPMI_ISR_HANDLE* outputp handle)
{
    if(!DPMI_PM || i < 0 || i > 255 || handle == NULL)
        return -1;
    //TODO:
    return -1;
}

uint16_t DPMI_UninstallISR(DPMI_ISR_HANDLE* inputp handle)
{
    return -1;
}

void DPMI_GetPhysicalSpace(DPMI_SPACE* outputp spc)
{
    //cs & ds based at 0.
    spc->base = 0;
    spc->limit = 0xFFFFFFFF;
    //TODO: get esp
    #error not implemented.
}

static void DPMI_Shutdown(void);

void DPMI_Init(void)
{
#if USE_DLMALLOC
    DOS_HeapHandle = DPMI_DOSMalloc(DOS_HEAP_SIZE>>4);

    uint32_t DOS_Base = (DOS_HeapHandle&0xFFFF)<<4;
    DOS_Space = create_mspace_with_base((void*)DOS_Base, DOS_HEAP_SIZE, 0);
#endif
}
static void DPMI_Shutdown(void)
{
#if USE_DLMALLOC
    DPMI_DOSFree(DOS_HeapHandle);
    DOS_HeapHandle = 0;
#endif
}
#endif
