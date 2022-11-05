#include "USBDDOS/DPMI/DPMI.h"
#if defined(__DJ2__)
#include "USBDDOS/DPMI/XMS.H"
#include <conio.h>
#include <stdlib.h>
#include <dpmi.h>
#include <sys/farptr.h>
#include <sys/segments.h>
#include <crt0.h>
#include <assert.h>
#include <stdio.h>

extern DPMI_ADDRESSING DPMI_Addressing;

int _crt0_startup_flags = _CRT0_FLAG_LOCK_MEMORY | _CRT0_FLAG_FILL_DEADBEEF;

static uint32_t DPMI_DSBase = 0;
static uint32_t DPMI_DSLimit = 0;
static uint16_t DPMI_Selector4G;
static int16_t DPMI_TSR_Inited = 0;

typedef struct _AddressMap
{
    uint32_t Handle;
    int32_t LinearAddr;
    int32_t PhysicalAddr;
    int32_t Size;
}AddressMap;

#define ADDRMAP_TABLE_SIZE (256 / sizeof(AddressMap))

static AddressMap AddresMapTable[ADDRMAP_TABLE_SIZE];
static int AddressMapCount = 0;

static void AddAddressMap(const __dpmi_meminfo* info, int32_t PhysicalAddr)
{
    if(AddressMapCount == ADDRMAP_TABLE_SIZE)
    {
        //error
    }
    AddressMap* map = &AddresMapTable[AddressMapCount++];

    map->Handle = info->handle;
    map->LinearAddr = info->address;
    map->PhysicalAddr = PhysicalAddr;
    map->Size = info->size;
}

static void DPMI_Shutdown(void);

#define NEW_IMPL 1
#if NEW_IMPL
extern uint32_t DPMI_InitTSR(uint32_t base, uint32_t newbase, uint32_t* poffset, uint32_t* psize);
extern BOOL DPMI_ShutdownTSR();
static uint32_t XMS_Bias;
#else
static __dpmi_meminfo XMS_Info;
#endif
#define ONLY_MSPACES 1
#define NO_MALLOC_STATS 1
#define USE_LOCKS 0
#define HAVE_MMAP 0
#include "dlmalloc.c.h"
static const int32_t XMS_HEAP_SIZE = 1024*1024*16;
static mspace XMS_Space;
static uint32_t XMS_Physical;
static uint16_t XMS_Handle; //handle to XMS API

//http://www.delorie.com/djgpp/v2faq/faq18_13.html choice 3
//choice 1 (DOS alloc) is not suitable here since we need near ptr but DJGPP's DS base is larger than that, (we may need __djgpp_nearptr_enable)
//to do that. for most DOS Extenders like DOS/4G is OK to use choice 1 since there DS have a base of 0, thus DOS memory are near ptr.
static void Init_XMS()
{
    #if NEW_IMPL
    //use XMS memory since we need know the physical base for some non-DPMI DOS handlers, or drivers.
    //it is totally physically mapped, and doesn't support realloc, making malloc/brk/sbrk not working.
    //it will work if djgpp support srbk hook. stdout buffer will be pre-alloced for msg output & debug use.
    //doesn't matter, this is a driver anyway.
    uint32_t size = 0;
    uint32_t offset = 0;
    DPMI_InitTSR(0, 0, &offset, &size);
    size = align(size, 4096);

    XMS_Handle = XMS_Alloc((size+XMS_HEAP_SIZE)/1024, &XMS_Physical);
    if(XMS_Handle == 0)
        exit(1);

    XMS_Bias = offset >= 4096 ? 4096 : 0; //re-enable null pointer page fault
    uint32_t XMSBase = DPMI_MapMemory(XMS_Physical, size + XMS_HEAP_SIZE);
    _LOG("XMS base %08lx, XMS lbase %08lx Pool base %08lx\n", XMS_Physical, XMSBase, XMS_Physical+size);
    DPMI_TSR_Inited = DPMI_InitTSR(DPMI_DSBase, XMSBase - XMS_Bias, &offset, &size);
    _LOG("TSR inited.\n");
    int ds; //reload ds incase this function inlined and ds optimized as previous
    asm __volatile__("mov %%ds, %0":"=r"(ds)::"memory");
    assert(__dpmi_get_segment_limit(ds) == size-1);
    __dpmi_set_segment_limit(ds, size + XMS_HEAP_SIZE - 1);
    XMS_Space = create_mspace_with_base((void*)size, XMS_HEAP_SIZE, 0);
    _LOG("XMS init done.\n");
    //update mapping
    DPMI_DSBase = XMSBase - XMS_Bias;
    DPMI_DSLimit = size + XMS_HEAP_SIZE;
    #else
    //the idea is to allocate XMS memory in physical addr and mapped it after current ds's limit region,
    //then expand current ds' limit so that the mapped addr are within the current ds segment,
    //and the mapped data can be directly accessed as normal pointer (near ptr)
    //another trick is to use dlmalloc with mapped based ptr to allocate arbitary memory.
    XMS_Handle = XMS_Alloc(XMS_HEAP_SIZE/1024, &XMS_Physical);
    if(XMS_Handle == 0)
        exit(1);
    __dpmi_meminfo info = {0};
    info.size = XMS_HEAP_SIZE;
    #if 0     //Not supported by CWSDPMI and Windows, but by DPMIONE or HDPMI
    info.address = (DPMI_DSBase + DPMI_DSLimit + 4095) / 4096 * 4096;
    if( __dpmi_allocate_linear_memory(&info, 0) == -1)
    #else
    if(__dpmi_allocate_memory(&info) == -1)
    #endif
    {
        XMS_Free(XMS_Handle);
        printf("Failed to allocate linear memory (%08lx). \n", info.address);
        exit(1);
    }

    __dpmi_meminfo info2 = info;
    info2.address = 0;
    info2.size = XMS_HEAP_SIZE / 4096;
    if( __dpmi_map_device_in_memory_block(&info2, XMS_Physical) == -1)
    {
        XMS_Free(XMS_Handle);
        printf("Error: Failed to map XMS memory %08lx, %08lx.\n", info.address, info.size);
        exit(1);
    }
    uint32_t XMSBase = info.address;
    XMS_Info = info;
    info.handle = -1;
    AddAddressMap(&info, XMS_Physical);
    __dpmi_set_segment_limit(_my_ds(), XMSBase + XMS_HEAP_SIZE - 1);
    //printf("DSBase: %08lx, XMS at physical base: %08lx\n", DPMI_DSBase, XMS_Physical);
    XMS_Space = create_mspace_with_base((void*)(XMSBase - DPMI_DSBase), XMS_HEAP_SIZE, 0);
    #endif
}

void DPMI_Init(void)
{
    atexit(&DPMI_Shutdown);
    
    DPMI_Selector4G = __dpmi_allocate_ldt_descriptors(1);
    __dpmi_set_segment_base_address(DPMI_Selector4G, 0);
    __dpmi_set_segment_limit(DPMI_Selector4G, 0xFFFFFFFF);
    DPMI_Addressing.selector = DPMI_Selector4G;
    DPMI_Addressing.physical = FALSE;

    __dpmi_get_segment_base_address(_my_ds(), &DPMI_DSBase);
    DPMI_DSLimit = __dpmi_get_segment_limit(_my_ds());

    Init_XMS();

    /*
    int32_t* ptr = (int32_t*)DPMI_MappedMalloc(256,16);
    *ptr = 0xDEADBEEF;
    int32_t addr = DPMI_PTR2L(ptr);
    int32_t val = DPMI_LoadD(addr);
    printf("%08lx:%08lx\n",addr, val);

    int32_t addr2 = DPMI_MapMemory(DPMI_L2P(addr), 256);
    printf("%08lx:%08lx", addr2, DPMI_LoadD(addr2));
    exit(0);
    */
}

static void DPMI_Shutdown(void)
{
    #if NEW_IMPL
    //printf("cleanup TSR...\n"); fflush(stdout);
    if(DPMI_TSR_Inited)
    {
        DPMI_ShutdownTSR();
        asm __volatile__("":::"memory");
        DPMI_TSR_Inited = FALSE;
    }
    #else
    //libc may expand this limit, if we restore it to a smaller value, it may cause crash
    //__dpmi_set_segment_limit(_my_ds(), DPMI_DSLimit);
    //printf("free mapped XMS space...\n"); fflush(stdout);
    if(XMS_Info.handle != 0)
    {
        __dpmi_free_memory(XMS_Info.handle);
        XMS_Info.handle = 0;
    }
    #endif

    //printf("free mapped space...\n"); fflush(stdout);
    for(int i = 0; i < AddressMapCount; ++i)
    {
        AddressMap* map = &AddresMapTable[i];
        if(map->Handle == -1)//XMS mapped
            continue;
        __dpmi_meminfo info;
        info.handle = map->Handle;
        info.address = map->LinearAddr;
        info.size = map->Size;
        __dpmi_free_physical_address_mapping(&info);
    }
    AddressMapCount = 0;
    //printf("free XMS memory...\n"); fflush(stdout);
    if(XMS_Handle != 0)
    {
        XMS_Free(XMS_Handle);
        XMS_Handle = 0;
    }
}

uint32_t DPMI_L2P(uint32_t vaddr)
{
    for(int i = 0; i < AddressMapCount; ++i)
    {
        AddressMap* map = &AddresMapTable[i];
        if(map->LinearAddr <= vaddr && vaddr <= map->LinearAddr + map->Size)
        {
            int32_t offset = vaddr - map->LinearAddr;
            return map->PhysicalAddr + offset;
        }
    }
    printf("Error mapping linear address to physical: %08lx (%08lx,%08lx).\n", vaddr, DPMI_DSBase, DPMI_DSBase+DPMI_DSLimit);
    printf("Exit\n");
    exit(1);
    return 0; //make compiler happy
}

uint32_t DPMI_P2L(uint32_t paddr)
{
    for(int i = 0; i < AddressMapCount; ++i)
    {
        AddressMap* map = &AddresMapTable[i];
        if(map->PhysicalAddr <= paddr && paddr <= map->PhysicalAddr + map->Size)
        {
            int32_t offset = paddr - map->PhysicalAddr;
            return map->LinearAddr + offset;
        }
    }
    printf("Error mapping physical address to linear: %08lx.\n", paddr);
    assert(FALSE);
    exit(1);
    return 0; //make compiler happy
}

uint32_t DPMI_PTR2L(void* ptr)
{
    return ptr ? DPMI_DSBase + (uint32_t)ptr : 0;
}

void* DPMI_L2PTR(uint32_t addr)
{
    return addr > DPMI_DSBase ? (void*)(addr - DPMI_DSBase) : NULL;
}


uint32_t DPMI_MapMemory(uint32_t physicaladdr, uint32_t size)
{
    __dpmi_meminfo info;
    info.address = physicaladdr;
    info.size = size;
    if( __dpmi_physical_address_mapping(&info) != -1)
    {
        AddAddressMap(&info, physicaladdr);
        return info.address;
    }
    assert(FALSE);
    return 0;
}

void* DPMI_MappedMalloc(unsigned int size, unsigned int alignment/* = 4*/)
{
    return mspace_memalign(XMS_Space, alignment, size);
}

void DPMI_MappedFree(void* ptr)
{
    return mspace_free(XMS_Space, ptr);
}

uint32_t DPMI_DOSMalloc(uint16_t size)
{
    int selector = 0;
    uint16_t segment = __dpmi_allocate_dos_memory(size, &selector);
    if(segment != -1)
        return (selector << 16) | segment;
    else
        return 0;
}

void DPMI_DOSFree(uint32_t segment)
{
    __dpmi_free_dos_memory((uint16_t)(segment>>16));
}

uint16_t DPMI_CallRealModeRETF(DPMI_REG* reg)
{
    reg->d._reserved = 0;
    return (uint16_t)__dpmi_simulate_real_mode_procedure_retf((__dpmi_regs*)reg);
}

uint16_t DPMI_CallRealModeINT(uint8_t i, DPMI_REG* reg)
{
    reg->d._reserved = 0;
    return (uint16_t)__dpmi_simulate_real_mode_interrupt(i, (__dpmi_regs*)reg);
}

uint16_t DPMI_CallRealModeIRET(DPMI_REG* reg)
{
    reg->d._reserved = 0;
    return (uint16_t)__dpmi_simulate_real_mode_procedure_iret((__dpmi_regs*)reg);
}

uint16_t DPMI_InstallISR(int i, void(*ISR)(void), uint16_t* outputp realCS, uint16_t* outputp realIP)
{
    if(i < 0 || i > 255 || realCS == NULL || realIP == NULL)
        return -1;
        
    __dpmi_raddr ra;
    __dpmi_get_real_mode_interrupt_vector(i, &ra);
    *realCS = ra.segment;
    *realIP = ra.offset16;

    _go32_dpmi_seginfo go32pa;
    go32pa.pm_selector = _my_cs();
    go32pa.pm_offset = (uintptr_t)ISR;
    _go32_interrupt_stack_size = 1024;  //no need 32k stack. and it will crash. @see DPMI_TSR.C, 234
    _go32_dpmi_allocate_iret_wrapper(&go32pa);

    return _go32_dpmi_set_protected_mode_interrupt_vector(i, &go32pa);
}

void DPMI_GetPhysicalSpace(DPMI_SPACE* outputp spc)
{
    #if NEW_IMPL//doesn't work with old method.
    spc->baseds = XMS_Physical - XMS_Bias;
    spc->limitds = __dpmi_get_segment_limit(_my_ds());
    spc->basecs = spc->baseds;
    spc->limitcs = __dpmi_get_segment_limit(_my_cs());

    extern uint32_t __djgpp_stack_top;
    _LOG("DPMI_GetPhysicalSpace: physical ds base: %08lx, limit: %08lx, esp %08lx\n", spc->baseds, __dpmi_get_segment_limit(_my_ds()), __djgpp_stack_top);
    spc->stackpointer = __djgpp_stack_top;
    #else
    #error not supported. cannot get physical base from DPMI.
    #endif
}

#endif
