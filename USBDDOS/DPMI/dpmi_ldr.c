#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <dos.h> //_argv
#include "USBDDOS/DPMI/dpmi_ldr.h"
#include "USBDDOS/DPMI/dpmi.h"
#if DEBUG
#define _LOG_ENABLE 0
#endif
#include "USBDDOS/dbgutil.h"

//Note: This file works on BC, but currently not built for BC (not included in Makeifle.BC)
//beause BC's exit routine doesn't reload real mode DS (DGROUP), and it 
//doesn't current DS (protected mode ds) on the stack.
//Just exclude it from BC to save code size.

MZHEADER DPMI_LOADER_Header;
static RELOC_TABLE* DPMI_LOADER_RelocTable;

#if defined(__BC__)
#define __ARGV _argv
#else
#define __ARGV __argv
#endif

BOOL DPMI_LOADER_Init()
{
    if(__ARGV == NULL || __ARGV[0] == NULL)
        return FALSE;
    _LOG("argv[0]: %s\n", __ARGV[0]);

    DPMI_REG r = {0};
    r.h.ah = 0x3D; //open file
    r.h.al = 0;
    r.w.ds = FP_SEG(__ARGV[0]);
    r.w.dx = FP_OFF(__ARGV[0]);

    //note: we're in real mode. we can call int86x/intdosx directly,
    //but DPMI_CallRealModeINT also works, also avoid linking int86x code
    if(DPMI_CallRealModeINT(0x21, &r) != 0 || (r.w.flags&CPU_CFLAG))
        return FALSE;

    BOOL done = FALSE;
    uint16_t handle = r.w.ax;
    do
    {
        r.h.ah = 0x3F; //read
        r.w.bx = handle;
        r.w.cx = sizeof(DPMI_LOADER_Header);
        r.w.ds = FP_SEG(&DPMI_LOADER_Header);
        r.w.dx = FP_OFF(&DPMI_LOADER_Header);
        if(DPMI_CallRealModeINT(0x21, &r) != 0 || (r.w.flags&CPU_CFLAG) || r.w.ax != r.w.cx)
            break;
        if(DPMI_LOADER_Header.MZ != 0x5A4D)
            break;
        
        r.h.ah = 0x42; //seek
        r.h.al = 0; //seek_set
        r.w.bx = handle;
        r.w.dx = DPMI_LOADER_Header.relocation_tbl;
        r.w.cx = 0;
        if(DPMI_CallRealModeINT(0x21, &r) != 0 || (r.w.flags&CPU_CFLAG))
            break;
        
        DPMI_LOADER_RelocTable = (RELOC_TABLE*)malloc(DPMI_LOADER_Header.relocations*sizeof(RELOC_TABLE));
        memset(DPMI_LOADER_RelocTable, 0, DPMI_LOADER_Header.relocations*sizeof(RELOC_TABLE));

        r.h.ah = 0x3F;
        r.w.bx = handle;
        r.w.cx = sizeof(RELOC_TABLE) * DPMI_LOADER_Header.relocations;
        r.w.ds = FP_SEG(DPMI_LOADER_RelocTable);
        r.w.dx = FP_OFF(DPMI_LOADER_RelocTable);
        if(DPMI_CallRealModeINT(0x21, &r) != 0 || (r.w.flags&CPU_CFLAG) || r.w.ax != r.w.cx)
            break;
        done = TRUE;
    } while(0);

    //close file
    r.h.ah = 0x3E;
    r.w.bx = handle;
    DPMI_CallRealModeINT(0x21, &r);

    #if DEBUG
    for(uint16_t i = 0; i < DPMI_LOADER_Header.relocations; ++i)
        _LOG("relocation %u: %04x:%04x\n", i, DPMI_LOADER_RelocTable[i].segment, DPMI_LOADER_RelocTable[i].offset);
    #endif
    _LOG("min_alloc: %08lx max_alloc: %08lx\n", ((uint32_t)DPMI_LOADER_Header.min_alloc)<<4, ((uint32_t)DPMI_LOADER_Header.max_alloc)<<4);
    _LOG("pages: %d\n", DPMI_LOADER_Header.pages);

    return done;
}

BOOL DPMI_LOADER_Patch(uint32_t code_size, uint16_t cs, uint16_t ds, uint16_t cs_sel, uint16_t ds_sel, uint32_t address, uint32_t size)
{
    _LOG("DPMI_LOADER: patching...\n");
    for(uint16_t i = 0; i < DPMI_LOADER_Header.relocations; ++i)
    {
        uint32_t offset = (((uint32_t)DPMI_LOADER_RelocTable[i].segment)<<4) + (uint32_t)DPMI_LOADER_RelocTable[i].offset;
        //_LOG("offset: %08lx, codesize: %08lx\n", offset, code_size);
        assert(offset < size); unused(size);
        BOOL incode = offset < code_size; unused(incode);
        uint16_t seg = DPMI_LoadW(address + offset);                    //this segment is in memory, already rellocated, 
        //_LOG("segment: %04x, cs: %04x, ds: %04x\n", seg, cs, ds);
        unused(ds);
        uint32_t relative = ((uint32_t)(seg-cs))<<4;                    //apply -start to get relative value
        uint16_t selector;

        BOOL iscode = relative < code_size;
        if(iscode)
        {
            uint16_t index = relative / (64UL*1024UL);
            selector = cs_sel + index*8;
        }
        else
        {
            uint16_t index = (relative-code_size) / (64UL*1024UL);
            selector = ds_sel + index*8;
        }
        //_LOG("selector: %04x, cs_sel: %04x, ds_sel: %04x\n", selector, cs_sel, ds_sel);
        _LOG("DPMI_LOADER: %d: [%08lx] segment %s in %s: %04x->%04x\n", i, offset, iscode ? "CODE" : "DATA", incode ? "CODE" : "DATA", seg, selector);
        DPMI_StoreW(address+offset, selector);
    }
    _LOG("DPMI_LOADER: patching done.\n");
    return TRUE;
}

BOOL DPMI_LOADER_Unpatch(uint32_t code_size, uint16_t cs, uint16_t ds, uint16_t cs_sel, uint16_t ds_sel, uint32_t address, uint32_t size)
{
    _LOG("DPMI_LOADER: unpatching...\n");
    for(uint16_t i = 0; i < DPMI_LOADER_Header.relocations; ++i)
    {
        uint32_t offset = (((uint32_t)DPMI_LOADER_RelocTable[i].segment)<<4) + (uint32_t)DPMI_LOADER_RelocTable[i].offset;
        //_LOG("offset: %08lx, codesize: %08lx, size: %08lx\n", offset, code_size, size);
        assert(offset < size); unused(size); //unpatch not done, assert will freeze on exit
        BOOL incode = offset < code_size; unused(incode);
        uint16_t selector = DPMI_LoadW(address + offset);
        //_LOG("selector: %04x, cs_sel: %04x, ds_sel: %04x\n", selector, cs_sel, ds_sel);
        uint16_t seg;
        BOOL iscode = selector < ds_sel;
        if(iscode)
        {
            uint16_t index = (selector-cs_sel)/8;
            seg = ((((uint32_t)(cs)) << 4) + ((uint32_t)index)*64UL*1024UL) >> 4;
        }
        else
        {
            uint16_t index = (selector-ds_sel)/8;
            seg = ((((uint32_t)(ds)) << 4) + ((uint32_t)index)*64UL*1024UL) >> 4;
        }
        //_LOG("segment: %04x, cs: %04x, ds: %04x\n", seg, cs, ds);
        _LOG("DPMI_LOADER: %d: [%08lx] segment %s in %s: %04x->%04x\n", i, offset, iscode ? "CODE" : "DATA", incode ? "CODE" : "DATA", selector, seg);
        DPMI_StoreW(address+offset, seg);
    }
    _LOG("DPMI_LOADER: unpatching done.\n");
    return TRUE;
}

BOOL DPMI_LOADER_UnpatchStack(uint16_t ds, uint16_t ds_sel, uint16_t sp, int depth)
{
    unused(ds);unused(ds_sel);unused(sp);unused(depth);
    #if DPMI_LOADER_METHOD == 1
    //unsafe & dirty hack for Watcom, as method 1 described in header of dpmi_ldr.h
    //there should be 3 of them, 2 DS and 1 ES pushed on stack:
    //open-watcom-v2/bld/clib/startup/c/initrtns.c
    uint16_t far* stack = (uint16_t far*)MK_FP(_SS, sp);
    uint16_t pmds = _DS;
    uint16_t pmes = _ES;
    for(int i = 0; i < depth; ++i)
    {
        if(stack[i] == pmds || stack[i] == pmes)
        {
            uint16_t index = (stack[i] - ds_sel)/8;
            #if defined(__LARGE__)
            if(index >= 16)
            #else
            if(index >= 1)
            #endif
                continue; //ES may point to RMCB_DS, we don't care about that
            uint16_t seg = ((((uint32_t)(ds)) << 4) + ((uint32_t)index)*64UL*1024UL) >> 4;
            _LOG("DPMI_LOADER: stack: %04x->%04x\n", stack[i], seg);
            stack[i] = seg;
        }
    }
    #endif
    return TRUE;
}

#if defined(_WC__) && DPMI_LOADER_METHOD == 2
//this hacks to WC's C lib to perform RM switch at last
//but it doesn't work because the finalizing routine
//saves PM DS on stack before calling every finializing fucntion,
//it doesn't know it's the last one and will realod PM DS and continue
//checking the list (although the list is empty), but with the invalid PM DS
extern void DPMI_Shutdown(void);
#pragma pack(__push,1)
#pragma pack(1)
struct rt_init
{
    uint8_t  rtn_type;
    uint8_t  priority;
    void*  rtn;
#if !defined( __MEDIUM__ ) && defined( __LARGE__ )
    uint16_t  padding;
#endif
};
#pragma pack(__pop)

struct rt_init __based( __segname("YI") ) shutdown =
{
    #if defined(__MEDIUM__) || defined(__LARGE__)
    1,
    #else
    0,
    #endif
    0, //priority 0
    &DPMI_Shutdown,
};
#endif

BOOL DPMI_LOADER_Shutdown()
{
    if(DPMI_LOADER_RelocTable)
    {
        free(DPMI_LOADER_RelocTable);
        DPMI_LOADER_RelocTable = NULL;
    }
    return TRUE;
}
