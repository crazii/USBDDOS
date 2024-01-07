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

extern int main(int argc, char** argv);

MZHEADER DPMI_LOADER_Header;
static RELOC_TABLE* DPMI_LOADER_RelocTable;

#if defined(__BC__)
#define __ARGV _argv
#else
#define __ARGV __argv
#endif

int DPMI_LOADER_CS_Count;
int DPMI_LOADER_DS_Count;
uint16_t DPMI_LOADER_SegMain;
uint16_t DPMI_LOADER_CS[DPMI_CS_MAX];
uint16_t DPMI_LOADER_DS[DPMI_DS_MAX];

static BOOL DPMI_LOADER_AddSeg(uint16_t* segs, int* pcount, uint16_t seg, int MAX)
{
    int count = *pcount;
    for(int i = 0; i < count; ++i)
    {
        if(segs[i] == seg)
            return TRUE;
    }
    assert(count < MAX);
    if(count >= MAX)
        return FALSE;
    segs[count] = seg;
    return (*pcount)++;
}

static uint16_t DPMI_LOADER_FindSeg(const uint16_t* segs, int count, uint16_t seg, BOOL debug)
{
    for(int i = 0; i < count; ++i)
    {
        if(segs[i] == seg)
            return i;
    }
    if(debug)
    {
        _LOG("invalid segment: %04x\nvalid segments:\n",seg);
        for(int j = 0; j < count; ++j)
            _LOG("segment %02d: %04x\n",j, segs[j]);
        assert(FALSE);
    }
    return -1;
}

static int DPMI_SortSeg(const void* l, const void* r)
{
    const uint16_t* lseg = (const uint16_t*)l;
    const uint16_t* rseg = (const uint16_t*)r;
    return (int16_t)(*lseg - *rseg);
}

BOOL DPMI_LOADER_Init(uint16_t cs, uint16_t ds, uint32_t code_size)
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

    #if DEBUG && 0
    for(uint16_t i = 0; i < DPMI_LOADER_Header.relocations; ++i)
        _LOG("relocation %u: %04x:%04x\n", i, DPMI_LOADER_RelocTable[i].segment, DPMI_LOADER_RelocTable[i].offset);
    #endif
    _LOG("min_alloc: %08lx max_alloc: %08lx\n", ((uint32_t)DPMI_LOADER_Header.min_alloc)<<4, ((uint32_t)DPMI_LOADER_Header.max_alloc)<<4);
    _LOG("pages: %d\n", DPMI_LOADER_Header.pages);
    _LOG("cs: %04x, size: %lu\n", cs, code_size);

    assert(done);
    if(done)
    {
        for(uint16_t i = 0; i < DPMI_LOADER_Header.relocations; ++i)
        {
            uint16_t seg = *(uint16_t far*)MK_FP(cs + DPMI_LOADER_RelocTable[i].segment, DPMI_LOADER_RelocTable[i].offset);
            uint32_t relative = ((uint32_t)(seg-cs))<<4; //apply -start to get relative value
            BOOL iscode = relative < code_size;
            //_LOG("%s segment: %04x\n", iscode?"CODE":"DATA", seg);
            if(iscode)
                DPMI_LOADER_AddSeg(DPMI_LOADER_CS, &DPMI_LOADER_CS_Count, seg, DPMI_CS_MAX);
            else
                DPMI_LOADER_AddSeg(DPMI_LOADER_DS, &DPMI_LOADER_DS_Count, seg, DPMI_DS_MAX);
        }

        if(DPMI_LOADER_CS_Count == 0) //no relocations, tiny/small/compact?
            DPMI_LOADER_AddSeg(DPMI_LOADER_CS, &DPMI_LOADER_CS_Count, cs, DPMI_CS_MAX);

        if(DPMI_LOADER_DS_Count == 0) //no relocations, tiny/small/medium?
            DPMI_LOADER_AddSeg(DPMI_LOADER_DS, &DPMI_LOADER_DS_Count, ds, DPMI_DS_MAX);

        //sort segments addresses in ascending order
        qsort(DPMI_LOADER_DS, DPMI_LOADER_DS_Count, sizeof(uint16_t), DPMI_SortSeg);
        qsort(DPMI_LOADER_CS, DPMI_LOADER_CS_Count, sizeof(uint16_t), DPMI_SortSeg);
        DPMI_LOADER_SegMain = FP_SEG(&main);

        _LOG("Total segments: CODE: %u, DATA: %u\n", DPMI_LOADER_CS_Count, DPMI_LOADER_DS_Count);
        for(int j = 0; j < DPMI_LOADER_CS_Count; ++j)
            _LOG("CODE segment %02d: %04x\n",j, DPMI_LOADER_CS[j]);
        _LOG("seg main: %04x\n", DPMI_LOADER_SegMain);
    }
    return done;
}

BOOL DPMI_LOADER_PatchRM(uint32_t code_size, uint16_t cs, uint16_t cs_sel, uint16_t ds_sel, uint32_t size)
{
    //we're pathcing in place, extern functions cannot be called.
    //cannot call _LOG while/after patching, to the log before patching.
    #if DEBUG && 0
    {for(uint16_t i = 0; i < DPMI_LOADER_CS_Count; ++i)
    {
        uint16_t seg = DPMI_LOADER_CS[i];
        uint32_t relative = ((uint32_t)(seg-cs))<<4; //apply -start to get relative value
        BOOL iscode = relative < code_size;
        uint16_t selector;
        if(iscode)
        {
            uint16_t index = DPMI_LOADER_FindSeg(DPMI_LOADER_CS, DPMI_LOADER_CS_Count, seg, TRUE);
            selector = cs_sel + index*8;
        }
        else
        {
            uint16_t index = DPMI_LOADER_FindSeg(DPMI_LOADER_DS, DPMI_LOADER_DS_Count, seg, TRUE);
            selector = ds_sel + index*8;
        }
        _LOG("%s segment: %04x->%04x\n", iscode?"CODE":"DATA", seg, selector);
    }}

    {for(uint16_t i = 0; i < DPMI_LOADER_Header.relocations; ++i)
    {
        uint16_t far* entry = (uint16_t far*)MK_FP(cs + DPMI_LOADER_RelocTable[i].segment, DPMI_LOADER_RelocTable[i].offset);
        uint16_t seg = *entry;
        uint32_t relative = ((uint32_t)(seg-cs))<<4; //apply -start to get relative value
        BOOL iscode = relative < code_size;
        uint16_t selector;
        if(iscode)
        {
            uint16_t index = DPMI_LOADER_FindSeg(DPMI_LOADER_CS, DPMI_LOADER_CS_Count, seg, TRUE);
            selector = cs_sel + index*8;
        }
        else
        {
            uint16_t index = DPMI_LOADER_FindSeg(DPMI_LOADER_DS, DPMI_LOADER_DS_Count, seg, TRUE);
            selector = ds_sel + index*8;
        }
        uint32_t linear = (((uint32_t)FP_SEG(entry))<<4) + (uint32_t)FP_OFF(entry);
        _LOG("[%08lx] %s segment: %04x->%04x\n", linear, iscode?"CODE":"DATA", seg, selector);
    }}

    #endif

    unused(size);
    for(uint16_t i = 0; i < DPMI_LOADER_Header.relocations; ++i)
    {
        assert((((uint32_t)DPMI_LOADER_RelocTable[i].segment)<<4) + (uint32_t)DPMI_LOADER_RelocTable[i].offset < size);

        uint16_t far* entry = (uint16_t far*)MK_FP(cs + DPMI_LOADER_RelocTable[i].segment, DPMI_LOADER_RelocTable[i].offset);
        uint16_t seg = *entry;
        uint32_t relative = ((uint32_t)(seg-cs))<<4; //apply -start to get relative value
        BOOL iscode = relative < code_size;
        uint16_t selector;
        if(iscode)
        {
            uint16_t index = DPMI_LOADER_FindSeg(DPMI_LOADER_CS, DPMI_LOADER_CS_Count, seg, TRUE);
            selector = cs_sel + index*8;
        }
        else
        {
            uint16_t index = DPMI_LOADER_FindSeg(DPMI_LOADER_DS, DPMI_LOADER_DS_Count, seg, TRUE);
            selector = ds_sel + index*8;
        }
        *entry = selector;
    }
    return TRUE;
}

int DPMI_LOADER_GetCSIndex(uint16_t segment)
{
    return DPMI_LOADER_FindSeg(DPMI_LOADER_CS, DPMI_LOADER_CS_Count, segment, TRUE);
}

int DPMI_LOADER_GetDSIndex(uint16_t segment)
{
    return DPMI_LOADER_FindSeg(DPMI_LOADER_DS, DPMI_LOADER_DS_Count, segment, TRUE);
}

int DPMI_LOADER_FindCSIndex(uint16_t segment)
{
    return DPMI_LOADER_FindSeg(DPMI_LOADER_CS, DPMI_LOADER_CS_Count, segment, FALSE);
}

int DPMI_LOADER_FindDSIndex(uint16_t segment)
{
    return DPMI_LOADER_FindSeg(DPMI_LOADER_DS, DPMI_LOADER_DS_Count, segment, FALSE);
}

uint16_t DPMI_LOADER_GetCS(int index)
{
    assert(index < DPMI_LOADER_CS_Count);
    return index < DPMI_LOADER_CS_Count ? DPMI_LOADER_CS[index] : 0xFFFF;
}

uint16_t DPMI_LOADER_GetDS(int index)
{
    assert(index < DPMI_LOADER_DS_Count);
    return index < DPMI_LOADER_DS_Count ? DPMI_LOADER_DS[index] : 0xFFFF;
}

BOOL DPMI_LOADER_Patch(uint32_t code_size, uint16_t cs, uint16_t cs_sel, uint16_t ds_sel, uint32_t address, uint32_t size)
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
        uint32_t relative = ((uint32_t)(seg-cs))<<4;                    //apply -start to get relative value
        uint16_t selector;
        BOOL iscode = relative < code_size;
        if(iscode)
        {
            uint16_t index = DPMI_LOADER_FindSeg(DPMI_LOADER_CS, DPMI_LOADER_CS_Count, seg, TRUE);
            selector = cs_sel + index*8;
        }
        else
        {
            uint16_t index = DPMI_LOADER_FindSeg(DPMI_LOADER_DS, DPMI_LOADER_DS_Count, seg, TRUE);
            selector = ds_sel + index*8;
        }
        //_LOG("selector: %04x, cs_sel: %04x, ds_sel: %04x\n", selector, cs_sel, ds_sel);
        _LOG("DPMI_LOADER: %d: [%08lx] segment %s in %s: %04x->%04x\n", i, offset, iscode ? "CODE" : "DATA", incode ? "CODE" : "DATA", seg, selector);
        DPMI_StoreW(address+offset, selector);
    }
    _LOG("DPMI_LOADER: patching done.\n");
    return TRUE;
}

BOOL DPMI_LOADER_Unpatch(uint32_t code_size, uint16_t cs_sel, uint16_t cs_selend, uint16_t ds_sel, uint16_t ds_selend, uint32_t address, uint32_t size)
{
    _LOG("DPMI_LOADER: unpatching...\n");
    for(uint16_t i = 0; i < DPMI_LOADER_Header.relocations; ++i)
    {
        uint32_t offset = (((uint32_t)DPMI_LOADER_RelocTable[i].segment)<<4) + (uint32_t)DPMI_LOADER_RelocTable[i].offset;
        //_LOG("offset: %08lx, codesize: %08lx, size: %08lx\n", offset, code_size, size);
        assert(offset < size); unused(size); //unpatch not done, assert will freeze on exit
        BOOL incode = offset < code_size; unused(incode);
        uint16_t selector = DPMI_LoadW(address + offset);
        //_LOG("selector: %04x in %s, cs_sel: %04x, ds_sel: %04x\n", selector, incode?"CODE":"DATA", cs_sel, ds_sel);
        uint16_t seg;
        assert((selector >= cs_sel && selector <= cs_selend) || (selector >= ds_sel && selector <= ds_selend)); unused(ds_selend);
        BOOL iscode = (selector >= cs_sel && selector <= cs_selend);
        if(iscode)
        {
            uint16_t index = (selector-cs_sel)/8;
            assert(index < DPMI_LOADER_CS_Count);
            seg = DPMI_LOADER_CS[index];
        }
        else
        {
            uint16_t index = (selector-ds_sel)/8;
            assert(index < DPMI_LOADER_DS_Count);
            seg = DPMI_LOADER_DS[index];
        }
        //_LOG("segment: %04x, cs: %04x, ds: %04x\n", seg, cs_sel, ds_sel);
        _LOG("DPMI_LOADER: %d: [%08lx] segment %s in %s: %04x->%04x\n", i, offset, iscode ? "CODE" : "DATA", incode ? "CODE" : "DATA", selector, seg);
        DPMI_StoreW(address+offset, seg);
    }
    _LOG("DPMI_LOADER: unpatching done.\n");
    return TRUE;
}

BOOL DPMI_LOADER_PatchStack(uint16_t cs_seg, uint16_t cs_sel, uint16_t sp, int depth, int count)
{
    uint16_t far* stack = (uint16_t far*)MK_FP(_SS, sp);
    int c = 0;
    for(int i = 0; i <= depth; ++i)
    {
        if(cs_seg == stack[i])
        {
            _LOG("DPMI_LOADER: stack [%04x:%04x]: %04x->%04x\n", _SS, &stack[i], stack[i], cs_sel);
            stack[i] = cs_sel;
            if(++c == count)
                break;
        }
    }
    return c>0;
}

#if DEBUG
void DPMI_LOADER_DumpSegments()
{
    for(int j = 0; j < DPMI_LOADER_CS_Count; ++j)
        DBG_Log("CODE segment %02d: %04x\n",j, DPMI_LOADER_CS[j]);
    for(int i = 0; i < DPMI_LOADER_DS_Count; ++i)
        DBG_Log("DATA segment %02d: %04x\n",i, DPMI_LOADER_DS[i]);
}
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
