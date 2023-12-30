#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <dos.h> //_argv
#include "USBDDOS/DPMI/dpmi_ldr.h"
#include "USBDDOS/DPMI/dpmi.h"
#if DEBUG
#define _LOG_ENABLE 1
#endif
#include "USBDDOS/dbgutil.h"

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

    FILE* fp = fopen(__ARGV[0], "rb");
    if(fp == NULL)
        return FALSE;
    if(fread(&DPMI_LOADER_Header, sizeof(DPMI_LOADER_Header), 1, fp) != 1)
    {
        fclose(fp);
        return FALSE;
    }
    if(DPMI_LOADER_Header.MZ != 0x5A4D)
    {
        fclose(fp);
        return FALSE;
    }
    if(fseek(fp, DPMI_LOADER_Header.relocation_tbl, SEEK_SET) != 0)
    {
        fclose(fp);
        return FALSE;
    }

    DPMI_LOADER_RelocTable = (RELOC_TABLE*)malloc(DPMI_LOADER_Header.relocations*sizeof(RELOC_TABLE));
    memset(DPMI_LOADER_RelocTable, 0, DPMI_LOADER_Header.relocations*sizeof(RELOC_TABLE));
    if( fread(DPMI_LOADER_RelocTable, sizeof(RELOC_TABLE), DPMI_LOADER_Header.relocations, fp) != DPMI_LOADER_Header.relocations)
    {
        free(DPMI_LOADER_RelocTable);
        DPMI_LOADER_RelocTable = NULL;
        fclose(fp);
        return FALSE;
    }

    #if DEBUG
    for(uint16_t i = 0; i < DPMI_LOADER_Header.relocations; ++i)
        _LOG("relocation %u: %04x:%04x\n", i, DPMI_LOADER_RelocTable[i].segment, DPMI_LOADER_RelocTable[i].offset);
    #endif
    _LOG("min_alloc: %08lx max_alloc: %08lx\n", ((uint32_t)DPMI_LOADER_Header.min_alloc)<<4, ((uint32_t)DPMI_LOADER_Header.max_alloc)<<4);
	_LOG("pages: %d\n", DPMI_LOADER_Header.pages);

    fclose(fp);
    return TRUE;
}

BOOL DPMI_LOADER_Patch(uint32_t code_size, uint16_t cs, uint16_t ds, uint16_t cs_sel, uint16_t ds_sel, uint32_t address, uint32_t size)
{
    _LOG("DPMI_LOADER: patching...\n");
    for(uint16_t i = 0; i < DPMI_LOADER_Header.relocations; ++i)
    {
        uint32_t offset = (((uint32_t)DPMI_LOADER_RelocTable[i].segment)<<4) + (uint32_t)DPMI_LOADER_RelocTable[i].offset;
        _LOG("offset: %08lx, codesize: %08lx\n", offset, code_size);
        assert(offset < size); unused(size);
        BOOL incode = offset < code_size;
        uint16_t seg = DPMI_LoadW(address + offset);                    //this segment is in memory, already rellocated, 
        _LOG("segment: %04x, cs: %04x, ds: %04x\n", seg, cs, ds);
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
        _LOG("selector: %04x, cs_sel: %04x, ds_sel: %04x\n", selector, cs_sel, ds_sel);
        _LOG("DPMI_LOADER: %s segment %d in %s: %04x->%04x\n", iscode ? "CODE" : "DATA", i, incode ? "CODE" : "DATA", seg, selector);
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
        _LOG("offset: %08lx, codesize: %08lx, size: %08lx\n", offset, code_size, size);
        assert(offset < size); unused(size); //unpatch not done, assert will freeze on exit
        BOOL incode = offset < code_size;
        uint16_t selector = DPMI_LoadW(address + offset);
        _LOG("selector: %04x, cs_sel: %04x, ds_sel: %04x\n", selector, cs_sel, ds_sel);
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
        _LOG("segment: %04x, cs: %04x, ds: %04x\n", seg, cs, ds);
        _LOG("DPMI_LOADER: %s segment %d in %s: %04x<-%04x\n", iscode ? "CODE" : "DATA", i, incode ? "CODE" : "DATA", selector, seg);
        DPMI_StoreW(address+offset, seg);
    }
    _LOG("DPMI_LOADER: unpatching done.\n");
    return TRUE;
}

BOOL DPMI_LOADER_Shutdown()
{
    if(DPMI_LOADER_RelocTable)
    {
        free(DPMI_LOADER_RelocTable);
        DPMI_LOADER_RelocTable = NULL;
    }
    return TRUE;
}
