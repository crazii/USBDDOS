#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "dpmi_ldr.h"
#include "USBDDOS/DPMI/dpmi.h"
#include "USBDDOS/dbgutil.h"

static MZHEADER DPMI_LOADER_Header;
static RELOC_TABLE* DPMI_LOADER_RelocTable;
static uint16_t* DPMI_LOADER_Backups;

BOOL DPMI_LOADER_Init()
{
    if(__argv == NULL || __argv[0] == NULL)
        return FALSE;
    _LOG("argv[0]: %s\n", __argv[0]);

    FILE* fp = fopen(__argv[0], "rb");
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

    fclose(fp);
    DPMI_LOADER_Backups = (uint16_t*)malloc(DPMI_LOADER_Header.relocations*sizeof(uint16_t));
    return TRUE;
}

#if 0
BOOL DPMI_LOADER_PatchRelocation(uint16_t cs, uint16_t ds, uint16_t cs_sel, uint16_t ds_sel, uint32_t address, uint32_t size)
{
    //uint32_t code_size = (((uint32_t)ds)<<4) - (((uint32_t)cs)<<4);
    //uint32_t code_segs = (code_size + 64UL*1024UL - 1UL) / (64UL*1024UL) - 1; //minus initial CS
    for(uint16_t i = 0; i < DPMI_LOADER_Header.relocations; ++i)
    {
        uint32_t offset = (((uint32_t)DPMI_LOADER_RelocTable[i].segment)<<4) + (uint32_t)DPMI_LOADER_RelocTable[i].offset;
        assert(offset < size); unused(size);
        uint16_t seg = DPMI_LoadW(address + offset);                    //this segment is in memory, already rellocated, 
        BOOL iscode = seg < ds;
        uint32_t start = iscode ? (((uint32_t)cs)<<4) : (((uint32_t)ds)<<4);
        uint32_t relative = (((uint32_t)seg)<<4) - start;               //apply -start to get relative value
        uint16_t index = relative / (64UL*1024UL);
        uint16_t selector = (iscode ? cs_sel : ds_sel) + index*8;
        _LOG("segment %d: %s: %04x->%04x", i, iscode ? "CODE" : "DATA", seg, selector);
        DPMI_StoreW(address+offset, selector);
    }
    return TRUE;
}
#endif

BOOL DPMI_LOADER_PatchCode(uint16_t cs, uint16_t ds, uint16_t cs_sel, uint32_t address, uint32_t size)
{
    for(uint16_t i = 0; i < DPMI_LOADER_Header.relocations; ++i)
    {
        uint32_t offset = (((uint32_t)DPMI_LOADER_RelocTable[i].segment)<<4) + (uint32_t)DPMI_LOADER_RelocTable[i].offset;
        assert(offset < size); unused(size);
        uint16_t seg = DPMI_LoadW(address + offset);                    //this segment is in memory, already rellocated, 
        BOOL iscode = seg < ds;
        if(!iscode)
            continue;
        DPMI_LOADER_Backups[i] = seg;
        uint32_t start = (((uint32_t)cs)<<4);
        uint32_t relative = (((uint32_t)seg)<<4) - start;               //apply -start to get relative value
        uint16_t index = relative / (64UL*1024UL);
        uint16_t selector = cs_sel + index*8;
        _LOG("segment %d: %s: %04x->%04x", i, "CODE", seg, selector);
        DPMI_StoreW(address+offset, selector);
    }
    return TRUE;
}

BOOL DPMI_LOADER_PatchData(uint16_t ds, uint16_t ds_sel, uint32_t address, uint32_t size)
{
    for(uint16_t i = 0; i < DPMI_LOADER_Header.relocations; ++i)
    {
        RELOC_TABLE entry = DPMI_LOADER_RelocTable[i];

        uint32_t offset = (((uint32_t)entry.segment)<<4) + (uint32_t)entry.offset;
        assert(offset < size); unused(size);
        uint16_t seg = DPMI_LoadW(address + offset);                    //this segment is in memory, already rellocated, 
        BOOL iscode = seg < ds;
        if(iscode)
            continue;
        DPMI_LOADER_Backups[i] = seg;
        uint32_t start = (((uint32_t)ds)<<4);
        uint32_t relative = (((uint32_t)seg)<<4) - start;               //apply -start to get relative value
        uint16_t index = relative / (64UL*1024UL);
        uint16_t selector = ds_sel + index*8;
        _LOG("code segment %d: %s: %04x->%04x", i, "DATA", seg, selector);
        DPMI_StoreW(address+offset, selector);
    }
    return TRUE;
}

BOOL DPMI_LOADER_UnpatchData(uint16_t ds, uint16_t ds_sel, uint32_t address, uint32_t size)
{
    for(uint16_t i = 0; i < DPMI_LOADER_Header.relocations; ++i)
    {
        RELOC_TABLE entry = DPMI_LOADER_RelocTable[i];

        uint32_t offset = (((uint32_t)entry.segment)<<4) + (uint32_t)entry.offset;
        assert(offset < size); unused(size);
        uint16_t selector = DPMI_LoadW(address + offset);
        BOOL iscode = selector < ds_sel;
        if(iscode)
            continue;
        _LOG("code segment %d: %s: %04x->%04x", i, "DATA", selector, DPMI_LOADER_Backups[i]);
        assert(DPMI_LOADER_Backups[i] >= ds);
        DPMI_StoreW(address+offset, DPMI_LOADER_Backups[i]);
    }
    return TRUE;
}

BOOL DPMI_LOADER_Shutdown()
{
    if(DPMI_LOADER_RelocTable)
    {
        free(DPMI_LOADER_RelocTable);
        DPMI_LOADER_RelocTable = NULL;
    }
    if(DPMI_LOADER_Backups)
    {
        free(DPMI_LOADER_Backups);
        DPMI_LOADER_Backups = NULL;
    }
    return TRUE;
}
