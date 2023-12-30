#ifndef _DPMI_LDR_H_
#define _DPMI_LDR_H_
#include "USBDDOS/platform.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MZHeader
{
    uint16_t MZ;
    uint16_t extra;
    uint16_t pages;
    uint16_t relocations;
    uint16_t size; //in paragraphs (16 bytes)
    uint16_t min_alloc; //min program size in paragraphs, including code, excluding PSP
    uint16_t max_alloc; //max program size in paragraphs, including code, excluding PSP
    uint16_t initSS;
    uint16_t initSP;
    uint16_t checksum;
    uint16_t initIP;
    uint16_t initCS;
    uint16_t relocation_tbl;
    uint16_t overlay;
    uint16_t overlay_num;

}MZHEADER;
_Static_assert(sizeof(MZHEADER) == 0x1E, "size error");

typedef struct RelocationTable
{
    uint16_t offset;
    uint16_t segment;
}RELOC_TABLE;

BOOL DPMI_LOADER_Init();

#if 0
//input:    cs, ds: real mode segments
//          cs_sel, ds_sel: protected mode selectors
//          assume selectors are contingous, i.e. next code selector is cs_sel+8
//          ds_sel should be larger than any code selectors
//          address: linear address of whole program data
//          size:    program data size
//MUST be called in PM mode.
BOOL DPMI_LOADER_PatchRelocation(uint16_t cs, uint16_t ds, uint16_t cs_sel, uint16_t ds_sel, uint32_t address, uint32_t size);
#endif

//only need patch code once, since code are not copied back to RM when switch back
BOOL DPMI_LOADER_PatchCode(uint16_t cs, uint16_t ds, uint16_t cs_sel, uint32_t address, uint32_t size);

//neeed patch/unpatch data every time when switching mode
BOOL DPMI_LOADER_PatchData(uint16_t ds, uint16_t ds_sel, uint32_t address, uint32_t size);
BOOL DPMI_LOADER_UnpatchData(uint16_t ds, uint16_t ds_sel, uint32_t address, uint32_t size);

BOOL DPMI_LOADER_Shutdown();

#ifdef __cplusplus
}
#endif

#endif//_DPMI_LDR_H_
