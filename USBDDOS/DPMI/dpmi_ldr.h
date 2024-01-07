//////////////////////////////////////////////////////////////////////////////
// A DPMI run time patcher to alt real mode relocation segments to protected
// mode selectors.
// It is a patch, not a REAL LOADER.
// The potential problems of using the pather is if app code (client)
// programatically copys the rellocation data before switching
// to proteced mode, then the copied data cannot be patched.
// Fortunately that usually doesn't happens.
// To handle that case perfectly we need a real loader before program
// startup, and also a stub utility to stub the executable,
// But that will not be done here.
//
// There's also a problem that the C library startup & cleanup
// routine may made a copy of the relocations on the stack, 
// we either do one of this:
//
// 1.walk the stack to modify the selector to segments befoe 
// switching to real mode. this not much reliable, but simpler.
//
// 2.user compiler/library specific feature such as init_seg to make
// DPMI shutdown at the last so that the protected mode DS is not
// accessed afterwards. (still it's not working for WC,
// because WC's c lib still accesses DS after finializing runtime)
// 
// 3.never going back to real mode(execpt on final exiting), and install
// a int 21 handler to translate DOS terminate & IO function call.
// This sounds intriguing but may have problems, because a 16 bit
// C library might potentially use any real mode segment besieds the CS/DS
// so it needs a customized C lib to work perfectly (like DJGPP).
// but if a customized C lib is used, the problem itself can
// be avoid without any hacks used here.
// This method tested OK for WC.
//
// 4.a custom C lib that take care of any use of real mode segments
// and so we don't need this loader patch as compiled in BC.
// It might also usefull for using SMALL memory models.
// Because the custom C lib could be tiny & compact enough,
// with unused functions removed, thus we don't need support MEDIUM/LARGE models.
// (Currently the iostream function is not used, except the this loader)
// the C lib of Public Domain Operation System (PDOS), PDPCLIB 
// https://sourceforge.net/projects/pdos/ (supports BC & WC)
// may be used as base to cusmize a small C lib
// 
// 5. when invalid segment happens, handle it in exception hander,
// changing real mode segments to selectors.
// this also may not work because
// although selectors value now are very small, 
// real mode lib may try to access the segment in very low memory(i.e IVT)
// thus made CPU treat the segments as selectors.
//
//
// Currently the combined methods of 3 and 5 are used.
//
//////////////////////////////////////////////////////////////////////////////
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
static_assert(sizeof(MZHEADER) == 0x1E, "size error");

typedef struct RelocationTable
{
    uint16_t offset;
    uint16_t segment;
}RELOC_TABLE;

extern MZHEADER DPMI_LOADER_Header;

#if defined(__MEDIUM__) || defined(__LARGE__)
#define DPMI_CS_MAX 32 //some compiler (i.e.) generate a segment for each compile unit, currently 32 is enough. for WC the link will merge segments and there'll be 2 or more.
#else
#define DPMI_CS_MAX 1
#endif

#if defined(__COMPACT__) || defined(__LARGE__)
#define DPMI_DS_MAX 32
#else
#define DPMI_DS_MAX 1
#endif

//segments might not be in full 64K sizes, need record the starts
extern int DPMI_LOADER_CS_Count;
extern int DPMI_LOADER_DS_Count;
extern uint16_t DPMI_LOADER_SegMain; //this a C wrapper to get address of main for C++
extern uint16_t DPMI_LOADER_CS[DPMI_CS_MAX];
extern uint16_t DPMI_LOADER_DS[DPMI_DS_MAX];

BOOL DPMI_LOADER_Init(uint16_t cs, uint16_t ds, uint32_t code_size);

//patch relocation in place, in conventional memory in real mode
//note: extern functions called be called in real mode anymore after patching.
BOOL DPMI_LOADER_PatchRM(uint32_t code_size, uint16_t cs, uint16_t cs_sel, uint16_t ds_sel, uint32_t size);

//patch relocations.
//input:
//          code_size: total size of the code segments
//          cs, ds: real mode segments
//          cs_sel, ds_sel: protected mode selectors
//          assume selectors are contingous, i.e. next code selector is cs_sel+8
//          address: proteced mode linear address of the beginging of whole program data in HIMEM
//          size:    size of whole program data
//MUST be called in PM mode.
BOOL DPMI_LOADER_Patch(uint32_t code_size, uint16_t cs, uint16_t cs_sel, uint16_t ds_sel, uint32_t address, uint32_t size);

int DPMI_LOADER_GetCSIndex(uint16_t segment);
int DPMI_LOADER_GetDSIndex(uint16_t segment);

//same as DPMI_LOADER_Get*, without debug checks
int DPMI_LOADER_FindCSIndex(uint16_t segment);
int DPMI_LOADER_FindDSIndex(uint16_t segment);

uint16_t DPMI_LOADER_GetCS(int index);
uint16_t DPMI_LOADER_GetDS(int index);

BOOL DPMI_LOADER_Unpatch(uint32_t code_size, uint16_t cs_sel, uint16_t cs_selend, uint16_t ds_sel, uint16_t ds_selend, uint32_t address, uint32_t size);

//replace cs_seg to cs_sel directly on stack, limit stack depth, and count of replacements
BOOL DPMI_LOADER_PatchStack(uint16_t cs_seg, uint16_t cs_sel, uint16_t sp, int depth, int count);

#if DEBUG
void DPMI_LOADER_DumpSegments();
#endif

BOOL DPMI_LOADER_Shutdown();

#ifdef __cplusplus
}
#endif

#endif//_DPMI_LDR_H_
