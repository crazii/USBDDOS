//////////////////////////////////////////////////////////////////////////////
//local 16 bit DPMI that implements API defined in dpmi.h
//works for 16 bit compilers (Borland C, OpenWatcom C)
//the local DPMI can work standalone without a DPMI host,
//which removes the dependency of external DPMI servers.
//it simply copy the code & data to protected mode segments,
//thus MEDIUM/LARGE model not supported
//
//a LOADER/patcher which read the EXE's rellocation info and 
//setup multiple protected segments and apply the relllocation will
//make it work for MEDIUM/LARGE model
//for LARGE model, there also need to be a #define which replace the default _fmalloc
//and uses the protected mode method.
//////////////////////////////////////////////////////////////////////////////
#ifndef _DPMI_BC_H_
#define _DPMI_BC_H_
#include <assert.h>
#include <dos.h>
#if defined(__BC__) || defined(__WC__)
#include <bios.h>
#endif
#include "USBDDOS/platform.h"
#include "USBDDOS/pic.h"
#include "USBDDOS/DPMI/dpmi_ldr.h"
#include "USBDDOS/dbgutil.h"

#if defined(__BC__)
#define __LIBCALL _Cdecl _FARFUNC
extern "C" int __LIBCALL vsprintf(char* buf, const char* fmt, va_list aptr);
#elif defined(__WC__)
#define __LIBCALL _WCRTLINK
extern "C" int __LIBCALL vsnprintf(char* buf, size_t size, const char* fmt, va_list aptr);
#else
#define __LIBCALL 
extern "C" int vsnprintf(char* buf, size_t size, const char* fmt, va_list aptr);
#endif
extern "C" int __LIBCALL puts(const char* str);
extern "C" int __LIBCALL printf(const char* fmt, ...);
extern "C" void __LIBCALL delay(unsigned millisec);

#ifndef NDEBUG
#if defined(__BC__)
extern "C" void __LIBCALL __assertfail(char * __msg, char * __cond, char * __file, int __line);
#else
void __LIBCALL _assert99(char* expr, char* func, char* file, int line);
#endif
#endif

#define DPMI_SELEXT_MAX 8

#if defined(__BC__)
enum SEL_INDEX
{
    SEL_4G = 1,
    SEL_SYS_DS,     //gdt/idt/paging
    SEL_INTR_DS,    //ds selector used by interrupt/exceptions, a copy of DS used by dpmi_bc

    SEL_VIDEO,   //debug
    SEL_TEMP,

    SEL_RMCB_CS,
    SEL_RMCB_DS,

    SEL_VCPI_CS,//VCPI*3
    SEL_VCPI_1,
    SEL_VCPI_2,
    SEL_LDT,    //VCPI dummy
    SEL_TSS,    //VCPI dummy

    SEL_DS = 16,
    SEL_DSE = SEL_DS+DPMI_DS_MAX-1,
    SEL_HIMEM_DS,
    SEL_HIMEM_DSE = SEL_HIMEM_DS+DPMI_DS_MAX-1,
    SEL_EXT_DS, //for runtime exceptions
    SEL_EXT_DSE = SEL_EXT_DS + DPMI_SELEXT_MAX - 1,

    //note: code + ext code selectors must be contigous (see DPMI_HWIRQHandler)
    SEL_CS, //seletor for real mode (conventional) memory
    SEL_CSE = SEL_CS+DPMI_CS_MAX-1,
    SEL_HIMEM_CS, //selector for himem
    SEL_HIMEM_CSE = SEL_HIMEM_CS+DPMI_CS_MAX-1,
    SEL_EXT_CS,     //for runtime exceptions
    SEL_EXT_CSE = SEL_EXT_CS + DPMI_SELEXT_MAX - 1,
    SEL_TOTAL,
};
#else //WC won't compile if enum used as a constant in inline asm
//selectors
#define SEL_4G 1
#define SEL_SYS_DS (SEL_4G+1)
#define SEL_INTR_DS (SEL_SYS_DS+1)
#define SEL_VIDEO (SEL_INTR_DS+1)
#define SEL_TEMP (SEL_VIDEO+1)
#define SEL_RMCB_CS (SEL_TEMP+1)
#define SEL_RMCB_DS (SEL_RMCB_CS+1)
#define SEL_VCPI_CS (SEL_RMCB_DS+1)
#define SEL_VCPI_1 (SEL_VCPI_CS+1)
#define SEL_VCPI_2 (SEL_VCPI_1+1)
#define SEL_LDT (SEL_VCPI_2+1)
#define SEL_TSS (SEL_LDT+1)
#define SEL_DS 16 //FIX WC: too many tokens in a line
#define SEL_DSE (SEL_DS+DPMI_DS_MAX-1)
#define SEL_HIMEM_DS (SEL_DSE+1)
#define SEL_HIMEM_DSE (SEL_HIMEM_DS+DPMI_DS_MAX-1)
#define SEL_EXT_DS (SEL_HIMEM_DSE+1)
#define SEL_EXT_DSE (SEL_EXT_DS+DPMI_SELEXT_MAX-1)
#define SEL_CS (SEL_EXT_DSE+1)
#define SEL_CSE (SEL_CS+DPMI_CS_MAX-1)
#define SEL_HIMEM_CS (SEL_CSE+1)
#define SEL_HIMEM_CSE (SEL_HIMEM_CS+DPMI_CS_MAX-1)
#define SEL_EXT_CS (SEL_HIMEM_CSE+1)
#define SEL_EXT_CSE (SEL_EXT_CS+DPMI_SELEXT_MAX-1)
#define SEL_TOTAL (SEL_EXT_CSE+1)
#endif
static_assert(SEL_DS > SEL_TSS, "invalid enum/constants");

typedef enum
{
    PM_NONE = 0,
    PM_DIRECT = 1, //raw
    PM_VCPI = 2,   //vcpi
}DPMI_PM_MODE;

typedef struct //layout required by VCPI. do not change
{
    uint32_t CR3; //page directory
    uint32_t GDTR_linear;
    uint32_t IDTR_linear;
    uint16_t LDT_Selector;
    uint16_t TSS_Selector;
    uint32_t EIP;
    uint32_t CS;
}DPMI_VCPIClientStruct;

#define DPMI_RMCB_STACK_SIZE 128 //for switching mode
#define DPMI_RMCB_TRASNLATION_STACK_SIZE 512 //need more of them to support reentrance (translation to RM, RMCB to PM, translation to RM ...)
#define DPMI_RMCB_COUNT 16
#define DPMI_RMCB_ENTRYCODE_SIZE 6
#define DPMI_RMCB_SIZE 4096 //total size including stack

typedef void (far* DPMI_RMCB_ENTRY)(void);

typedef struct
{
    DPMI_RMCB_ENTRY Target; //asm optimization based the layout, do not change
    void far* UserReg;  //for user call backs. pm mode DS's offset
    char Code[DPMI_RMCB_ENTRYCODE_SIZE]; //call common entry, iret
}DPMI_RMCBTable;

//RMCB is used for real mode call backs, and as comptatible interface to switch modes inside the program.
//RMCB memory is alloated from DOS and stay resident to handle INT calls
//it contains the code to switch between modes.
//1. rm => pm: rm => CB => pm
//2. pm => rm (INT,RETF,IRET) => pm:
//3. INT (rm) => pm => rm:
//4. rm => pm (allocated user call backs)
typedef struct //real mode callback
{
    //FIXED OFFSET BEGIN used for inline asm
    IDTR OldIDTR;
    GDTR NullGDTR;
    GDTR GDTR; //the RMCB_ prefix is added to prevent ambiguous name for ASM
    IDTR IDTR;

    uint32_t CR3; //reserved for raw mode (xms mode) paging.

    uint16_t SwitchPM; //function offset
    uint16_t SwitchRM; //function offset
    uint16_t Translation; //function offset: PM2RM translation

    uint16_t PM_SP; //stack pointer
    uint16_t PM_SS; //stack segment. must follows PM_SP

    uint16_t RM_SP; //fixed stack for translation, temporary, do not save any thing on it after mode switch
    uint16_t RM_SEG; //real mode segment of RMCB (DS=CS=SS), //must follows RM_SP, as an SS

    uint32_t VcpiInterface; //used in V86
    uint16_t VcpiInterfaceCS; //grouped with VcpiInterface as a farcall, don't break
    uint16_t ClientWrapperCS; //selector of DPMI_RMCbClientWrapper, 0 if not MEDIUM model
    DPMI_VCPIClientStruct VcpiClient;
    uint32_t VcpiClientAddr; //linear address of VcpiClient

    uint16_t CommonEntry; //common entry: save context & swtich to PM to call target entry
    DPMI_RMCBTable Table[DPMI_RMCB_COUNT];
    //add new fixed offset for inline asm here
    //FIXED OFFSET END

    uint16_t TranslationSP;
    uint16_t Size;
    uint8_t RealModeIRQ0Vec;
    uint8_t ProtectedModeIRQ0Vec;
    uint8_t RealModeIRQ8Vec;
    uint8_t ProtectedModeIRQ8Vec;

}DPMI_RMCB;

#if defined(__BC__)

#pragma option -k-
static BOOL DPMI_IsV86() //should call before init pm
{
    _ASM_BEGIN //note: VM in eflags(V86) bit 17 never push on stack, so pushf/pop won't work
        _ASM(smsw ax)    //machine state word (low word of cr0)
        _ASM2(and ax, 0x1) //PE
    _ASM_END
    return _AX;
}
#pragma option -k

#elif defined(__WC__)

static BOOL DPMI_IsV86();
#pragma aux DPMI_IsV86 = \
"smsw ax" \
"and ax, 1" \
value[ax]

extern "C" uint16_t _STACKTOP;
#else

#define DPMI_IsV86() 0

#endif //__BC__

//another dirty hack for open watcom. no structure support in __asm bock. define offset and manually apply
//this also works for BC, we can use unified code
#define DPMI_REG_OFF_EDI 0
#define DPMI_REG_OFF_ESI 4
#define DPMI_REG_OFF_EBP 8
#define DPMI_REG_OFF_EBX 16
#define DPMI_REG_OFF_EDX 20
#define DPMI_REG_OFF_ECX 24
#define DPMI_REG_OFF_EAX 28
#define DPMI_REG_OFF_FLAGS 32
#define DPMI_REG_OFF_ES 34
#define DPMI_REG_OFF_DS 36
#define DPMI_REG_OFF_FS 38
#define DPMI_REG_OFF_GS 40
#define DPMI_REG_OFF_IP 42
#define DPMI_REG_OFF_CS 44
#define DPMI_REG_OFF_SP 46
#define DPMI_REG_OFF_SS 48
static_assert(DPMI_REG_OFF_EDI == offsetof(DPMI_REG, d.edi), "constant error");
static_assert(DPMI_REG_OFF_ESI == offsetof(DPMI_REG, d.esi), "constant error");
static_assert(DPMI_REG_OFF_EBP == offsetof(DPMI_REG, d.ebp), "constant error");
static_assert(DPMI_REG_OFF_EBX == offsetof(DPMI_REG, d.ebx), "constant error");
static_assert(DPMI_REG_OFF_EDX == offsetof(DPMI_REG, d.edx), "constant error");
static_assert(DPMI_REG_OFF_ECX == offsetof(DPMI_REG, d.ecx), "constant error");
static_assert(DPMI_REG_OFF_EAX == offsetof(DPMI_REG, d.eax), "constant error");
static_assert(DPMI_REG_OFF_FLAGS == offsetof(DPMI_REG, w.flags), "constant error");
static_assert(DPMI_REG_OFF_ES == offsetof(DPMI_REG, w.es), "constant error");
static_assert(DPMI_REG_OFF_DS == offsetof(DPMI_REG, w.ds), "constant error");
static_assert(DPMI_REG_OFF_FS == offsetof(DPMI_REG, w.fs), "constant error");
static_assert(DPMI_REG_OFF_GS == offsetof(DPMI_REG, w.gs), "constant error");
static_assert(DPMI_REG_OFF_CS == offsetof(DPMI_REG, w.cs), "constant error");
static_assert(DPMI_REG_OFF_IP == offsetof(DPMI_REG, w.ip), "constant error");
static_assert(DPMI_REG_OFF_SS == offsetof(DPMI_REG, w.ss), "constant error");
static_assert(DPMI_REG_OFF_SP == offsetof(DPMI_REG, w.sp), "constant error");

#define DPMI_REG_SIZE 50
static_assert(DPMI_REG_SIZE == sizeof(DPMI_REG), "error constant");

#define DPMI_PUSHAD_SIZE 32

#define RMCB_TABLE_ENTRY_SIZE 14
static_assert(RMCB_TABLE_ENTRY_SIZE == sizeof(DPMI_RMCBTable), "error constant");

#define DPMI_SIZEOF_GDT 8
static_assert(DPMI_SIZEOF_GDT == sizeof(GDT), "error constant");

#define VCPI_SIZEOF_CLIENTSTRUCT 24
static_assert(VCPI_SIZEOF_CLIENTSTRUCT == sizeof(DPMI_VCPIClientStruct), "constant error");

#define VCPI_CLIENTSTRUCT_OFF_EIP 16
static_assert(VCPI_CLIENTSTRUCT_OFF_EIP == offsetof(DPMI_VCPIClientStruct, EIP), "constant error");

#define RMCB_OFF_OldIDTR 2 //+2: extra padding
#define RMCB_OFF_NullGDTR 10 //+2: extra padding
#define RMCB_OFF_GDTR 18 //+2: extra padding
#define RMCB_OFF_IDTR 26
#define RMCB_OFF_CR3 32
#define RMCB_OFF_SwitchPM 36
#define RMCB_OFF_SwitchRM 38
#define RMCB_OFF_Translation 40
#define RMCB_OFF_PMSP 42
#define RMCB_OFF_RMSP 46
#define RMCB_OFF_RMSEG 48
#define RMCB_OFF_VcpiInterface 50
#define RMCB_OFF_ClientWrapperCS 56
#define RMCB_OFF_VcpiClient 58
#define RMCB_OFF_VcpiClientAddr (RMCB_OFF_VcpiClient+VCPI_SIZEOF_CLIENTSTRUCT)
#define RMCB_OFF_CommonEntry (RMCB_OFF_VcpiClientAddr+4)
#define RMCB_OFF_Table (RMCB_OFF_CommonEntry+2)

static_assert(RMCB_OFF_OldIDTR == offsetof(DPMI_RMCB, OldIDTR) + 2, "constant error"); //2: extra padding
static_assert(RMCB_OFF_NullGDTR == offsetof(DPMI_RMCB, NullGDTR) + 2, "constant error"); //2: extra padding
static_assert(RMCB_OFF_GDTR == offsetof(DPMI_RMCB, GDTR) + 2, "constant error");
static_assert(RMCB_OFF_IDTR == offsetof(DPMI_RMCB, IDTR) + 2, "constant error");
static_assert(RMCB_OFF_CR3 == offsetof(DPMI_RMCB, CR3), "constant error");
static_assert(RMCB_OFF_SwitchPM == offsetof(DPMI_RMCB, SwitchPM), "constant error");
static_assert(RMCB_OFF_SwitchRM == offsetof(DPMI_RMCB, SwitchRM), "constant error");
static_assert(RMCB_OFF_Translation == offsetof(DPMI_RMCB, Translation), "constant error");
static_assert(RMCB_OFF_PMSP == offsetof(DPMI_RMCB, PM_SP), "constant error");
static_assert(RMCB_OFF_RMSP == offsetof(DPMI_RMCB, RM_SP), "constant error");
static_assert(RMCB_OFF_RMSEG == offsetof(DPMI_RMCB, RM_SEG), "constant error");
static_assert(RMCB_OFF_VcpiInterface == offsetof(DPMI_RMCB, VcpiInterface), "constant error");
static_assert(RMCB_OFF_ClientWrapperCS == offsetof(DPMI_RMCB, ClientWrapperCS), "constant error");
static_assert(RMCB_OFF_VcpiClient == offsetof(DPMI_RMCB, VcpiClient), "constant error");
static_assert(RMCB_OFF_VcpiClientAddr == offsetof(DPMI_RMCB, VcpiClientAddr), "constant error");
static_assert(RMCB_OFF_CommonEntry == offsetof(DPMI_RMCB, CommonEntry), "constant error");
static_assert(RMCB_OFF_Table == offsetof(DPMI_RMCB, Table), "constant error");

//default descriptors
static const uint32_t DPMI_CodeDesc[2] = {0x0000FFFF, 0x000F9A00};
static const uint32_t DPMI_DataDesc[2] = {0x0000FFFF, 0x000F9200};
static const uint32_t DPMI_Data4GDesc[2] = {0x0000FFFF, 0x00CF9200}; //4G ds, Granularity, 32 bit
static_assert(sizeof(DPMI_Data4GDesc) == sizeof(GDT), "size error");
static_assert(sizeof(DPMI_DataDesc) == sizeof(GDT), "size error");
static_assert(sizeof(DPMI_CodeDesc) == sizeof(GDT), "size error");

typedef struct //group temporary data and allocate from DOS, exit on release. thus save the DS space after TSR
{
    GDT gdt[SEL_TOTAL];
    IDT idt[256];
    GDTR gdtr;
    IDTR idtr;

    uint32_t TempMemoryHandle;
    uint32_t LinearCS;  //real mode (initial) cs linear base
    uint32_t LinearDS;  //real mode ds linear base
    uint8_t RealModeIRQ0Vec;
    uint8_t ProtectedModeIRQ0Vec;
    uint8_t RealModeIRQ8Vec;
    uint8_t ProtectedModeIRQ8Vec;

    //below should be NULL if not v86
    uint32_t VcpiInterface; //used in V86
    uint16_t VcpiInterfaceCS; //grouped with VcpiInterface as a farcall, don't break
    uint16_t PageMemory; //dos alloc handle
    uint32_t PageTable0LAddr;
    uint32_t RMCBLAddr;
    PDE far* PageDir;   //page table
    PTE far* PageTable0;   //first 4M page, inited by VCPI
    PTE far* PageTalbeHimem;
    PTE far* PageTalbeHimem2; //in case XMS meory cross 4M boudnary
    uint16_t far* XMSPageHandle; //XMS handle for extra page tables
}DPMI_TempData;

extern "C" DPMI_ADDRESSING DPMI_Addressing;
uint32_t DPMI_XMS_Size = 64UL*1024UL;   //first 64k of system data, program data will added in run time

static uint16_t _DATA_SEG;  //first data segment for real mode
static uint16_t _CODE_SEG;  //first code segment for real mode
static uint16_t _INTR_SEG;  //a copy of real mode data segment used by dpmi_bc/interrupt
static uint16_t _STACK_PTR;
static uint32_t _DATA_SIZE; //total size of all data segments
static uint32_t _CODE_SIZE; //total size of all code segments
static uint32_t DPMI_SystemDS = 0;  //keep GDT, IDT, page table
static uint32_t DPMI_HimemCode = 0;  //starting address of himem code (also starting address of the whole program data in himem)
static uint32_t DPMI_HimemData = 0;  //starting address of himem data, right following the himem code
static uint16_t DPMI_XMSHimemHandle;
static uint16_t DPMI_XMSBelow4MHandle;
static uint8_t DPMI_V86;
static uint8_t DPMI_PM;
static uint8_t DPMI_TSRed;
static uint8_t DPMI_ExceptionPatch = FALSE;
static uint8_t DPMI_ExtDSCount = 0;
static uint8_t DPMI_ExtCSCount = 0;
static uint32_t DPMI_RmcbMemory;
static DPMI_RMCB far* DPMI_Rmcb; //can be accessed in PM (segment is SEL_RMCB_DS*8)
static DPMI_TempData far* DPMI_Temp;
static void (far*DPMI_UserINTHandler[256])(void); //TODO: software int handlers
static uint16_t DPMI_UserINTHandlerDS[256];

static uint32_t DPMI_GetSelectorBase(uint16_t selector, uint32_t* limit)
{
    const GDT far* gdt = DPMI_PM ? (GDT far*)MK_FP(SEL_SYS_DS*8, 0) : DPMI_Temp->gdt;
    //const GDT far* gdt = DPMI_PM ? (GDT far*)MK_FP(SEL_TEMP*8, offsetof(DPMI_TempData, gdt)) : DPMI_Temp->gdt; //debug: during the 1st phase when himem data not ready
    selector /= 8;
    uint32_t base = ((uint32_t)gdt[selector].base_low) |
        (((uint32_t)gdt[selector].base_middle)<<16) |
        (((uint32_t)gdt[selector].base_high)<<24);

    if(limit)
    {
        *limit = ((uint32_t)gdt[selector].limit_low) | (((uint32_t)gdt[selector].limit_high)<<16);
        *limit = (*limit + 1) * (gdt[selector].granuarity ? 4096UL : 1) - 1;
    }
    return base;
}

//convert a 16 bit far ptr to linear addr
#define DPMI_PTR16R2L(ptr) ((((uint32_t)FP_SEG((ptr)))<<4)+(uint32_t)FP_OFF((ptr)))
//convert a linear addr to a far pointer
static void far* near DPMI_L2PTR16R(uint32_t addr)
{
    for(uint16_t i = 0; i < DPMI_LOADER_DS_Count; ++i)
    {
        //addr are sorted, we iterate each to find the base
        uint32_t base = ((uint32_t)DPMI_LOADER_DS[i])<<4;
        if(addr >= base && addr-base < 0xFFFF)
            return MK_FP(DPMI_LOADER_DS[i], addr-base);
    }
    //arbitary addr?
    assert(addr <= 0xFFFFFL);//below 1M
    if(addr > 0xFFFFFL)
        return NULL;
    uint32_t seg = addr >> 4;
    uint32_t off = addr - (seg<<4);
    return MK_FP(seg, off);
}

static uint32_t near DPMI_SEGOFFP2L(uint16_t selector, uint16_t off)
{
    uint32_t base = DPMI_GetSelectorBase(selector, NULL);
    return base + (uint32_t)off;
}

#pragma off (unreferenced)
static uint32_t near DPMI_PTR16P2L(void far* ptr)
{
    return DPMI_SEGOFFP2L(FP_SEG(ptr), FP_OFF(ptr));
}

static void far* near DPMI_L2PTR16P(uint32_t addr)
{
    if(addr <= 0xFFFFFL)
    {
        for(uint16_t i = SEL_DS; i <= SEL_DSE; ++i)
        {
            uint32_t base = DPMI_GetSelectorBase(i*8, NULL);
            if(addr >= base && addr-base < 0xFFFF)
                return MK_FP(i*8, addr-base);
        }        
    }
    for(uint16_t i = SEL_HIMEM_DS; i <= SEL_HIMEM_DSE; ++i)
    {
        uint32_t base = DPMI_GetSelectorBase(i*8, NULL);
        if(addr >= base && addr-base < 0xFFFF)
            return MK_FP(i*8, addr-base);
    }
    assert(FALSE);
    return NULL;
}
#pragma pop (unreferenced)

//only works for protected mode.
static inline PDE DPMI_LoadPDE(uint32_t addr) { PDE pde; pde.value = DPMI_LoadD(addr); return pde; }
static inline PDE DPMI_LoadPDE(uint32_t addr, uint32_t i) { return DPMI_LoadPDE(addr+i*sizeof(PDE)); }
static inline void DPMI_StorePDE(uint32_t addr, const PDE* pde) { DPMI_StoreD(addr, pde->value); }
static inline void DPMI_StorePDE(uint32_t addr, uint32_t i, const PDE* pde) { DPMI_StorePDE(addr+i*sizeof(PDE), pde); }

#define DPMI_LoadPTE DPMI_LoadPDE
#define DPMI_StorePTE DPMI_StorePDE

//UMB might not be 1:1 mapped.
//#define DPMI_PTUnmap(pt, laddr) DPMI_V86 ? PTE_ADDR(DPMI_PM ? DPMI_LoadPTE(DPMI_PTR16R2L(pt), (laddr)>>12) : pt[(laddr)>>12]) : laddr; //laddr to paddr
//#define DPMI_PDUnmap(pdir, laddr) DPMI_PTUnmap(&pdir[(laddr)>>20], laddr)
//simplified to works only for real mode before init
#define DPMI_PTUnmap(pt, laddr) (DPMI_V86 ? PTE_ADDR((pt)[(laddr)>>12L]) : laddr);

//////////////////////////////////////////////////////////////////////////////
//direct (raw) mode
//////////////////////////////////////////////////////////////////////////////
#pragma disable_message (13) //WC: unreachable code, but seems invalid
#pragma option -k- //BC
static void __NAKED far DPMI_DirectProtectedMode()
{
    _ASM_BEGIN
        _ASM(pushad)
        _ASM(pushfd)
        _ASM(cli)

        //enable A20 through XMS
        _ASM(call enableA20)

        _ASM(sidt fword ptr ds:[RMCB_OFF_OldIDTR]);//';' is added to work around vscode highlight problem
        _ASM(lgdt fword ptr ds:[RMCB_OFF_GDTR]);
        _ASM(lidt fword ptr ds:[RMCB_OFF_IDTR]);
        _ASM(push SEL_RMCB_CS*8)
#if defined(__BC__)
        _ASM2(mov ax, offset reload_cs)
        _ASM2(sub ax, offset DPMI_DirectProtectedMode)
        _ASM2(add ax, ds:[RMCB_OFF_SwitchPM]);
#else
        //thanks to joncampbell123
        //https://github.com/open-watcom/open-watcom-v2/issues/1179#issuecomment-1868556806
        _ASM(call hack)
        _ASM(jmp reload_cs)
    _ASMLBL(hack:) //hack for watcom, not working with offset + asm label
        _ASM(pop ax)
        _ASM(push bx)
        _ASM(mov bx, ax)
        _ASM(add ax, word ptr cs:[bx+1]);
        _ASM(add ax, 3) //size of jmp instruction itself
        _ASM(pop bx) //it's a runtime hack using relative offset, so no need to adjust offset as for BC
#endif
        _ASM(push ax)
        _ASM2(mov eax, cr0)
        _ASM2(or al, 0x1)      //PE
        _ASM2(mov cr0, eax)
        _ASM(retf)
    _ASMLBL(reload_cs:)
        _ASM2(mov ax, SEL_RMCB_DS*8)
        _ASM2(mov ds, ax)
        _ASM2(mov es, ax)
        _ASM2(mov ss, ax)
        _ASM2(mov fs, ax)
        _ASM2(mov gs, ax)

    _ASMLBL(directPMdone:)
        _ASM(popfd)
        _ASM(popad)
        _ASM(retf)  //return successfully

    _ASMLBL(enableA20:)
        _ASM2(mov ax, 0x4300)
        _ASM(int 0x2F)
        _ASM2(cmp al, 0x80) //XMS installed?
        _ASM(jne enableA20End)
        _ASM2(mov ax, 0x4310)
        _ASM(int 0x2F) //get entry function in es:bx

        _ASM(push es)
        _ASM(push bx)
        _ASM2(mov bx, sp)
        _ASM2(mov ah, 0x07) //query a20
        _ASM(call dword ptr ss:[bx]);
        _ASM2(cmp ax, 1)
        _ASM(je enableA20End) //already enabled

        _ASM2(mov ah, 0x03) //global enable a20
        _ASM(call dword ptr ss:[bx]);
    _ASMLBL(enableA20End:)
        _ASM2(add sp, 4)
        _ASM(retn)
    _ASM_END
}
static void __NAKED DPMI_DirectProtectedModeEnd() {}

static void __NAKED far DPMI_DirectRealMode()
{
    _ASM_BEGIN
        _ASM(pushad)
        _ASM(pushfd)
        _ASM(cli)
    _ASM_END

    _ASM_BEGIN
        _ASM(lgdt fword ptr ds:[RMCB_OFF_NullGDTR]); //load null gdt
        _ASM(lidt fword ptr ds:[RMCB_OFF_OldIDTR]);
        _ASM(push word ptr ds:[RMCB_OFF_RMSEG]);
#if defined(__BC__)
        _ASM2(mov ax, offset reload_cs2)
        _ASM2(sub ax, offset DPMI_DirectRealMode)
        _ASM2(add ax, ds:[RMCB_OFF_SwitchRM]);
#else
        _ASM(call hack)
        _ASM(jmp reload_cs2)
    _ASMLBL(hack:) //hack for watcom, not working with offset + asm label
        _ASM(pop ax)
        _ASM(push bx)
        _ASM(mov bx, ax)
        _ASM(add ax, word ptr cs:[bx+1]);
        _ASM(add ax, 3) //size of jmp instruction itself
        _ASM(pop bx) //it's a runtime hack using relative offset, so no need to adjust offset as for BC
#endif
        _ASM(push ax)
        _ASM2(mov eax, cr0)
        _ASM2(and al, 0xFE)
        _ASM2(mov cr0, eax)
        _ASM(retf)
        //jmp reload_cs2 //WTF. inline asm can only perform near jump via C label.
    _ASMLBL(reload_cs2:) //reload_cs2 label far
        _ASM2(mov ax, ds:[RMCB_OFF_RMSEG]); //rmds
        _ASM2(mov ds, ax)
        _ASM2(mov es, ax)
        _ASM2(mov ss, ax)
        _ASM2(mov gs, ax)
        _ASM2(mov fs, ax)

        //note: don't disable A20 as some DOS extender doesn't like it.

        _ASM(popfd)
        _ASM(popad)
        _ASM(retf) //return successfully
    _ASM_END
}
static void __NAKED DPMI_DirectRealModeEnd() {}
#pragma option -k //BC
#pragma enable_message (13) //WC

//////////////////////////////////////////////////////////////////////////////
//VCPI mode
//////////////////////////////////////////////////////////////////////////////
static uint32_t DPMI_4MPageTableLAddr;
static uint16_t far* DPMI_XMSPageHandle; //XMS handle for extra page tables
static uint16_t DPMI_MappedPages = 0; //num physical maps
static uint16_t PageTable0Offset = 0; //offset of 4M page after VCPI init, < 1024.
#define VCPI_PAGING_MEM_SIZE (16L*1024L) //4K PD, 8K page table for XMS, 4k aligment (also for DPMI_XMSPageHandle)

//https://www.edm2.com/index.php/Virtual_Control_Program_Interface_specification_v1
uint32_t __CDECL DPMI_GetVCPIInterface(PTE far* First4M, GDT far* VcpiGDT)
{
    uint16_t table_seg = FP_SEG(First4M);
    uint16_t table_off = FP_OFF(First4M);
    uint16_t new_off = 0;
    uint16_t gdt_seg = FP_SEG(VcpiGDT);
    uint16_t gdt_off = FP_OFF(VcpiGDT);
    uint32_t interface = 0;
    uint8_t result = -1;
    _ASM_BEGIN
        _ASM(push ds)
        _ASM(push ax)
        _ASM(push es)
        _ASM(push di)
        _ASM(push si)
        _ASM(push ebx)
        _ASM2(mov ax, table_seg)
        _ASM2(mov es, ax)
        _ASM2(mov di, table_off)
        _ASM2(mov ax, gdt_seg)
        _ASM2(mov ds, ax)
        _ASM2(mov si, gdt_off)
        _ASM2(mov ax, 0xDE01)
        _ASM(int 0x67) //AH, DI, EBX modified
        _ASM2(mov dword ptr interface, ebx)
        _ASM2(mov new_off, di)
        _ASM2(mov result, ah)
        _ASM(pop ebx)
        _ASM(pop si)
        _ASM(pop di)
        _ASM(pop es)
        _ASM(pop ax)
        _ASM(pop ds)
    _ASM_END
    if(result != 0)
    {
        printf("Error: faield to get VCPI interface.\n");
        exit(1);
    }
    assert(sizeof(PTE) == 4);
    PageTable0Offset = (new_off - table_off)/sizeof(PTE);
    #if 0 //dump vcpi pages
    {
        int start = 0;
        for(uint32_t j = start; j < PageTable0Offset; ++j)
            _LOG(((((j-start+1)%8)==0)?"%08lx\n":"%08lx "), (*(uint32_t far*)(First4M+j)));
        _LOG("\n");
    }
    #endif
    return interface;
}

static BOOL DPMI_InitVCPI()
{
    assert(DPMI_Temp);
    if(DPMI_Temp->VcpiInterface)
        return TRUE;

    volatile BOOL VCPIPresent = FALSE;
    _ASM_BEGIN //check vcpi
        _ASM(push ax)
        _ASM(push bx)
        _ASM2(mov ax, 0xDE00)
        _ASM(int 0x67) //AH, BX modified
        _ASM(not ax)
        _ASM2(mov VCPIPresent, ax)
        _ASM(pop bx)
        _ASM(pop ax)
    _ASM_END

    if(!VCPIPresent)
    {
        printf("Error: Virtual 8086 mode detected but no VCPI.\n");
        exit(1);
    }
    DPMI_Temp->PageMemory = 0; //dos alloc handle
    DPMI_Temp->PageDir = NULL; //page table
    DPMI_Temp->PageTable0 = NULL;   //first 4M page
    DPMI_Temp->PageTalbeHimem = NULL;   //mapped for himem
    DPMI_Temp->XMSPageHandle = NULL; //page table for physical maps
    //prepare paging, used dos malloc to save space for the driver. it will be freed on TSR (execept interrupt handler needed)
    DPMI_Temp->PageMemory = (uint16_t)DPMI_HighMalloc((VCPI_PAGING_MEM_SIZE+15)>>4L, FALSE);
    _LOG("Page memory: %08x\n", DPMI_Temp->PageMemory);
    if(DPMI_Temp->PageMemory == 0)
    {
        printf("Error: Failed to allocate memory.\n");
        exit(1);
    }
    uint16_t seg = (uint16_t)align(DPMI_Temp->PageMemory, 4096>>4);
    DPMI_Temp->PageDir = (PDE far*)MK_FP(seg, 0);
    DPMI_Temp->PageTalbeHimem = (PTE far*)(DPMI_Temp->PageDir + 1024);
    DPMI_Temp->PageTalbeHimem2 = (PTE far*)(DPMI_Temp->PageTalbeHimem + 1024);
    uint32_t PageTable0Seg = DPMI_Temp->PageTable0LAddr>>4;
    DPMI_Temp->PageTable0 = (PDE far*)MK_FP(PageTable0Seg, 0);
    assert((DPMI_PTR16R2L(DPMI_Temp->PageDir)&0xFFF)==0);
    _fmemset(DPMI_Temp->PageDir, 0, 4096);
    _fmemset(DPMI_Temp->PageTable0, 0, 4096);
    _fmemset(DPMI_Temp->PageTalbeHimem, 0, 4096);
    _fmemset(DPMI_Temp->PageTalbeHimem2, 0, 4096);
    //find an alignment gap to place xms map handle
    uint32_t PageDirLAddr = DPMI_PTR16R2L(DPMI_Temp->PageDir);
    if(PageDirLAddr - ((uint32_t)DPMI_Temp->PageMemory<<4L) >= 1024L*sizeof(uint16_t))
    {
        DPMI_Temp->XMSPageHandle = (uint16_t far*)MK_FP(DPMI_Temp->PageMemory,0);
        assert(DPMI_PTR16R2L(DPMI_Temp->XMSPageHandle+1024) <= PageDirLAddr);
    }
    else
    {
        DPMI_Temp->XMSPageHandle = (uint16_t far*)(DPMI_Temp->PageTalbeHimem2 + 1024);
        assert(DPMI_PTR16R2L(DPMI_Temp->XMSPageHandle) >= DPMI_PTR16R2L(DPMI_Temp->PageTalbeHimem2)+4096L);
        assert(DPMI_PTR16R2L(DPMI_Temp->XMSPageHandle+1024) <= ((uint32_t)DPMI_Temp->PageMemory<<4)+VCPI_PAGING_MEM_SIZE);
    }
    _fmemset(DPMI_Temp->XMSPageHandle, 0, 1024L*sizeof(uint16_t));
    DPMI_XMSPageHandle = DPMI_Temp->XMSPageHandle;

    //dummy. not used, make it looks valid. (with invalid 0 base)
    GDT far* Gdt = DPMI_Temp->gdt;
    Gdt[SEL_LDT].limit_low = sizeof(LDT)-1;
    Gdt[SEL_LDT].read_write = 1;
    Gdt[SEL_LDT].present = 1;
    //TODO: TSS type in descriptor.
    Gdt[SEL_TSS].limit_low = sizeof(TSS32)-1;
    Gdt[SEL_TSS].type = 1; //0x89
    Gdt[SEL_TSS].accessed = 1;
    Gdt[SEL_TSS].present = 1;

    DPMI_Temp->VcpiInterface = DPMI_GetVCPIInterface(DPMI_Temp->PageTable0, &Gdt[SEL_VCPI_CS]);
    DPMI_Temp->VcpiInterfaceCS = SEL_VCPI_CS*8;

    uint32_t pt0 = DPMI_PTUnmap(DPMI_Temp->PageTable0, DPMI_PTR16R2L(DPMI_Temp->PageTable0));
    _LOG("Page table 0: %08lx %08lx\n", pt0, DPMI_PTR16R2L(DPMI_Temp->PageTable0));
    _LOG("Page dir: %08lx\n", DPMI_PTR16R2L(DPMI_Temp->PageDir));

    assert((pt0&0xFFF) == 0);
    PDE pde = PDE_INIT(pt0);
    DPMI_Temp->PageDir[0] = pde;

    //init himem page after 4M page table inited by vcpi
    {
        uint32_t pdi = DPMI_SystemDS >> 22L;
        if(pdi == 0) //himem below 4M
            pdi = 1; //append next to pt0 in case himem cross table boundary
        //pre-add pt
        {
            uint32_t pt = DPMI_PTUnmap(DPMI_Temp->PageTable0, DPMI_PTR16R2L(DPMI_Temp->PageTalbeHimem));
            assert((pt&0xFFF) == 0);
            PDE pde = PDE_INIT(pt);
            DPMI_Temp->PageDir[pdi] = pde;
        }
        {
            uint32_t pt = DPMI_PTUnmap(DPMI_Temp->PageTable0, DPMI_PTR16R2L(DPMI_Temp->PageTalbeHimem2));
            assert((pt&0xFFF) == 0);
            PDE pde = PDE_INIT(pt);
            DPMI_Temp->PageDir[pdi+1] = pde;
        }
        DPMI_MapMemory(DPMI_SystemDS, DPMI_XMS_Size);
    }
    //CLI; //soft int will set IF, need cli if called in DPMI_VCPIProtectedMode().
    return TRUE;
}

#pragma disable_message (13) //WC: unreachable code, but seems invalid
#pragma option -k- //BC
static void __NAKED far DPMI_VCPIProtectedMode()
{
    //note: DO NOT access global data, since it's executed in isolated real mode segment
    _ASM_BEGIN
        _ASM(push eax)
        _ASM(push ebx)
        _ASM(push ecx)
        _ASM(push esi)
        _ASM(pushf)
        _ASM(cli)

        _ASM2(xor ecx, ecx)
#if defined(__BC__)
        _ASM2(mov cx, offset VCPI_PMDone) //mov ebx, offset _VCPI_PMDone //linking error
        _ASM2(sub cx, offset DPMI_VCPIProtectedMode)
        _ASM2(add cx, ds:[RMCB_OFF_SwitchPM]);
#else
        _ASM(call hack)
        _ASM(jmp VCPI_PMDone)
    _ASMLBL(hack:) //hack for watcom, not working with offset + asm label
        _ASM(pop cx)
        _ASM(push bx)
        _ASM(mov bx, cx)
        _ASM(add cx, word ptr cs:[bx+1]);
        _ASM(add cx, 3) //size of jmp instruction itself
        _ASM(pop bx) //it's a runtime hack using relative offset, so no need to adjust offset as for BC
#endif
        _ASM2(mov dword ptr ds:[RMCB_OFF_VcpiClient+VCPI_CLIENTSTRUCT_OFF_EIP], ecx);

        _ASM2(mov esi, dword ptr ds:[RMCB_OFF_VcpiClientAddr]);
        _ASM2(mov ecx, esp) //save SP to CX
        _ASM2(mov ax, 0xDE0C)
        _ASM(int 0x67) //EAX, ESI, DS ES FS GS modified
    _ASMLBL(VCPI_PMDone:)
        _ASM2(mov ax, SEL_RMCB_DS*8)
        _ASM2(mov ds, ax)
        _ASM2(mov es, ax)
        _ASM2(mov ss, ax)
        _ASM2(mov fs, ax)
        _ASM2(mov gs, ax)
        _ASM2(mov esp, ecx) //restore to SP (point at pushf before cli)

        _ASM(push cx)
    _ASM_END

    _ASM_BEGIN
        _ASM(pop bx)

        _ASM(pop ax)  //note: V86 mode change involves flags change. don't popf directly, only keep IF. or VCPI will crash on return v86
        _ASM2(and ax, CPU_IFLAG)//IF
        _ASM(pushf)
        _ASM2(or word ptr ss:[bx], ax); //BX==SP
        _ASM(popf)

        _ASM(pop esi)
        _ASM(pop ecx)
        _ASM(pop ebx)
        _ASM(pop eax)
        _ASM(retf)
    _ASM_END
}
static void __NAKED DPMI_VCPIProtectedModeEnd() {}

static void __NAKED far DPMI_VCPIRealMode() //VCPI PM to RM. input ds=ss=SEL_RMCB_DS*8,cs=SEL_RMCB_CS*8
{
    //http://www.edm2.com/index.php/Virtual_Control_Program_Interface_specification_v1#5.2_Switch_to_V86_Mode

    _ASM_BEGIN
        _ASM(push eax)
        _ASM(push ebx)
        _ASM(pushf)
        _ASM(cli)

        _ASM2(movzx ebx, sp)
        _ASM(clts) //clear TS bits. we have no task.
        _ASM2(xor eax, eax)
        _ASM2(mov ax, ds:[RMCB_OFF_RMSEG]); //ds:[RM_SEG], ds should be SEL_RMCB_DS*8

        _ASM(push eax)    //GS
        _ASM(push eax)    //FS
        _ASM(push eax)    //DS
        _ASM(push eax)    //ES
        _ASM(push eax)    //SS
        _ASM(push ebx)    //ESP
        _ASM(pushfd)      //EFLAGS, not used, filled by VCPI
        _ASM(push eax)    //CS
        _ASM2(mov ax, ds)
        _ASM2(mov es, ax)
#if defined(__BC__)
        _ASM2(mov ax, offset VCPI_to_real) //mov eax, offset back_to_real //link err
        _ASM2(sub ax, offset DPMI_VCPIRealMode)
        _ASM2(add ax, es:[RMCB_OFF_SwitchRM]);
#else
        _ASM(call hack)
        _ASM(jmp VCPI_to_real)
    _ASMLBL(hack:) //hack for watcom, not working with offset + asm label
        _ASM(pop ax)
        _ASM(push bx)
        _ASM(mov bx, ax)
        _ASM(add ax, word ptr cs:[bx+1]);
        _ASM(add ax, 3) //size of jmp instruction itself
        _ASM(pop bx) //it's a runtime hack using relative offset, so no need to adjust offset as for BC
#endif
        _ASM(push eax)    //EIP

        _ASM2(mov ax, SEL_4G*8)
        _ASM2(mov ds, ax)
        _ASM2(mov ax, 0xDE0C)

        _ASM(call fword ptr es:[RMCB_OFF_VcpiInterface]); //EAX modified (and segments loaded from stack)
    _ASMLBL(VCPI_to_real:)
        _ASM(pop ax)
        _ASM2(and ax, CPU_IFLAG) //IF
        _ASM(pushf)
        _ASM2(or word ptr ss:[bx], ax); //BX=SP

        _ASM(popf)
        _ASM(pop ebx)
        _ASM(pop eax)
        _ASM(retf)
    _ASM_END
}

static void __NAKED DPMI_VCPIRealModeEnd() {}
#pragma option -k //BC
#pragma enable_message (13) //WC

//////////////////////////////////////////////////////////////////////////////
//segment selector mapping
//////////////////////////////////////////////////////////////////////////////
static inline uint16_t DPMI_GetCodeSelector(uint16_t segment)
{
    return (SEL_HIMEM_CS + DPMI_LOADER_GetCSIndex(segment))<<3;
}
static inline uint16_t DPMI_GetDataSelector(uint16_t segment)
{
    return (SEL_HIMEM_DS + DPMI_LOADER_GetDSIndex(segment))<<3;
}
static inline uint16_t DPMI_GetCodeSegment(uint16_t selector)
{
    return DPMI_LOADER_GetCS((selector>>3) - SEL_HIMEM_CS);
}
static inline uint16_t DPMI_GetDataSegment(uint16_t selector)
{
    if((selector >= SEL_EXT_DS*8 && selector <= SEL_EXT_DSE*8) || (selector >= SEL_EXT_CS*8 && selector <= SEL_EXT_CSE*8))
        return (uint16_t)(DPMI_GetSelectorBase(selector,NULL)>>4);
    else if(selector == SEL_INTR_DS*8)
        return _INTR_SEG;
    else
        return DPMI_LOADER_GetDS((selector>>3) - SEL_HIMEM_DS);
}
#if DEBUG
#pragma off (unreferenced)
static void DPMI_DumpSelector(uint16_t selector)
{
    uint32_t limit = 0;
    uint32_t base = DPMI_GetSelectorBase(selector, &limit);
    _LOG("sel: %x, base: %08lx, limit: %08lx\n", selector, base, limit);
}
#pragma pop (unreferenced)
#endif

//////////////////////////////////////////////////////////////////////////////
//mode switch
//////////////////////////////////////////////////////////////////////////////
//swtich to protected mode.
static int16_t __CDECL near DPMI_SwitchProtectedMode()
{ //care on printf msg in mode switch, because printf will switch back and forth and cause recursion
    if(DPMI_PM)
        return DPMI_PM;
    if(!DPMI_V86)
    { //add pre-test for A20. during the real mode switching in RMCB, there's no error torlances
        if(!XMS_EnableA20())
        {
            printf("Error enabling A20.\n");
            exit(1);
        }
    }

    //map real mode segments to protected mode selectors
    //_LOG("ds: %04x, ss: %04x, cs: %04x\n",_DS,_SS,_CS);
    uint16_t ds_sel = DPMI_GetDataSelector(_DS);
    uint16_t ss_sel = DPMI_GetDataSelector(_SS);
    uint16_t cs_sel = DPMI_GetCodeSelector(_CS);
    //switch to conventional memory first, then copy conventional memory to himem, finally switch to himem
    uint16_t lowds_sel = (SEL_DS + DPMI_LOADER_GetDSIndex(_DS))<<3;
    uint16_t lowss_sel = (SEL_DS + DPMI_LOADER_GetDSIndex(_SS))<<3;
    uint16_t lowcs_sel = (SEL_CS + DPMI_LOADER_GetCSIndex(_CS))<<3;
    uint16_t intr_ds = DPMI_GetDataSelector(_INTR_SEG);
    //_LOG("ds: %04x %04x, ss: %04x %04x, cs: %04x, %04x\n",_DS,ds_sel,_SS,ss_sel,_CS,cs_sel);
    //DPMI_DumpSelector(SEL_INTR_DS*8);

    CLIS();
    //patch low mem in place, otherwise extern functions are not callable after switching to PM
    DPMI_LOADER_PatchRM(_CODE_SIZE, _CODE_SEG, SEL_CS*8, SEL_DS*8, _CODE_SIZE+_DATA_SIZE);
    //DO NOT access any static data befoe switch, because this may be called from a RMCB block from outside
    _ASM_BEGIN
        _ASM(push ax)
        _ASM(push bx)
        _ASM(push cx)
        _ASM(push bp)

        _ASM2(mov ax, lowcs_sel) //load auto vars to registers
        _ASM2(mov cx, lowss_sel)
        _ASM2(mov bp, sp)
        _ASM2(lss bx, DPMI_Rmcb); //changing stack
        _ASM2(mov sp, word ptr ss:[bx + RMCB_OFF_RMSP]);//stack changed, no access to auto vars/parameters from now on
        _ASM(push cx)

        _ASM(push ax)
#if defined(__BC__)
        _ASM(push offset SwitchPMDone)
#else
        _ASM(call hack)
        _ASM(jmp SwitchPMDone)
    _ASMLBL(hack:) //hack for watcom, not working with offset + asm label
        _ASM(push bp)
        _ASM(mov bp, sp)
        _ASM(xchg bx, word ptr ss:[bp+2]);
        _ASM(add bx, word ptr cs:[bx+1]);
        _ASM(add bx, 3) //size of jmp instruction itself
        _ASM(xchg bx, word ptr ss:[bp+2]);
        _ASM(pop bp)
#endif
        _ASM(push ss) //rmcb cs=ss
        _ASM(push word ptr ss:[bx + RMCB_OFF_SwitchPM]);
        _ASM2(mov ax, ss)
        _ASM2(mov ds, ax)
        _ASM(retf)
    _ASMLBL(SwitchPMDone:)
        _ASM(pop ax) //lowss_sel
        _ASM2(mov ss, ax)
        _ASM2(mov sp, bp) //back to function stack
        _ASM(pop bp) //back to function stack frame, access to auto vars/paramters available

        _ASM2(mov ax, lowds_sel)
        _ASM2(mov ds, ax)
        _ASM2(mov es, ax)
        _ASM2(mov fs, ax)
        _ASM2(mov gs, ax)
        _ASM(pop cx)
        _ASM(pop bx)
        _ASM(pop ax)
    _ASM_END

    DPMI_PM = DPMI_V86 ? PM_VCPI : PM_DIRECT;
    DPMI_Rmcb = (DPMI_RMCB far*)MK_FP(SEL_RMCB_DS*8, 0);
    DPMI_Addressing.physical = TRUE;        //linear=physical
    DPMI_Addressing.selector = SEL_4G*8;    //4G ds

    DPMI_CopyLinear(DPMI_HimemCode, ((uint32_t)_CODE_SEG)<<4, _CODE_SIZE+_DATA_SIZE); //copy code & data to himem
    DPMI_LOADER_Unpatch(_CODE_SIZE, SEL_CS*8, SEL_CSE*8, SEL_DS*8, SEL_DSE*8, DPMI_HimemCode, _CODE_SIZE+_DATA_SIZE); //remove previous copied patch
    DPMI_LOADER_Patch(_CODE_SIZE, _CODE_SEG, SEL_HIMEM_CS*8, SEL_HIMEM_DS*8, DPMI_HimemCode, _CODE_SIZE+_DATA_SIZE); //apply himem patch

    //switch to himem selectors
    _ASM_BEGIN//switch immediately after copy, or the stack won't match
        _ASM2(mov ax, ds_sel)
        _ASM2(mov ds, ax)
        _ASM2(mov es, ax)
        _ASM2(mov fs, ax)
        _ASM2(mov gs, ax)
        _ASM2(mov ax, ss_sel)
        _ASM2(mov ss, ax)

        _ASM(push word ptr cs_sel)
#if defined(__BC__)
        _ASM(push offset SwitchHimemCode)
#else
        _ASM(call hack)
        _ASM(jmp SwitchHimemCode)
    _ASMLBL(hack:) //hack for watcom, not working with offset + asm label
        _ASM(push bp)
        _ASM(mov bp, sp)
        _ASM(xchg bx, word ptr ss:[bp+2]);
        _ASM(add bx, word ptr cs:[bx+1]);
        _ASM(add bx, 3) //size of jmp instruction itself
        _ASM(xchg bx, word ptr ss:[bp+2]);
        _ASM(pop bp)
#endif
        _ASM(retf)
    _ASMLBL(SwitchHimemCode:)
    _ASM_END

    //modify SEL_INTRL_DS immediate after switching to himem stack, for small model needs SS=DS to work, otherwise exception handler won't work properly
    GDT far* gdt = DPMI_Temp != NULL ? (GDT far*)MK_FP(SEL_TEMP*8, 0) : (GDT far*)MK_FP(SEL_SYS_DS*8, 0);
    gdt[SEL_INTR_DS] = gdt[intr_ds>>3];

    STIL();
    return DPMI_PM;
}

//back to real/v86 mode. this function only should be called on init or exit
//for swtich modes in between execution, use DPMI_CallRealMode* functions.
static void __CDECL near DPMI_SwitchRealMode()
{
    if(!DPMI_PM)
        return;
    CLIS();

    uint16_t RmcbSeg = DPMI_Rmcb->RM_SEG; //save on stack. DPMI_Rmcb won't be accessible after switching mode
    //_LOG("rmcb: %04x, %04x\n", FP_SEG(DPMI_Rmcb), RmcbSeg);
    //map protected mode selectors to real mode segments
    uint16_t StackSeg = DPMI_GetDataSegment(_SS);
    uint16_t DataSeg = DPMI_GetDataSegment(_DS);
    uint16_t CodeSeg = DPMI_GetCodeSegment(_CS);
    uint16_t lowds_sel = (SEL_DS + DPMI_LOADER_GetDSIndex(DataSeg))<<3;
    uint16_t lowss_sel = (SEL_DS + DPMI_LOADER_GetDSIndex(StackSeg))<<3;
    //_LOG("ds: %04x %04x, ss: %04x %04x, cs: %04x, %04x\n",_DS, DataSeg,_SS, StackSeg,_CS, CodeSeg);

    DPMI_CopyLinear(((uint32_t)_CODE_SEG)<<4, DPMI_HimemCode, _CODE_SIZE+_DATA_SIZE); //copy code & data back
    DPMI_LOADER_Unpatch(_CODE_SIZE, SEL_HIMEM_CS*8, SEL_HIMEM_CSE*8,  SEL_HIMEM_DS*8, SEL_HIMEM_DSE*8, ((uint32_t)_CODE_SEG)<<4, _CODE_SIZE+_DATA_SIZE); //remove previous copied patch
    //switch on copied data, himem data discarded
    _ASM_BEGIN
        _ASM2(mov ax, lowss_sel)
        _ASM2(mov ss, ax)
        _ASM2(mov ax, lowds_sel)
        _ASM2(mov ds, ax)
        _ASM2(mov es, ax)
        _ASM2(mov fs, ax)
        _ASM2(mov gs, ax)
    _ASM_END

    _ASM_BEGIN
        _ASM(push ax)
        _ASM(push bx)
        _ASM(push cx)
        _ASM2(mov ax, StackSeg) //load autovars to regsters
        _ASM2(mov cx, CodeSeg)  //before switching stack

        _ASM(push bp)
        _ASM2(mov bp, sp) //stack frame uanvailable
        _ASM2(mov bx, SEL_RMCB_DS*8)
        _ASM2(mov ss, bx)
        _ASM2(mov ss:[RMCB_OFF_PMSP], sp);
        _ASM2(mov sp, ss:[RMCB_OFF_RMSP]); //stack changed

        _ASM(push ax)
        //push real mode far return address from RMCB_OFF_SwitchRM
        _ASM(push cx)
#if defined(__BC__)
        _ASM(push offset SwitchRMDone)
#else
        _ASM(call hack)
        _ASM(jmp SwitchRMDone)
    _ASMLBL(hack:) //hack for watcom, not working with offset + asm label
        _ASM(push bp)
        _ASM(mov bp, sp)
        _ASM(xchg bx, word ptr ss:[bp+2]);
        _ASM(add bx, word ptr cs:[bx+1]);
        _ASM(add bx, 3) //size of jmp instruction itself
        _ASM(xchg bx, word ptr ss:[bp+2]);
        _ASM(pop bp)
#endif
        //call pm entry of RMCB_OFF_SwitchRM by push cs+ip and retf
        _ASM(push SEL_RMCB_CS*8)
        _ASM(push word ptr ss:[RMCB_OFF_SwitchRM]);
        _ASM2(mov ax, ss)
        _ASM2(mov ds, ax)
        _ASM(retf)
    _ASMLBL(SwitchRMDone:)

        _ASM(pop ax) //pushed StackSeg
        _ASM2(mov ss, ax)
        _ASM2(mov sp, bp) //switching from real mode RMCB stack to real mode stack
        _ASM(pop bp) //restore stack frame

        _ASM2(mov ax, DataSeg)
        _ASM2(mov ds, ax)
        _ASM2(mov es, ax)
        _ASM2(mov fs, ax)
        _ASM2(mov gs, ax)

        _ASM(pop cx)
        _ASM(pop bx)
        _ASM(pop ax)
    _ASM_END

    //note: only changing settings in conventional mem, the himem copy not changed (as DPMI_PM=true)
    DPMI_Rmcb = (DPMI_RMCB far*)MK_FP(RmcbSeg, 0);
    DPMI_PM = PM_NONE;
    DPMI_Addressing.selector = 0;
    STIL();
}

//////////////////////////////////////////////////////////////////////////////
//IDT
//////////////////////////////////////////////////////////////////////////////
static uint16_t DPMI_ExceptAddSegment(uint16_t segment, BOOL data)
{
    GDT far* gdt = (GDT far*)MK_FP(SEL_SYS_DS*8, 0);
    uint8_t& count = data ? DPMI_ExtDSCount : DPMI_ExtCSCount;
    int base = data ? SEL_EXT_DS : SEL_EXT_CS;
    for(int i = 0; i < count; ++i)
    {
        if((DPMI_GetSelectorBase((base+i)*8, NULL)>>4) == segment)
            return (base+i)*8;
    }
    assert(count < DPMI_SELEXT_MAX);
    uint16_t index = base + count;
    _fmemcpy(&gdt[index], data ? DPMI_DataDesc : DPMI_CodeDesc, sizeof(GDT));
    uint32_t baseaddr = ((uint32_t)segment)<<4;
    gdt[index].base_low = (uint16_t)baseaddr;
    gdt[index].base_middle = (uint8_t)(baseaddr>>16);
    uint16_t selector = index*8;
    ++count;
    _LOG("add %s selector: %x: %x\n", data?"DATA":"CODE", selector, segment);
    return selector;
}

static uint16_t DPMI_ExceptFindOrAndSegment(uint16_t segment, BOOL data)
{
    if(((segment&0x7) == 0 && segment >= SEL_CS*8 && segment <= SEL_HIMEM_DSE*8)) //already a selector?
        return segment;
    int count = data ? DPMI_ExtDSCount : DPMI_ExtCSCount;
    int base = data ? SEL_EXT_DS : SEL_EXT_CS;
    if(((segment&0x7) == 0 && segment >= base*8 && segment < (base+count)*8)) //newly added?
    {
        if((DPMI_GetSelectorBase(segment,NULL)>>4) == segment) //extactly the same one
            return segment;
    }
    uint16_t selector;
    int index = data ? DPMI_LOADER_FindDSIndex(segment) : DPMI_LOADER_FindCSIndex(segment);
    if(index == -1)
        selector = DPMI_ExceptAddSegment(segment, data);
    else
        selector = data ? DPMI_GetDataSelector(segment) : DPMI_GetCodeSelector(segment);
    return selector;
}

struct DPMI_Opcodes
{
    const char* name;
    uint8_t bytes[2];
    uint8_t len;
    uint8_t faraddr; //0,1
    uint8_t modifyCS;
    uint8_t stackonly;
};

static BOOL DPMI_ExceptPatchSegment(DPMI_REG far* r, const uint8_t far* csip, uint16_t far* sssp)
{
#if DEBUG0
    _ASM_BEGIN
        _ASM(push ss)
        _ASM(push r)
        _ASM(call DBG_DumpREG)
        _ASM2(add sp, 4)
    _ASM_END
    int base = 0;
    _LOG("CS:IP:");
    {for(int i = 0; i < 16; ++i)
        _LOG(" %02x", csip[base+i]);}
    _LOG("\n");
    _LOG("SS:SP:");
    {for(int i = 0; i < 8; ++i)
        _LOG(" %02x", sssp[i]);}
    _LOG("\n");
#endif

    BOOL Continue = FALSE;
    const int OPCODE_FCALL = 8;
    const int OPCODE_MOVSREG = 9;
    static const DPMI_Opcodes opcodes[] = 
    {
        {"unkown", {0}, 2, 0, 0, 0},
        //stack operations at first
        {"retf", {0xCB}, 1, 1, 1, 1},
        {"iret", {0xCF}, 1, 1, 1, 1},
        {"pop ds", {0x1F}, 1, 0, 0, 1},
        {"pop es", {0x07}, 1, 0, 0, 1},
        {"pop ss", {0x17}, 1, 0, 0, 1},
        {"pop fs", {0x0F, 0xA1}, 2, 0, 0, 1},
        {"pop gs", {0x0F, 0xA9}, 2, 0, 0, 1},

        {"call far", {0xFF}, 1, 1, 1, 0}, //call far 0xFF ModRM r(3), need special check, don't change the location as OPCODE_FCALL
        {"mov sreg", {0x8E}, 1, 0, 0, 0}, //don't change the location as OPCODE_MOVSREG
        {"lds", {0xC5}, 1, 1, 0, 0},
        {"les", {0xC4}, 1, 1, 0, 0},
        {"lss", {0x0F, 0xB2}, 2, 1, 0, 0},
        {"lfs", {0x0F, 0xB4}, 2, 1, 0, 0},
        {"lgs", {0x0F, 0xB5}, 2, 1, 0, 0},
    };
    int prefix = 0;
    int opcode = 0;
    int modrm = 0;
    int operand = 0;
    int faraddr = 0;
    for(int i  = 0; (i < _countof(opcodes)) && !opcode; ++i)
    {
        for(int j = 0; j <= (opcodes[i].stackonly ? 0 : 1); ++j) //don't check prefix for stack ops
        {
            if(_fmemcmp(csip+j, opcodes[i].bytes, opcodes[i].len) == 0)
            {
                modrm = opcodes[i].stackonly ? 0 : csip[opcodes[i].len+j];
                if((i == OPCODE_FCALL) && (((modrm>>3)&0x7) != 3)) continue; //special case for call far R=3
                prefix = j == 0 ? prefix : csip[0];
                operand = opcodes[i].stackonly ? 0 : opcodes[i].len + j + 1;
                faraddr = opcodes[i].faraddr;
                opcode = i;
                break;
            }
        }
    }
    if(opcode == 0)
        return FALSE;
    Continue = TRUE;
    if(opcodes[opcode].stackonly)
    {
        uint16_t far* segaddr = sssp;
        uint16_t selector = DPMI_ExceptFindOrAndSegment(segaddr[faraddr], !opcodes[opcode].modifyCS);
        _LOG("%s: segment addr: %04x->%04x\n", opcodes[opcode].name, segaddr[faraddr], selector);
        Continue = segaddr[faraddr] != selector;
        segaddr[faraddr] = selector;
        return Continue;
    }
    assert(operand != 0);

    //https://wiki.osdev.org/X86-64_Instruction_Encoding#ModR.2FM
    //displacements
    #define I16 (*(uint16_t far*)&csip[operand])
    #define I8 (csip[operand])
    #define BX (r->w.bx)
    #define SI (r->w.si)
    #define DI (r->w.di)
    #define BP (r->w.bp)
    #define AX (r->w.ax)
    #define CX (r->w.cx)
    #define DX (r->w.dx)
    #define SP (r->w.sp)
    uint16_t tables[3][8] =
    {
        {BX+SI,     BX+DI,      BP+SI,      BP+DI,      SI,     DI,     I16,    BX,},
        {BX+SI+I8,  BX+DI+I8,   BP+SI+I8,   BP+DI+I8,   SI+I8,  DI+I8,  BP+I8,  BX+I8,},
        {BX+SI+I16, BX+DI+I16,  BP+SI+I16,  BP+DI+I16,  SI+I16, DI+I16, BP+I16, BX+I16,},
    };
    uint16_t far* table3[8] = {&AX,        &CX,         &DX,         &BX,         &SP,     &BP,     &SI,     &DI};
    static const char* segnames[8] = {"es", "cs", "ss", "ds", "fs", "gs", "invalid", "invalid"}; unused(segnames);

    uint8_t mod = (uint8_t)(modrm >> 6);
    uint8_t reg = (uint8_t)((modrm>>3)&0x7);
    uint8_t rm = (uint8_t)(modrm&0x7);
    if(mod == 3) //move reg to segments
    {
        assert(opcode == OPCODE_MOVSREG);
        uint16_t selector = DPMI_ExceptFindOrAndSegment(*table3[rm], TRUE/*cannot mov to cs directly*/);
        Continue = *table3[rm] != selector;
        _LOG("%s: %x->%x", opcodes[opcode].name, *table3[rm], selector);
        *table3[rm] = selector; //modify registers
        return Continue;
    }

    uint16_t offset = tables[mod][rm];
    uint16_t far* addr;
    //default segment: BP use ss, other use ds
    if(prefix == 0 ) prefix = ((mod < 3 && (rm == 2 || rm == 3)) || (mod == 1 || mod == 2) && rm == 6) ? 0x36 : 0x3E;
    switch(prefix)
    {
        case 0x2E: //cs
        addr = (uint16_t far*)MK_FP(r->w.cs, offset); break;
        case 0x36: //ss
        addr = (uint16_t far*)MK_FP(r->w.ss, offset); break;
        case 0x3E: //ds
        addr = (uint16_t far*)MK_FP(r->w.ds, offset); break;
        case 0x26: //es
        addr = (uint16_t far*)MK_FP(r->w.es, offset); break;
        case 0x64: //fs
        addr = (uint16_t far*)MK_FP(r->w.fs, offset); break;
        default: assert(FALSE); Continue = FALSE; break;
    }
    if(Continue)
    {
        //note: no need to check whether prefix segment is valid, if it is not, an exception will occur on previouly loading it.
        _LOG("addr: %x %x %x\n", addr[faraddr], SEL_CS*8, SEL_HIMEM_DSE*8);
        uint16_t selector = DPMI_ExceptFindOrAndSegment(addr[faraddr], !opcodes[opcode].modifyCS);
        _LOG("prefix %02x ofset: %04x %s: %s addr: %04x->%04x\n", prefix, offset, opcodes[opcode].name, opcode == OPCODE_MOVSREG ? segnames[reg] : "segment", addr[faraddr], selector);
        Continue = addr[faraddr] != selector;
        //code segment (i.e. mov es, cs:[bx]) cannot be modified (RE without W), use 4G to write.
        DPMI_StoreW(DPMI_PTR16P2L(addr+faraddr), selector);//addr[faraddr] = selector;
    }
    return Continue;
}

static void near DPMI_Shutdown(void);

//dump exception and also try to fix it
//the expn will be modified to 0xFFFF if exception is fixed and may try to continue
extern "C" void __CDECL near DPMI_Except(short expn, short error, short ip_, short cs_, short flags)
{
    unused(expn);unused(error);unused(flags);
    #define OFF 14 //offset after flags. current stack: bp(setup by compiler), return addr, expn, error, ip_, cs_, flags
    _ASM_BEGIN
        //save context, also make a DPMI_REG ptr for dump
        _ASM(push ss)
        _ASM2(add bp, OFF)
        _ASM(push bp) //ss: sp
        _ASM2(sub bp, OFF)
        _ASM(push word ptr cs_)
        _ASM(push word ptr ip_)
        _ASM(push gs)
        _ASM(push fs)
        _ASM(push ds)
        _ASM(push es)
        _ASM(pushf)
        _ASM(pushad) //<- DPMI_REG ptr

        _ASM(push ax)
        _ASM(push bx)
        _ASM2(mov bx, sp)
        _ASM2(add bx, 4) //bx = DPMI_REG ptr
        _ASM2(mov ax, word ptr [bp]); //set the right cient bp before exception, for debug dump.
        _ASM2(mov word ptr ss:[bx+DPMI_REG_OFF_EBP], ax);
        _ASM2(mov ax, SEL_INTR_DS*8) //DBG_DumpREG may also use SEL_INTR_DS for small/medium model
        _ASM2(mov ds, ax)
        _ASM(pop bx)
        _ASM(pop ax)
    _ASM_END

    const uint8_t far* csip = (uint8_t far*)MK_FP(cs_, ip_); unused(csip);
    uint8_t far* ssbp = (uint8_t far*)MK_FP(_SS, _BP);
    BOOL Continue = FALSE;
    if(expn == 0x0D && DPMI_ExceptionPatch)
    {
        DPMI_REG far* reg = (DPMI_REG far*)MK_FP(_SS, _SP);
        //BC startup/terminate routines stores return address in memory, patching will not help
        //we patch it here in exception handler
        Continue = DPMI_ExceptPatchSegment(reg, csip, (uint16_t far*)(ssbp+OFF));
        //flags |= CPU_TFLAG; //single step DEBUG
    }
#if DEBUG //single step debug
    else if(expn == 3) //int 3 triggers single step debug mode.
    {
        //note: flags/ip are the real ones pushed on exception, because we use the exception stack frame directly in this function
        flags |= CPU_TFLAG; 
        Continue = TRUE;
    }
    else if(expn == 1 && (flags&CPU_TFLAG))
    {
        _ASM_BEGIN
            _ASM2(mov ax, sp)
            _ASM(push ss) //push ss for compact/large model. for small/medium it is ignored beause DBG_DumpREG has ONLY 1 parameter
            _ASM(push ax)
            _ASM(call DBG_DumpREG)
            _ASM2(add sp, 4)
        _ASM_END
        int base = 0;
        _LOG("CS:IP:");
        {for(int i = 0; i < 16; ++i)
            _LOG(" %02x", csip[base+i]);}
        _LOG("\n");
        _LOG("SS:SP:");
        {for(int i = 0; i < 16; ++i)
            _LOG(" %02x", ssbp[OFF+i]);}
        _LOG("\n");
        uint16_t mask = PIC_GetIRQMask(); //disable timer etc. (int 16h will enable interrupt)
        PIC_SetIRQMask(PIC_IRQ_UNMASK(0xFFFF,1));
        _bios_keybrd(0); //single step debug for each key press. DPMI_INT16H for keyboard function is installed for debug mode.
        PIC_SetIRQMask(mask);
        Continue = TRUE;
    }
#endif
    if(Continue)
        *(uint16_t far*)(ssbp + 4) = 0xFFFF;//*(&expn) = 0xFFFF;
    else
    {
        #if DEBUG //TODO: non debug output
        //dump registers
        _ASM_BEGIN
            _ASM2(mov ax, sp)
            _ASM(push ss) //push ss for compact/large model, for small/medium it is ignored beause DBG_DumpREG has ONLY 1 parameter
            _ASM(push ax)
            _ASM(call DBG_DumpREG)
            _ASM2(add sp, 4)
        _ASM_END
        _LOG("Excpetion: %02x, Error: %04x, CS:IP: %04x:%04x, FLAGS: %04x\n", expn, error, cs_, ip_, flags);
        int base = 0; //modify this, i.e. -8 to get prevous instuctions
        _LOG("CS:IP:");
        {for(int i = 0; i < 16; ++i)
            _LOG(" %02x", csip[base+i]);}
        _LOG("\n");
        _LOG("SS:SP:");
        {for(int i = 0; i < 16; ++i)
            _LOG(" %02x", ssbp[OFF+i]);}
        _LOG("\n");
        //DPMI_LOADER_DumpSegments();
        #endif//#if DEBUG
    }
    #undef OFF
    _ASM_BEGIN
        _ASM2(mov bx, sp)
        _ASM2(mov word ptr ss:[bx+DPMI_REG_OFF_EBP], bp);
        _ASM(popad)
        _ASM(popf)
        _ASM(pop es)
        _ASM(pop ds)
        _ASM(pop fs)
        _ASM(pop gs)
        _ASM2(add sp, 8)
    _ASM_END
}

#pragma option -k- //BC
extern "C" void __NAKED __CDECL near DPMI_ExceptionHandlerImpl();

//add extra wrapper to make the same stack when DPMI_HWIRQHandler called
extern "C" void __NAKED __CDECL near DPMI_ExceptionHandlerImplWrapper()
{
    _ASM_BEGIN
        _ASM(call DPMI_ExceptionHandlerImpl)
    _ASM_END
}

static void __NAKED DPMI_NullINTHandler()
{
    _ASM_BEGIN 
        _ASM(iret)
    _ASM_END
}

extern "C" void __NAKED __CDECL near DPMI_HWIRQHandler();

#define DPMI_EXCEPT(n) extern "C" void __NAKED __CDECL DPMI_ExceptionHandler##n() \
{\
    _asm {call DPMI_ExceptionHandlerImplWrapper} \
}

#define DPMI_IRQHANDLER(n) extern "C" void __NAKED __CDECL DPMI_IRQHandler##n() \
{\
    _asm {call DPMI_HWIRQHandler} \
}

DPMI_EXCEPT(0x00)
DPMI_EXCEPT(0x01)
DPMI_EXCEPT(0x02)
DPMI_EXCEPT(0x03)
DPMI_EXCEPT(0x04)
DPMI_EXCEPT(0x05)
DPMI_EXCEPT(0x06)
DPMI_EXCEPT(0x07)
DPMI_EXCEPT(0x08)
DPMI_EXCEPT(0x09)
DPMI_EXCEPT(0x0A)
DPMI_EXCEPT(0x0B)
DPMI_EXCEPT(0x0C)
DPMI_EXCEPT(0x0D)
DPMI_EXCEPT(0x0E)

DPMI_IRQHANDLER(0x08)
DPMI_IRQHANDLER(0x09)
DPMI_IRQHANDLER(0x0A)
DPMI_IRQHANDLER(0x0B)
DPMI_IRQHANDLER(0x0C)
DPMI_IRQHANDLER(0x0D)
DPMI_IRQHANDLER(0x0E)
DPMI_IRQHANDLER(0x0F)
DPMI_IRQHANDLER(0x70)
DPMI_IRQHANDLER(0x71)
DPMI_IRQHANDLER(0x72)
DPMI_IRQHANDLER(0x73)
DPMI_IRQHANDLER(0x74)
DPMI_IRQHANDLER(0x75)
DPMI_IRQHANDLER(0x76)
DPMI_IRQHANDLER(0x77)

#define DPMI_EXCEPT_ADDR(n) FP_OFF(&DPMI_ExceptionHandler##n)
#define DPMI_IRQHANDLER_ADDR(n) FP_OFF(&DPMI_IRQHandler##n)

void __NAKED __CDECL near DPMI_ExceptionHandlerImpl()
{
    _ASM_BEGIN
        //calc exception number using offset
        //restore modified registers (ax,dx,bp) before calling DPMI_Except, so that DPMI_Except can get the original reigsters info
        _ASM2(add sp, 2) //discard return address in DPMI_ExceptionHandlerImplWrapper/DPMI_HWIRQHandler
        _ASM(push bp)
        _ASM2(mov bp, sp)
        _ASM(push dx)
        _ASM(push ax)
        _ASM2(mov ax, [bp+2]); //get return address in DPMI_ExceptionHandlerXX/DPMI_IRQHandlerXX
        _ASM2(sub ax, 3); //back to caller addr when call was made: 'call near [addr]'= 3 bytes
        _ASM2(sub ax, offset DPMI_ExceptionHandler0x00);
        //size of DPMI_ExceptionHandlerXX/DPMI_IRQHandlerXX
        _ASM2(mov dx, offset DPMI_ExceptionHandler0x01)
        _ASM2(sub dx, offset DPMI_ExceptionHandler0x00)
        _ASM(div dl) //get exception number
        _ASM2(xor ah,ah)

        _ASM2(cmp ax, 0x0E) //called from DPMI_IRQHandlerXX?
        _ASM(jbe CallExcept) //no, continue
        _ASM2(sub ax, 0x7)
    _ASMLBL(CallExcept:)
        _ASM2(cmp ax, 0x08) //exception below 8 have no errors.
        _ASM(ja HaveError)
        //no error, push/insert 0 before bp
        _ASM2(mov dx, word ptr [bp]);
        _ASM2(xchg dx, word ptr [bp-2]);
        _ASM2(xchg dx, word ptr [bp-4]);
        _ASM(push dx)
        _ASM2(sub bp, 2)
        _ASM2(mov word ptr [bp+4], 0);
    _ASMLBL(HaveError:)
        _ASM2(mov [bp+2],ax); //discard return address in DPMI_ExceptionHandlerXX/DPMI_IRQHandlerXX, change to exp no
        _ASM(pop ax)
        _ASM(pop dx)
        _ASM(pop bp)
        _ASM(call DPMI_Except)
        _ASM2(xchg bp, sp)
        _ASM2(add bp, 4) //pop exp no, errorcode
        _ASM2(cmp word ptr ss:[bp-4], 0xFFFF); //expno changed to 0xFFFF?
        _ASM2(xchg bp, sp)
        _ASM(je ExeptContinue)
        _ASM2(mov ax, SEL_INTR_DS*8)
        _ASM2(mov ds, ax)
        _ASM2(cmp DPMI_TSRed, 1)
        _ASM(jne exitnow)
    _ASMLBL(deadloop:)
        _ASM(jmp deadloop)
    _ASMLBL(exitnow:)
        _ASM(push 1)
        _ASM(call DPMI_Exit)
    _ASMLBL(ExeptContinue:)
        _ASM(iret);
    _ASM_END
}

#pragma option -k //BC


extern "C" void __CDECL near DPMI_HWIRQHandlerInternal()
{
    uint8_t irq = PIC_GetIRQ();
    //_LOG("HWIRQ %d\n", irq);
    uint8_t vec = PIC_IRQ2VEC(irq);
    //assert(vec >= 0x08 && vec <= 0x0F || vec >= 0x70 && vec <= 0x77);
    void (far* handler)(void) = DPMI_UserINTHandler[vec]; //save on stack, we are going to modify ds
    if(handler)
    {
        uint16_t userds = DPMI_UserINTHandlerDS[vec];
        _ASM_BEGIN
            _ASM(push ds)
            _ASM(push es)
            _ASM(push ax)
            _ASM2(mov ax, userds)
            _ASM2(mov ds, ax)
            _ASM2(mov es, ax)
            _ASM(pop ax)
        _ASM_END

        handler();

        _ASM_BEGIN
            _ASM(pop es)
            _ASM(pop ds)
        _ASM_END
    }
    else
    {
        DPMI_REG r = {0};
        DPMI_CallRealModeINT(vec, &r); //call real mode IRQ handler
    }
}

#pragma option -k- //BC
#pragma disable_message (13) //WC: unreachable code, but seems invalid
extern "C" void __NAKED __CDECL near DPMI_HWIRQHandler()
{
    //test if it is a normal INT or exception.
    _ASM_BEGIN
        _ASM(push bp)
        _ASM2(mov bp, sp)
        _ASM(push ax)

        //stack:
        //normal int: [bp,] DPMI_IRQHandlerXX, ip,          cs, *flags*,
        //exception:  [bp,] DPMI_IRQHandlerXX, errorcode,   ip, *cs*,     flags
        //not handling ring3 excptions, we're always in ring0
        //note: flags always has bit1 set, but selector last 4 bits are always 0 or 8
        _ASM2(cmp word ptr [bp+8], SEL_RMCB_CS*8)
        _ASM(je Except)
        _ASM2(test word ptr [bp+8], 0x7)
        _ASM(jnz NormalINT)
        _ASM2(cmp word ptr [bp+8], SEL_CS*8)
        _ASM(jb NormalINT)
        _ASM2(movzx ax, byte ptr DPMI_ExtCSCount)
        _ASM2(add ax, SEL_EXT_CS)
        _ASM2(shl ax, 3)
        _ASM2(cmp word ptr [bp+8], ax)
        _ASM(jae NormalINT)
    _ASMLBL(Except:)
        _ASM(pop ax)
        _ASM(pop bp)
        _ASM(call DPMI_ExceptionHandlerImpl) //never returns
    _ASMLBL(NormalINT:)
        _ASM(pop ax)
        _ASM(pop bp)
    _ASM_END

    _ASM_BEGIN
        _ASM2(add sp, 2) //disacard return address in DPMI_IRQHandlerXX
        _ASM(pushf)
        _ASM(cli)
        _ASM(pushad)
        _ASM(push ds)
        _ASM(push es)
        _ASM(push fs)
        _ASM(push gs)

        _ASM2(mov bp, sp)
        _ASM2(and word ptr [bp+8+DPMI_PUSHAD_SIZE], 0xBFFF) //remove NT flag for flags or iret will do a task return
        _ASM2(mov ax, SEL_INTR_DS*8) //load the DS we're using
        _ASM2(mov ds, ax)
        _ASM(call DPMI_HWIRQHandlerInternal)

        _ASM(pop gs)
        _ASM(pop fs)
        _ASM(pop es)
        _ASM(pop ds)
        _ASM(popad)
        _ASM(popf)
        _ASM(iret)
    _ASM_END
}
#pragma enable_message (13) //WC
#pragma option -k //BC

static void DPMI_SetupPaging()
{
    if(DPMI_V86) //no paging in raw protected mode
        DPMI_InitVCPI(); //VCPI will init first 4M page.
}

#include "USBDDOS/DPMI/dpmi_i21.h"
static void near DPMI_Shutdown(void);

static void DPMI_SetupIDT()
{
    assert(DPMI_Temp);
    _fmemset(DPMI_Temp->idt, 0, sizeof(IDT)*256);
    for(int i = 0; i < 256; ++i)
    {
        DPMI_Temp->idt[i].gate_type = IDT_GT_IG;
        DPMI_Temp->idt[i].selector = (SEL_CS+DPMI_LOADER_GetCSIndex(FP_SEG(&DPMI_NullINTHandler)))<<3;
        DPMI_Temp->idt[i].offset_low = FP_OFF(&DPMI_NullINTHandler);
        DPMI_Temp->idt[i].present = 1;
    }

    DPMI_Temp->RealModeIRQ0Vec = 0x08;
    DPMI_Temp->ProtectedModeIRQ0Vec = 0x08;
    DPMI_Temp->RealModeIRQ8Vec = 0x70;
    DPMI_Temp->ProtectedModeIRQ8Vec = 0x70;

    uint16_t MasterVec = DPMI_Temp->RealModeIRQ0Vec;
    uint16_t SlaveVec = DPMI_Temp->RealModeIRQ8Vec;
    if(DPMI_V86)
    {
        _ASM_BEGIN //Get 8259A Interrupt Vector Mappings
            _ASM(push ax)
            _ASM(push bx)
            _ASM(push cx)
            _ASM2(mov ax, 0xDE0A)
            _ASM(int 0x67)
            _ASM2(mov MasterVec, bx)
            _ASM2(mov SlaveVec, cx)
            _ASM(pop cx)
            _ASM(pop bx)
            _ASM(pop ax)
        _ASM_END
        if(MasterVec != DPMI_Temp->RealModeIRQ0Vec) //if already remapped, just use it
            DPMI_Temp->RealModeIRQ0Vec = (uint8_t)MasterVec;
        if(SlaveVec != DPMI_Temp->RealModeIRQ8Vec)
            DPMI_Temp->RealModeIRQ8Vec = (uint8_t)SlaveVec;
    }

    uint16_t size = DPMI_EXCEPT_ADDR(0x01) - DPMI_EXCEPT_ADDR(0x00);
    for(int e = 0x00; e < 0x0E; ++e)
        DPMI_Temp->idt[e].offset_low = DPMI_EXCEPT_ADDR(0x00) + size * e;

    //now irq 0~7 & exceptions 0x8~0xF share the same handler. set HWIRQHandler setup will overwrite part of the exception.
    size = DPMI_IRQHANDLER_ADDR(0x09) - DPMI_IRQHANDLER_ADDR(0x08);
    for(int j = DPMI_Temp->ProtectedModeIRQ0Vec; j <= DPMI_Temp->ProtectedModeIRQ0Vec+7; ++j)
        DPMI_Temp->idt[j].offset_low = DPMI_IRQHANDLER_ADDR(0x08) + size * (j-DPMI_Temp->ProtectedModeIRQ0Vec);
    for(int k = DPMI_Temp->ProtectedModeIRQ8Vec; k <= DPMI_Temp->ProtectedModeIRQ8Vec+7; ++k)
        DPMI_Temp->idt[k].offset_low = DPMI_IRQHANDLER_ADDR(0x70) + size * (k-DPMI_Temp->ProtectedModeIRQ8Vec);

    DPMI_Temp->idt[0x21].offset_low = FP_OFF(DPMI_INT21H);
    DPMI_I21H_pfnTerminate = DPMI_Shutdown;
    DPMI_I21H_PM_FP2L = DPMI_SEGOFFP2L;
    #if DEBUG
    DPMI_Temp->idt[0x16].offset_low = FP_OFF(DPMI_INT16H);
    #endif

    DPMI_Temp->idtr.size = sizeof(IDT)*256 - 1;
    DPMI_Temp->idtr.offset = DPMI_PTR16R2L(DPMI_Temp->idt);
}

//////////////////////////////////////////////////////////////////////////////
//GDT
//////////////////////////////////////////////////////////////////////////////

static void DPMI_SetupGDT()
{
    assert(DPMI_Temp);

    DPMI_Temp->gdtr.offset = DPMI_PTR16R2L(DPMI_Temp->gdt);
    DPMI_Temp->gdtr.size = sizeof(GDT)*SEL_TOTAL - 1;

    GDT far* gdt = DPMI_Temp->gdt;
    _fmemset(gdt, 0, sizeof(GDT)*SEL_TOTAL);

    _fmemcpy(&gdt[SEL_4G], DPMI_Data4GDesc, sizeof(GDT));

    _fmemcpy(&gdt[SEL_SYS_DS], DPMI_DataDesc, sizeof(GDT));
    gdt[SEL_SYS_DS].base_low = (uint16_t)DPMI_SystemDS;
    gdt[SEL_SYS_DS].base_middle = (uint8_t)(DPMI_SystemDS>>16L);
    gdt[SEL_SYS_DS].base_high = (uint8_t)(DPMI_SystemDS>>24L);

    _fmemcpy(&gdt[SEL_RMCB_CS], DPMI_CodeDesc, sizeof(GDT));
    gdt[SEL_RMCB_CS].base_low = (uint16_t)DPMI_Temp->RMCBLAddr;
    gdt[SEL_RMCB_CS].base_middle = (uint8_t)(DPMI_Temp->RMCBLAddr>>16L);
    gdt[SEL_RMCB_CS].limit_low = (uint16_t)(DPMI_RMCB_SIZE - 1);
    _fmemcpy(&gdt[SEL_RMCB_DS], DPMI_DataDesc, sizeof(GDT));
    gdt[SEL_RMCB_DS].base_low = (uint16_t)DPMI_Temp->RMCBLAddr;
    gdt[SEL_RMCB_DS].base_middle = (uint8_t)(DPMI_Temp->RMCBLAddr>>16L);
    gdt[SEL_RMCB_DS].limit_low = DPMI_RMCB_SIZE - 1;

    int codeCount = DPMI_LOADER_CS_Count;
    assert(codeCount<=DPMI_CS_MAX);
    int dataCount = DPMI_LOADER_DS_Count;
    assert(dataCount<=DPMI_DS_MAX);
    {for(int i = 0; i < codeCount; ++i)
    {
        _fmemcpy(&gdt[SEL_CS+i], DPMI_CodeDesc, sizeof(GDT));
        uint32_t base = ((uint32_t)DPMI_LOADER_CS[i])<<4;
        gdt[SEL_CS+i].base_low = (uint16_t)base;
        gdt[SEL_CS+i].base_middle = (uint8_t)(base>>16L);
    }}
    {for(int i = 0; i < dataCount; ++i)
    {
        _fmemcpy(&gdt[SEL_DS+i], DPMI_DataDesc, sizeof(GDT));
        uint32_t base = ((uint32_t)DPMI_LOADER_DS[i])<<4;
        gdt[SEL_DS+i].base_low = (uint16_t)base;
        gdt[SEL_DS+i].base_middle = (uint8_t)(base>>16L);
    }}

    {for(int i = 0; i < codeCount; ++i)
    {
        _fmemcpy(&gdt[SEL_HIMEM_CS+i], DPMI_CodeDesc, sizeof(GDT));
        uint32_t base = DPMI_HimemCode + (((uint32_t)DPMI_LOADER_CS[i]-(uint32_t)_CODE_SEG)<<4);
        gdt[SEL_HIMEM_CS+i].base_low = (uint16_t)base;
        gdt[SEL_HIMEM_CS+i].base_middle = (uint8_t)(base>>16L);
        gdt[SEL_HIMEM_CS+i].base_high = (uint8_t)(base>>24L);
    }}

    {for(int i = 0; i < dataCount; ++i)
    {
        _fmemcpy(&gdt[SEL_HIMEM_DS+i], DPMI_DataDesc, sizeof(GDT));
        uint32_t base = DPMI_HimemData + ((((uint32_t)DPMI_LOADER_DS[i]-(uint32_t)_DATA_SEG)<<4));
        gdt[SEL_HIMEM_DS+i].base_low = (uint16_t)base;
        gdt[SEL_HIMEM_DS+i].base_middle = (uint8_t)(base>>16L);
        gdt[SEL_HIMEM_DS+i].base_high = (uint8_t)(base>>24L);
    }}

    gdt[SEL_INTR_DS] = gdt[SEL_DS + DPMI_LOADER_GetDSIndex(FP_SEG(DPMI_UserINTHandler))];

    _fmemcpy(&gdt[SEL_VIDEO], DPMI_DataDesc, sizeof(GDT));
    gdt[SEL_VIDEO].base_low = 0x8000;
    gdt[SEL_VIDEO].base_middle = 0xB;
    gdt[SEL_VIDEO].limit_low = 80*25*2-1;

    _fmemcpy(&gdt[SEL_TEMP], DPMI_DataDesc, sizeof(GDT));
    gdt[SEL_TEMP].base_low = (uint16_t)DPMI_PTR16R2L(DPMI_Temp);
    gdt[SEL_TEMP].base_middle = (uint8_t)(DPMI_PTR16R2L(DPMI_Temp)>>16L);

    #if DEBUG && 0
    {for(int i = 0; i < SEL_TOTAL; ++i)
    {
        uint32_t far* entry = (uint32_t far*)&gdt[i];
        _LOG("GDT: %d: %08lx %08lx\n", i, entry[0], entry[1]);
    }}
    #endif
}
#endif//_DPMI_BC_H_