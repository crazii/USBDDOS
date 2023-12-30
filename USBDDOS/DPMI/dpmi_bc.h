#ifndef _DPMI_BC_H_
#define _DPMI_BC_H_
#include <assert.h>
#include <string.h>
#include <dos.h>
#include <process.h>
#include "USBDDOS/DPMI/dpmi.h"
#include "USBDDOS/pic.h"
#include "USBDDOS/dbgutil.h"

#if defined(__BC__)
#define __LIBCALL _Cdecl _FARFUNC
extern "C" int __LIBCALL vsprintf(char* buf, const char* fmt, va_list aptr);
#elif defined(__WC__)
#define __LIBCALL _WCRTLINK
extern "C" int __LIBCALL vsnprintf(char* buf, size_t size, const char* fmt, va_list aptr);
#else
#define __LIBCALL 
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

typedef enum
{
    PM_NONE = 0,
    PM_DIRECT = 1, //raw
    PM_VCPI = 2,   //vcpi
}DPMI_PM_MODE;

#if defined(__BC__)
enum
{
    SEL_4G = 1,
    SEL_SYS_DS, //gdt/idt/paging
    SEL_CS, //real time initial cs
    SEL_DS,
    SEL_HIMEM_CS,
    SEL_HIMEM_DS,
    SEL_RMCB_CS,
    SEL_RMCB_DS,

    SEL_VCPI_CS,//VCPI*3
    SEL_VCPI_1,
    SEL_VCPI_2,
    SEL_LDT,    //VCPI dummy
    SEL_TSS,    //VCPI dummy

    SEL_TOTAL,
};
#else
//WC won't compile if enum used as a constant in inline asm
#define SEL_4G 1
#define SEL_SYS_DS 2
#define SEL_CS 3
#define SEL_DS 4
#define SEL_HIMEM_CS 5
#define SEL_HIMEM_DS 6
#define SEL_RMCB_CS 7
#define SEL_RMCB_DS 8
#define SEL_VCPI_CS 9
#define SEL_VCPI_1 10
#define SEL_VCPI_2 11
#define SEL_LDT 12
#define SEL_TSS 13
#define SEL_TOTAL 14
#endif

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

typedef void (near* DPMI_RMCB_ENTRY)(void);

typedef struct
{
    DPMI_RMCB_ENTRY Target; //asm optimization based the layout, do not change
    uintptr_t UserReg;//for user call backs. pm mode DS's offset
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

    DPMI_RMCB_ENTRY CommonEntry; //common entry: save context & swtich to PM to call target entry
    DPMI_RMCBTable Table[DPMI_RMCB_COUNT];

    uint16_t SwitchPM; //function offset
    uint16_t SwitchRM; //function offset
    uint16_t Translation; //function offset: PM2RM translation

    uint16_t PM_SP; //stack pointer
    uint16_t PM_SS; //stack segment. must follows PM_SP

    uint16_t RM_SP; //fixed stack for translation, temporary, do not save any thing on it after mode switch
    uint16_t RM_SEG; //real mode segment of RMCB (DS=CS=SS), //must follows RM_SP, as an SS

    uint32_t VcpiInterface; //used in V86
    uint16_t VcpiInterfaceCS; //grouped with VcpiInterface as a farcall, don't break
    DPMI_VCPIClientStruct VcpiClient;
    uint32_t VcpiClientAddr; //linear address of VcpiClient

    //add new fixed offset for inline asm here
    //FIXED OFFSET END

    uint32_t LinearDS; //initial ds
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

#else

#error not implemented.

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
_Static_assert(DPMI_REG_OFF_EDI == offsetof(DPMI_REG, d.edi), "constant error");
_Static_assert(DPMI_REG_OFF_ESI == offsetof(DPMI_REG, d.esi), "constant error");
_Static_assert(DPMI_REG_OFF_EBP == offsetof(DPMI_REG, d.ebp), "constant error");
_Static_assert(DPMI_REG_OFF_EBX == offsetof(DPMI_REG, d.ebx), "constant error");
_Static_assert(DPMI_REG_OFF_EDX == offsetof(DPMI_REG, d.edx), "constant error");
_Static_assert(DPMI_REG_OFF_ECX == offsetof(DPMI_REG, d.ecx), "constant error");
_Static_assert(DPMI_REG_OFF_EAX == offsetof(DPMI_REG, d.eax), "constant error");
_Static_assert(DPMI_REG_OFF_FLAGS == offsetof(DPMI_REG, w.flags), "constant error");
_Static_assert(DPMI_REG_OFF_ES == offsetof(DPMI_REG, w.es), "constant error");
_Static_assert(DPMI_REG_OFF_DS == offsetof(DPMI_REG, w.ds), "constant error");
_Static_assert(DPMI_REG_OFF_FS == offsetof(DPMI_REG, w.fs), "constant error");
_Static_assert(DPMI_REG_OFF_GS == offsetof(DPMI_REG, w.gs), "constant error");
_Static_assert(DPMI_REG_OFF_CS == offsetof(DPMI_REG, w.cs), "constant error");
_Static_assert(DPMI_REG_OFF_IP == offsetof(DPMI_REG, w.ip), "constant error");
_Static_assert(DPMI_REG_OFF_SS == offsetof(DPMI_REG, w.ss), "constant error");
_Static_assert(DPMI_REG_OFF_SP == offsetof(DPMI_REG, w.sp), "constant error");

#define DPMI_REG_SIZE 50
_Static_assert(DPMI_REG_SIZE == sizeof(DPMI_REG), "error constant");

#define RMCB_TABLE_ENTRY_SIZE 10
_Static_assert(RMCB_TABLE_ENTRY_SIZE == sizeof(DPMI_RMCBTable), "error constant");

#define DPMI_SIZEOF_GDT 8
_Static_assert(DPMI_SIZEOF_GDT == sizeof(GDT), "error constant");

#define VCPI_SIZEOF_CLIENTSTRUCT 24
_Static_assert(VCPI_SIZEOF_CLIENTSTRUCT == sizeof(DPMI_VCPIClientStruct), "constant error");

#define VCPI_CLIENTSTRUCT_OFF_EIP 16
_Static_assert(VCPI_CLIENTSTRUCT_OFF_EIP == offsetof(DPMI_VCPIClientStruct, EIP), "constant error");


#define RMCB_OFF_OldIDTR 2 //+2: extra padding
#define RMCB_OFF_NullGDTR 10 //+2: extra padding
#define RMCB_OFF_GDTR 18 //+2: extra padding
#define RMCB_OFF_IDTR 26
#define RMCB_OFF_CR3 32
#define RMCB_OFF_CommonEntry 36
#define RMCB_OFF_Table 38
#define RMCB_OFF_SwitchPM 198
#define RMCB_OFF_SwitchRM 200
#define RMCB_OFF_Translation 202
#define RMCB_OFF_PMSP 204
#define RMCB_OFF_RMSP 208
#define RMCB_OFF_RMSEG 210
#define RMCB_OFF_VcpiInterface 212
#define RMCB_OFF_VcpiClient 218
#define RMCB_OFF_VcpiClientAddr 242

_Static_assert(RMCB_OFF_OldIDTR == offsetof(DPMI_RMCB, OldIDTR) + 2, "constant error"); //2: extra padding
_Static_assert(RMCB_OFF_NullGDTR == offsetof(DPMI_RMCB, NullGDTR) + 2, "constant error"); //2: extra padding
_Static_assert(RMCB_OFF_GDTR == offsetof(DPMI_RMCB, GDTR) + 2, "constant error");
_Static_assert(RMCB_OFF_IDTR == offsetof(DPMI_RMCB, IDTR) + 2, "constant error");
_Static_assert(RMCB_OFF_CR3 == offsetof(DPMI_RMCB, CR3), "constant error");
_Static_assert(RMCB_OFF_CommonEntry == offsetof(DPMI_RMCB, CommonEntry), "constant error");
_Static_assert(RMCB_OFF_Table == offsetof(DPMI_RMCB, Table), "constant error");
_Static_assert(RMCB_OFF_SwitchPM == offsetof(DPMI_RMCB, SwitchPM), "constant error");
_Static_assert(RMCB_OFF_SwitchRM == offsetof(DPMI_RMCB, SwitchRM), "constant error");
_Static_assert(RMCB_OFF_Translation == offsetof(DPMI_RMCB, Translation), "constant error");
_Static_assert(RMCB_OFF_PMSP == offsetof(DPMI_RMCB, PM_SP), "constant error");
_Static_assert(RMCB_OFF_RMSP == offsetof(DPMI_RMCB, RM_SP), "constant error");
_Static_assert(RMCB_OFF_RMSEG == offsetof(DPMI_RMCB, RM_SEG), "constant error");
_Static_assert(RMCB_OFF_VcpiInterface == offsetof(DPMI_RMCB, VcpiInterface), "constant error");
_Static_assert(RMCB_OFF_VcpiClient == offsetof(DPMI_RMCB, VcpiClient), "constant error");
_Static_assert(RMCB_OFF_VcpiClientAddr == offsetof(DPMI_RMCB, VcpiClientAddr), "constant error");

//default descriptors
static const uint32_t DPMI_CodeDesc[2] = {0x0000FFFF, 0x000F9A00};
static const uint32_t DPMI_DataDesc[2] = {0x0000FFFF, 0x000F9200};
static const uint32_t DPMI_Data4GDesc[2] = {0x0000FFFF, 0x00CF9200}; //4G ds, Granularity, 32 bit
_Static_assert(sizeof(DPMI_Data4GDesc) == sizeof(GDT), "size error");
_Static_assert(sizeof(DPMI_DataDesc) == sizeof(GDT), "size error");
_Static_assert(sizeof(DPMI_CodeDesc) == sizeof(GDT), "size error");

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
const uint32_t DPMI_XMS_Size = 192L*1024L;   //64k code+data, first 64k is system data.

static uint16_t _DATA_SEG;
static uint16_t _CODE_SEG;
static uint32_t DPMI_SystemDS = 0;  //keep GDT, IDT, page table
static uint32_t DPMI_HimemCS = 0;
static uint32_t DPMI_HimemDS = 0;
static uint16_t DPMI_XMSHimemHandle;
static uint16_t DPMI_XMSBelow4MHandle;
static uint8_t DPMI_V86;
static uint8_t DPMI_PM;
static uint8_t DPMI_TSRed;
static uint32_t DPMI_RmcbMemory;
static DPMI_RMCB far* DPMI_Rmcb; //can be accessed in PM (segment is SEL_RMCB_DS*8)
static DPMI_TempData far* DPMI_Temp;

#if defined(__WC__) && 0
//this method is tiny optimized than the macro method, but it uses more code spaces..
//the macro uses 16 bit register ops and loops, maybe not efficient but compact in code size.
uint32_t DPMI_Ptr16ToLinear(void far* ptr);
#pragma aux DPMI_Ptr16ToLinear = \
"push edx" \
"push eax" \
"movzx eax, ax" \
"movzx edx, dx" \
"shl edx, 4" \
"add edx, eax" \
"pop eax" \
"mov ax, dx" \
"pop dx" \
"shr edx, 16" \
"push dx" \
"pop edx" \
Parm[dx ax] \
Value[dx ax]
#endif

//convert a 16 bit far ptr to linear addr
#define DPMI_Ptr16ToLinear(ptr) ((((uint32_t)FP_SEG((ptr)))<<4)+(uint32_t)FP_OFF((ptr)))
//convert a linear addr to a far pointer
void far* DPMI_LinearToPtr16(uint32_t addr)
{
    uint32_t linearDS = ((uint32_t)_DS)<<4L;
    if(addr >= linearDS && addr <= linearDS + 0xFFFFL)
        return MK_FP(_DS, addr-linearDS);

    assert(addr <= 0xFFFFFL);//below 1M
    if(addr > 0xFFFFFL)
        return NULL;
    uint32_t seg = addr >> 4;
    uint32_t off = addr - (seg<<4);
    return MK_FP(seg, off);
}

//only works for protected mode.
static inline PDE DPMI_LoadPDE(uint32_t addr) { PDE pde; pde.value = DPMI_LoadD(addr); return pde; }
static inline PDE DPMI_LoadPDE(uint32_t addr, uint32_t i) { return DPMI_LoadPDE(addr+i*sizeof(PDE)); }
static inline void DPMI_StorePDE(uint32_t addr, const PDE* pde) { DPMI_StoreD(addr, pde->value); }
static inline void DPMI_StorePDE(uint32_t addr, uint32_t i, const PDE* pde) { DPMI_StorePDE(addr+i*sizeof(PDE), pde); }

#define DPMI_LoadPTE DPMI_LoadPDE
#define DPMI_StorePTE DPMI_StorePDE

//UMB might not be 1:1 mapped.
//#define DPMI_PTUnmap(pt, laddr) DPMI_V86 ? PTE_ADDR(DPMI_PM ? DPMI_LoadPTE(DPMI_Ptr16ToLinear(pt), (laddr)>>12) : pt[(laddr)>>12]) : laddr; //laddr to paddr
//#define DPMI_PDUnmap(pdir, laddr) DPMI_PTUnmap(&pdir[(laddr)>>20], laddr)
//simplified to works only for real mode before init
#define DPMI_PTUnmap(pt, laddr) (DPMI_V86 ? PTE_ADDR((pt)[(laddr)>>12L]) : laddr);

//////////////////////////////////////////////////////////////////////////////
//direct (raw) mode
//////////////////////////////////////////////////////////////////////////////
#pragma option -k-
static void __NAKED far DPMI_DirectProtectedMode()
{
    _ASM_BEGIN
        _ASM(push eax)
        _ASM(push ebx) //the C code uses bx but never restore it
        _ASM(pushfd)
        _ASM(cli)

        //XMS global enable A20
        _ASM(call XMS_EnableA20)
        _ASM2(test al, al)
        _ASM(je Enable20Done)
        _ASM(push 1)
        _ASM(call exit)
    _ASMLBL(Enable20Done:)

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
    _ASM_END

    _ASM_BEGIN
        _ASM(popfd)
        _ASM(pop ebx)
        _ASM(pop eax)
        _ASM(retf)
    _ASM_END
}
static void __NAKED DPMI_DirectProtectedModeEnd() {}

static void __NAKED far DPMI_DirectRealMode()
{
    _ASM_BEGIN
        _ASM(push eax)
        _ASM(push ebx)
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

        //XMS global enable A20
        _ASM(call XMS_DisableA20)
        _ASM2(test al, al)
        _ASM(je Disable20Done)
        _ASM(push 1)
        _ASM(call exit)
    _ASMLBL(Disable20Done:)

        _ASM(popfd)
        _ASM(pop ebx)
        _ASM(pop eax)
        _ASM(retf)
    _ASM_END
}
static void __NAKED DPMI_DirectRealModeEnd() {}
#pragma option -k

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
    /*//fill the un-inited table entry if VCPI didn't init them all
    //note: now allowed by VCPI spec
    for(uint32_t i = PageTable0Offset; i < 1024; ++i)
    {
        PTE pte = {0};
        PTE_INIT(pte, i<<12);//1:1 map
        First4M[i] = pte;
    }//*/
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
    assert((DPMI_Ptr16ToLinear(DPMI_Temp->PageDir)&0xFFF)==0);
    _fmemset(DPMI_Temp->PageDir, 0, 4096);
    _fmemset(DPMI_Temp->PageTable0, 0, 4096);
    _fmemset(DPMI_Temp->PageTalbeHimem, 0, 4096);
    _fmemset(DPMI_Temp->PageTalbeHimem2, 0, 4096);
    //find an alignment gap to place xms map handle
    uint32_t PageDirLAddr = DPMI_Ptr16ToLinear(DPMI_Temp->PageDir);
    if(PageDirLAddr - ((uint32_t)DPMI_Temp->PageMemory<<4L) >= 1024L*sizeof(uint16_t))
    {
        DPMI_Temp->XMSPageHandle = (uint16_t far*)MK_FP(DPMI_Temp->PageMemory,0);
        assert(DPMI_Ptr16ToLinear(DPMI_Temp->XMSPageHandle+1024) <= PageDirLAddr);
    }
    else
    {
        DPMI_Temp->XMSPageHandle = (uint16_t far*)(DPMI_Temp->PageTalbeHimem2 + 1024);
        assert(DPMI_Ptr16ToLinear(DPMI_Temp->XMSPageHandle) >= DPMI_Ptr16ToLinear(DPMI_Temp->PageTalbeHimem2)+4096L);
        assert(DPMI_Ptr16ToLinear(DPMI_Temp->XMSPageHandle+1024) <= ((uint32_t)DPMI_Temp->PageMemory<<4)+VCPI_PAGING_MEM_SIZE);
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

    uint32_t pt0 = DPMI_PTUnmap(DPMI_Temp->PageTable0, DPMI_Ptr16ToLinear(DPMI_Temp->PageTable0));
    _LOG("Page table 0: %08lx %08lx\n", pt0, DPMI_Ptr16ToLinear(DPMI_Temp->PageTable0));
    _LOG("Page dir: %08lx\n", DPMI_Ptr16ToLinear(DPMI_Temp->PageDir));

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
            uint32_t pt = DPMI_PTUnmap(DPMI_Temp->PageTable0, DPMI_Ptr16ToLinear(DPMI_Temp->PageTalbeHimem));
            assert((pt&0xFFF) == 0);
            PDE pde = PDE_INIT(pt);
            DPMI_Temp->PageDir[pdi] = pde;
        }
        {
            uint32_t pt = DPMI_PTUnmap(DPMI_Temp->PageTable0, DPMI_Ptr16ToLinear(DPMI_Temp->PageTalbeHimem2));
            assert((pt&0xFFF) == 0);
            PDE pde = PDE_INIT(pt);
            DPMI_Temp->PageDir[pdi+1] = pde;
        }
        DPMI_MapMemory(DPMI_SystemDS, 192L*1024L);
    }
    //CLI; //soft int will set IF, need cli if called in DPMI_VCPIProtectedMode().
    return TRUE;
}

#pragma option -k-
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
#pragma option -k

//////////////////////////////////////////////////////////////////////////////
//mode switch
//////////////////////////////////////////////////////////////////////////////

//swtich to protected mode.
static int16_t __CDECL DPMI_SwitchProtectedMode(uint32_t LinearDS)
{ //care on printf msg in mode switch, because printf will switch back and forth and cause recursion
    if(DPMI_PM)
        return DPMI_PM;

    CLIS();
    //DO NOT access any static data befoe switch, because this may be called from a RMCB block from outside
    _ASM_BEGIN
        _ASM(push bx)
        _ASM(push bp)
        _ASM2(mov bp, sp)
        _ASM2(lss bx, DPMI_Rmcb);
        _ASM2(mov sp, ss:[bx + RMCB_OFF_RMSP]);
        _ASM(push SEL_CS*8)
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
        _ASM(push ss)
        _ASM(push word ptr ss:[bx + RMCB_OFF_SwitchPM]);
        _ASM2(mov ax, ss)
        _ASM2(mov ds, ax)
        _ASM(retf)
    _ASMLBL(SwitchPMDone:)
        _ASM2(mov ax, SEL_DS*8)
        _ASM2(mov ds, ax)
        _ASM2(mov es, ax)
        _ASM2(mov ss, ax)
        _ASM2(mov fs, ax)
        _ASM2(mov gs, ax)
        _ASM2(mov sp, bp)
        _ASM(pop bp)
        _ASM(pop bx)
    _ASM_END
    DPMI_PM = DPMI_V86 ? PM_VCPI : PM_DIRECT;
    DPMI_Rmcb = (DPMI_RMCB far*)MK_FP(SEL_RMCB_DS*8, 0);

    DPMI_Addressing.physical = TRUE;    //linear=physical
    DPMI_Addressing.selector = SEL_4G*8; //4G ds

    DPMI_CopyLinear(DPMI_HimemDS, LinearDS, 64L*1024L); //copy data to himem

    _ASM_BEGIN//switch immediately after copy, or the stack won't match
        _ASM2(mov ax, SEL_HIMEM_DS*8)
        _ASM2(mov ds, ax)
        _ASM2(mov es, ax)
        _ASM2(mov ss, ax)
        _ASM2(mov fs, ax)
        _ASM2(mov gs, ax)
    _ASM_END

    //_LOG("FLAGS:%x\n", CPU_FLAGS());
    STIL();
    return DPMI_PM;
}

//back to real/v86 mode. this function only should be called on init or exit
//for swtich modes in between execution, use DPMI_CallRealMode* functions.
static void __CDECL DPMI_SwitchRealMode(uint32_t LinearDS)
{
    if(!DPMI_PM)
        return;
    CLIS();

    uint16_t segment = DPMI_Rmcb->RM_SEG; //save on stack. DPMI_Rmcb won't be accessible after switching mode
    //_LOG("SS:SP %x:%x\n:", _DATA_SEG, _SP);

    DPMI_CopyLinear(LinearDS, DPMI_HimemDS, 64L*1024L); //copy data back
    _ASM_BEGIN
        _ASM2(mov ax, SEL_DS*8)
        _ASM2(mov ds, ax)
        _ASM2(mov es, ax)
        _ASM2(mov ss, ax)
        _ASM2(mov fs, ax)
        _ASM2(mov gs, ax)
    _ASM_END

    _ASM_BEGIN
        _ASM(push bx)
        _ASM(push bp)
        _ASM2(mov bp, sp)
        _ASM2(lss bx, DPMI_Rmcb);

        _ASM2(mov ss:[bx + RMCB_OFF_PMSP], sp);
        _ASM2(mov sp, ss:[bx + RMCB_OFF_RMSP]);
        _ASM(push _DATA_SEG)
        _ASM(push _CODE_SEG)
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
        _ASM(push SEL_RMCB_CS*8)
        _ASM(push word ptr ss:[bx + RMCB_OFF_SwitchRM]);
        _ASM2(mov ax, ss)
        _ASM2(mov ds, ax)
        _ASM(retf)
    _ASMLBL(SwitchRMDone:)
        //_ASM2(mov ax, _DATA_SEG) //_DATA_SEG is in _DS, not accessible since DS not set
        _ASM(pop ax) //pushed _DATA_SEG
        _ASM2(mov ds, ax)
        _ASM2(mov es, ax)
        _ASM2(mov ss, ax)
        _ASM2(mov fs, ax)
        _ASM2(mov gs, ax)
        _ASM2(mov sp, bp)
        _ASM(pop bp)
        _ASM(pop bx)
    _ASM_END
    //note: only changing settings in LinearDS, himemDS not changed (as DPMI_PM=true)
    DPMI_Rmcb = (DPMI_RMCB far*)MK_FP(segment, 0);
    DPMI_PM = PM_NONE;
    DPMI_Addressing.selector = 0;
    STIL();
#if 0 //debug code
    _asm
     {
    jmp end
    printdx:
        push bp
        mov bp, sp
        sub sp, 4

        mov cx, 4
    xword:
        mov ax, dx

        push cx
        dec cx
        shl cx, 2
        shr ax, cl
        pop cx

        and ax, 0xF
        cmp ax, 0xA
        jae alpha
        add ax, '0'
        jmp savechar
    alpha:
        add ax, 'A'-0xA
    savechar:
        sub bp, cx
        mov [bp], ax
        add bp, cx
        loop xword

        mov cx, 4
    pnt:
        mov ah, 0x0E
        sub bp, cx
        mov al, [bp]
        add bp, cx
        int 0x10
        loop pnt
        mov sp, bp
        pop bp
        ret
    end:
        mov dx, ss
        call printdx

        mov ah,0x0E
        mov al,':'
        int 0x10

        mov dx, sp
        call printdx
    looop:
        jmp looop
    }
#endif
}

//////////////////////////////////////////////////////////////////////////////
//IDT
//////////////////////////////////////////////////////////////////////////////
static void (*DPMI_UserINTHandler[256])(void); //TODO: software int handlers

extern "C" void __CDECL DPMI_DumpExcept(short expn, short error, short ip, short cs, short flags)
{
    unused(expn);unused(error);unused(ip);unused(cs);unused(flags);
    _LOG("Excpetion: 0x%02x, Error: %x, CS:IP: %04x:%04x, FLAGS: %04x\n", expn, error, cs, ip, flags);
}

#pragma option -k-
extern "C" void __NAKED __CDECL DPMI_ExceptionHandlerImpl();

//add extra wrapper to make the same stack when DPMI_HWIRQHandler called
extern "C" void __NAKED __CDECL DPMI_ExceptionHandlerImplWrapper()
{
    _asm {call DPMI_ExceptionHandlerImpl}
}

static void __NAKED DPMI_NullINTHandler()
{
    _ASM_BEGIN 
        _ASM(iret)
    _ASM_END
}

extern "C" void __NAKED __CDECL DPMI_HWIRQHandler();

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

void __NAKED __CDECL DPMI_ExceptionHandlerImpl()
{
    _ASM_BEGIN
        _ASM(pop ax) //discard return address in DPMI_ExceptionHandlerImplWrapper/DPMI_HWIRQHandler
        _ASM(pop ax) //get & discard return address in DPMI_ExceptionHandlerXX/DPMI_IRQHandlerXX, never returns.
        //_ASM2(sub ax, 3) //back to caller addr when call was made - use dec later
        _ASM2(sub ax, offset DPMI_ExceptionHandler0x00)
#if defined(__BC__)
        _ASM2(shr ax, 2) //BC has a 'ret' at the end
#else
        _ASM2(mov bl, 3) //size of DPMI_ExceptionHandlerXX/DPMI_IRQHandlerXX
        _ASM(div bl) //get exception number
#endif
        _ASM(dec ax)
        _ASM2(cmp ax, 0x0E) //called from DPMI_IRQHandlerXX?
        _ASM(jbe DumpExcept) //no, continue
        _ASM2(sub ax, 0x7)
    _ASMLBL(DumpExcept:)
        _ASM(push ax)
        _ASM(call DPMI_DumpExcept)
        //don't bother the stack, exit
        _ASM2(test DPMI_TSRed, 1)
        _ASM(jz exitnow)
    _ASMLBL(deadloop:)
        _ASM(jmp deadloop)
    _ASMLBL(exitnow:)
        _ASM(push 1)
        _ASM(call exit)
    _ASM_END
}

#pragma option -k


extern "C" void __CDECL DPMI_HWIRQHandlerInternal()
{
    uint8_t irq = PIC_GetIRQ();
    //_LOG("HWIRQ %d\n", irq);
    //if(irq < 16)
    {
        uint8_t vec = PIC_IRQ2VEC(irq);
        //assert(vec >= 0x08 && vec <= 0x0F || vec >= 0x70 && vec <= 0x77);
        if(DPMI_UserINTHandler[vec])
            DPMI_UserINTHandler[vec]();
        else
        {
            DPMI_REG r = {0};
            DPMI_CallRealModeINT(vec, &r); //call real mode IRQ handler
        }
    }
}

#pragma option -k-
void __NAKED DPMI_HWIRQHandler()
{
    //test if it is a normal INT or exception.
    _ASM_BEGIN
        #ifdef DEBUG1
        _ASM(push eax)
        _ASM2(mov eax, 'AAAA')
        _ASM2(and eax, 0x00FFFFFF) //log print 'AAA'
        _ASM(push eax)
        _ASM2(mov ax, sp)
        _ASM(push ax)
        _ASM(call DBG_Log)
        _ASM2(add sp, 6)
        _ASM(pop eax)
        #endif

        _ASM(push bp)
        _ASM2(mov bp, sp)

        //stack:
        //normal int: [bp,] DPMI_IRQHandlerXX, ip,          cs, *flags*,
        //exception:  [bp,] DPMI_IRQHandlerXX, errorcode,   ip, *cs*,     flags
        //not handling ring3 excptions, we're always in ring0
        //note: flags always has bit1 set, but selector last 4 bits are always 0 or 8
        _ASM2(cmp word ptr [bp+8], SEL_HIMEM_CS*8)
        _ASM(je Except)
        _ASM2(cmp word ptr [bp+8], SEL_RMCB_CS*8) //DPMI_CallRealMode
        _ASM(je Except)
        _ASM2(cmp word ptr [bp+8], SEL_CS*8) //DPMI_CallRealMode
        _ASM(je Except)
        _ASM(jmp NormalINT)
    _ASMLBL(Except:)
        _ASM(pop bp)
        _ASM(call DPMI_ExceptionHandlerImpl)
    _ASMLBL(NormalINT:)
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
        _ASM2(and word ptr [bp+40], 0xBFFF) //remove NT flag for flags or iret will do a task return

        _ASM2(mov ax, SEL_HIMEM_DS*8)
        _ASM2(mov ds, ax)
        _ASM2(mov es, ax)
        _ASM(call DPMI_HWIRQHandlerInternal)

        #ifdef DEBUG1
        _ASM2(mov eax, 'BBBB')
        _ASM2(and eax, 0x00FFFFFF) //log print 'BBB'
        _ASM(push eax)
        _ASM2(mov ax, sp)
        _ASM(push ax)
        _ASM(call DBG_Log)
        _ASM2(add sp, 6)
        #endif

        _ASM(pop gs)
        _ASM(pop fs)
        _ASM(pop es)
        _ASM(pop ds)
        _ASM(popad)
        _ASM(popf)
        _ASM(iret)
    _ASM_END
}

#pragma option -k

//////////////////////////////////////////////////////////////////////////////
//RMCB
//////////////////////////////////////////////////////////////////////////////
#pragma option -k-
#define DPMI_RMCB_TEST 0
#if DPMI_RMCB_TEST
extern "C" void __NAKED DPMI_RmcbLog()
{
    _LOG("RMCB TEST");
    _ASM_BEGIN _ASM(retf) _ASM_END
}
#endif
extern "C" void __NAKED DPMI_RMCbIRet() //wrapper function for user rmcb (allow user uses normal ret). copy DPMI_REG registers and iret
{
    _ASM_BEGIN
        //save IRET frame to register
        _ASM(pop ax)
        _ASM(pop dx)
        _ASM(pop bx)

        _ASM(pop ecx)
        _ASM(pop esi)
        _ASM(pop ds)
        _ASM(pop edi)
        _ASM(pop es) //copy back reverse ds:si with es:di

        _ASM(cld)
        _ASM2(rep movs byte ptr es:[edi], byte ptr ds:[esi]);

        //_ASM(push bx)
        _ASM(push dx)
        _ASM(push ax)
        _ASM(retf)
    _ASM_END
}
static void __NAKED DPMI_RMCBCommonEntry() //commen entry for call back
{
    _ASM_BEGIN
        //_ASM2(add sp, 4) //ugly fix compiler: BC will push si/di if inline asm uses si/di - use EDI,ESI to avoid the problem

        //save context and make a DPMI_REG struct
        _ASM(push ss)
        _ASM(push sp) //ss: sp
        _ASM(push cs)
        _ASM(push ax) //cs : (fake) ip
        _ASM(push gs)
        _ASM(push fs)
        _ASM(push ds)
        _ASM(push es)
        _ASM(pushf)
        _ASM(pushad)

        //get caller IP and calc target. target addr = TableOffset + (IP-TableOffset)/TableSize*TableSize + 0 (offsetof(DPMI_RMCBTable, Target))
        //note: this should be done before swtich stack
        _ASM2(mov bp, sp);
        _ASM2(mov ax, ss:[bp+DPMI_REG_SIZE]); //caller IP (RMCB entry)
        _ASM2(sub bp, dx)
        _ASM2(mov cx, RMCB_TABLE_ENTRY_SIZE) //optmized as imm
        _ASM2(sub ax, RMCB_OFF_Table) //.Table == offsetof(DPMI_RMCB, Table)
        _ASM2(xor dx, dx)
        _ASM(div cx)
        _ASM(mul cx)
        _ASM2(add ax, RMCB_OFF_Table) //calc offset done
        _ASM2(mov bp, ax)
        _ASM2(mov bx, cs:[bp+2]); //reg ptr
        _ASM2(mov bp, cs:[bp]); //load from memory & save to BP
    _ASM_END

    _ASM_BEGIN
        _ASM(cli) //RMCB stack are temporary so disable interrup on RMCB stack
        //switch to RMCB stack
        _ASM2(movzx esi, sp)
        _ASM2(mov ax, ss)
        _ASM2(movzx edi, ax)
        _ASM2(lss sp, cs:[RMCB_OFF_RMSP]);

        _ASM(push cs)
        _ASM(pop ds)  
        _ASM(push SEL_RMCB_CS*8) //far return from switch
        _ASM(call word ptr cs:[RMCB_OFF_SwitchPM]); //far call with pushed cs
        //now in pm, fs=gs=es=ss=ds=SEL_RMCB_DS*8

        //switch himem stack. ss=SEL_HIMEM_DS*8
        _ASM2(lss sp, ds:[RMCB_OFF_PMSP]);
        //save original ss,sp to himem stack. rmcb stack are temporary and should not use after mode switch
        _ASM(push edi)
        _ASM(push esi)

        //copy DPMI_REG to UserReg if it is not null
        _ASM2(test bx, bx)
        _ASM(jz SkipCopyREG)

        _ASM2(mov ax, SEL_4G*8) //setup flat mode to copy reg [oldss:oldsp] => ss:[bx]
        _ASM2(mov ds, ax)
        _ASM2(mov es, ax)

        _ASM2(shl edi, 4) //originial realtime ss:sp to linear
        _ASM2(add esi, edi)
        //restore interrupt flag here. if bx is null, then it is a interrupt, don't restore flags.
        _ASM2(mov ax, ds:[esi+32]); //TODO: something is wrong on VCPI mode, find out why
        _ASM2(and ax, 0xBFFF) //mode switching needs clear NT flag 
        _ASM(push ax)
        _ASM(popf)

        _ASM2(movzx edi, bx) //pm ss:[bx] to linear
        _ASM2(add edi, dword ptr ss:[DPMI_HimemDS]);
        _ASM2(mov ecx, DPMI_REG_SIZE)
        
        _ASM(push ds)
        _ASM(push esi)
        _ASM(push es)
        _ASM(push edi)
        _ASM(push ecx) //popped by DPMI_RMCbIRet

        _ASM(cld)
        _ASM2(rep movs byte ptr es:[edi], byte ptr ds:[esi]);
    _ASMLBL(SkipCopyREG:)
        //call target pm function
        _ASM(pushf) //push return address. assume target is IRET
        _ASM(push cs)
#if defined(__BC__)
        _ASM2(mov ax, offset RMCB_PmReturn)
        _ASM2(sub ax, offset DPMI_RMCBCommonEntry)
        _ASM2(add ax, fs:[RMCB_OFF_CommonEntry]);
#else
        _ASM(call hack)
        _ASM(jmp RMCB_PmReturn)
    _ASMLBL(hack:) //hack for watcom, not working with offset + asm label
        _ASM(pop ax)
        _ASM(push bx)
        _ASM(mov bx, ax)
        _ASM(add ax, word ptr cs:[bx+1]);
        _ASM(add ax, 3) //size of jmp instruction itself
        _ASM(pop bx)
#endif
        _ASM(push ax)

        _ASM2(test bx, bx)
        _ASM(jz SkipIRetWrapper)
        _ASM(push offset DPMI_RMCbIRet)

    _ASMLBL(SkipIRetWrapper:)
        _ASM(push SEL_HIMEM_CS*8)
        _ASM(push bp) //saved target
        _ASM2(mov ax, ss)
        _ASM2(mov ds, ax)
        _ASM2(mov es, ax)
        _ASM2(mov fs, ax)
        _ASM2(mov gs, ax)
        _ASM(retf)
    _ASMLBL(RMCB_PmReturn:)
        
    /*#if DPMI_RMCB_TEST //BC doesn't handle macro with asm well
        _ASM(push SEL_RMCB_DS*8)
        _ASM(pop ds)
        _ASM(push cs)
        _ASM2(mov ax, offset RMCB_PmReturn2)
        _ASM2(sub ax, offset DPMI_RMCBCommonEntry)
        _ASM2(add ax, ds:[RMCB_OFF_CommonEntry]);
        _ASM(push ax)
        _ASM(push SEL_HIMEM_CS*8)
        _ASM(push offset DPMI_RmcbLog)
        _ASM2(mov ax, ss)
        _ASM2(mov ds, ax)
        _ASM2(mov es, ax)
        _ASM(retf)
        _ASMLBL(RMCB_PmReturn2:)
    #endif*/

        _ASM(pop ebp) //ss:sp
        _ASM(pop ebx)
        _ASM2(mov ax, SEL_RMCB_DS*8) //restore RMCB ds,ss
        _ASM2(mov ds, ax)
        _ASM(cli)
        _ASM2(mov ss, ax) //pm selector for ss
        _ASM2(mov sp, ds:[RMCB_OFF_RMSP]);
        _ASM(push word ptr ds:[RMCB_OFF_RMSEG]);
        _ASM(call word ptr ds:[RMCB_OFF_SwitchRM]);
        //now in rm. fs=gs=es=ss=ds=RMCB segement
    _ASM_END

    _ASM_BEGIN
        _ASM2(mov ss, bx) //ss:sp
        _ASM2(mov sp, bp)

        _ASM(popad)
        _ASM(popf)
        _ASM(pop es)
        _ASM(pop ds)
        _ASM(pop fs)
        _ASM(pop gs)
        _ASM2(add sp, 8) //cs, ip, sp, ss
        _ASM(ret) //WC doesn't generate ret for naked function, and adding it doesn't hurt for BC
    _ASM_END
}
static void __NAKED DPMI_RMCBCommonEntryEnd() {}

static void __NAKED DPMI_RMCBEntryIRET() //keep it as small as possible
{
    _ASM_BEGIN
        _ASM(call word ptr cs:[RMCB_OFF_CommonEntry]); //word ptr=near, dword ptr=far cs16:off16, fword=far cs16:off32
        //there's a RET generated by BC compiler, will be patched to IRET later.
#if !defined(__BC__)//#if defined(__WC__) //theres a bug in BC in _asm block, the __WC__ is defined?
        _ASM(ret)
#endif
    _ASM_END
}
static void __NAKED DPMI_RMCBEntryIRETEnd() {}
#pragma option -k
static const int DPMI_RMCBEntrySize = (uintptr_t)DPMI_RMCBEntryIRETEnd - (uintptr_t)DPMI_RMCBEntryIRET;

//INTn < 256 (hi byte 0): interrupt(cs:ip) with iret, push flags.
static void far _pascal DPMI_RMCBTranslation(DPMI_REG* reg, unsigned INTn, BOOL directcall)
{
    _ASM_BEGIN
        _ASM(pushfd)
        _ASM(pushad)
        _ASM(push es)
        _ASM(push fs)
        _ASM(push gs)

        _ASM(push ds)
        _ASM2(mov bx, reg) //bp[xxx]
        _ASM(push ebx)    //save DPMI_REG ptr

        _ASM2(mov ax, INTn)
        _ASM2(test ah, ah)
        _ASM(jnz skip_flags)
        _ASM(push word ptr [bx + DPMI_REG_OFF_FLAGS])
        _ASM2(and word ptr [bx + DPMI_REG_OFF_FLAGS], 0xFCFF) //clear IF TF (by Intel INTn instruction reference).(AC in hiword)
    _ASMLBL(skip_flags:)
        _ASM(push cs)
#if defined(__BC__)
        _ASM2(mov ax, offset callreturn)
#else
        _ASM(call hack)
        _ASM(jmp callreturn)
    _ASMLBL(hack:) //hack for watcom, not working with offset + asm label
        _ASM(pop ax)
        _ASM(push bx)
        _ASM(mov bx, ax)
        _ASM(add ax, word ptr cs:[bx+1]);
        _ASM(add ax, 3) //size of jmp instruction itself
        _ASM(pop bx)
        _ASM(mov directcall, ax); //directcall=!0, it's a runtime hack using relative offset, so no need to adjust offset as for BC
#endif
        _ASM2(mov cx, directcall)
        _ASM2(test cx, cx)
        _ASM(jnz skip_offset_adjst)
        _ASM2(sub ax, offset DPMI_RMCBTranslation)
        _ASM2(add ax, ds:[RMCB_OFF_Translation]);
    _ASMLBL(skip_offset_adjst:)
        _ASM(push ax)

        _ASM(push word ptr [bx + DPMI_REG_OFF_CS])
        _ASM(push word ptr [bx + DPMI_REG_OFF_IP])

//input: BX as DPMI_REG PTR (seems BC doesn't need type override)
//output: full register set loaded from ptr, inclluding EBX itself
//SS:ESP excluded. load ds at last.
        _ASM(push word ptr ds:[bx + DPMI_REG_OFF_DS]);
        _ASM(push word ptr ds:[bx + DPMI_REG_OFF_FLAGS]);
        _ASM(popf)
        _ASM2(mov ax, word ptr ds:[bx + DPMI_REG_OFF_GS]);
        _ASM2(mov gs, ax)
        _ASM2(mov ax, word ptr ds:[bx + DPMI_REG_OFF_FS]);
        _ASM2(mov fs, ax)
        _ASM2(mov ax, word ptr ds:[bx + DPMI_REG_OFF_ES]);
        _ASM2(mov es, ax)
        _ASM2(mov eax, dword ptr ds:[bx + DPMI_REG_OFF_EAX]);
        _ASM2(mov ecx, dword ptr ds:[bx + DPMI_REG_OFF_ECX]);
        _ASM2(mov edx, dword ptr ds:[bx + DPMI_REG_OFF_EDX]);
        _ASM2(mov ebp, dword ptr ds:[bx + DPMI_REG_OFF_EBP]);
        _ASM2(mov esi, dword ptr ds:[bx + DPMI_REG_OFF_ESI]);
        _ASM2(mov edi, dword ptr ds:[bx + DPMI_REG_OFF_EDI]);
        _ASM2(mov ebx, dword ptr ds:[bx + DPMI_REG_OFF_EBX]);
        _ASM(pop ds)

        #ifndef DEBUG //test code
        push ax
        push bx
        xor bx, bx
        mov ah, 0x0E
        mov al, '_'
        int 0x10
        pop bx
        pop ax
        #endif //-test code

        _ASM(retf)
    _ASMLBL(callreturn:)
        #ifndef DEBUG //test code
        push ax
        push bx
        xor bx, bx
        mov ah, 0x0E
        mov al, '-'
        int 0x10
        pop bx
        pop ax
        #endif //-test code

        //load DPMI_REG ptr(DS:EBX) at stack top and store new reg by xchg
        _ASM(push ds) //ss[bp+4]
        _ASM(push ax) //ss[bp+2]
        _ASM(push bp) //ss[bp]
        _ASM2(mov bp, sp)
        _ASM2(xchg dword ptr ss:[bp+6], ebx); //DPMI_REG ptr
        _ASM2(mov ax, ss:[bp+10]); //pushed ds
        _ASM2(xchg ss:[bp+4], ax); //change ds back
        _ASM2(mov ss:[bp+10], ax); //store new ds
        _ASM(pop bp)
        _ASM(pop ax)
        _ASM(pop ds) //ds changed


//input: DS:BX as DPMI_REG ptr
//output: full register set stored to the ptr, except DS, EBX
        _ASM(push gs)
        _ASM(push fs)
        _ASM(pop word ptr ds:[bx + DPMI_REG_OFF_FS]);
        _ASM(push es)
        _ASM(pop word ptr ds:[bx + DPMI_REG_OFF_ES]);
        _ASM(pushf)
        _ASM(pop word ptr ds:[bx + DPMI_REG_OFF_FLAGS]);
        _ASM(pop word ptr ds:[bx + DPMI_REG_OFF_GS]);
        _ASM2(mov dword ptr ds:[bx + DPMI_REG_OFF_EAX], eax);
        _ASM2(mov dword ptr ds:[bx + DPMI_REG_OFF_ECX], ecx);
        _ASM2(mov dword ptr ds:[bx + DPMI_REG_OFF_EDX], edx);
        _ASM2(mov dword ptr ds:[bx + DPMI_REG_OFF_EBP], ebp);
        _ASM2(mov dword ptr ds:[bx + DPMI_REG_OFF_ESI], esi);
        _ASM2(mov dword ptr ds:[bx + DPMI_REG_OFF_EDI], edi);


        _ASM(pop eax) //new ebx
        _ASM2(mov dword ptr ds:[bx + DPMI_REG_OFF_EBX], eax);
        _ASM(pop ax) //new ds
        _ASM2(mov word ptr ds:[bx + DPMI_REG_OFF_DS], ax);
    _ASM_END

    _ASM_BEGIN
        _ASM(pop gs)
        _ASM(pop fs)
        _ASM(pop es)
        _ASM(popad)
        _ASM(popfd)
    _ASM_END
}
#pragma option -k-
static void __NAKED DPMI_RMCBTranslationEnd() {}
#pragma option -k

static void DPMI_SetupRMCB()
{
    assert(DPMI_Temp != NULL);
    assert(DPMI_RmcbMemory != 0);
    assert(DPMI_Rmcb == NULL);

    char* buff = (char*)malloc(DPMI_RMCB_SIZE);
    memset(buff, 0, DPMI_RMCB_SIZE);
    DPMI_RMCB rmcb;
    memset(&rmcb, 0, sizeof(rmcb));
    rmcb.GDTR = DPMI_Temp->gdtr;
    rmcb.IDTR = DPMI_Temp->idtr;
    rmcb.CR3 =  DPMI_PTUnmap(DPMI_Temp->PageTable0, DPMI_Ptr16ToLinear(DPMI_Temp->PageDir)); //in case DPMI_Temp->PageDir is in UMB (already mapped). CR3 need physical addr.
    rmcb.VcpiInterface = DPMI_Temp->VcpiInterface;
    rmcb.VcpiInterfaceCS = DPMI_Temp->VcpiInterfaceCS;
    rmcb.LinearDS = DPMI_Temp->LinearDS;
    rmcb.RealModeIRQ0Vec = DPMI_Temp->RealModeIRQ0Vec;
    rmcb.ProtectedModeIRQ0Vec = DPMI_Temp->ProtectedModeIRQ0Vec;
    rmcb.RealModeIRQ8Vec = DPMI_Temp->RealModeIRQ8Vec;
    rmcb.ProtectedModeIRQ8Vec = DPMI_Temp->ProtectedModeIRQ8Vec;

    uint16_t offset = 0;
    offset += sizeof(rmcb);

    rmcb.CommonEntry = (DPMI_RMCB_ENTRY)offset;
    void* Code = (void*)&DPMI_RMCBCommonEntry;
    void* CodeEnd = (void*)&DPMI_RMCBCommonEntryEnd;
    uint16_t CodeSize = (uintptr_t)CodeEnd - (uintptr_t)Code;
    memcpy_c2d(buff + offset, &DPMI_RMCBCommonEntry, CodeSize);
    offset += CodeSize;

    rmcb.SwitchPM = offset;
    Code = (void*)(DPMI_V86 ? &DPMI_VCPIProtectedMode : &DPMI_DirectProtectedMode);
    CodeEnd = (void*)(DPMI_V86 ? &DPMI_VCPIProtectedModeEnd : &DPMI_DirectProtectedModeEnd);
    CodeSize = (uintptr_t)CodeEnd - (uintptr_t)Code;
    memcpy_c2d(buff + offset, Code, CodeSize);
    offset += CodeSize;

    rmcb.SwitchRM = offset;
    Code = (void*)(DPMI_V86 ? &DPMI_VCPIRealMode : &DPMI_DirectRealMode);
    CodeEnd = (void*)(DPMI_V86 ? &DPMI_VCPIRealModeEnd : &DPMI_DirectRealModeEnd);
    CodeSize = (uintptr_t)CodeEnd - (uintptr_t)Code;
    memcpy_c2d(buff + offset, Code, CodeSize);
    offset += CodeSize;

    rmcb.Translation = offset;
    Code = (void*)&DPMI_RMCBTranslation;
    CodeEnd = (void*)&DPMI_RMCBTranslationEnd;
    CodeSize = (uintptr_t)CodeEnd - (uintptr_t)Code;
    memcpy_c2d(buff + offset, Code, CodeSize);
    offset += CodeSize;

    //_LOG("%d %d %d\n", offset, DPMI_RMCB_STACK_SIZE, DPMI_RMCB_SIZE);
    assert(offset + sizeof(sizeof(DPMI_REG))*4 + DPMI_RMCB_TRASNLATION_STACK_SIZE*4 + DPMI_RMCB_STACK_SIZE < DPMI_RMCB_SIZE);
    for(int i = 0; i < DPMI_RMCB_COUNT; ++i)
    {
        Code = (void*)&DPMI_RMCBEntryIRET;
        CodeEnd = (void*)&DPMI_RMCBEntryIRETEnd;
        CodeSize = (uintptr_t)CodeEnd - (uintptr_t)Code;
        //_LOG("%d\n", CodeSize);
        assert(CodeSize <= DPMI_RMCB_ENTRYCODE_SIZE);
        memcpy_c2d(rmcb.Table[i].Code, Code, CodeSize);
        rmcb.Table[i].Target = 0;
        rmcb.Table[i].UserReg = 0;
        rmcb.Table[i].Code[CodeSize-1] = 0xCF; //patch IRET (opcode=0xCF)
    }
    memcpy(buff, &rmcb, sizeof(rmcb));

    uint16_t segment = (uint16_t)(DPMI_Temp->RMCBLAddr>>4);
    DPMI_Rmcb = (DPMI_RMCB far*)MK_FP(segment, 0);
    _fmemcpy(DPMI_Rmcb, buff, DPMI_RMCB_SIZE);
    DPMI_Rmcb->Size = offset;
    DPMI_Rmcb->TranslationSP = DPMI_RMCB_SIZE;
    //PM_SP updated each time translation to real mode.
    DPMI_Rmcb->PM_SS = SEL_HIMEM_DS*8;
    DPMI_Rmcb->RM_SP = offset + DPMI_RMCB_STACK_SIZE;
    DPMI_Rmcb->RM_SEG = segment;

    DPMI_Rmcb->VcpiClient.CR3 = rmcb.CR3;
    DPMI_Rmcb->VcpiClient.GDTR_linear = DPMI_Ptr16ToLinear(&DPMI_Rmcb->GDTR)+2;
    DPMI_Rmcb->VcpiClient.IDTR_linear = DPMI_Ptr16ToLinear(&DPMI_Rmcb->IDTR)+2;
    DPMI_Rmcb->VcpiClient.LDT_Selector = SEL_LDT*8;
    DPMI_Rmcb->VcpiClient.TSS_Selector = SEL_TSS*8;
    DPMI_Rmcb->VcpiClient.EIP = 0; //set on mode swtiching
    DPMI_Rmcb->VcpiClient.CS = SEL_RMCB_CS*8;
    DPMI_Rmcb->VcpiClientAddr = DPMI_Ptr16ToLinear(&DPMI_Rmcb->VcpiClient);

    //_LOG("RMCB: %08lx\n", (DPMI_RmcbMemory&0xFFFF)<<4);
    _LOG("CR3: %08lx\n", DPMI_Rmcb->VcpiClient.CR3);
    _LOG("GDTR: %08lx, size %d, offset %08lx\n", DPMI_Ptr16ToLinear(&DPMI_Rmcb->GDTR), DPMI_Rmcb->GDTR.size, DPMI_Rmcb->GDTR.offset);
    _LOG("IDTR: %08lx, size %d, offset %08lx\n", DPMI_Ptr16ToLinear(&DPMI_Rmcb->IDTR), DPMI_Rmcb->IDTR.size, DPMI_Rmcb->IDTR.offset);
    free(buff);
}

static int DPMI_FindEmptyRMCBTableEntry()
{
    for(int i = 0; i < DPMI_RMCB_COUNT; ++i)
    {
        if(DPMI_Rmcb->Table[i].Target == NULL)
            return i;
    }
    return -1;
}

#endif//_DPMI_BC_H_