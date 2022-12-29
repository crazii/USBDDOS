#ifndef _DPMI_BC_H_
#define _DPMI_BC_H_
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <dos.h>
#include <process.h>
#include "USBDDOS/DPMI/DPMI.h"
#include "USBDDOS/PIC.H"
#include "USBDDOS/DBGUTIL.H"

typedef enum
{
    PM_NONE = 0,
    PM_DIRECT = 1, //raw
    PM_VCPI = 2,   //vcpi
}DPMI_PM_MODE;

enum
{
    SEL_4G = 1,
    SEL_SYS_DS, //gdt/idt/paging
    SEL_CS, //real time initial cs
    SEL_DS,
    SEL_0x28,   //avoid for FLAGS
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

//default descriptors
static const uint32_t DPMI_CodeDesc[2] = {0x0000FFFF, 0x000F9A00};
static const uint32_t DPMI_DataDesc[2] = {0x0000FFFF, 0x000F9200};
static const uint32_t DPMI_Data4GDesc[2] = {0x0000FFFF, 0x00CF9200}; //4G ds, Granularity, 32 bit

#define DPMI_RMCB_STACK_SIZE 128 //for switching mode
#define DPMI_RMCB_TRASNLATION_STACK_SIZE 512 //need more of them to support reentrance (translation to RM, RMCB to PM, translation to RM ...)
#define DPMI_RMCB_COUNT 16
#define DPMI_RMCB_ENTRYCODE_SIZE 6
#define DPMI_RMCB_SIZE 4096 //total size including stack
#define DPMI_REMAP_PIC 0 //not working properly for some drivers (which need handle rm interrupt in pm mode.)

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
    IDTR RMCB_OldIDTR;
    GDTR RMCB_Gdtr; //the RMCB_ prefix is added to prevent ambiguous name for ASM
    IDTR RMCB_Idtr;
    uint32_t LinearDS; //initial ds
    uint32_t CR3;
    uint32_t RMCB_VCPIInterface; //used in V86
    uint16_t RMCB_VCPIInterfaceCS; //grouped with VCPIInterface as a farcall, don't break
    uint16_t SwitchPM; //function offset
    uint16_t SwitchRM; //function offset
    uint16_t Translation; //function offset: PM2RM translation
    uint16_t PM_SP; //stack pointer
    uint16_t PM_SS; //stack segment. must follows PM_SP
    uint16_t RM_SP; //fixed stack for translation, temporary, do not save any thing on it after mode switch
    uint16_t RM_SEG; //real mode segment of RMCB (DS=CS=SS), //must follows RM_SP, as an SS

    uint16_t TranslationSP;
    uint16_t Size;
    uint8_t RealModeIRQ0Vec;
    uint8_t ProtectedModeIRQ0Vec;
    uint8_t RealModeIRQ8Vec;
    uint8_t ProtectedModeIRQ8Vec;
    //uint8_t Interrupt : 1; //interrupt status. don't remap PIC in int handling. make sure IF is always cleared: use pushf/popf instead of sti.
    uint8_t NeedRemapPIC : 1;
    uint8_t Ready : 1;
    uint8_t Unused : 6;
    uint8_t Unused2;
    DPMI_RMCB_ENTRY CommonEntry; //common entry: save context & swtich to PM to call target entry
    DPMI_RMCBTable Table[DPMI_RMCB_COUNT];
}DPMI_RMCB;

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
    uint8_t NeedRemapPIC;

    //below should be NULL if not v86
    uint32_t VCPIInterface; //used in V86
    uint16_t VCPIInterfaceCS; //grouped with VCPIInterface as a farcall, don't break
    uint16_t PageMemory; //dos alloc handle
    uint32_t PageTable0LAddr;
    uint32_t RMCBLAddr;
    PDE far* PageDir;   //page table
    PTE far* PageTable0;   //first 4M page, inited by VCPI
    PTE far* PageTalbeHimem;
    PTE far* PageTalbeHimem2; //in case XMS meory cross 4M boudnary
    uint16_t far* XMSPageHandle; //XMS handle for extra page tables
}DPMI_TempData;

extern DPMI_ADDRESSING DPMI_Addressing;
const uint32_t DPMI_XMS_Size = 192L*1024L;   //64k code+data, first 64k is system data.

static uint32_t DPMI_SystemDS = 0;  //keep GDT, IDT, page table
static uint32_t DPMI_HimemCS = 0;
static uint32_t DPMI_HimemDS = 0;
static uint16_t DPMI_XMSHimemHandle;
static uint8_t DPMI_V86;
static uint8_t DPMI_PM;
static uint8_t DPMI_TSRed;
static uint32_t DPMI_RmcbMemory;
static DPMI_RMCB far* DPMI_Rmcb; //can be accessed in PM (segment is SEL_RMCB_DS*8)
static DPMI_TempData far* DPMI_Temp;

//convert a 16 bit ptr to linear addr
#define DPMI_Ptr16ToLinear(ptr) ((((uint32_t)FP_SEG(ptr))<<4)+(uint32_t)FP_OFF(ptr))

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

//wiki.osdev.org/8259_PIC
//only remap master pic. will informs the VCPI server in v86 mode.
//use macro because the switch function need have no function calls (for simplification)
//assume outp() is optimized to out dx,al, or better use asm.
#if DPMI_REMAP_PIC
#define DPMI_RemapPIC(rmcb, pm, v86) do {\
    if(!rmcb->NeedRemapPIC) \
        break;\
    int master = pm ? rmcb->ProtectedModeIRQ0Vec : rmcb->RealModeIRQ0Vec; \
    int slave = pm ? rmcb->ProtectedModeIRQ8Vec : rmcb->RealModeIRQ8Vec; \
    int oldmask = inp(0x21); \
    outp(0x20, 0x11); \
    outp(0x21, master); \
    outp(0x21, 4); \
    outp(0x21, 1); \
    outp(0x21, oldmask); \
    if(!v86) \
        break; \
    _ASM_BEGIN \
        _ASM(push ax) \
        _ASM(push bx) \
        _ASM(push cx) \
        _ASM2(mov bx, master) \
        _ASM2(mov cx, slave) \
        _ASM2(mov ax, 0xDE0B) \
        _ASM(call fword ptr ds:[.RMCB_VCPIInterface]) \
        _ASM(pop cx)\
        _ASM(pop bx)\
        _ASM(pop ax)\
    _ASM_END \
} while(0)
#endif

#define DPMI_EnableA20() do { \
    while(inp(0x64) & 2)\
    outp(0x64, 0xd1);\
    while(inp(0x64) & 2)\
    outp(0x60, 0xdf);\
    while(inp(0x64) & 2)\
    outp(0x64, 0xff);\
} while(0)

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

//////////////////////////////////////////////////////////////////////////////
//direct (raw) mode
//////////////////////////////////////////////////////////////////////////////

static void far DPMI_DirectProtectedMode()
{
    _ASM_BEGIN
        _ASM(push ebx) //the C code uses bx but never restore it
        _ASM(pushf)
        _ASM(cli)
    _ASM_END

    DPMI_EnableA20(); //no function calls for simplification

    _ASM_BEGIN
        _ASM(sidt fword ptr ds:[.RMCB_OldIDTR+2]);//';' is added to work around vscode highlight problem
        _ASM(lgdt fword ptr ds:[.RMCB_Gdtr+2]); //2: extra padding
        _ASM(lidt fword ptr ds:[.RMCB_Idtr+2]);
        _ASM(push SEL_RMCB_CS*8)
        _ASM2(mov ax, offset reload_cs)
        _ASM2(sub ax, offset DPMI_DirectProtectedMode)
        _ASM2(add ax, ds:[.SwitchPM]);
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

#if DPMI_REMAP_PIC
    DPMI_RMCB* rmcb = (DPMI_RMCB*)0; //ds:[0]
    #pragma warn -rch //unreachable code
    #pragma warn -ccc //condition is always true/false
    DPMI_RemapPIC(rmcb, TRUE, FALSE);
    #pragma warn .rch
    #pragma warn .ccc
#endif

    _ASM_BEGIN
        _ASM(popf)
        _ASM(pop ebx)
    _ASM_END
}
#pragma option -k-
static void DPMI_DirectProtectedModeEnd() {}
#pragma option -k

static void far DPMI_DirectRealMode()
{
    _ASM_BEGIN
        _ASM(push ebx) //the C code uses bx but never restore it
        _ASM(pushf)
        _ASM(cli)
    _ASM_END

    GDTR nullgdt; // = {0}; call BC builtin and freeze - no function calls!
    nullgdt.offset = 0; nullgdt.size = 0;

#if DPMI_REMAP_PIC
    DPMI_RMCB* rmcb = (DPMI_RMCB*)0; //ds:[0]
    #pragma warn -rch
    #pragma warn -ccc
    DPMI_RemapPIC(rmcb, FALSE, FALSE);
    #pragma warn .rch
    #pragma warn .ccc
#endif

    _ASM_BEGIN
        _ASM(lgdt fword ptr nullgdt+2)
        _ASM(lidt fword ptr ds:[.RMCB_OldIDTR+2]);
        _ASM(push word ptr ds:[.RM_SEG]);
        _ASM2(mov ax, offset reload_cs2)
        _ASM2(sub ax, offset DPMI_DirectRealMode)
        _ASM2(add ax, ds:[.SwitchRM]);
        _ASM(push ax)
        _ASM2(mov eax, cr0)
        _ASM2(and al, 0xFE)
        _ASM2(mov cr0, eax)
        _ASM(retf)
        //jmp reload_cs2 //WTF. inline asm can only perform near jump via C label.
    _ASMLBL(reload_cs2:) //reload_cs2 label far
        _ASM2(mov ax, ds:[.RM_SEG]); //rmds
        _ASM2(mov ds, ax)
        _ASM2(mov es, ax)
        _ASM2(mov ss, ax)
        _ASM2(mov gs, ax)
        _ASM2(mov fs, ax)
        _ASM(popf)
        _ASM(pop ebx)
    _ASM_END
}
#pragma option -k-
static void DPMI_DirectRealModeEnd() {}
#pragma option -k

//////////////////////////////////////////////////////////////////////////////
//VCPI mode
//////////////////////////////////////////////////////////////////////////////
static uint32_t DPMI_4MPageTableLAddr;
static uint16_t far* DPMI_XMSPageHandle; //XMS handle for extra page tables
static uint16_t DPMI_MappedPages = 0; //num physical maps
static uint16_t PageTable0Offset = 0; //offset of 4M page after VCPI init, < 1024.
#define VCPI_PAGING_MEM_SIZE (16L*1024L) //4K PD, 8K page table for XMS, 4k aligment (also for DPMI_XMSPageHandle)

typedef struct //layout required by VCPI. do not change
{
    uint32_t CR3; //page directory
    uint32_t gdtr_linear;
    uint32_t idtr_linear;
    uint16_t ldt_selector;
    uint16_t tss_selector;
    uint32_t EIP;
    uint32_t CS;
}DPMI_VCPIClientStruct;

//https://www.edm2.com/index.php/Virtual_Control_Program_Interface_specification_v1
uint32_t DPMI_GetVCPIInterface(PTE far* First4M, GDT far* VcpiGDT)
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
    if(DPMI_Temp->VCPIInterface)
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
    DPMI_Temp->PageMemory = DPMI_DOSMalloc((VCPI_PAGING_MEM_SIZE+15)>>4L);
    if(DPMI_Temp->PageMemory == 0)
    {
        printf("Error: Failed to allocate memory.\n");
        exit(1);
    }
    uint16_t seg = align(DPMI_Temp->PageMemory, 4096>>4);
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

    DPMI_Temp->VCPIInterface = DPMI_GetVCPIInterface(DPMI_Temp->PageTable0, &Gdt[SEL_VCPI_CS]);
    DPMI_Temp->VCPIInterfaceCS = SEL_VCPI_CS*8;

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

static void far DPMI_VCPIProtectedMode() //intended use struct as paramter
{
    _ASM_BEGIN _ASM(push ebx) _ASM_END

    DPMI_RMCB* rmcb = (DPMI_RMCB*)0; //ds:[0], ds should be FP_SEG(rmcb) or rmcb->RM_SEG.
    DPMI_VCPIClientStruct ClientStruct; //= {0}; will call BC helper function but we cannot do that
    ClientStruct.CR3 = rmcb->CR3;
    ClientStruct.gdtr_linear = DPMI_Ptr16ToLinear(&rmcb->RMCB_Gdtr)+2;
    ClientStruct.idtr_linear = DPMI_Ptr16ToLinear(&rmcb->RMCB_Idtr)+2;
    ClientStruct.ldt_selector = SEL_LDT*8;
    ClientStruct.tss_selector = SEL_TSS*8;
    ClientStruct.CS = SEL_RMCB_CS*8;
    uint32_t ClientStructLAddr = DPMI_Ptr16ToLinear(&ClientStruct);

    //note: DO NOT access global data, since it's executed in isolated real mode segment
    _ASM_BEGIN
        _ASM(push esi)
        _ASM(pushf)
        _ASM(cli)

        _ASM2(xor ecx, ecx)
        _ASM2(mov cx, offset VCPI_PMDone) //mov ebx, offset _VCPI_PMDone //linking error
        _ASM2(sub cx, offset DPMI_VCPIProtectedMode)
        _ASM2(add cx, ds:[.SwitchPM]);
        _ASM2(mov dword ptr ClientStruct.EIP, ecx)
        _ASM2(mov esi, ClientStructLAddr)
        _ASM2(mov cx, sp) //save SP to CX
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

#if DPMI_REMAP_PIC
    #pragma warn -rch
    #pragma warn -ccc
    DPMI_RemapPIC(rmcb, TRUE, TRUE); //note: remap PIC after switch to PM & before popf
    #pragma warn .rch
    #pragma warn .ccc
#endif

    _ASM_BEGIN
        _ASM(pop bx)

        _ASM(pop ax)  //note: V86 mode change involves flags change. don't popf directly, only keep IF. or VCPI will crash on return v86
        _ASM2(and ax, CPU_IFLAG)//IF
        _ASM(pushf)
        _ASM2(or word ptr ss:[bx], ax); //SI==SP
        _ASM(popf)

        _ASM(pop esi)
        _ASM(pop ebx)
    _ASM_END
}
#pragma option -k-
static void DPMI_VCPIProtectedModeEnd() {}
#pragma option -k

static void far DPMI_VCPIRealMode() //VCPI PM to RM
{
    //http://www.edm2.com/index.php/Virtual_Control_Program_Interface_specification_v1#5.2_Switch_to_V86_Mode

#if DPMI_REMAP_PIC
    #pragma warn -rch
    #pragma warn -ccc
    DPMI_RemapPIC(rmcb, FALSE, TRUE); //note: remap PIC before switch to RM
    #pragma warn .rch
    #pragma warn .ccc
#endif

    _ASM_BEGIN
        _ASM(push ebx)
        _ASM(pushf)
        _ASM(cli)

        _ASM2(movzx ebx, sp)
        _ASM(clts) //clear TS bits. we have no task.
        _ASM2(xor eax, eax)
        _ASM2(mov ax, ds:[.RM_SEG]); //ds:[RM_SEG], ds should be SEL_RMCB_DS*8

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
        _ASM2(mov ax, offset VCPI_to_real) //mov eax, offset back_to_real //link err
        _ASM2(sub ax, offset DPMI_VCPIRealMode)
        _ASM2(add ax, es:[.SwitchRM]);
        _ASM(push eax)    //EIP

        _ASM2(mov ax, SEL_4G*8)
        _ASM2(mov ds, ax)
        _ASM2(mov ax, 0xDE0C)
        _ASM(call fword ptr es:[.RMCB_VCPIInterface]); //EAX modified (and segments push on stack)
    _ASMLBL(VCPI_to_real:)
        _ASM(pop ax)
        _ASM2(and ax, CPU_IFLAG) //IF
        _ASM(pushf)
        _ASM2(or word ptr ss:[bx], ax); //BX=SP

        _ASM(popf)
        _ASM(pop ebx)
    _ASM_END
}
#pragma option -k-
static void DPMI_VCPIRealModeEnd() {}
#pragma option -k

//////////////////////////////////////////////////////////////////////////////
//mode switch
//////////////////////////////////////////////////////////////////////////////

//swtich to protected mode.
static int16_t DPMI_SwitchProtectedMode(uint32_t LinearDS)
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
        _ASM2(mov sp, ss:[bx.RM_SP]);
        _ASM(push SEL_CS*8)
        _ASM(push offset SwitchPMDone)
        _ASM(push ss)
        _ASM(push word ptr ss:[bx.SwitchPM]);
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
static void DPMI_SwitchRealMode(uint32_t LinearDS)
{
    if(!DPMI_PM)
        return;
    CLIS();
    uint16_t segment = DPMI_Rmcb->RM_SEG; //save on stack. DPMI_Rmcb won't be accessible after switching mode

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
        _ASM2(mov ss:[bx.PM_SP], sp);
        _ASM2(mov sp, ss:[bx.RM_SP]);
        _ASM(push _TEXT)
        _ASM(push offset SwitchRMDone)
        _ASM(push SEL_RMCB_CS*8)
        _ASM(push word ptr ss:[bx.SwitchRM]);
        _ASM2(mov ax, ss)
        _ASM2(mov ds, ax)
        _ASM(retf)
    _ASMLBL(SwitchRMDone:)
        _ASM2(mov ax, _DATA)
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
}

//////////////////////////////////////////////////////////////////////////////
//IDT
//////////////////////////////////////////////////////////////////////////////
static void (*DPMI_UserINTHandler[256])(void); //TODO: software int handlers

#pragma option -k-
static void DPMI_NullINTHandler()
{
    _ASM_BEGIN _ASM(iret) _ASM_END
}
#pragma option -k

#define DPMI_EXCEPT(n) static void DPMI_ExceptionHandler##n()\
{\
    _ASM_BEGIN _ASM(push ds) _ASM(pushad) _ASM2(mov ax, SEL_HIMEM_DS*8) _ASM2(mov ds, ax) _ASM2(mov ss, ax) _ASM2(mov sp, 0xFFF8) _ASM_END \
    printf("Exception: %02x\n", n);\
    if(!DPMI_TSRed) exit(1);\
    else while(1);\
}

#pragma option -k-
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
#pragma option -k

#define DPMI_EXCEPT_ADDR(n) FP_OFF(&DPMI_ExceptionHandler##n)

static void DPMI_HWIRQHandlerInternal()
{
    uint8_t irq = PIC_GetIRQ();
    //if(irq < 16)
    {
        uint8_t vec = PIC_IRQ2VEC(irq);
        //assert(vec >= 0x08 && vec <= 0x0F || vec >= 0x70 && vec <= 0x77);
        if(DPMI_UserINTHandler[vec])
            DPMI_UserINTHandler[vec]();
        else
        {
            static DPMI_REG r = {0};
            DPMI_CallRealModeINT(vec, &r); //call real mode IRQ handler
        }
    }
}

#pragma option -k-
static void DPMI_HWIRQHandler()
{
    //test if it is a normal INT or exception.
    _ASM_BEGIN
        _ASM(push bp)
        _ASM2(mov bp, sp)
        _ASM2(cmp word ptr [bp+6], SEL_HIMEM_CS*8) //normal int: [bp,] ip,cs,flags, exception: [bp,] errorcode,ip,cs,flags (0x08~0x0F). not handling ring3 excptions.
        _ASM(jne NormalINT)
        //what if flags == SEL_HIMEM_CS*8 by accident? need test [bp+4] for normal INT. add emtpy SEL_0x28 avoid conflict
        _ASM2(cmp word ptr [bp+4], SEL_HIMEM_CS*8)
        _ASM(je NormalINT)
        _ASM2(cmp word ptr [bp+4], SEL_RMCB_CS*8)
        _ASM(je NormalINT)
    _ASMLBL(Except:)
        _ASM(call DPMI_ExceptionHandler0x09)
    _ASMLBL(NormalINT:)
        _ASM(pop bp)
    _ASM_END

    _ASM_BEGIN _ASM(pushf) _ASM(cli) _ASM(pushad) _ASM(push ds) _ASM(push es) _ASM(push fs) _ASM(push gs) _ASM_END
    _ASM_BEGIN _ASM2(mov bp, sp) _ASM2(and word ptr [bp+40], 0xBFFF) _ASM_END //remove NT flag for flags or iret will do a task return
    _ASM_BEGIN _ASM2(mov ax, SEL_HIMEM_DS*8) _ASM2(mov ds, ax) _ASM2(mov es, ax) _ASM_END

    DPMI_HWIRQHandlerInternal();

    _ASM_BEGIN _ASM(pop gs) _ASM(pop fs) _ASM(pop es) _ASM(pop ds) _ASM(popad) _ASM(popf) _ASM(iret) _ASM_END
}
#pragma option -k

//////////////////////////////////////////////////////////////////////////////
//RMCB
//////////////////////////////////////////////////////////////////////////////
#define DPMI_RMCB_TEST 0
static const int RMCB_TableSize = sizeof(DPMI_RMCBTable); //use C to get stable size.
static const int DPMI_REG_Size = sizeof(DPMI_REG);
#pragma option -k-
#if DPMI_RMCB_TEST
static void DPMI_RmcbLog()
{
    _LOG("RMCB TEST");
    _ASM_BEGIN _ASM(retf) _ASM_END
}
#endif
static void DPMI_RMCbIRet() //wrapper function for user rmcb (allow user uses normal ret). copy DPMI_REG registers and iret
{
    _ASM_BEGIN
        //save IRET frame to register
        _ASM(pop ax)
        _ASM(pop dx)
        _ASM(pop bx)

        _ASM(pop ecx) _ASM(pop esi) _ASM(pop ds) _ASM(pop edi) _ASM(pop es) //copy back reverse ds:si with es:di
        _ASM(cld)
        _ASM2(rep movs byte ptr es:[edi], byte ptr ds:[esi]);

        //_ASM(push bx)
        _ASM(push dx)
        _ASM(push ax)
        _ASM(retf)
    _ASM_END
}
static void DPMI_RMCBCommonEntry() //commen entry for call back
{
    _ASM_BEGIN
        //_ASM2(add sp, 4) //ugly fix compiler: BC will push si/di if inline asm uses si/di - use EDI,ESI to avoid the problem

        //save context and make a DPMI_REG struct
        _ASM(push ss) _ASM(push sp) //ss: sp
        _ASM(push cs) _ASM(push ax) //cs : (fake) ip
        _ASM(push gs) _ASM(push fs) _ASM(push ds) _ASM(push es)
        _ASM(pushf)
        _ASM(pushad)

        //get caller IP and calc target. target addr = TableOffset + (IP-TableOffset)/TableSize*TableSize + 0 (offsetof(DPMI_RMCBTable, Target))
        //note: this should be done before swtich stack
        _ASM2(mov bp, sp);
        _ASM2(mov ax, ss:[bp+DPMI_REG_Size]); //caller IP (RMCB entry)
        _ASM2(mov cx, RMCB_TableSize) //optmized as imm
        _ASM2(sub ax, .Table) //.Table == offsetof(DPMI_RMCB, Table)
        _ASM2(xor dx, dx)
        _ASM(div cx)
        _ASM(mul cx)
        _ASM2(add ax, .Table) //calc offset done
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
        _ASM2(lss sp, cs:[.RM_SP]);

        _ASM(push cs)
        _ASM(pop ds)  
        _ASM(push SEL_RMCB_CS*8) //far return from switch
        _ASM(call word ptr cs:[.SwitchPM]); //far call with pushed cs
        //now in pm, fs=gs=es=ss=ds=SEL_RMCB_DS*8

        //switch himem stack. ss=SEL_HIMEM_DS*8
        _ASM2(lss sp, ds:[.PM_SP]);
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
        _ASM2(mov ax, ds:[esi+32]);
        _ASM(push ax)
        _ASM(popf)

        _ASM2(movzx edi, bx) //pm ss:[bx] to linear
        _ASM2(add edi, dword ptr ss:[DPMI_HimemDS]);
        _ASM2(mov ecx, DPMI_REG_Size)
        _ASM(push ds) _ASM(push esi) _ASM(push es) _ASM(push edi) _ASM(push ecx) //popped by DPMI_RMCbIRet
        _ASM(cld)
        _ASM2(rep movs byte ptr es:[edi], byte ptr ds:[esi]);
    _ASMLBL(SkipCopyREG:)
        //call target pm function
        _ASM(pushf) //push return address. assume target is IRET
        _ASM(push cs)
        _ASM2(mov ax, offset RMCB_PmReturn)
        _ASM2(sub ax, offset DPMI_RMCBCommonEntry)
        _ASM2(add ax, fs:[.CommonEntry]);
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
        _ASM2(add ax, ds:[.CommonEntry]);
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
        _ASM2(mov sp, ds:[.RM_SP]);
        _ASM(push word ptr ds:[.RM_SEG]);
        _ASM(call word ptr ds:[.SwitchRM]);
        //now in rm. fs=gs=es=ss=ds=RMCB segement
    _ASM_END

    _ASM_BEGIN
        _ASM2(mov ss, bx) //ss:sp
        _ASM2(mov sp, bp)

        _ASM(popad)
        _ASM(popf)
        _ASM(pop es) _ASM(pop ds) _ASM(pop fs) _ASM(pop gs)
        _ASM2(add sp, 8) //cs, ip, sp, ss
    _ASM_END
}
static void DPMI_RMCBCommonEntryEnd() {}
#pragma option -k

#pragma option -k-
static void DPMI_RMCBEntryIRET() //keep it as small as possible
{
    _ASM_BEGIN
        _ASM(call word ptr cs:[.CommonEntry]); //word ptr=near, dword ptr=far cs16:off16, fword=far cs16:off32
        //there's a RET generated by compiler, will be patched to IRET later.
    _ASM_END
}
static void DPMI_RMCBEntryIRETEnd() {}
#pragma option -k
static const int DPMI_RMCBEntrySize = (uintptr_t)DPMI_RMCBEntryIRETEnd - (uintptr_t)DPMI_RMCBEntryIRET;

//input: BX as DPMI_REG PTR (seems BC doesn't need type override)
//output: full register set loaded from ptr, inclluding EBX itself
//SS:ESP excluded. load ds at last.
#define DPMI_LoadRealModeRegsPtr() do {\
    _ASM_BEGIN\
        _ASM(push word ptr ds:[bx.w.ds])\
        \
        _ASM(push word ptr ds:[bx.w.flags])\
        _ASM(popf)\
        _ASM2(mov ax, word ptr ds:[bx.w.gs])\
        _ASM2(mov gs, ax)\
        _ASM2(mov ax, word ptr ds:[bx.w.fs])\
        _ASM2(mov fs, ax)\
        _ASM2(mov ax, word ptr ds:[bx.w.es])\
        _ASM2(mov es, ax)\
        _ASM2(mov eax, dword ptr ds:[bx.d.eax])\
        _ASM2(mov ecx, dword ptr ds:[bx.d.ecx])\
        _ASM2(mov edx, dword ptr ds:[bx.d.edx])\
        _ASM2(mov ebp, dword ptr ds:[bx.d.ebp])\
        _ASM2(mov esi, dword ptr ds:[bx.d.esi])\
        _ASM2(mov edi, dword ptr ds:[bx.d.edi])\
        \
        _ASM2(mov ebx, dword ptr ds:[bx.d.ebx])\
        _ASM(pop ds)\
    _ASM_END \
} while(0)

//input: DS:BX as DPMI_REG ptr
//output: full register set stored to the ptr, except DS, EBX
#define DPMI_StoreRealModeRegsPtr() do {\
    _ASM_BEGIN \
        _ASM(push gs)\
        _ASM(push fs)\
        _ASM(pop word ptr ds:[bx.w.fs])\
        _ASM(push es)\
        _ASM(pop word ptr ds:[bx.w.es])\
        _ASM(pushf)\
        _ASM(pop word ptr ds:[bx.w.flags])\
        _ASM(pop word ptr ds:[bx.w.gs])\
        _ASM2(mov dword ptr ds:[bx.d.eax], eax)\
        _ASM2(mov dword ptr ds:[bx.d.ecx], ecx)\
        _ASM2(mov dword ptr ds:[bx.d.edx], edx)\
        _ASM2(mov dword ptr ds:[bx.d.ebp], ebp)\
        _ASM2(mov dword ptr ds:[bx.d.esi], esi)\
        _ASM2(mov dword ptr ds:[bx.d.edi], edi)\
    _ASM_END \
} while(0)

//INTn < 256 (hi byte 0): interrupt(cs:ip) with iret, push flags.
static void far _pascal DPMI_RMCBTranslation(DPMI_REG* reg, unsigned INTn, BOOL directcall)
{
   _ASM_BEGIN
        _ASM(pushf)
        _ASM(pushad)
        _ASM(push es) _ASM(push fs) _ASM(push gs)

        _ASM(push ds)
        _ASM2(mov bx, reg) //bp[xxx]
        _ASM(push ebx)    //save DPMI_REG ptr
        _ASM2(mov ax, INTn)
        _ASM2(test ah, ah)
        _ASM(jnz skip_flags)
        _ASM(push word ptr [bx.w.flags])
        _ASM2(and word ptr [bx.w.flags], 0xFCFF) //clear IF TF (by Intel INTn instruction reference).(AC in hiword)
    _ASMLBL(skip_flags:)
        _ASM(push cs)
        _ASM2(mov ax, offset callreturn)
        _ASM2(mov cx, directcall)
        _ASM2(test cx, cx)
        _ASM(jnz skip_offset_adjst)
        _ASM2(sub ax, offset DPMI_RMCBTranslation)
        _ASM2(add ax, ds:[.Translation]);
    _ASMLBL(skip_offset_adjst:)
        _ASM(push ax)
        _ASM(push word ptr [bx.w.cs])
        _ASM(push word ptr [bx.w.ip])
    _ASM_END

    DPMI_LoadRealModeRegsPtr();//DS:BX

    _ASM_BEGIN
        _ASM(retf)
    _ASMLBL(callreturn:)
        //load DPMI_REG ptr(DS:EBX) at stack top and store new reg by xchg
        _ASM(push ds)
        _ASM(push ax)
        _ASM(push bp)
        _ASM2(mov bp, sp)
        _ASM2(xchg dword ptr ss:[bp+6], ebx);
        _ASM2(mov ax, ss:[bp+10]);
        _ASM2(xchg ss:[bp+4], ax); //change ds back
        _ASM2(mov ss:[bp+10], ax);
        _ASM(pop bp)
        _ASM(pop ax)
        _ASM(pop ds) //ds changed
    _ASM_END

    DPMI_StoreRealModeRegsPtr();//DS:BX

    _ASM_BEGIN
        _ASM(pop eax) //new ebx
        _ASM2(mov dword ptr ds:[bx.d.ebx], eax);
        _ASM(pop ax) //new ds
        _ASM2(mov word ptr ds:[bx.w.ds], ax);
    _ASM_END

    _ASM_BEGIN
        _ASM(pop gs) _ASM(pop fs) _ASM(pop es)
        _ASM(popad)
        _ASM(popf)
    _ASM_END
}
#pragma option -k-
static void DPMI_RMCBTranslationEnd() {}
#pragma option -k

static void DPMI_SetupRMCB()
{
    assert(DPMI_Temp != NULL);
    assert(DPMI_RmcbMemory != 0);
    assert(DPMI_Rmcb == NULL);

    char buff[DPMI_RMCB_SIZE];
    memset(buff, 0, sizeof(buff));
    DPMI_RMCB rmcb;
    memset(&rmcb, 0, sizeof(rmcb));
    rmcb.RMCB_Gdtr = DPMI_Temp->gdtr;
    rmcb.RMCB_Idtr = DPMI_Temp->idtr;
    rmcb.LinearDS = DPMI_Temp->LinearDS;
    rmcb.RMCB_VCPIInterface = DPMI_Temp->VCPIInterface;
    rmcb.RMCB_VCPIInterfaceCS = DPMI_Temp->VCPIInterfaceCS;
    rmcb.CR3 = DPMI_PTUnmap(DPMI_Temp->PageTable0, DPMI_Ptr16ToLinear(DPMI_Temp->PageDir)); //in case DPMI_Temp->PageDir is in UMB (already mapped). CR3 need physical addr.
    rmcb.NeedRemapPIC = DPMI_Temp->NeedRemapPIC;
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
    assert(offset + sizeof(sizeof(DPMI_REG))*2 + DPMI_RMCB_TRASNLATION_STACK_SIZE*4 + DPMI_RMCB_STACK_SIZE < DPMI_RMCB_SIZE);
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

    uint16_t segment = DPMI_Temp->RMCBLAddr>>4;
    DPMI_Rmcb = (DPMI_RMCB far*)MK_FP(segment, 0);
    _fmemcpy(DPMI_Rmcb, buff, DPMI_RMCB_SIZE);
    DPMI_Rmcb->Size = offset;
    DPMI_Rmcb->TranslationSP = DPMI_RMCB_SIZE;
    //PM_SP updated each time translation to real mode.
    DPMI_Rmcb->PM_SS = SEL_HIMEM_DS*8;
    DPMI_Rmcb->RM_SP = offset + DPMI_RMCB_STACK_SIZE;
    DPMI_Rmcb->RM_SEG = segment;

    //_LOG("RMCB: %08lx\n", (DPMI_RmcbMemory&0xFFFF)<<4);
    _LOG("CR3: %08lx\n", DPMI_Rmcb->CR3);
    _LOG("GDTR: %08lx, size %d, offset %08lx\n", DPMI_Ptr16ToLinear(&DPMI_Rmcb->RMCB_Gdtr), DPMI_Rmcb->RMCB_Gdtr.size, DPMI_Rmcb->RMCB_Gdtr.offset);
    _LOG("IDTR: %08lx, size %d, offset %08lx\n", DPMI_Ptr16ToLinear(&DPMI_Rmcb->RMCB_Idtr), DPMI_Rmcb->RMCB_Idtr.size, DPMI_Rmcb->RMCB_Idtr.offset);
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