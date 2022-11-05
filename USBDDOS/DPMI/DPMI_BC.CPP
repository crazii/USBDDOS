#include "USBDDOS/DPMI/DPMI.h"
#if defined(__BC__) || 1
//don't use DPMI on Boarland C, instead use 16bit protected mode and keep the code and data near (no far calls/far data).
//note: the inline assember won't reconginze 386 instructions and registers, code need compiled with -B flag (compile via assembly) in BC31
//
//if in real mode, direct switch to protected mode, without paging, otherwise use VCPI to switch to 16 bit protected mode with 1:1 paging.
//small model, 64K+64K
#include "USBDDOS/DPMI/XMS.H"
#include <conio.h>
#include <stdlib.h>
#include <dos.h>
#include <alloc.h>
#include <malloc.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>

#define DPMI_HIMEM 1
#define DPMI_DEBUG 1

//we need IF store on stack because only stack is copied on mode switch
#undef CLIS
#undef STIL
static const int DPMI_NIFLAG = ~CPU_IFLAG;
#define CLIS() __asm{pushf; cli;}

/*#define STIL() __asm{push ax; push bp; pushf; pop ax; mov bp, sp;\
xchg ax, [bp+4]; and ax, CPU_IFLAG; or [bp+4], ax;\
pop bp; pop ax; popf;} //general case, preserves all registers*/

#define STIL() __asm{ pop ax; and ax, CPU_IFLAG;\
pushf; pop dx; and dx, DPMI_NIFLAG;\
or dx, ax; push dx; popf;} //function exit case, uses ax,dx*/

typedef enum
{
    PM_NONE = 0,
    PM_DIRECT = 1, //raw
    PM_VCPI = 2,   //vcpi
}DPMI_PM_MODE;

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

static inline PDE DPMI_LoadPDE(uint32_t addr) {PDE pde; *(uint32_t*)&pde = DPMI_LoadD(addr); return pde;}
static inline PDE DPMI_LoadPDE(const PDE far* pd, uint32_t i) { return DPMI_LoadPDE(DPMI_Ptr16ToLinear(pd)+i*sizeof(PDE));}
static inline void DPMI_StorePDE(uint32_t addr, const PDE* pde) {DPMI_StoreD(addr, *(uint32_t*)pde);}
static inline void DPMI_StorePDE(PDE far* pd, uint32_t i, const PDE* pde) {DPMI_StorePDE(DPMI_Ptr16ToLinear(pd)+i*sizeof(PDE), pde);}

#define DPMI_LoadPTE DPMI_LoadPDE
#define DPMI_StorePTE DPMI_StorePDE

enum
{
    SEL_4G = 1,
    SEL_CS,
    SEL_DS,
    SEL_XMS_CS,
    SEL_XMS_DS,

    SEL_VCPI_CS,//VCPI*3
    SEL_VCPI_1,
    SEL_VCPI_2,
    SEL_LDT,    //VCPI dummy
    SEL_TSS,    //VCPI dummy

    #if DPMI_HIMEM
    SEL_CUR_DS = SEL_XMS_DS,
    #else
    SEL_CUR_DS = SEL_DS,
    #endif
};

static uint32_t DPMI_GDT[] =
{
    0,0,
    0x0000FFFF, 0x00CF9200,  //4G ds, Granularity, 32 bit
    0x0000FFFF, 0x000F9A00,  //current cs, 16bit
    0x0000FFFF, 0x000F9200,  //current ds, 16bit
    0x0000FFFF, 0x000F9A00,  //XMS cs, 16bit
    0x0000FFFF, 0x000F9200,  //XMS ds, 16bit
};//no break here, consecutive with DPMI_VCPIGDT
static GDT DPMI_VCPIGDT[] =
{
    {0},
    {0},
    {0},
    {0},
    {0},
};

extern DPMI_ADDRESSING DPMI_Addressing;
const uint32_t DPMI_XMS_Size = 128L*1024L;   //64k code+data, first 64k is code.

static uint32_t DPMI_LinearCS = 0;  //real mode cs linear base
static uint32_t DPMI_LinearDS = 0;  //real mode ds linear base
static uint32_t DPMI_HimemCS = 0;
static uint32_t DPMI_HimemDS = 0;
static uint16_t DPMI_XMSHimemHandle;
static BOOL DPMI_INT = FALSE;    //interrupt status. don't remap PIC in int handling. make sure IF is always cleared: use pushf/popf instead of sti.
static DPMI_PM_MODE DPMI_PM = PM_NONE;

static GDTR DPMI_GDTR = {0, sizeof(DPMI_GDT)-1, 0};
static IDTR DPMI_IDTR = {0, 0, 0};
static IDT far* DPMI_IDT = NULL;
static uint32_t DPMI_PM_PIC_MIVEC = 0x28;
static const uint32_t DPMI_RM_PIC_MIVEC = 0x08;
static uint32_t DPMI_PM_PIC_SIVEC = 0x70; //don't change slave pic
static const uint32_t DPMI_RM_PIC_SIVEC = 0x70;
static uint16_t DPMI_IDTBuffer = 0;
static BOOL DPMI_V86 = 0;
static BOOL DPMI_PIC_NeedRemap = TRUE;

static BOOL DPMI_IsV86() //should call before init pm
{
    __asm { //note: VM in eflags(V86) bit 17 never push on stack, so pushf/pop won't work
        smsw ax    //machine state word (low word of cr0)
        and ax, 0x1 //PE
    }
    return _AX;
}


static void DPMI_RemapPIC(int vec) //wki.osdev.org/8259_PIC
{
    if(!DPMI_INT && DPMI_PIC_NeedRemap)
    {
        int oldmask = inp(0x21);
        outp(0x20, 0x11);
        outp(0x21, vec);
        outp(0x21, 4);
        outp(0x21, 1);
        outp(0x21, oldmask);

        if(DPMI_V86)
        {
            __asm { //this only informs the VCPI server
                push bx
                mov bx, vec
                mov cx, DPMI_PM_PIC_SIVEC
                mov ax, 0xDE0B
                call fword ptr ds:DPMI_VCPIInterface //declared backwards
                pop bx
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////
//direct (raw) mode
//////////////////////////////////////////////////////////////////////////////
static IDTR DPMI_IDTR_IVT = {0, 0, 0};

static BOOL DPMI_EnableA20(void)
{
    if(!DPMI_INT)
    {
        while(inp(0x64) & 2)
        outp(0x64, 0xd1);
        while(inp(0x64) & 2)
        outp(0x60, 0xdf);
        while(inp(0x64) & 2)
        outp(0x64, 0xff);
    }
    return TRUE;
}

static BOOL DPMI_DirectProtectedMode(void)
{
    assert(!DPMI_PM);
    if(!DPMI_EnableA20())
    {
        printf("Error: failed enable A20.\n");
        return FALSE;
    }
    __asm {
        pushf
        cli
        sidt fword ptr DPMI_IDTR_IVT+2
        lgdt fword ptr DPMI_GDTR+2 //extra padding
        lidt fword ptr DPMI_IDTR+2
        push DPMI_PM_PIC_MIVEC
        call DPMI_RemapPIC
        add sp, 2

        mov eax, cr0
        or al, 0x1      //PE
        mov cr0, eax
        push SEL_CS*8
        push offset reload_cs
        retf
        reload_cs:
        mov ax, SEL_DS*8
        mov ds, ax
        mov es, ax
        mov ss, ax
        mov fs, ax
        mov gs, ax
        popf
    }
    return TRUE;
}

static void DPMI_DirectRealMode()
{
    GDTR nullgdt = {0};
    __asm {
        pushf
        cli
        push DPMI_RM_PIC_MIVEC
        call DPMI_RemapPIC
        add sp, 2

        lgdt fword ptr nullgdt+2
        lidt fword ptr DPMI_IDTR_IVT+2;
        mov eax, cr0
        and eax, 0xFFFFFFFE
        mov cr0, eax
        push _TEXT
        push offset reload_cs2
        retf
        //jmp reload_cs2 //WTF. inline asm can only perform near jump via C label.
    reload_cs2 label far
        mov ax, _DATA //rmds
        mov ds, ax
        mov es, ax
        mov ss, ax
        mov gs, ax
        mov fs, ax
        popf
    }
}

//////////////////////////////////////////////////////////////////////////////
//VCPI mode
//////////////////////////////////////////////////////////////////////////////
static uint16_t DPMI_PageBuffer = 0; //dos alloc handle
static PDE far* DPMI_PageDir = NULL; //page table
static PTE far* DPMI_PageTable4M = NULL;   //first 4M page
static PTE far* DPMI_PageTableXMS = NULL;   //mapped for himem
static uint16_t far* DPMI_XMSPageHandle = NULL; //page table for physical maps
static uint16_t DPMI_MappedPages = 0; //num physical maps
static uint16_t DPMI_PageTable4MOffset = 0; //offset of 4M page after VCPI init, < 1024.
struct
{
    uint32_t CR3;//page directory
    uint32_t gdtr_linear;
    uint32_t idtr_linear;
    uint16_t ldt_selector;
    uint16_t tss_selector;
    uint32_t EIP;
    uint32_t CS;
}DPMI_VCPIClientStruct = {0};

static uint32_t DPMI_VCPIInterface = 0;
static uint16_t DPMI_VCPIInterfaceCS = 0; //grouped with DPMI_VCPIInterface as a farcall, don't break

//https://www.edm2.com/index.php/Virtual_Control_Program_Interface_specification_v1
uint32_t DPMI_GetVCPIInterface(PTE far* First4M, GDT* VcpiGDT)
{
    uint16_t table_seg = FP_SEG(First4M);
    uint16_t table_off = FP_OFF(First4M);
    uint16_t new_off = 0;
    uint16_t gdt_off = FP_OFF(VcpiGDT);
    uint32_t interface = 0;
    uint8_t result = -1;
    __asm {
        push ax
        push es
        push di
        push si
        push ebx
        mov ax, table_seg
        mov es, ax
        mov di, table_off
        mov si, gdt_off
        mov ax, 0xDE01
        int 0x67
        mov dword ptr interface, ebx
        mov new_off, di
        mov result, ah
        pop ebx
        pop si
        pop di
        pop es
        pop ax
    }
    if(result != 0)
    {
        printf("Error: faield to get VCPI interface.\n");
        exit(1);
    }
    assert(sizeof(PTE) == 4);
    DPMI_PageTable4MOffset = (new_off - table_off)/sizeof(PTE);
    #if 0 //dump vcpi pages
    {
        int start = 0;
        for(uint32_t j = start; j < DPMI_PageTable4MOffset; ++j)
            printf(((((j-start+1)%8)==0)?"%08lx\n":"%08lx "), (*(uint32_t far*)(First4M+j)));
        printf("\n");
    }
    #endif
    //fill the un-inited table entry if VCPI didn't init them all
    /*for(uint32_t i = DPMI_PageTable4MOffset; i < 1024; ++i)
    {
        PTE pte = {0};
        PTE_INIT(pte, i<<12);//1:1 map
        First4M[i] = pte;
    }//*/
    return interface;
}

static BOOL DPMI_InitVCPI()
{
    if(DPMI_VCPIInterface)
        return TRUE;

    volatile BOOL VCPIPresent = FALSE;
    __asm { //check vcpi
        push ax
        mov ax, 0xDE00
        int 0x67
        not ax
        mov VCPIPresent, ax
        pop ax
    }
    if(!VCPIPresent)
    {
        printf("Error: Virtual 8086 mode detected but no VCPI.\n");
        exit(1);
    }
    //prepare paging, used dos malloc to save space for the driver. it will be freed on TSR (execept interrupt handler needed)
    DPMI_PageBuffer = DPMI_DOSMalloc((16*1024)>>4); //4K PD, 4K page table for VCPI, 4K page table for XMS, 4k aligment
    if(DPMI_PageBuffer == 0)
    {
        printf("Error: Failed to allocate memory.\n");
        exit(1);
    }
    DPMI_PageDir = (PDE far*)MK_FP(align(DPMI_PageBuffer, 4096>>4),0);
    DPMI_PageTable4M = (PTE far*)MK_FP(align(DPMI_PageBuffer, 4096>>4), 4096);
    DPMI_PageTableXMS = (PTE far*)MK_FP(align(DPMI_PageBuffer, 4096>>4), 4096+4096);
    _fmemset(DPMI_PageDir, 0, 4096);
    _fmemset(DPMI_PageTable4M, 0, 4096);
    _fmemset(DPMI_PageTableXMS, 0, 4096);
    //find an align gap to place xms map handle
    uint32_t linearPD = DPMI_Ptr16ToLinear(DPMI_PageDir); //linear&physical
    if(linearPD - ((uint32_t)DPMI_PageBuffer<<4L) >= 1024L*sizeof(uint16_t))
    {
        DPMI_XMSPageHandle = (uint16_t far*)MK_FP(DPMI_PageBuffer,0);
        assert(DPMI_Ptr16ToLinear(DPMI_XMSPageHandle+1024) <= linearPD);
    }
    else
    {
        DPMI_XMSPageHandle = (uint16_t far*)MK_FP(align(DPMI_PageBuffer, 4096>>4), 4096L+4096L+4096L);
        assert(DPMI_Ptr16ToLinear(DPMI_XMSPageHandle) >= DPMI_Ptr16ToLinear(DPMI_PageTableXMS)+4096L);
        assert(DPMI_Ptr16ToLinear(DPMI_XMSPageHandle+1024) <= ((uint32_t)DPMI_PageBuffer<<4)+16L*1024L);
    }
    _fmemset(DPMI_XMSPageHandle, 0, 1024L*sizeof(uint16_t));

    PDE pde = {0};
    uint32_t pt0 = DPMI_Ptr16ToLinear(DPMI_PageTable4M);
    assert((pt0&0xFFF) == 0);
    PDE_INIT(pde, pt0);
    DPMI_PageDir[0] = pde;

    //dummy. not used, make it looks valid. (with invalid 0 base)
    DPMI_VCPIGDT[SEL_LDT-SEL_VCPI_CS].limit_low = sizeof(LDT)-1;
    DPMI_VCPIGDT[SEL_LDT-SEL_VCPI_CS].read_write = 1;
    DPMI_VCPIGDT[SEL_LDT-SEL_VCPI_CS].present = 1;
    //TODO: TSS type in descriptor.
    DPMI_VCPIGDT[SEL_TSS-SEL_VCPI_CS].limit_low = sizeof(TSS32)-1;
    DPMI_VCPIGDT[SEL_TSS-SEL_VCPI_CS].type = 1; //0x89
    DPMI_VCPIGDT[SEL_TSS-SEL_VCPI_CS].accessed = 1;
    DPMI_VCPIGDT[SEL_TSS-SEL_VCPI_CS].present = 1;

    DPMI_GDTR.size += sizeof(DPMI_VCPIGDT);

    assert((DPMI_Ptr16ToLinear(DPMI_PageDir)&0xFFF)==0);
    DPMI_VCPIClientStruct.CR3 = linearPD;
    DPMI_VCPIClientStruct.gdtr_linear = DPMI_Ptr16ToLinear(&DPMI_GDTR)+2;
    DPMI_VCPIClientStruct.idtr_linear = DPMI_Ptr16ToLinear(&DPMI_IDTR)+2;
    DPMI_VCPIClientStruct.ldt_selector = SEL_LDT*8;
    DPMI_VCPIClientStruct.tss_selector = SEL_TSS*8;
    DPMI_VCPIClientStruct.CS = SEL_CS*8;
    //DPMI_VCPIClientStruct.EIP = _VCPI_PMDone;
    DPMI_VCPIInterface = DPMI_GetVCPIInterface(DPMI_PageTable4M, DPMI_VCPIGDT);
    DPMI_VCPIInterfaceCS = SEL_VCPI_CS*8;

    #if DPMI_HIMEM //init himem page after 4M page table inited by vcpi
    {
        uint32_t pdi = DPMI_HimemCS >> 22L;
        if(pdi == 0) //himem below 4M
            pdi = 1; //append next to tb0 in case 128k himem cross table boundary
        PDE pde = {0};
        uint32_t pt = DPMI_Ptr16ToLinear(DPMI_PageTableXMS);
        assert((pt&0xFFF) == 0);
        PDE_INIT(pde, pt);
        DPMI_PageDir[pdi] = pde;
        DPMI_MapMemory(DPMI_HimemCS, 128L*1024L);
    }
    #endif
    //CLI; //soft int will set IF, need cli if called in DPMI_VCPIProtectedMode().
    return TRUE;
}

static BOOL DPMI_VCPIProtectedMode(void)
{
    uint32_t client_struct = DPMI_Ptr16ToLinear(&DPMI_VCPIClientStruct);
    __asm {
        push esi
        push ebx
        pushf
        cli
        mov esi, client_struct
        xor ebx, ebx
        mov bx, offset VCPI_PMDone //mov ecx, offset _VCPI_PMDone //linking error
        mov dword ptr DPMI_VCPIClientStruct.EIP, ebx
        mov bx, sp
        mov ax, 0xDE0C
        int 0x67
    VCPI_PMDone:
        mov ax, SEL_DS*8
        mov ds, ax
        mov es, ax
        mov ss, ax
        mov fs, ax
        mov gs, ax
        mov esp, ebx

        push DPMI_PM_PIC_MIVEC
        call DPMI_RemapPIC
        add sp, 2

        pop ax  //note: V86 mode change involves flags change. don't popf directly, only keep IF. or VCPI will crash on return v86
        and ax, CPU_IFLAG//IF
        pushf
        or word ptr ss:[bx], ax
        popf

        pop ebx
        pop esi
    }
    return TRUE;
}

static void DPMI_VCPIRealMode() //VCPI PM to RM
{
    //http://www.edm2.com/index.php/Virtual_Control_Program_Interface_specification_v1#5.2_Switch_to_V86_Mode
    __asm {
        push ebx
        pushf
        cli
        movzx ebx, sp

        push DPMI_RM_PIC_MIVEC
        call DPMI_RemapPIC
        add sp, 2

        //clts //clear TS bits. we have no task.
        xor eax, eax
        mov ax, _DATA

        push eax    //GS
        push eax    //FS
        push eax    //DS
        push eax    //ES
        push eax    //SS
        push ebx    //ESP
        pushfd      //EFLAGS, not used, filled by VCPI
        mov ax, _TEXT
        push eax    //CS
        mov ax, offset VCPI_to_real//mov eax, offset back_to_real //link err
        push eax    //EIP

        mov ax, SEL_4G*8
        mov ds, ax
        mov ax, 0xDE0C
        call fword ptr es:DPMI_VCPIInterface
    VCPI_to_real:
        pop ax
        and ax, CPU_IFLAG//IF
        pushf
        or word ptr ss:[bx], ax
        popf
        pop ebx
    }
}

//////////////////////////////////////////////////////////////////////////////
//mode switch
//////////////////////////////////////////////////////////////////////////////
static int16_t DPMI_SwitchProtectedMode(BOOL FullHimemCopy = FALSE)
{ //care on printf msg in mode switch, because printf will switch back and forth and cause recursion
    assert(!DPMI_PM);
    CLIS();

    if(DPMI_V86)
    {
        if(DPMI_VCPIProtectedMode())
            DPMI_PM = PM_VCPI;
    }
    else if(DPMI_DirectProtectedMode())
        DPMI_PM = PM_DIRECT;

    if(DPMI_PM)
    {
        DPMI_Addressing.physical = TRUE;    //linear=physical
        DPMI_Addressing.selector = SEL_4G*8; //4G ds

        #if DPMI_HIMEM
        //copy only stack during interrupt/simple functions (i.e. printf,delay).
        //full copy should be performed on senarios: A.call rm INT/RETF with data pointer; B.custom INT (SW/HW) handlers that modify data; C.printf with dynamic gen/mod %s that not on stack.
        //full copy should not: during driver operations (MMIO data will be overwritten corrupted)
        //senario A,B doesn't involve driver behavior (A.doesn't need to. B.copy happens BEFORE/AFTER INT which may have MMIO op)
        //C, usually for debug purpose, may happen during driver ops. do a pre-translation in PM mode to string (vsprintf) will work. see translation of printf()
        //A: not actuall happens, not handled. B, custom INT handler not supported yet
        if(!FullHimemCopy)
            DPMI_CopyLinear(DPMI_HimemDS + (uint16_t)_SP, DPMI_LinearDS + (uint16_t)_SP, 64L*1024L - (uint16_t)_SP);
        else
            DPMI_CopyLinear(DPMI_HimemDS, DPMI_LinearDS, 64L*1024L); //copy data to himem
        __asm {//switch immediately after copy, or the stack won't match
            mov ax, SEL_XMS_DS*8
            mov ds, ax
            mov es, ax
            mov ss, ax
            mov fs, ax
            mov gs, ax
        }
        //_LOG("FLAGS:%x\n", CPU_FLAGS());
        #endif
    }
    STIL();
    return DPMI_PM;
}

static void DPMI_SwitchRealMode(BOOL FullHimemCopy = FALSE) //back to real/v86 mode
{
    if(!DPMI_PM)
        return;
    CLIS();

    #if DPMI_HIMEM
    if(!FullHimemCopy)
        DPMI_CopyLinear(DPMI_LinearDS + (uint16_t)_SP, DPMI_HimemDS + (uint16_t)_SP, 64L*1024L - (uint16_t)_SP);
    else
        DPMI_CopyLinear(DPMI_LinearDS, DPMI_HimemDS, 64L*1024L); //copy data back
    __asm {
        mov cx, DPMI_INT
        mov ax, SEL_DS*8
        mov ds, ax
        mov es, ax
        mov ss, ax
        mov fs, ax
        mov gs, ax
        mov DPMI_INT, cx //sync DPMI_INT to new ds (if data not fully copied)
    }
    #endif

    if(DPMI_V86)
    {
        assert(DPMI_PM == PM_VCPI);
        DPMI_VCPIRealMode();
    }
    else
        DPMI_DirectRealMode();

    DPMI_PM = PM_NONE;
    DPMI_Addressing.selector = 0;
    STIL();
}

//////////////////////////////////////////////////////////////////////////////
//IDT
//////////////////////////////////////////////////////////////////////////////
static void DPMI_NullINTHandler()
{
    __asm {//TODO: custom handler installation, need full himem copy
        mov sp, bp
        pop bp //dirty hack as naked function
        iret
    }
}

static void DPMI_HWIRQ(int vec)
{
    __asm {pushad; push ds; push es;}
    __asm {mov ax, SEL_CUR_DS*8; mov ds, ax; mov es, ax}
    DPMI_INT = TRUE;    //interrupt should be disabled all the way, not reentrant like DOS
    DPMI_REG r = {0};
    DPMI_CallRealModeINT(vec, &r);
    DPMI_INT = FALSE;
    __asm {pop es; pop ds; popad;}
}
#define DPMI_HW_INT(n, rn) static void DPMI_HwINTHandler##n()\
{\
    __asm {push rn; call DPMI_HWIRQ;}\
    __asm {mov sp, bp; pop bp; iret;}\
}

DPMI_HW_INT(0x28, 0x08)    //IRQ0: PIT
DPMI_HW_INT(0x29, 0x09)    //IRQ1: KBD
DPMI_HW_INT(0x2A, 0x0A)
DPMI_HW_INT(0x2B, 0x0B)
DPMI_HW_INT(0x2C, 0x0C)
DPMI_HW_INT(0x2D, 0x0D)
DPMI_HW_INT(0x2E, 0x0E)
DPMI_HW_INT(0x2F, 0x0F)
DPMI_HW_INT(0x70, 0x70)
DPMI_HW_INT(0x71, 0x71)
DPMI_HW_INT(0x72, 0x72)
DPMI_HW_INT(0x73, 0x73)
DPMI_HW_INT(0x74, 0x74)
DPMI_HW_INT(0x75, 0x75)
DPMI_HW_INT(0x76, 0x76)
DPMI_HW_INT(0x77, 0x77)

#define DPMI_HW_INT_ADDR(n) FP_OFF(&DPMI_HwINTHandler##n)

#define DPMI_EXCEPT(n) static void DPMI_ExpetionHandler##n()\
{\
    __asm {mov ax, SEL_CUR_DS*8; mov ds, ax}\
    DPMI_SwitchRealMode(TRUE);\
    printf("Exception: %02x\n", n);\
    exit(1);\
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

#define DPMI_EXCEPT_ADDR(n) FP_OFF(&DPMI_ExpetionHandler##n)

static void DPMI_SetupIDT()
{
    assert(sizeof(IDT)==8);
    DPMI_IDTBuffer = DPMI_DOSMalloc((sizeof(IDT)*256)>>4);
    DPMI_IDT = (IDT far*)MK_FP(DPMI_IDTBuffer,0);
    _fmemset(DPMI_IDT, 0, sizeof(IDT)*256);
    for(int i = 0; i < 256; ++i)
    {
        DPMI_IDT[i].gate_type = IDT_GT_IG;
        DPMI_IDT[i].selector = SEL_CS*8;
        DPMI_IDT[i].offset_low = FP_OFF(&DPMI_NullINTHandler);
        DPMI_IDT[i].present = 1;
    }

    uint16_t MasterVec = DPMI_RM_PIC_MIVEC;
    uint16_t SlaveVec = DPMI_RM_PIC_SIVEC;
    if(DPMI_V86)
    {
        __asm { //Get 8259A Interrupt Vector Mappings
            push ax
            push bx
            push cx
            mov ax, 0xDE0A
            int 0x67
            mov MasterVec, bx
            mov SlaveVec, cx
            pop cx
            pop bx
            pop ax
        }
        if(MasterVec != DPMI_RM_PIC_MIVEC) //if VCPI already remap it, just use it
        {
            DPMI_PM_PIC_MIVEC = MasterVec;
            DPMI_PIC_NeedRemap = FALSE;
        }
        if(SlaveVec != DPMI_RM_PIC_SIVEC)
            DPMI_PM_PIC_SIVEC = SlaveVec;
    }

    int size = DPMI_HW_INT_ADDR(0x29) - DPMI_HW_INT_ADDR(0x28);
    for(int j = DPMI_PM_PIC_MIVEC; j <= DPMI_PM_PIC_MIVEC+7; ++j)
        DPMI_IDT[j].offset_low = DPMI_HW_INT_ADDR(0x28) + size * (j-DPMI_PM_PIC_MIVEC);
    for(int k = DPMI_PM_PIC_SIVEC; k <= DPMI_PM_PIC_SIVEC+7; ++k)
        DPMI_IDT[k].offset_low = DPMI_HW_INT_ADDR(0x70) + size * (k-DPMI_PM_PIC_SIVEC);
    size = DPMI_EXCEPT_ADDR(0x01) - DPMI_EXCEPT_ADDR(0x00);
    for(int e = 0x00; e < 0x0E; ++e)
        DPMI_IDT[e].offset_low = DPMI_EXCEPT_ADDR(0x00) + size * (e-0x00);

    DPMI_IDTR.size = sizeof(IDT)*256-1;
    DPMI_IDTR.offset = ((uint32_t)DPMI_IDTBuffer)<<4;
}

//////////////////////////////////////////////////////////////////////////////
//common code
//////////////////////////////////////////////////////////////////////////////
//input: BX as DPMI_REG PTR (seems BC doesn't need type override)
//output: full register set loaded from ptr, inclluding EBX itself
//SS:ESP excluded. load ds at last.
#define DPMI_LoadRealModeRegsPtr() \
{\
    __asm {\
        push word ptr ds:[bx.w.ds];\
        \
        push word ptr ds:[bx.w.flags];\
        popf;\
        mov ax, word ptr ds:[bx.w.gs];\
        mov gs, ax;\
        mov ax, word ptr ds:[bx.w.fs];\
        mov fs, ax;\
        mov ax, word ptr ds:[bx.w.es];\
        mov es, ax;\
        mov eax, dword ptr ds:[bx.d.eax];\
        mov ecx, dword ptr ds:[bx.d.ecx];\
        mov edx, dword ptr ds:[bx.d.edx];\
        mov ebp, dword ptr ds:[bx.d.ebp];\
        mov esi, dword ptr ds:[bx.d.esi];\
        mov edi, dword ptr ds:[bx.d.edi];\
        \
        mov ebx, dword ptr ds:[bx.d.ebx];\
        pop ds;\
    }\
}

//input: BX as DPMI_REG ptr
//output: full register set stored to the ptr, except EBX
#define DPMI_StoreRealModeRegsPtr()\
{\
    __asm {\
        push ds;\
        \
        push _DATA;\
        pop ds;\
        \
        push gs;\
        pop word ptr ds:[bx.w.gs];\
        push fs;\
        pop word ptr ds:[bx.w.fs];\
        push es;\
        pop word ptr ds:[bx.w.es];\
        pushf;\
        pop word ptr ds:[bx.w.flags];\
        mov dword ptr ds:[bx.d.eax], eax;\
        mov dword ptr ds:[bx.d.ecx], ecx;\
        mov dword ptr ds:[bx.d.edx], edx;\
        mov dword ptr ds:[bx.d.ebp], ebp;\
        mov dword ptr ds:[bx.d.esi], esi;\
        mov dword ptr ds:[bx.d.edi], edi;\
        \
        pop ax;\
        mov word ptr ds:[bx.w.ds], ax;\
    }\
}

static void DPMI_CallRealMode(DPMI_REG* reg, unsigned INTn) //INTn < 256 (hi byte 0): interrupt(cs:ip) with iret. push flags.
{
    int16_t pm = DPMI_PM;
    reg->w.flags &= 0x3ED7;
    reg->w.flags |= 0x3002;
    {//keep original IF
        uint16_t flags = CPU_FLAGS();
        reg->w.flags &= ~((~flags)&CPU_IFLAG);
    }
    if(pm)
        DPMI_SwitchRealMode();
    assert(!DPMI_PM);
    //_LOG("CS:IP: %04x:%04x\n", reg->w.cs, reg->w.ip);

    __asm {pushf}
    #if DPMI_DEBUG && 0
    if(INTn < 256 && pm)
    {
        if(!(CPU_FLAGS()&CPU_IFLAG))
        {
            fprintf(stdout, "before %x, FLAGS :%x, reg.flags:%x\n", INTn, CPU_FLAGS(), reg->w.flags);fflush(stdout);
            CLI();
            INTn |= 0xDB00;
        }
    }
    #endif
    __asm {
        pushad
        push ds
        push es
        //push fs //Turbo Debugger bug
        mov ax, fs; push ax
        mov ax, gs; push ax

        mov bx, reg //bp[xxx]
        push ebx    //save DPMI_REG ptr
        mov ax, INTn
        test ah, ah
        jnz skip_flags
        push word ptr [bx.w.flags]
        and word ptr [bx.w.flags], 0xFCFF //clear IF TF (by Intel INTn instruction reference).(AC in hiword)
    }
    skip_flags: __asm {
        push cs
        push offset callreturn
        push word ptr [bx.w.cs]
        push word ptr [bx.w.ip]
    }
    DPMI_LoadRealModeRegsPtr();//BX
    __asm {
        retf
        callreturn:
        //load DPMI_REG ptr(EBX) at stack top and store new ebx by xchg
        push bp
        mov bp, sp
        xchg dword ptr ss:[bp+2], ebx
        pop bp
    }
    DPMI_StoreRealModeRegsPtr();//BX
    __asm {
        pop eax //new ebx
        mov dword ptr ds:[bx.d.ebx], eax
    }
    __asm {
        pop ax; mov gs, ax;
        pop ax; mov fs, ax;
        pop es
        pop ds
        popad
        popf
    }
    #if DPMI_DEBUG && 0
    if((INTn&0xFF00) == 0xDB00)
    {
        fprintf(stdout, "after %x, FLAGS :%x, reg.flags:%x\n", (INTn&0xFF), CPU_FLAGS(), reg->w.flags);fflush(stdout);
        CLI();
    }
    #endif

    if(pm)
        DPMI_SwitchProtectedMode();
    assert(DPMI_PM == pm);
}

static void DPMI_Cleanup()
{ //TODO: dont't release those data when interrupt handler installed, need get back to PM. (interrupt handler not supported yet)
    for(int i = 0; i < DPMI_MappedPages; ++i)
        XMS_Free(DPMI_XMSPageHandle[i]);

    DPMI_DOSFree(DPMI_PageBuffer);
    DPMI_PageBuffer = 0;
    DPMI_PageDir = NULL;
    DPMI_PageTable4M = NULL;
    DPMI_PageTableXMS = NULL;
    DPMI_XMSPageHandle = NULL;
    DPMI_MappedPages = 0;

    DPMI_DOSFree(DPMI_IDTBuffer);
    DPMI_IDTBuffer = 0;
    DPMI_IDT = NULL;
}

static void DPMI_Shutdown(void);

uint32_t DPMI_PTR2L(void* ptr)
{
    assert(ptr != NULL);
    if(DPMI_PM)
        return DPMI_HIMEM ? (DPMI_HimemDS + (uint32_t)FP_OFF(ptr)) : (DPMI_LinearDS + (uint32_t)FP_OFF(ptr));
    else
    {
        _LOG("real mode ptr mapping called.\n");
        return DPMI_Ptr16ToLinear(ptr);
    }
}

void* DPMI_L2PTR(uint32_t addr)
{
    if(DPMI_PM)
    {
        assert((DPMI_HIMEM && addr >= DPMI_HimemDS && addr <= DPMI_HimemDS+0xFFFFL)
         || (!DPMI_HIMEM && addr >= DPMI_LinearDS && addr <= DPMI_LinearDS+0xFFFFL));
        return DPMI_HIMEM ? ((void near*)MK_FP(0, addr - DPMI_HimemDS)) : ((void near*)MK_FP(0, addr - DPMI_LinearDS));
    }
    else
    {
        _LOG("real mode ptr mapping called.\n");
        return (void near*)DPMI_LinearToPtr16(addr);
    }
}

void DPMI_Init(void)
{
    //_LOG("sbrk: %x, SP: %x, stack: %u\n", FP_OFF(sbrk(0)), _SP, stackavail());//small model: static data : heap : stack
    atexit(&DPMI_Shutdown);
    DPMI_V86 = DPMI_IsV86();

    //setup gdt
    DPMI_GDTR.offset = DPMI_Ptr16ToLinear(DPMI_GDT);
    DPMI_LinearCS = ((uint32_t)_CS)<<4L;
    DPMI_LinearDS = ((uint32_t)_DS)<<4L;
    //_LOG("cs:%08lx, ss:%08lx, ds:%08lx\n", DPMI_LinearCS, (uint32_t)_SS<<4L, DPMI_LinearDS);
    ((GDT*)DPMI_GDT)[SEL_CS].base_low = DPMI_LinearCS;
    ((GDT*)DPMI_GDT)[SEL_CS].base_middle = DPMI_LinearCS>>16L;
    ((GDT*)DPMI_GDT)[SEL_DS].base_low = DPMI_LinearDS;
    ((GDT*)DPMI_GDT)[SEL_DS].base_middle = DPMI_LinearDS>>16L;
    //_LOG("GDT: %08lx %08lx %08lx %08lx\n", DPMI_GDT[4], DPMI_GDT[5], DPMI_GDT[6], DPMI_GDT[7]);
    #if DPMI_HIMEM //pre allocate XMS memory. CS not used for now, use it on TSR
    DPMI_XMSHimemHandle = XMS_Alloc(DPMI_XMS_Size/1024L, &DPMI_HimemCS);
    /*if(DPMI_HimemCS + 128L*1024L < 0x400000L) //align to 4M (1th page table)
    {
        if( XMS_Realloc(DPMI_XMSHimemHandle, (0x400000L-DPMI_HimemCS)/1024, &DPMI_HimemCS))
        {
            uint16_t handle = XMS_Alloc(DPMI_XMS_Size/1024L, &DPMI_HimemCS);
            XMS_Free(DPMI_XMSHimemHandle);
            DPMI_XMSHimemHandle = handle;
        }
    }*/
    if(DPMI_XMSHimemHandle == 0)
    {
        printf("Error: unable to allocate XMS memory.\n");
        exit(1);
    }
    DPMI_HimemDS = DPMI_HimemCS + 64L*1024L;
    //_LOG("HimemCS: %08lx, HimemDS: %08lx\n", DPMI_HimemCS, DPMI_HimemDS);
    ((GDT*)DPMI_GDT)[SEL_XMS_CS].base_low = DPMI_HimemCS;
    ((GDT*)DPMI_GDT)[SEL_XMS_CS].base_middle = DPMI_HimemCS>>16L;
    ((GDT*)DPMI_GDT)[SEL_XMS_CS].base_high = DPMI_HimemCS>>24L;
    ((GDT*)DPMI_GDT)[SEL_XMS_DS].base_low = DPMI_HimemDS;
    ((GDT*)DPMI_GDT)[SEL_XMS_DS].base_middle = DPMI_HimemDS>>16L;
    ((GDT*)DPMI_GDT)[SEL_XMS_DS].base_high =DPMI_HimemDS>>24L;
    #endif

    DPMI_SetupIDT();

    if(DPMI_V86)
        DPMI_InitVCPI();

    if(!DPMI_SwitchProtectedMode(TRUE))
    {
        printf("Error: unable to enter protected mode.\n");
        exit(1);
    }
    _LOG("Entered protected mode from %s.\n", DPMI_PM == PM_VCPI ? "virtual 8086 mode"  : "real mode");
}

static void DPMI_Shutdown(void)
{
    DPMI_SwitchRealMode(TRUE);
    DPMI_Cleanup();
    if(DPMI_XMSHimemHandle)
    {
        XMS_Free(DPMI_XMSHimemHandle);
        DPMI_XMSHimemHandle = 0;
    }
    //_LOG("(pseudo)DPMI terminating...\n");
}

uint32_t DPMI_MapMemory(uint32_t physicaladdr, uint32_t size)
{
    if(!DPMI_V86)
        return physicaladdr;

    assert(DPMI_XMSPageHandle != NULL);
    assert(physicaladdr >= 640L*1024L);
    uint32_t pdi = physicaladdr >> 22L;
    uint32_t pdi2 = (physicaladdr+size) >> 22L;

    uint32_t pagestart = physicaladdr >> 12L;
    uint32_t pageend = align(physicaladdr + size, 4096) >> 12L;
    uint32_t pagecount = pageend - pagestart;
    pagestart -= pdi << 10L;
    for(uint32_t i = pdi; i <= pdi2; ++i)
    {
        uint32_t addr = (i << 22L);
        uint32_t start = max(pagestart, 0);
        uint32_t count = min(pagecount, 1024);
        uint32_t end = start+count;
        pagecount -= 1024;
        pagestart = 0;
        assert(i < 1024);
        assert(i != 0 || start >= DPMI_PageTable4MOffset); //make sure not overwriting VCPI entry
        //_LOG("mmap pt:%04ld, entry: %04ld-%04ld, addr: %08lx-%08lx\n", i, start, end-1, addr+(start<<12), addr+((end-1)<<12)+0xFFF);

        PDE pde = DPMI_PM ? DPMI_LoadPDE(DPMI_PageDir, i) : DPMI_PageDir[i];
        if(!pde.present)
        {
            assert(DPMI_PM == PM_VCPI);//need in pm mode to access cr3
            uint32_t tbaddr = 0;
            uint16_t handle = XMS_Alloc(4, &tbaddr);
            assert(handle);
            if((tbaddr&0xFFF))//not 4k aligned
            {
                BOOL ret = XMS_Realloc(handle, 8, &tbaddr);//waste.TODO: use VCPI to alloc 4K page
                assert(ret);
                tbaddr = align(tbaddr, 4096);
                assert((tbaddr&0xFFF) == 0);
            }
            DPMI_StoreW(DPMI_Ptr16ToLinear(DPMI_XMSPageHandle)+DPMI_MappedPages*sizeof(uint16_t), handle);
            ++DPMI_MappedPages;

            PDE_INIT(pde, tbaddr);
            DPMI_StorePDE(DPMI_PageDir, i, &pde);

            PTE ptetmp;//temporarily map it to end of 4M
            PTE_INIT(ptetmp, tbaddr);
            CLI();
            PTE pteold = DPMI_LoadPTE(DPMI_PageTable4M, 1023);
            DPMI_StorePTE(DPMI_PageTable4M, 1023, &ptetmp);
            __asm{ push eax; mov eax, cr3; mov cr3, eax; pop eax; } //flush TLB. TODO: invlpg(486+)
            DPMI_SetLinear(4L*1024L*1023L, 0, 4096); //clear mapped tb
            DPMI_StorePTE(DPMI_PageTable4M, 1023, &pteold);
            __asm{ push eax; mov eax, cr3; mov cr3, eax; pop eax; } //flush TLB. TODO: invlpg(486+)
            STI();
        }

        if(DPMI_PM)
        {
            PTE ptetmp;//temporarily map it to end of 4M
            PTE_INIT(ptetmp, PDE_ADDR(pde));
            CLI();
            PTE pteold = DPMI_LoadPTE(DPMI_PageTable4M, 1023);
            DPMI_StorePTE(DPMI_PageTable4M, 1023, &ptetmp);
            __asm{ push eax; mov eax, cr3; mov cr3, eax; pop eax; } //flush TLB. TODO: invlpg(486+)
            for(uint32_t j = start; j < end; ++j)
            {
                PTE pte;
                PTE_INIT(pte, (addr + (j<<12L)) ); //1:1 map
                pte.disable_cache = 1; //for device memroy map
                //_LOG("PTE: %08lx => %08lx\n", DPMI_LoadD(4L*1024L*1023L+j*4L), addr + (j<<12L));
                DPMI_StorePTE(4L*1024L*1023L+j*sizeof(PTE), &pte);
            }
            DPMI_StorePTE(DPMI_PageTable4M, 1023, &pteold);
            __asm{ push eax; mov eax, cr3; mov cr3, eax; pop eax; } //flush TLB. TODO: invlpg(486+)
            STI();
        }
        else
        { //real mode, taking effect after next switch to PM.
            uint32_t linear = PDE_ADDR(pde);
            assert(linear <= 640L*1024L-4096L);
            PTE far* pt = (PTE far*)DPMI_LinearToPtr16(linear);
            //_LOG("%Fp, %Fp\n", pt, DPMI_PageTable4M);
            for(uint32_t j = start; j < end; ++j)
            {
                PTE pte;
                PTE_INIT(pte, (addr + (j<<12L)) ); //1:1 map
                //_LOG("PTE: %ld,  %08lx => %08lx\n", j, *(uint32_t far*)(pt+j), *(uint32_t*)&pte);
                pt[j] = pte;
            }
        }
    }
    return physicaladdr;
}

void* DPMI_DMAMalloc(unsigned int size, unsigned int alignment)
{
    int8_t* ptr = (int8_t*)malloc(size + alignment + 2) + 2;
    uint16_t addr = DPMI_PTR2L(ptr);
    uint16_t offset = align(addr, alignment) - addr;
    void* aligned = ptr + offset;
    ((uint16_t*)aligned)[-1] = offset + 2;

    #if DPMI_DEBUG && 0
    uint32_t linear = DPMI_PTR2L(aligned);
    uint32_t pdi = linear>>22;
    uint32_t pti = (linear>>12)&0x3FF;
    PDE pde = DPMI_PM ? DPMI_LoadPDE(DPMI_PageDir, pdi) : DPMI_PageDir[pdi];
    uint32_t ptaddr = PDE_ADDR(pde);
    uint32_t lptaddr = (TRUE) ? ptaddr : 0; //pt should be in dos memory (pre mapped).
    //_LOG("pde %08lx, pt addr: %08lx\n", *(uint32_t*)&pde, DPMI_Ptr16ToLinear(DPMI_PageTable4M));
    PTE pte = DPMI_PM ? DPMI_LoadPTE(lptaddr+pti*sizeof(PTE)) : ((PTE far*)DPMI_LinearToPtr16(ptaddr))[pti];
    PTE p = DPMI_PM ? DPMI_LoadPTE(DPMI_PageTable4M, pti) : DPMI_PageTable4M[pti];
    //_LOG("4M pt: %ld, %lx\n", pti, *(uint32_t*)&p);

    _LOG("mapped malloc: %04x %08lx, pdi:%ld, pti:%ld, pte:%lx\n",
        aligned, linear, pdi, pti, *(uint32_t*)&pte);
    #endif
    return aligned;
}

void DPMI_DMAFree(void* ptr)
{
    uint16_t offset = ((uint16_t*)ptr)[-1];
    free((int8_t*)ptr - offset);
}

uint32_t DPMI_DOSMalloc(uint16_t size)
{
    //BC doc: malloc cannot coeistwith _dos_allocmem / allocmem (? probably for model small+)
    //_dos_allocmem may not call int 21h 48h, the memory allocated will be free even in TSR (repeated TSR use the same addr).
    //so need implement another translation
    uint16_t seg = 0;
    DPMI_REG r = {0};
    r.h.ah = 0x48;
    r.w.bx = size;
    DPMI_CallRealModeINT(0x21,&r);
    seg = (r.w.flags&CPU_CFLAG) ? 0 : r.w.ax;
    return seg;
}

void DPMI_DOSFree(uint32_t segment)
{
    if(segment&0xFFFF)
    {
        DPMI_REG r = {0};
        r.h.ah = 0x49;
        r.w.es = segment;
        DPMI_CallRealModeINT(0x21,&r);
    }
}

uint16_t DPMI_CallRealModeRETF(DPMI_REG* reg)
{   //TODO: to support call to predefined USE16 segment (C or asm, not dynamic allocation), need enable FullimemCopy on mode switch to enable data share
    //even dynamlic allocated USE16 seg could be called with shared datas
    DPMI_CallRealMode(reg, 0xFF00);
    return 0;
}

uint16_t DPMI_CallRealModeINT(uint8_t i, DPMI_REG* reg)
{
    uint16_t off = (DPMI_PM) ? DPMI_LoadW(i*4) : *(uint16_t far*)MK_FP(0,i*4);
    uint16_t seg = (DPMI_PM) ? DPMI_LoadW(i*4+2) : *(uint16_t far*)MK_FP(0,i*4+2);
    reg->w.cs = seg;
    reg->w.ip = off;
    DPMI_CallRealMode(reg, i);
    return 0;
}

uint16_t DPMI_CallRealModeIRET(DPMI_REG* reg)
{
    DPMI_CallRealMode(reg, 0);
    return 0;
}

uint16_t DPMI_InstallISR(int i, void(*ISR)(void), DPMI_ISR_HANDLE* outputp handle)
{
    if(!DPMI_PM || i < 0 || i > 255 || handle == NULL)
        return -1;

    uint16_t off = (DPMI_PM) ? DPMI_LoadW(i*4) : *(uint16_t far*)MK_FP(0,i*4);
    uint16_t seg = (DPMI_PM) ? DPMI_LoadW(i*4+2) : *(uint16_t far*)MK_FP(0,i*4+2);
    handle->rm_cs = seg;
    handle->rm_offset = off;
    handle->cs = handle->offset = 0;
    //TODO: preserve GDT & IDT & page table into XMS, and preserve switching code (RMCB) in DOS to switch betwen rm/pm outside
    return -1;
}

uint16_t DPMI_UninstallISR(DPMI_ISR_HANDLE* inputp handle)
{
    return -1;
}

void DPMI_GetPhysicalSpace(DPMI_SPACE* outputp spc)
{
    spc->baseds = DPMI_HIMEM ? DPMI_HimemDS : DPMI_LinearDS;
    spc->limitds = 64L*1024L - 1;
    spc->basecs = DPMI_HIMEM ? DPMI_HimemCS : DPMI_LinearCS;
    spc->limitcs = 64L*1024L - 1;
    spc->stackpointer = 0xFFF8;
    return;
}

BOOL DPMI_TSR(void)
{
    if(DPMI_HIMEM && !DPMI_PM)
        return FALSE;

    if(DPMI_PM)
    {
        #if DPMI_HIMEM
        DPMI_CopyLinear(DPMI_HimemCS, DPMI_LinearCS, 64L*1024L);
        #endif
        DPMI_SwitchRealMode(TRUE);
    }
    DPMI_Cleanup();
    //_LOG("FLAGS: %04x\n", CPU_FLAGS());

    int residence = DPMI_HIMEM ? 0 : (128L*1024L)>>4L; //resident 64K+64K if !Himem
    __asm {
        mov ax, 0x3100
        mov dx, residence
        int 0x21
    }
    return FALSE;
}
/*
//simple PM translation handling basic io in protected mode
unsigned _dos_open(const char _FAR* filename, unsigned flags, int* handle)
{
    DPMI_REG r = {0};
    r.h.al = flags>>1; //Borland O_*** to dos value
    r.h.ah = 0x3D;
    r.w.ds = FP_SEG(filename);
    r.w.dx = FP_OFF(filename);
    r.h.cl = 0;
    DPMI_CallRealModeINT(0x21,&r);
    BOOL error = (r.w.flags&CPU_CFLAG);
    if(!error)
        *handle = r.w.ax;
    return error  ? r.w.ax : 0;
}

unsigned _dos_write(int fd, const void far* buf, unsigned len, unsigned _FAR* nwritten)
{
    DPMI_REG r = {0};
    r.h.ah = 0x40;
    r.w.bx = fd;
    r.w.cx = len;
    r.w.ds = FP_SEG(buf);
    r.w.dx = FP_OFF(buf);
    DPMI_CallRealModeINT(0x21, &r);
    BOOL error = r.w.flags&CPU_CFLAG;
    if(!error)
        *nwritten = r.w.ax;
    return error ? r.w.ax : 0;
}

unsigned _dos_close(int handle)
{
    DPMI_REG r = {0};
    r.w.bx = handle;
    r.h.ah = 0x3E;
    DPMI_CallRealModeINT(0x21, &r);
    BOOL error = r.w.flags&CPU_CFLAG;
    return error ? r.w.ax : 0;
}

int ioctl(int handle, int func, void _FAR* argdx, int argcx)
{
    DPMI_REG r = {0};
    r.w.bx = handle;
    r.h.ah = 0x44;
    r.h.al = func;
    r.w.ds = FP_SEG(argdx);
    r.w.dx = FP_OFF(argdx);
    r.w.cx = argcx;
    DPMI_CallRealModeINT(0x21, &r);
    BOOL error = r.w.flags&CPU_CFLAG;
    if(!error)
        return r.w.ax;
    else
        return -1;
}
//*/
//PM to RM translation for debug output
int _Cdecl _FARFUNC printf(const char* fmt, ...)
{
    static int recursion_gard = 0;
    if(!recursion_gard) //avoid reentrance in mode switching
    {
        recursion_gard = 1;
        //himem mode: data not copied during translation, so string buffers (%s) need to de-referenced before mode switch (otherwise ds/himemds data don't match)
        //use vsprintf to do it. BC doesn't have vsnprintf, potential buffer overflow exists.
        char* buff = (char*)alloca(min(max(512, stackavail()-1024), stackavail()-64));
        va_list aptr;
        va_start(aptr, fmt);
        int len = vsprintf(buff, fmt, aptr);
        va_end(aptr);

        int16_t pm = DPMI_PM;
        if(pm) DPMI_SwitchRealMode();

        char c;
        while( (c=*(buff++)) != '\0') putchar(c);
        fflush(stdout); //extra

        if(pm) DPMI_SwitchProtectedMode();
        recursion_gard = 0;
        return len;
    }
    return 0;
}

void _Cdecl _FARFUNC delay(unsigned millisec)
{
    uint32_t usec = (uint32_t)millisec*1000L;
    DPMI_REG r = {0};
    r.w.ax = 0x8600; //bios delay function, 976 usec resolution
    r.w.dx = usec&0xFFFF;
    r.w.cx = usec>>16;
    DPMI_CallRealModeINT(0x15,&r);
}

#if !defined(NDEBUG)
void _Cdecl _FARFUNC __assertfail(char _FAR* __msg,
                                  char _FAR* __cond,
                                  char _FAR* __file,
                                  int __line)
{//BC's __assertfail will cause #GP, probably caused by INTn/changing segment
    printf(__msg, __cond, __file, __line);
    exit(1);
}
#endif

#endif
