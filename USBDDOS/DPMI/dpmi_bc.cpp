#include "USBDDOS/DPMI/dpmi.h"
#if 1
//don't use DPMI on Boarland C, instead use 16bit protected mode and keep the code and data near (no far calls/far data).
//note: the inline assember won't reconginze 386 instructions and registers, code need compiled with -B flag (compile via assembly) in BC31
//
//if in real mode, direct switch to protected mode, without paging, otherwise use VCPI to switch to 16 bit protected mode with 1:1 paging.
//small model, 64K+64K

//the _ASM* macros are not needed since this file is intended for BC only, but stil add them to workaround the syntax inteligence problem of vscode
#include <conio.h>
#include <stdlib.h>
#include <dos.h>
#if !defined(_WIN32) && !defined(__linux__) && !defined(__WC__)//make editor happy
#include <alloc.h>
#endif
#include <malloc.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include "USBDDOS/DPMI/xms.h"
#include "USBDDOS/DPMI/dpmi_bc.h"

static BOOL DPMI_InitProtectedMode()
{
    //phase 1: temporary protected mode & copy GDT/IDT/Paging to Himem
    //no PM seg for DPMI_Temp, once in PM, DPMI_Temp is inaccessible. save on stack
    uint32_t GDTLAddr = DPMI_Temp->gdtr.offset;
    uint32_t IDTLAddr = DPMI_Temp->idtr.offset;
    uint16_t GDTSize = DPMI_Temp->gdtr.size + 1;
    uint16_t IDTSize = DPMI_Temp->idtr.size + 1;
    uint32_t PageDirLAddr = DPMI_Ptr16ToLinear(DPMI_Temp->PageDir);
    //uint16_t Page4MOffset = DPMI_Ptr16ToLinear(DPMI_Temp->PageTable0) - PageDirLAddr; //VCPI spec don't allow copy 1st 4M page
    uint32_t Page4MLAddr = DPMI_Ptr16ToLinear(DPMI_Temp->PageTable0);
    uint16_t PageXMSOffset = (uint16_t)(DPMI_Ptr16ToLinear(DPMI_Temp->PageTalbeHimem) - PageDirLAddr);
    uint16_t PageXMS2Offset = (uint16_t)(DPMI_Ptr16ToLinear(DPMI_Temp->PageTalbeHimem2) - PageDirLAddr);
    uint16_t PageHanldeOffset = (uint16_t)(DPMI_Ptr16ToLinear(DPMI_Temp->XMSPageHandle) - PageDirLAddr);
    uint32_t LinearCS = DPMI_Temp->LinearCS;
    uint32_t LinearDS = DPMI_Temp->LinearDS;

    CLIS();
    for(int i = 0; i < 256; ++i)
        DPMI_Temp->idt[i].selector = SEL_HIMEM_CS*8; //change IDT selector, codes are the same after change
    DPMI_SwitchProtectedMode(LinearDS);
    DPMI_CopyLinear(DPMI_HimemCS, LinearCS, 64L*1024L); //copy code
    STIL();

    uint32_t offset = 0;

    //_LOG("Copy IDT.\n");
    DPMI_CopyLinear(DPMI_SystemDS + offset, IDTLAddr, IDTSize);
    DPMI_Rmcb->RMCB_Idtr.offset = DPMI_SystemDS + offset;
    offset += IDTSize;

    //we're 1:1 mapped, linear address of gdt/idt is the same physical address
    //_LOG("Copy GDT.\n");
    DPMI_CopyLinear(DPMI_SystemDS + offset, GDTLAddr, GDTSize);
    DPMI_Rmcb->RMCB_Gdtr.offset = DPMI_SystemDS + offset;
    offset += GDTSize;

    if(DPMI_V86)
    {
        //_LOG("Copy page tables.\n");
        offset = align(DPMI_SystemDS + offset, 4096) - DPMI_SystemDS; //BUGFIX DPMI_Rmcb->CR3 need align to 4K not offset or initial memory
        DPMI_CopyLinear(DPMI_SystemDS + offset, PageDirLAddr, VCPI_PAGING_MEM_SIZE);
        DPMI_Rmcb->CR3 = DPMI_SystemDS + offset; //physical addr
        DPMI_4MPageTableLAddr = Page4MLAddr;
        uint32_t XMSLAddr = DPMI_Rmcb->CR3 + PageXMSOffset;
        uint32_t XMS2LAddr = DPMI_Rmcb->CR3 + PageXMS2Offset;
        DPMI_XMSPageHandle = (uint16_t far*)MK_FP(SEL_SYS_DS*8,  PageDirLAddr + PageHanldeOffset);
        //adjust paging
        PDE far* pagedir = (PDE far*)MK_FP(SEL_SYS_DS*8, offset);
        //PDE_INITA(pagedir[0], DPMI_4MPageTableLAddr);
        long pdi = (DPMI_SystemDS>>22) ? (DPMI_SystemDS>>22) : 1;
        PDE_INITA(pagedir[pdi], XMSLAddr);
        PDE_INITA(pagedir[pdi+1], XMS2LAddr);
        offset += VCPI_PAGING_MEM_SIZE;
    }

    DPMI_SwitchRealMode(LinearDS);
    _LOG("Himem data setup ready.\n");

    //phase 2: return to real mode, and perform the switch again using new himem data
    DPMI_DOSFree(DPMI_Temp->PageMemory);
    DPMI_DOSFree(DPMI_Temp->TempMemoryHandle);
    DPMI_Temp = NULL;

    DPMI_SwitchProtectedMode(LinearDS); //final PM and also pre-test the adjustment above.
    return TRUE;
}

static void DPMI_SetupPaging()
{
    if(DPMI_V86) //no paging in raw protected mode
        DPMI_InitVCPI(); //VCPI will init first 4M page.
}

static void DPMI_SetupIDT()
{
    assert(sizeof(IDT)==8);
    assert(DPMI_Temp);
    _fmemset(DPMI_Temp->idt, 0, sizeof(IDT)*256);
    for(int i = 0; i < 256; ++i)
    {
        DPMI_Temp->idt[i].gate_type = IDT_GT_IG;
        DPMI_Temp->idt[i].selector = SEL_CS*8;
        DPMI_Temp->idt[i].offset_low = FP_OFF(&DPMI_NullINTHandler);
        DPMI_Temp->idt[i].present = 1;
    }

    DPMI_Temp->NeedRemapPIC = FALSE;    //don't do remap, incompatible with TSR's interupt handling: pm interrupt handler need enter pm, if
                                        //do remapping, the PIC are re-inited and status (IMR/ISR) are cleared. but we also need handling interrupt in pm after in pm, so the best
                                        //way is to leave the interrupt and detect IRQ/EXC in the handler.
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
        {
            DPMI_Temp->RealModeIRQ0Vec = (uint8_t)MasterVec;
            DPMI_Temp->NeedRemapPIC = FALSE;
        }
        if(SlaveVec != DPMI_Temp->RealModeIRQ8Vec)
            DPMI_Temp->RealModeIRQ8Vec = (uint8_t)SlaveVec;
    }

    uint16_t size = DPMI_EXCEPT_ADDR(0x01) - DPMI_EXCEPT_ADDR(0x00);
    for(int e = 0x00; e < 0x0E; ++e)
        DPMI_Temp->idt[e].offset_low = DPMI_EXCEPT_ADDR(0x00) + size * (e-0x00);

    //now irq 0~7 & exceptions 0x8~0xF share the same handler. set HWIRQHandler setup will overwrite part of the exception.
    for(int j = DPMI_Temp->ProtectedModeIRQ0Vec; j <= DPMI_Temp->ProtectedModeIRQ0Vec+7; ++j)
        DPMI_Temp->idt[j].offset_low = (uintptr_t)(void near*)&DPMI_HWIRQHandler;
    for(int k = DPMI_Temp->ProtectedModeIRQ8Vec; k <= DPMI_Temp->ProtectedModeIRQ8Vec+7; ++k)
        DPMI_Temp->idt[k].offset_low = (uintptr_t)(void near*)&DPMI_HWIRQHandler;

    DPMI_Temp->idtr.size = sizeof(IDT)*256 - 1;
    DPMI_Temp->idtr.offset = DPMI_Ptr16ToLinear(DPMI_Temp->idt);
}

static void DPMI_SetupGDT()
{
    assert(DPMI_Temp);
    assert(sizeof(DPMI_Data4GDesc) == sizeof(GDT));
    assert(sizeof(DPMI_DataDesc) == sizeof(GDT));
    assert(sizeof(DPMI_CodeDesc) == sizeof(GDT));

    DPMI_Temp->gdtr.offset = DPMI_Ptr16ToLinear(DPMI_Temp->gdt);
    DPMI_Temp->gdtr.size = sizeof(GDT)*SEL_TOTAL - 1;

    GDT far* gdt = DPMI_Temp->gdt;
    _fmemset(gdt, 0, sizeof(GDT)*SEL_TOTAL);

    _fmemcpy(&gdt[SEL_4G], DPMI_Data4GDesc, sizeof(GDT));
    _fmemcpy(&gdt[SEL_CS], DPMI_CodeDesc, sizeof(GDT));
    gdt[SEL_CS].base_low = (uint16_t)DPMI_Temp->LinearCS;
    gdt[SEL_CS].base_middle = (uint8_t)(DPMI_Temp->LinearCS>>16L);
    _fmemcpy(&gdt[SEL_DS], DPMI_DataDesc, sizeof(GDT));
    gdt[SEL_DS].base_low = (uint16_t)DPMI_Temp->LinearDS;
    gdt[SEL_DS].base_middle = (uint8_t)(DPMI_Temp->LinearDS>>16L);
    //_LOG("GDT: %08lx %08lx %08lx %08lx\n", DPMI_GDT[4], DPMI_GDT[5], DPMI_GDT[6], DPMI_GDT[7]);

    _fmemcpy(&gdt[SEL_SYS_DS], DPMI_DataDesc, sizeof(GDT));
    gdt[SEL_SYS_DS].base_low = (uint16_t)DPMI_SystemDS;
    gdt[SEL_SYS_DS].base_middle = (uint8_t)(DPMI_SystemDS>>16L);
    gdt[SEL_SYS_DS].base_high = (uint8_t)(DPMI_SystemDS>>24L);
    _fmemcpy(&gdt[SEL_HIMEM_CS], DPMI_CodeDesc, sizeof(GDT));
    gdt[SEL_HIMEM_CS].base_low = (uint16_t)DPMI_HimemCS;
    gdt[SEL_HIMEM_CS].base_middle = (uint8_t)(DPMI_HimemCS>>16L);
    gdt[SEL_HIMEM_CS].base_high = (uint8_t)(DPMI_HimemCS>>24L);
    _fmemcpy(&gdt[SEL_HIMEM_DS], DPMI_DataDesc, sizeof(GDT));
    gdt[SEL_HIMEM_DS].base_low = (uint16_t)DPMI_HimemDS;
    gdt[SEL_HIMEM_DS].base_middle = (uint8_t)(DPMI_HimemDS>>16L);
    gdt[SEL_HIMEM_DS].base_high = (uint8_t)(DPMI_HimemDS>>24L);
    _fmemcpy(&gdt[SEL_RMCB_CS], DPMI_CodeDesc, sizeof(GDT));
    gdt[SEL_RMCB_CS].base_low = (uint16_t)DPMI_Temp->RMCBLAddr;
    gdt[SEL_RMCB_CS].base_middle = (uint8_t)(DPMI_Temp->RMCBLAddr>>16L);
    gdt[SEL_RMCB_CS].limit_low = (uint16_t)(DPMI_RMCB_SIZE - 1);
    _fmemcpy(&gdt[SEL_RMCB_DS], DPMI_DataDesc, sizeof(GDT));
    gdt[SEL_RMCB_DS].base_low = (uint16_t)DPMI_Temp->RMCBLAddr;
    gdt[SEL_RMCB_DS].base_middle = (uint8_t)(DPMI_Temp->RMCBLAddr>>16L);
    gdt[SEL_RMCB_DS].limit_low = DPMI_RMCB_SIZE - 1;

    #if DEBUG && 0
    for(int i = 0; i < SEL_TOTAL; ++i)
    {
        uint32_t far* entry = (uint32_t far*)&gdt[i];
        _LOG("GDT: %d: %08lx %08lx\n", i, entry[0], entry[1]);
    }
    #endif
}

//////////////////////////////////////////////////////////////////////////////
//common code
//////////////////////////////////////////////////////////////////////////////
static void DPMI_CallRealMode(DPMI_REG* reg, unsigned INTn) //INTn < 256: interrupt call
{
    reg->w.flags &= 0x3ED7;
    reg->w.flags |= 0x3002;
    {//keep original IF
        reg->w.flags &= CPU_FLAGS()&(~CPU_IFLAG);
    }
    if(!DPMI_PM)
    {
        DPMI_RMCBTranslation(reg, INTn, TRUE);
        return;
    }

    //make it re-entrant. reentrance hapeens on interrupt with 2 scenarios:
    //A, proteced mode IRQ handling will call realmode handler through here
    //B, real mode IRQ handling, if hooked, will call to PM handler through RMCB, then the PM handler might call realmode functions (i.e.debug print)
    //senario B might not work since BIOS/DOS interrupt are not reentrant

    //copy struct to dest stack
    DPMI_Rmcb->TranslationSP -= sizeof(DPMI_REG);
    uint8_t far* stackREG = (uint8_t far*)MK_FP(SEL_RMCB_DS*8, DPMI_Rmcb->TranslationSP);
    _fmemcpy(stackREG, reg, sizeof(DPMI_REG));
    DPMI_Rmcb->TranslationSP -= DPMI_RMCB_TRASNLATION_STACK_SIZE; //reserve spece for re-entrant

    _ASM_BEGIN
        _ASM(push es) _ASM(push fs) _ASM(push gs)
        _ASM(push bx)
        _ASM(push bp)
        _ASM2(mov dx, ss)

        _ASM2(mov cx, INTn)    //INTn stored at ss:bp[xx]. save parameters to register before switching stack
        _ASM2(shl ecx, 16)
        _ASM2(mov ax, sp) //save SP before switching stack
        _ASM2(lss sp, stackREG) //stackREG stored at ss:bp[xx]. load ss:sp in one shot, or interrutp will be on wrong stack
        _ASM2(mov cx, sp) //DPMI_REG near* for translation call
        _ASM2(les bx, DPMI_Rmcb)
        _ASM2(cmp dx, SEL_HIMEM_DS*8) //if not on the himem stack, skip set sp. coulde be on translation stack. (rmcb stack is temporary and interrupt should not be enabled on it)
        _ASM(jne TranslationSkipSaveSP)
        _ASM2(mov bp, es:[bx + RMCB_OFF_PMSP]);
        _ASM2(mov es:[bx + RMCB_OFF_PMSP], ax); //if real mode interrupt reflect to pm, it will use this sp pointer
    _ASMLBL(TranslationSkipSaveSP:)
        _ASM(push dx)
        _ASM(push ax)
        _ASM(push dx)

        //return frame
        _ASM(push cs)
#if defined(__BC__)
        _ASM(push offset TranslationDone)
#else
        _ASM(call hack)
        _ASM(jmp TranslationDone)
    _ASMLBL(hack:) //hack for watcom, not working with offset + asm label
        _ASM(push bp)
        _ASM(mov bp, sp)
        _ASM(xchg bx, word ptr ss:[bp+2]);
        _ASM(add bx, word ptr[bx+1])
        _ASM(xchg bx, word ptr ss:[bp+2]);
        _ASM(pop bp)
#endif
        //push parameter for translation
        _ASM(push cx) //note: pascal param left to right order
        _ASM2(shr ecx, 16)
        _ASM(push cx)
        _ASM(push 0)
        //chained call switch pm (return of translation)
        _ASM(push word ptr es:[bx + RMCB_OFF_RMSEG]);
        _ASM(push word ptr es:[bx + RMCB_OFF_SwitchPM]);
        //chained call translation. NOTE: use pascal calling convention so that stack parameters are clared by callee
        _ASM(push word ptr es:[bx + RMCB_OFF_RMSEG]);
        _ASM(push word ptr es:[bx + RMCB_OFF_Translation]);
        //chained call switch rm
        _ASM(push SEL_RMCB_CS*8)
        _ASM(push word ptr es:[bx + RMCB_OFF_SwitchRM]);
        _ASM2(mov ax, ss)
        _ASM2(mov ds, ax)
        _ASM(retf)
    _ASMLBL(TranslationDone:)
        _ASM2(mov ax, SEL_HIMEM_DS*8)
        _ASM2(mov ds, ax)

        _ASM(pop dx) //old ss
        _ASM2(cmp dx, SEL_HIMEM_DS*8)
        _ASM(jne TranslationSkipSaveSP2)
        _ASM2(mov ss:[bx + RMCB_OFF_PMSP], bp);
    _ASMLBL(TranslationSkipSaveSP2:)

        _ASM2(mov bx, sp)
        _ASM2(lss sp, ss:[bx]);

        _ASM(pop bp)
        _ASM(pop bx)
        _ASM(pop gs) _ASM(pop fs) _ASM(pop es)
    _ASM_END
    _fmemcpy(reg, stackREG, sizeof(DPMI_REG));
    DPMI_Rmcb->TranslationSP += sizeof(DPMI_REG) + DPMI_RMCB_TRASNLATION_STACK_SIZE;
}

static void DPMI_Shutdown(void);

uint32_t DPMI_PTR2L(void* ptr)
{
    assert(ptr != NULL);
    if(DPMI_PM)
        return (DPMI_HimemDS + FP_OFF(ptr));
    else
    {
        assert(0 && "real mode ptr mapping called.\n");
        return DPMI_Ptr16ToLinear(ptr);
    }
}

void* DPMI_L2PTR(uint32_t addr)
{
    if(DPMI_PM)
    {
        if(!(addr >= DPMI_HimemDS && addr <= DPMI_HimemDS+0xFFFFL))
            _LOG("%08lx ", addr);
        assert(addr >= DPMI_HimemDS && addr <= DPMI_HimemDS+0xFFFFL);
        //printf("%08lx %04x ", addr, (void*)(uint16_t)(addr - DPMI_HimemDS));
        return (void*)(uint16_t)(addr - DPMI_HimemDS);
    }
    else
    {
        assert(0 && "real mode ptr mapping called.\n");
        return (void near*)DPMI_LinearToPtr16(addr);
    }
}

void DPMI_Init(void)
{
    #if defined(__WC__)
    _DATA = _DS;
    #endif

    //_LOG("sbrk: %x, SP: %x, stack: %u\n", FP_OFF(sbrk(0)), _SP, stackavail());//small model: static data : heap : stack
    atexit(&DPMI_Shutdown);

    DPMI_V86 = DPMI_IsV86();
    DPMI_PM = 0;

    uint32_t TempMemory = DPMI_DOSMalloc((sizeof(DPMI_TempData)+15)>>4);
    DPMI_Temp = (DPMI_TempData far*)MK_FP(TempMemory, 0);
    uint16_t RMCBSize = DPMI_V86 ? (DPMI_RMCB_SIZE<=2048 ? 4096 : 4096 + DPMI_RMCB_SIZE)+4096 : DPMI_RMCB_SIZE; //combine 1st 4k page
    DPMI_RmcbMemory = DPMI_HighMalloc((RMCBSize+15)>>4, FALSE);
    if(TempMemory == 0 || DPMI_RmcbMemory == 0)
    {
        printf("Error: failed allocating memory.\n");
        exit(-1);
    }
    //pre allocate XMS memory. CS not used for now, use it on TSR
    DPMI_XMSHimemHandle = XMS_Alloc(DPMI_XMS_Size/1024L, &DPMI_SystemDS);
    /*if(DPMI_SystemDS + 128L*1024L < 0x400000L) //align to 4M (1st page table)
    {
        if( XMS_Realloc(DPMI_XMSHimemHandle, (0x400000L-DPMI_SystemDS)/1024, &DPMI_SystemDS))
        {
            uint16_t handle = XMS_Alloc(DPMI_XMS_Size/1024L, &DPMI_SystemDS);
            XMS_Free(DPMI_XMSHimemHandle);
            DPMI_XMSHimemHandle = handle;
        }
    }*/
    if(DPMI_XMSHimemHandle == 0)
    {
        printf("Error: unable to allocate XMS memory.\n");
        exit(1);
    }
    DPMI_HimemCS = DPMI_SystemDS + 64L*1024L;
    DPMI_HimemDS = DPMI_HimemCS + 64L*1024L;

    _LOG("HimemCS: %08lx, HimemDS: %08lx\n", DPMI_HimemCS, DPMI_HimemDS);
    _LOG("Temp: %08lx\n", (uint32_t)TempMemory<<4);
    //_LOG("RMCB: %08lx\n", (DPMI_RmcbMemory&0xFFFF)<<4);

    _fmemset(DPMI_Temp, 0, sizeof(DPMI_TempData));
    uint32_t RMCBMemroyLAddr = (DPMI_RmcbMemory&0xFFFFL)<<4;
    uint32_t PageTable0LAddr = align(RMCBMemroyLAddr, 4096);
    DPMI_Temp->RMCBLAddr = (!DPMI_V86 || PageTable0LAddr - RMCBMemroyLAddr >= DPMI_RMCB_SIZE) ? RMCBMemroyLAddr : PageTable0LAddr + 4096; //real mode direct to pm with no paging, only v86 uses page table.
    DPMI_Temp->PageTable0LAddr = PageTable0LAddr;

    _LOG("RMCB: %08lx\n", DPMI_Temp->RMCBLAddr);
    _LOG("PageTable0: %08lx\n", PageTable0LAddr);

    DPMI_Temp->LinearCS = ((uint32_t)_CS)<<4L;
    DPMI_Temp->LinearDS = ((uint32_t)_DS)<<4L;
    DPMI_Temp->TempMemoryHandle = TempMemory;
    _LOG("LinearCS: %08lx, LinearDS: %08lx\n", DPMI_Temp->LinearCS, DPMI_Temp->LinearDS);

    DPMI_SetupGDT();
    DPMI_SetupIDT();
    DPMI_SetupPaging();
    DPMI_SetupRMCB();
    DPMI_Rmcb->Ready = TRUE;

    if(!DPMI_InitProtectedMode())
    {
        printf("Error: unable to enter protected mode.\n");
        exit(1);
    }
    _LOG("Entered protected mode from %s.\n", DPMI_PM == PM_VCPI ? "virtual 8086 mode"  : "real mode");
}

static void DPMI_Shutdown(void)
{
    //assert(DPMI_PM);
    for(int i = 0; i < DPMI_MappedPages; ++i)
        XMS_Free(DPMI_XMSPageHandle[i]);

    DPMI_SwitchRealMode(DPMI_Rmcb->LinearDS);
    if(DPMI_XMSHimemHandle)
    {
        XMS_Free(DPMI_XMSHimemHandle);
        DPMI_XMSHimemHandle = 0;
    }
    DPMI_HighFree(DPMI_RmcbMemory);
    _LOG("(pseudo)DPMI terminating...\n");
}

uint32_t DPMI_MapMemory(uint32_t physicaladdr, uint32_t size)
{
    if(!DPMI_V86)
        return physicaladdr;

    _LOG("DPMI_MapMemory: %lx,%lx\n", physicaladdr,size);
    assert(physicaladdr >= 640L*1024L); //some OHCI controller has a BAR below 1M
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
        //assert(i != 0 || start >= PageTable0Offset); //make sure not overwriting VCPI entry
        //_LOG("mmap pt:%04ld, entry: %04ld-%04ld, addr: %08lx-%08lx\n", i, start, end-1, addr+(start<<12), addr+((end-1)<<12)+0xFFF);

        PDE pde = DPMI_PM ? DPMI_LoadPDE(DPMI_Rmcb->CR3, i) : DPMI_Temp->PageDir[i];
        if(!pde.bits.present)
        {
            assert(DPMI_PM == PM_VCPI); //need in pm mode to access cr3. otherwise need pre-add PT to PD
            assert(DPMI_XMSPageHandle != 0);
            uint32_t tbaddr = 0;
            uint16_t handle = XMS_Alloc(4, &tbaddr);
            assert(handle);
            if((tbaddr&0xFFF))//not 4k aligned
            {
                BOOL ret = XMS_Realloc(handle, 8, &tbaddr); //waste.TODO: use VCPI to alloc 4K page
                assert(ret);unused(ret);
                tbaddr = align(tbaddr, 4096);
                assert((tbaddr&0xFFF) == 0);
            }
            assert(DPMI_MappedPages < 1024);
            DPMI_XMSPageHandle[DPMI_MappedPages++] = handle;

            PDE_INITA(pde, tbaddr);
            DPMI_StorePDE(DPMI_Rmcb->CR3, i, &pde);

            PTE ptetmp = PTE_INIT(tbaddr); //temporarily map it to end of 4M
            CLIS();
            PTE pteold = DPMI_LoadPTE(DPMI_4MPageTableLAddr, 1023);
            DPMI_StorePTE(DPMI_4MPageTableLAddr, 1023, &ptetmp);
            _ASM_BEGIN
                _ASM(push eax)
                _ASM2(mov eax, cr3)
                _ASM2(mov cr3, eax)
                _ASM(pop eax)
            _ASM_END //flush TLB. TODO: invlpg(486+)
            DPMI_SetLinear(4L*1024L*1023L, 0, 4096); //clear mapped tb
            DPMI_StorePTE(DPMI_4MPageTableLAddr, 1023, &pteold);
            _ASM_BEGIN
                _ASM(push eax)
                _ASM2(mov eax, cr3)
                _ASM2(mov cr3, eax)
                _ASM(pop eax)
            _ASM_END //flush TLB. TODO: invlpg(486+)
            STIL();
        }

        if(DPMI_PM)
        {
            PTE ptetmp = PTE_INIT(PDE_ADDR(pde)); //temporarily map it to end of 4M
            CLIS();
            PTE pteold = DPMI_LoadPTE(DPMI_4MPageTableLAddr, 1023);
            DPMI_StorePTE(DPMI_4MPageTableLAddr, 1023, &ptetmp);
            _ASM_BEGIN
                _ASM(push eax)
                _ASM2(mov eax, cr3)
                _ASM2(mov cr3, eax)
                _ASM(pop eax)
            _ASM_END //flush TLB. TODO: invlpg(486+)
            for(uint32_t j = start; j < end; ++j)
            {
                PTE pte = PTE_INIT(addr + (j<<12L)); //1:1 map
                pte.bits.disable_cache = 1; //for device memory map
                //_LOG("PTE: %08lx => %08lx\n", DPMI_LoadD(4L*1024L*1023L+j*4L), addr + (j<<12L));
                DPMI_StorePTE(4L*1024L*1023L, j, &pte);
            }
            DPMI_StorePTE(DPMI_4MPageTableLAddr, 1023, &pteold);
            _ASM_BEGIN
                _ASM(push eax)
                _ASM2(mov eax, cr3)
                _ASM2(mov cr3, eax)
                _ASM(pop eax)
            _ASM_END //flush TLB. TODO: invlpg(486+)
            STIL();
        }
        else
        { //real mode, taking effect after init PM.
            uint32_t physical = PDE_ADDR(pde); //TODO: physical to linear (if UMB)
            assert(physical <= 1024L*1024L-4096L);
            PTE far* pt = (PTE far*)DPMI_LinearToPtr16(physical);
            for(uint32_t j = start; j < end; ++j)
            {
                PTE pte = PTE_INIT(addr + (j<<12L)); //1:1 map
                //pte.bits.disable_cache = 1;
                //_LOG("PTE: %ld,  %08lx => %08lx\n", j, *(uint32_t far*)(pt+j), *(uint32_t*)&pte);
                pt[j] = pte;
            }
        }
    }
    return physicaladdr;
}


uint32_t DPMI_UnmapMemory(uint32_t addr)
{
    unused(addr);
    return 0;
}

void* DPMI_DMAMalloc(unsigned int size, unsigned int alignment)
{
    CLIS();
    alignment = max(alignment,4);
    uint8_t* ptr = (uint8_t*)malloc(size + alignment + 2) + 2;
    uint32_t addr = DPMI_PTR2L(ptr);
    uint16_t offset = (uint16_t)(align(addr, alignment) - addr);
    void* aligned = ptr + offset;
    ((uint16_t*)aligned)[-1] = offset + 2;
    STIL();
    return aligned;
}

void* DPMI_DMAMallocNCPB(unsigned int size, unsigned int alignment)
{
    CLIS();
    alignment = max(alignment,4);
    uint8_t* ptr = (uint8_t*)malloc(size + alignment + 2) + 2;
    uint32_t addr = DPMI_PTR2L(ptr);
    uint16_t offset = (uint16_t)(align(addr, alignment) - addr);
    //_LOG("%lx %lx %lx", DPMI_HimemDS, addr, align(addr,alignment));

    uint32_t extra = 0;
    while(((align(addr,alignment)+extra)&~0xFFFUL) != ((align(addr,alignment)+extra+size-1)&~0xFFFUL))
    {
        //_LOG("*** PAGE BOUNDARY *** ");
        //_LOG("%lx %d %d ", align(addr,alignment), size, alignment);
        free(ptr-2);
        extra += align(size,alignment);
        ptr = (uint8_t*)malloc((size_t)(size + extra + alignment + 2)) + 2;
        addr = DPMI_PTR2L(ptr);
        offset = (uint16_t)(align(addr, alignment) - addr + extra);
        //_LOG("%lx ", align(addr,alignment)+extra);
        //_LOG("%lx %lx ", ((align(addr,alignment)+extra)&~0xFFFUL), ((align(addr,alignment)+extra+size-1)&~0xFFFUL));
    };

    void* aligned = ptr + offset;
    ((uint16_t*)aligned)[-1] = offset + 2;
    STIL();
    return aligned;
}


void DPMI_DMAFree(void* ptr)
{
    CLIS();
    uint16_t offset = ((uint16_t*)ptr)[-1];
    free((int8_t*)ptr - offset);
    STIL();
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
        r.w.es = (uint16_t)segment;
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

uint16_t DPMI_InstallISR(uint8_t i, void(*ISR)(void), DPMI_ISR_HANDLE* outputp handle)
{
    if(!DPMI_PM || handle == NULL)
    {
        assert(FALSE);
        return -1;
    }
    int RmcbIndex = DPMI_FindEmptyRMCBTableEntry();
    if(RmcbIndex == -1)
    {
        assert(FALSE);
        return -1;
    }
    CLIS();
    uint16_t off = DPMI_LoadW(i*4);
    uint16_t seg = DPMI_LoadW(i*4+2);
    handle->rm_cs = seg;
    handle->rm_offset = off;
    handle->cs = _CS;
    handle->offset = (uintptr_t)(void*)DPMI_UserINTHandler[i];
    handle->n = i;
    handle->extra = RmcbIndex;

    DPMI_UserINTHandler[i] = ISR;
    DPMI_Rmcb->Table[RmcbIndex].Target = &DPMI_HWIRQHandler; //resue PM handler. temporary implementation for IRQs
    DPMI_Rmcb->Table[RmcbIndex].UserReg = 0;
    
    //patch return code to IRET. not needed for now but if we support uninstall later, slot may be reused and the code need update each time on installation
    DPMI_Rmcb->Table[RmcbIndex].Code[DPMI_RMCBEntrySize-1] = 0xCF;

    DPMI_StoreW(i*4, offsetof(DPMI_RMCB, Table) + RmcbIndex * sizeof(DPMI_RMCBTable) + offsetof(DPMI_RMCBTable, Code));
    DPMI_StoreW(i*4+2, DPMI_Rmcb->RM_SEG);
    STIL();
    return 0;
}

uint16_t DPMI_UninstallISR(DPMI_ISR_HANDLE* inputp handle)
{
    if(handle == NULL || handle->cs == 0 || handle->rm_cs == 0 || handle->rm_offset == 0
        || DPMI_Rmcb->Table[handle->extra].Target == NULL)
    {
        assert(FALSE);
        return -1;
    }
    CLIS();
    uint8_t vec = handle->n;
    DPMI_UserINTHandler[vec] = (void(*)())handle->offset; //TODO: support chained call? for now restore to the first uninstall call.
    DPMI_Rmcb->Table[handle->extra].Target = NULL;

    if(DPMI_UserINTHandler[vec] == NULL) //no handler installed, restore default
    {
        if(DPMI_PM)
        {
            DPMI_StoreW(vec*4, handle->rm_offset);
            DPMI_StoreW(vec*4+2, handle->rm_cs);
        }
        else
        {
            *(uint16_t far*)MK_FP(0, vec*4) = handle->rm_offset;
            *(uint16_t far*)MK_FP(0, vec*4+2) = handle->rm_cs;
        }
    }
    STIL();
    return 0;
}

uint32_t DPMI_AllocateRMCB_RETF(void(*Fn)(void), DPMI_REG* reg)
{
    if(Fn == NULL || reg == NULL)
        return 0;
    int RmcbIndex = DPMI_FindEmptyRMCBTableEntry();
    if(RmcbIndex == -1)
    {
        assert(FALSE);
        return 0;
    }
    CLIS();
    DPMI_Rmcb->Table[RmcbIndex].UserReg = (uintptr_t)reg;
    DPMI_Rmcb->Table[RmcbIndex].Target = Fn;
    //patch return code to RETF (default is IRET)
    DPMI_Rmcb->Table[RmcbIndex].Code[DPMI_RMCBEntrySize-1] = 0xCB;

    _LOG("RMCB Index:%d, addr: %lx\n", RmcbIndex, (((uint32_t)DPMI_Rmcb->RM_SEG)<<4) + offsetof(DPMI_RMCB, Table) +  (offsetof(DPMI_RMCBTable, Code) + sizeof(DPMI_RMCBTable)*RmcbIndex));
    STIL();
    return (((uint32_t)DPMI_Rmcb->RM_SEG) << 16) | (offsetof(DPMI_RMCB, Table) + offsetof(DPMI_RMCBTable, Code) + sizeof(DPMI_RMCBTable)*RmcbIndex);
}

uint32_t DPMI_AllocateRMCB_IRET(void(*Fn)(void), DPMI_REG* reg)
{
    if(Fn == NULL || reg == NULL)
        return 0;
    int RmcbIndex = DPMI_FindEmptyRMCBTableEntry();
    if(RmcbIndex == -1)
    {
        assert(FALSE);
        return 0;
    }
    CLIS();
    DPMI_Rmcb->Table[RmcbIndex].UserReg = (uintptr_t)reg;
    DPMI_Rmcb->Table[RmcbIndex].Target = Fn;
    //patch return code to IRET. not needed for now but if we support uninstall later, slot may be reused and the code need update each time on installation
    DPMI_Rmcb->Table[RmcbIndex].Code[DPMI_RMCBEntrySize-1] = 0xCF;
    STIL();
    return (((uint32_t)DPMI_Rmcb->RM_SEG) << 16) | (offsetof(DPMI_RMCB, Table) + offsetof(DPMI_RMCBTable, Code) + sizeof(DPMI_RMCBTable)*RmcbIndex);
}

void DPMI_GetPhysicalSpace(DPMI_SPACE* outputp spc)
{
    spc->baseds = DPMI_HimemDS;
    spc->limitds = 64L*1024L - 1;
    spc->basecs = DPMI_HimemCS;
    spc->limitcs = 64L*1024L - 1;
    spc->stackpointer = 0xFFF8;
    return;
}

BOOL DPMI_TSR(void)
{
    if(!DPMI_PM)
        return FALSE;

    //assert(!DPMI_Rmcb->Interrupt);
    CLIS();
    DPMI_Rmcb->PM_SP = 0xFFF8;
    DPMI_TSRed = TRUE;
    DPMI_SwitchRealMode(DPMI_Rmcb->LinearDS);
    //DPMI_Rmcb->NeedRemapPIC = FALSE; //make sure no remmaping after TSR, it might conflict with other VCPI clients
    //_LOG("FLAGS: %04x\n", CPU_FLAGS());
    STIL();
    //printf("HimemCS: %08lx, HimemDS: %08lx\n", DPMI_HimemCS, DPMI_HimemDS);
    //printf("CR3: %08lx\n", DPMI_Rmcb->CR3);
    
    _ASM_BEGIN
        _ASM2(mov ax, 0x3100)
        _ASM2(mov dx, 0)//all stay in himem, RMCB uses INT 21h 48h that won't be released.
        _ASM(int 0x21)
    _ASM_END
    DPMI_TSRed = FALSE;
    return FALSE;
}

#if defined(__BC__)
#define __LIBCALL _Cdecl _FARFUNC
#elif defined(__WC__)
#define __LIBCALL _WCRTLINK
#else
#define __LIBCALL 
#endif

#if defined(__WC__)
namespace std {
#endif

//PM to RM translation for debug output
int __LIBCALL puts(const char* str)
{
    if(str == NULL)
        return 0;
    DPMI_REG r = {0};
    while(*str)
    {
        r.h.ah = 0x0E;
        r.h.al = *str;
        DPMI_CallRealModeINT(0x10, &r);
        if(*str =='\n')
        {
            r.h.ah = 0x0E;
            r.h.al = '\r';
            DPMI_CallRealModeINT(0x10, &r);
        }
        ++str;
    }
    return 0;
}

int __LIBCALL printf(const char* fmt, ...)
{
    static int recursion_gard = 0;
    if(!recursion_gard) //avoid reentrance in mode switching
    {
        recursion_gard = 1;

        //himem mode: data not copied during translation, so string buffers (%s) need to de-referenced before mode switch (otherwise ds/himemds data don't match)
        //use vsprintf to do it. BC doesn't have vsnprintf, potential buffer overflow exists.
        char* buff = (char*)malloc(2048);
        va_list aptr;
        va_start(aptr, fmt);
        int len = vsprintf(buff, fmt, aptr);
        va_end(aptr);
        DPMI_REG r = {0};
        for(int i = 0; i < len; ++i)
        {
            r.h.ah = 0x0E;
            r.h.al = buff[i];
            DPMI_CallRealModeINT(0x10, &r);
            if(buff[i] =='\n')
            {
                r.h.ah = 0x0E;
                r.h.al = '\r';
                DPMI_CallRealModeINT(0x10, &r);
            }
        }
        free(buff);
        recursion_gard = 0;
        return len;
    }
    return 0;
}

void __LIBCALL delay(unsigned millisec)
{
    uint32_t usec = (uint32_t)millisec*1000L;
    DPMI_REG r = {0};
    r.w.ax = 0x8600; //bios delay function, 976 usec resolution
    r.w.dx = (uint16_t)(usec&0xFFFF);
    r.w.cx = (uint16_t)(usec>>16);
    DPMI_CallRealModeINT(0x15,&r);
}

#if !defined(NDEBUG)
void __LIBCALL __assertfail(char * __msg,
                                  char * __cond,
                                  char * __file,
                                  int __line)
{//BC's __assertfail will cause #GP, probably caused by INTn (abort) / changing segment
    if(DPMI_PM)
    {
        printf("CR3: %08lx, DS: %04x, HimemCS: %08lx, HimemDS: %08lx\n", DPMI_Rmcb->CR3, _DS, DPMI_HimemCS, DPMI_HimemDS);
        printf(__msg, __cond, __file, __line);
        fflush(stdout);
    }
    else
    {
        printf(__msg, __cond, __file, __line);
        fflush(stdout);
    }
    if(!DPMI_TSRed)
        exit(1);
    else
        while(1);
}
#endif

#if defined(__WC__)
}//namespace std
#endif

//internal used, for debug
extern "C" BOOL DPMI_IsInProtectedMode()
{
    return DPMI_PM;
}

#endif
