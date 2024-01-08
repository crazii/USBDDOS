//don't use DPMI on Boarland C, instead use 16bit protected mode and keep the code and data near (no far calls/far data).
//note: the inline assember won't reconginze 386 instructions and registers, code need compiled with -B flag (compile via assembly) in BC31
//
//if in real mode, direct switch to protected mode, without paging, otherwise use VCPI to switch to 16 bit protected mode with 1:1 paging.
//small model, 64K+64K

//the _ASM* macros are not needed since this file is intended for BC only, but stil add them to workaround the syntax inteligence problem of vscode
#include <stdlib.h>
#include <dos.h>
#include <malloc.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>
#include "USBDDOS/DPMI/xms.h"
#include "USBDDOS/DPMI/dpmi.h"
#include "USBDDOS/DPMI/dpmi_bc.h"

//////////////////////////////////////////////////////////////////////////////
//RMCB
//////////////////////////////////////////////////////////////////////////////
#pragma option -k- //BC
#define DPMI_RMCB_TEST 0
#if DPMI_RMCB_TEST
extern "C" void __NAKED DPMI_RmcbLog()
{
    _LOG("RMCB TEST");
    _ASM_BEGIN _ASM(retf) _ASM_END
}
#endif
extern "C" void __NAKED DPMI_RMCbClientWrapper() //wrapper function for user rmcb (allow user uses normal ret/retf). copy DPMI_REG registers and return from a IRET frame
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
#pragma disable_message (13) //WC: unreachable code, but seems invalid
static void __NAKED near DPMI_RMCBCommonEntry() //commen entry for call back
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
        _ASM2(sub ax, RMCB_OFF_Table) //offsetof(DPMI_RMCB, Table)
        _ASM2(xor dx, dx)
        _ASM2(mov cx, RMCB_TABLE_ENTRY_SIZE)
        _ASM(div cx)
        _ASM(mul cx)
        _ASM2(add ax, RMCB_OFF_Table) //calc offset done
        _ASM2(mov bp, ax)
        _ASM2(mov ebx, dword ptr cs:[bp+4]); //reg far ptr
        _ASM2(mov ebp, dword ptr cs:[bp]); //load from memory & save to BP
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

        //switch himem stack. ss=(SEL_HIMEM_DS+x)*8
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
        _ASM2(mov ax, ds:[esi+DPMI_REG_OFF_FLAGS]);
        _ASM2(and ax, 0xBFFF) //mode switching needs clear NT flag 
        _ASM(push ax)
        _ASM(popf)
        //_ASM(int 0x03) //we can debug in pm

        _ASM(push ebx)
        _ASM(push fs)
        _ASM2(movzx edi, bx) //pm ss:[ebx] to linear: offset
        _ASM2(shr ebx, 16) //selector: hiword of ebx
        _ASM2(mov ax, SEL_SYS_DS*8)  //gdt
        _ASM2(mov fs, ax)
        _ASM2(mov eax, fs:[bx+2]); //drop limit, use base_low+base_middle
        _ASM2(mov ebx, fs:[bx+4]);
        _ASM2(and eax, 0x00FFFFFF)
        _ASM2(and ebx, 0xFF000000) //base_high
        _ASM2(or eax, ebx) //base address
        _ASM2(add edi, eax); //linear = offset + base
        _ASM2(mov ecx, DPMI_REG_SIZE)
        _ASM(pop fs)
        _ASM(pop ebx)
        
        _ASM(push ds)
        _ASM(push esi)
        _ASM(push es)
        _ASM(push edi)
        _ASM(push ecx) //popped by DPMI_RMCbClientWrapper

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
        _ASM2(mov ax, word ptr fs:[RMCB_OFF_ClientWrapperCS]);
        _ASM2(test ax, ax)
        _ASM(jz SkipCSForNearCall) //should be in tiny/small/compact model
        _ASM(push ax)
    _ASMLBL(SkipCSForNearCall:)
        _ASM(push offset DPMI_RMCbClientWrapper)

    _ASMLBL(SkipIRetWrapper:)
        _ASM(push ebp) //saved target far ptr
        _ASM2(mov eax, ebx)
        _ASM2(shr eax, 16) //setup user DS using reg far ptr's segment
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
        _ASM(push SEL_HIMEM_CS*8) //SEL_HIMEM_CS not working for medium model
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
#pragma enable_message (13) //WC

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
#pragma option -k //BC
static const int DPMI_RMCBEntrySize = FP_OFF(DPMI_RMCBEntryIRETEnd) - FP_OFF(DPMI_RMCBEntryIRET);

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

        _ASM(retf)
    _ASMLBL(callreturn:)
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
#pragma option -k- //BC
static void __NAKED DPMI_RMCBTranslationEnd() {}
#pragma option -k //BC

static void DPMI_SetupRMCB()
{
    assert(DPMI_Temp != NULL);
    assert(DPMI_RmcbMemory != 0);
    assert(DPMI_Rmcb == NULL);

    char* buff = (char *)malloc(DPMI_RMCB_SIZE);
    memset(buff, 0, DPMI_RMCB_SIZE);
    DPMI_RMCB rmcb;
    memset(&rmcb, 0, sizeof(rmcb));
    rmcb.GDTR = DPMI_Temp->gdtr;
    rmcb.IDTR = DPMI_Temp->idtr;
    rmcb.CR3 =  DPMI_PTUnmap(DPMI_Temp->PageTable0, DPMI_PTR16R2L(DPMI_Temp->PageDir)); //in case DPMI_Temp->PageDir is in UMB (already mapped). CR3 need physical addr.
    rmcb.VcpiInterface = DPMI_Temp->VcpiInterface;
    rmcb.VcpiInterfaceCS = DPMI_Temp->VcpiInterfaceCS;
    rmcb.RealModeIRQ0Vec = DPMI_Temp->RealModeIRQ0Vec;
    rmcb.ProtectedModeIRQ0Vec = DPMI_Temp->ProtectedModeIRQ0Vec;
    rmcb.RealModeIRQ8Vec = DPMI_Temp->RealModeIRQ8Vec;
    rmcb.ProtectedModeIRQ8Vec = DPMI_Temp->ProtectedModeIRQ8Vec;

    uint16_t offset = 0;
    offset += sizeof(rmcb);

    rmcb.CommonEntry = offset;
    uint8_t far* Code = (uint8_t far*)(void (far*)(void))&DPMI_RMCBCommonEntry;
    uint8_t far* CodeEnd = (uint8_t far*)(void (far*)(void))&DPMI_RMCBCommonEntryEnd;
    uint16_t CodeSize = CodeEnd - Code;
    _fmemcpy(buff + offset, Code, CodeSize);
    offset += CodeSize;

    rmcb.SwitchPM = offset;
    Code = (uint8_t far*)(void (far*)(void))(DPMI_V86 ? &DPMI_VCPIProtectedMode : &DPMI_DirectProtectedMode);
    CodeEnd = (uint8_t far*)(void (far*)(void))(DPMI_V86 ? &DPMI_VCPIProtectedModeEnd : &DPMI_DirectProtectedModeEnd);
    CodeSize = CodeEnd - Code;
    _fmemcpy(buff + offset, Code, CodeSize);
    offset += CodeSize;

    rmcb.SwitchRM = offset;
    Code = (uint8_t far*)(void (far*)(void))(DPMI_V86 ? &DPMI_VCPIRealMode : &DPMI_DirectRealMode);
    CodeEnd = (uint8_t far*)(void (far*)(void))(DPMI_V86 ? &DPMI_VCPIRealModeEnd : &DPMI_DirectRealModeEnd);
    CodeSize = CodeEnd - Code;
    _fmemcpy(buff + offset, Code, CodeSize);
    offset += CodeSize;

    rmcb.Translation = offset;
    Code = (uint8_t far*)(void (far*)(void))(&DPMI_RMCBTranslation);
    CodeEnd = (uint8_t far*)(void (far*)(void))(&DPMI_RMCBTranslationEnd);
    CodeSize = CodeEnd - Code;
    _fmemcpy(buff + offset, Code, CodeSize);
    offset += CodeSize;

    //_LOG("%d %d %d\n", offset, DPMI_RMCB_STACK_SIZE, DPMI_RMCB_SIZE);
    assert(offset + sizeof(sizeof(DPMI_REG))*4 + DPMI_RMCB_TRASNLATION_STACK_SIZE*4 + DPMI_RMCB_STACK_SIZE < DPMI_RMCB_SIZE);
    for(int i = 0; i < DPMI_RMCB_COUNT; ++i)
    {
        Code = (uint8_t far*)(void (far*)(void))&DPMI_RMCBEntryIRET;
        CodeEnd = (uint8_t far*)(void (far*)(void))&DPMI_RMCBEntryIRETEnd;
        CodeSize = CodeEnd - Code;
        //_LOG("%d\n", CodeSize);
        assert(CodeSize <= DPMI_RMCB_ENTRYCODE_SIZE);
        _fmemcpy(rmcb.Table[i].Code, Code, CodeSize);
        rmcb.Table[i].Target = 0;
        rmcb.Table[i].UserReg = 0;
        rmcb.Table[i].Code[CodeSize-1] = 0xCF; //patch IRET (opcode=0xCF)
    }
    _fmemcpy(buff, &rmcb, sizeof(rmcb));

    uint16_t segment = (uint16_t)(DPMI_Temp->RMCBLAddr>>4);
    DPMI_Rmcb = (DPMI_RMCB far*)MK_FP(segment, 0);
    _fmemcpy(DPMI_Rmcb, buff, DPMI_RMCB_SIZE);
    DPMI_Rmcb->Size = offset;
    DPMI_Rmcb->TranslationSP = DPMI_RMCB_SIZE;
    //PM_SP updated each time translation to real mode.
    DPMI_Rmcb->PM_SS = DPMI_GetDataSelector(_SS);
    DPMI_Rmcb->RM_SP = offset + DPMI_RMCB_STACK_SIZE;
    DPMI_Rmcb->RM_SEG = segment;

    DPMI_Rmcb->VcpiClient.CR3 = rmcb.CR3;
    DPMI_Rmcb->VcpiClient.GDTR_linear = DPMI_PTR16R2L(&DPMI_Rmcb->GDTR)+2;
    DPMI_Rmcb->VcpiClient.IDTR_linear = DPMI_PTR16R2L(&DPMI_Rmcb->IDTR)+2;
    DPMI_Rmcb->VcpiClient.LDT_Selector = SEL_LDT*8;
    DPMI_Rmcb->VcpiClient.TSS_Selector = SEL_TSS*8;
    DPMI_Rmcb->VcpiClient.EIP = 0; //set on mode swtiching
    DPMI_Rmcb->VcpiClient.CS = SEL_RMCB_CS*8;
    DPMI_Rmcb->VcpiClientAddr = DPMI_PTR16R2L(&DPMI_Rmcb->VcpiClient);
#if defined(__MEDIUM__) || defined(__LARGE__)
    DPMI_Rmcb->ClientWrapperCS = DPMI_GetCodeSelector(FP_SEG(DPMI_RMCbClientWrapper));
#endif

    //_LOG("RMCB: %08lx\n", (DPMI_RmcbMemory&0xFFFF)<<4);
    _LOG("CR3: %08lx\n", DPMI_Rmcb->VcpiClient.CR3);
    _LOG("GDTR: %08lx, size %d, offset %08lx\n", DPMI_PTR16R2L(&DPMI_Rmcb->GDTR), DPMI_Rmcb->GDTR.size, DPMI_Rmcb->GDTR.offset);
    _LOG("IDTR: %08lx, size %d, offset %08lx\n", DPMI_PTR16R2L(&DPMI_Rmcb->IDTR), DPMI_Rmcb->IDTR.size, DPMI_Rmcb->IDTR.offset);
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

static BOOL near DPMI_InitProtectedMode()
{
    //phase 1: temporary protected mode & copy GDT/IDT/Paging to Himem
    //no PM seg for DPMI_Temp, once in PM, DPMI_Temp is inaccessible. save on stack
    uint32_t GDTLAddr = DPMI_Temp->gdtr.offset;
    uint32_t IDTLAddr = DPMI_Temp->idtr.offset;
    uint16_t GDTSize = DPMI_Temp->gdtr.size + 1;
    uint16_t IDTSize = DPMI_Temp->idtr.size + 1;
    uint32_t PageDirLAddr = DPMI_PTR16R2L(DPMI_Temp->PageDir);
    //uint16_t Page4MOffset = DPMI_PTR16R2L(DPMI_Temp->PageTable0) - PageDirLAddr; //VCPI spec don't allow copy 1st 4M page
    uint32_t Page4MLAddr = DPMI_PTR16R2L(DPMI_Temp->PageTable0);
    uint16_t PageXMSOffset = (uint16_t)(DPMI_PTR16R2L(DPMI_Temp->PageTalbeHimem) - PageDirLAddr);
    uint16_t PageXMS2Offset = (uint16_t)(DPMI_PTR16R2L(DPMI_Temp->PageTalbeHimem2) - PageDirLAddr);
    uint16_t PageHanldeOffset = (uint16_t)(DPMI_PTR16R2L(DPMI_Temp->XMSPageHandle) - PageDirLAddr);
    //uint32_t LinearCS = DPMI_Temp->LinearCS;
    //uint32_t LinearDS = DPMI_Temp->LinearDS;

    DPMI_SwitchProtectedMode();

    #if 0//exception test
    _LOG("Exception test\n");
    _ASM_BEGIN
        _ASM(push 0xFFF8)
        _ASM(pop ds)
    _ASM_END
    #endif

    uint32_t offset = 0;

    //we're 1:1 mapped, linear address of gdt/idt is the same physical address
    //_LOG("Copy GDT.\n");
    DPMI_CopyLinear(DPMI_SystemDS + offset, GDTLAddr, GDTSize);
    DPMI_Rmcb->GDTR.offset = DPMI_SystemDS + offset;
    offset += GDTSize;

    //_LOG("Copy IDT.\n");
    DPMI_CopyLinear(DPMI_SystemDS + offset, IDTLAddr, IDTSize);
    DPMI_Rmcb->IDTR.offset = DPMI_SystemDS + offset;
    offset += IDTSize;

    if(DPMI_V86)
    {
        //_LOG("Copy page tables.\n");
        offset = align(DPMI_SystemDS + offset, 4096) - DPMI_SystemDS; //BUGFIX DPMI_Rmcb->CR3 need align to 4K not offset or initial memory
        DPMI_CopyLinear(DPMI_SystemDS + offset, PageDirLAddr, VCPI_PAGING_MEM_SIZE);
        DPMI_Rmcb->CR3 = DPMI_SystemDS + offset; //physical addr
        DPMI_Rmcb->VcpiClient.CR3 = DPMI_Rmcb->CR3;
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

    DPMI_SwitchRealMode();
    _LOG("Himem data setup ready.\n");

    //phase 2: return to real mode, and perform the switch again using new himem data
    DPMI_HighFree(DPMI_Temp->PageMemory);
    DPMI_HighFree(DPMI_Temp->TempMemoryHandle);
    DPMI_Temp = NULL;

    {
        CLIS();
        //get segments before switch PM, incase addr being patched, or compiler use CS/DS directly instead of using 'seg addr'
        uint16_t cs = DPMI_GetCodeSelector(FP_SEG(DPMI_NullINTHandler));

        DPMI_SwitchProtectedMode(); //final PM and also pre-test the adjustment above.
        IDT far* idt = (IDT far*)MK_FP(SEL_SYS_DS*8, GDTSize);
        for(int i = 0; i < 256; ++i)
            idt[i].selector = cs; //change IDT selector to himem
        STIL();
    }
    //asm int 0x21;
    return TRUE;
}

//////////////////////////////////////////////////////////////////////////////
//common code
//////////////////////////////////////////////////////////////////////////////
static void __CDECL DPMI_CallRealMode(DPMI_REG* reg, unsigned INTn) //INTn < 256: interrupt call
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
    DPMI_Rmcb->TranslationSP -= DPMI_RMCB_TRASNLATION_STACK_SIZE; //reserve space for re-entrant

    _ASM_BEGIN
        _ASM(push es)
        _ASM(push fs)
        _ASM(push gs)
        _ASM(push bx)
        _ASM(push bp)
        _ASM(push ecx)
        _ASM2(mov dx, ss)

        _ASM2(mov cx, INTn)    //INTn stored at ss:bp[xx]. save parameters to register before switching stack
        _ASM2(shl ecx, 16)
        _ASM2(mov ax, sp) //save SP before switching stack
        _ASM2(lss sp, stackREG) //stackREG stored at ss:bp[xx]. load ss:sp in one shot, or interrutp will be on wrong stack
        _ASM2(mov cx, sp) //DPMI_REG near* for translation call
        _ASM2(les bx, DPMI_Rmcb)
        _ASM2(cmp dx, SEL_HIMEM_DS*8) //if not on the himem stack, skip set sp. coulde be on translation stack. (rmcb stack is temporary and interrupt should not be enabled on it)
        _ASM(jb TranslationSkipSaveSP)
        _ASM2(mov bp, es:[bx + RMCB_OFF_PMSP]);
        _ASM2(mov es:[bx + RMCB_OFF_PMSP], ax); //if real mode interrupt reflect to pm, it will use this sp pointer
    _ASMLBL(TranslationSkipSaveSP:)
        _ASM(push dx)
        _ASM(push ax)
        _ASM(push dx)

        _ASM(push ds) //save ds
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
        _ASM(add bx, word ptr cs:[bx+1]);
        _ASM(add bx, 3) //size of jmp instruction itself
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
        _ASM(pop ax) //pushed ds
        _ASM2(mov ds, ax)

        _ASM(pop dx) //old ss
        _ASM2(cmp dx, SEL_HIMEM_DS*8)
        _ASM(jb TranslationSkipSaveSP2)
        _ASM2(mov ss:[bx + RMCB_OFF_PMSP], bp);
    _ASMLBL(TranslationSkipSaveSP2:)

        _ASM2(mov bx, sp)
        _ASM2(lss sp, ss:[bx]);

        _ASM(pop ecx)
        _ASM(pop bp)
        _ASM(pop bx)
        _ASM(pop gs)
        _ASM(pop fs)
        _ASM(pop es)
    _ASM_END
    _fmemcpy(reg, stackREG, sizeof(DPMI_REG));
    DPMI_Rmcb->TranslationSP += (sizeof(DPMI_REG) + DPMI_RMCB_TRASNLATION_STACK_SIZE);
}

uint32_t DPMI_PTR2L(void* ptr)
{
    assert(ptr != NULL);
    if(DPMI_PM)
        return (DPMI_HimemData + FP_OFF(ptr));
    else
    {
        assert(0 && "real mode ptr mapping called.\n");
        return DPMI_PTR16R2L(ptr);
    }
}

void* DPMI_L2PTR(uint32_t addr)
{
    if(DPMI_PM)
    {
        return (void*)DPMI_L2PTR16P(addr);
    }
    else
    {
        assert(0 && "real mode ptr mapping called.\n");
        return (void near*)DPMI_L2PTR16R(addr);
    }
}

int DPMI_Exit(int c)
{
    //don't enable segment patch for normal execution (normal code may cause seg fault by bug), only use it at exit.
    DPMI_ExceptionPatch = TRUE; 
    if(!DPMI_PM) //clean up memory for real mode. //in PM shutdown is called via INT21H
        DPMI_Shutdown();
    #undef exit
    exit(c);
    #define exit(c) DPMI_Exit(c)
}

void DPMI_Init(void)
{
    _DATA_SEG = _SS;
    _CODE_SEG = _psp + 0x10; //+0x100 as segment
    _INTR_SEG = FP_SEG(DPMI_UserINTHandler);
    #if defined(__LARGE__) || defined(__COMPACT__)
    _DATA_SIZE = 1024UL*1024UL; //1M for dynamic increments
    #else
    _DATA_SIZE = 64UL*1024UL;
    #endif
    _CODE_SIZE = (((uint32_t)_SS)<<4) - (((uint32_t)_CODE_SEG)<<4);

    #if defined(__WC__)
    //avoid further DOS mem call in PM mode for malloc
    _heapgrow(); //sbrk won't work. the heap doesn't know the gap between heap end and the brk. it just assmes brk is the end of the heap.
    _STACK_PTR = _STACKTOP;
    #else
    _STACK_PTR = 0xFFF8;
    #endif

    BOOL loaded = DPMI_LOADER_Init(_CODE_SEG, _DATA_SEG, _CODE_SIZE);
    assert(loaded);unused(loaded);
    if(!loaded)
    {
        printf("Error loading relocation info.\n");
        exit(1);
    }

    //small model:
    //BC: static data : heap : stack; fixed 64K data seg size.
    //WC: static data : stack (fixed size, set on linker) : heap; incremental: memory allocated from DOS on request.
    _LOG("sbrk: %x, SP: %x, stack: %u\n", FP_OFF(sbrk(0)), _SP, stackavail());
    DPMI_V86 = (uint8_t)DPMI_IsV86();
    DPMI_PM = 0;

    uint16_t RMCBSize = DPMI_V86 ? (DPMI_RMCB_SIZE<=2048 ? 4096 : 4096 + DPMI_RMCB_SIZE)+4096 : DPMI_RMCB_SIZE; //combine 1st 4k page
    //use high malloc because WC's malloc may expand DS size on request, if normal block is allocated, expanding will fail
    //the sbrk above will solve the problem but still it's better to allocate memory from high address.
    DPMI_RmcbMemory = DPMI_HighMalloc((RMCBSize+15)>>4, FALSE);
    uint32_t TempMemory = DPMI_HighMalloc((sizeof(DPMI_TempData)+15)>>4, FALSE);

    if(TempMemory == 0 || DPMI_RmcbMemory == 0)
    {
        if(DPMI_RmcbMemory) DPMI_HighFree(DPMI_RmcbMemory);
        if(TempMemory) DPMI_HighFree(TempMemory);
        printf("Error: failed allocating memory.\n");
        exit(-1);
    }

    DPMI_Temp = (DPMI_TempData far*)MK_FP(TempMemory, 0);
    //pre allocate XMS memory. CS not used for now, use it on TSR
    DPMI_XMS_Size += _CODE_SIZE + _DATA_SIZE;
    DPMI_XMS_Size = align(DPMI_XMS_Size, 4UL*1024UL);
    DPMI_XMSHimemHandle = XMS_Alloc((uint16_t)(DPMI_XMS_Size/1024UL), &DPMI_SystemDS);

    if(DPMI_V86 && DPMI_XMSHimemHandle) //since we need 1:1 map of memory, and VCPI spec doesn't allow modify the first page table, we need allocate memory above 4M
    {
        if(DPMI_SystemDS < 0x400000UL) //align to 4M (1st page table)
        {
            if( XMS_Realloc(DPMI_XMSHimemHandle, (uint16_t)((0x400000UL-DPMI_SystemDS)/1024UL), &DPMI_SystemDS))
            {
                uint16_t handle = XMS_Alloc((uint16_t)(DPMI_XMS_Size/1024UL), &DPMI_SystemDS);
                DPMI_XMSBelow4MHandle = DPMI_XMSHimemHandle; //keep to free on exit
                //XMS_Free(DPMI_XMSHimemHandle); //don't free, so that all later xms alloc will be above 4M.
                DPMI_XMSHimemHandle = handle;
            }
        }
    }

    if(DPMI_XMSHimemHandle == 0)
    {
        printf("Error: unable to allocate XMS memory.\n");
        exit(1);
    }
    //DPMI_SystemDS = align(DPMI_SystemDS, 4096UL);
    DPMI_HimemCode = DPMI_SystemDS + 64UL*1024UL;
    DPMI_HimemData = DPMI_HimemCode + _CODE_SIZE;

    _LOG("HimemCS: %08lx, HimemDS: %08lx\n", DPMI_HimemCode, DPMI_HimemData);
    _LOG("Temp: %08lx\n", (uint32_t)TempMemory<<4);
    //_LOG("RMCB: %08lx\n", (DPMI_RmcbMemory&0xFFFF)<<4);

    _fmemset(DPMI_Temp, 0, sizeof(DPMI_TempData));
    uint32_t RMCBMemroyLAddr = (DPMI_RmcbMemory&0xFFFFL)<<4;
    uint32_t PageTable0LAddr = align(RMCBMemroyLAddr, 4096);
    DPMI_Temp->RMCBLAddr = (!DPMI_V86 || PageTable0LAddr - RMCBMemroyLAddr >= DPMI_RMCB_SIZE) ? RMCBMemroyLAddr : PageTable0LAddr + 4096; //real mode direct to pm with no paging, only v86 uses page table.
    DPMI_Temp->PageTable0LAddr = PageTable0LAddr;

    _LOG("RMCB: %08lx\n", DPMI_Temp->RMCBLAddr);
    _LOG("PageTable0: %08lx\n", PageTable0LAddr);

    DPMI_Temp->LinearCS = ((uint32_t)_CODE_SEG)<<4L;
    DPMI_Temp->LinearDS = ((uint32_t)_DATA_SEG)<<4L;
    DPMI_Temp->TempMemoryHandle = TempMemory;
    _LOG("LinearCS: %08lx, LinearDS: %08lx\n", DPMI_Temp->LinearCS, DPMI_Temp->LinearDS);

    DPMI_SetupGDT();
    DPMI_SetupIDT();
    DPMI_SetupPaging();
    DPMI_SetupRMCB();

    if(!DPMI_InitProtectedMode())
    {
        printf("Error: unable to enter protected mode.\n");
        exit(1);
    }
    _LOG("Entered protected mode from %s.\n", DPMI_PM == PM_VCPI ? "virtual 8086 mode"  : "real mode");

    _LOG("main seg: %04x->%04x\n", DPMI_LOADER_SegMain, DPMI_GetCodeSelector(DPMI_LOADER_SegMain));
#if (defined(__MEDIUM__) || defined(__LARGE__))
    //called from real mode, real mode code segments on stack (far call), from C startup to here.
    //we can handle it in exception but exception is intended for C finalizing routine, besides it's more safe to patch it precisely
    DPMI_LOADER_PatchStack(DPMI_LOADER_SegMain, DPMI_GetCodeSelector(DPMI_LOADER_SegMain), _BP, 16, 1);
#endif
    //atexit((void(far*)(void))DPMI_Shutdown);
}

static void near DPMI_Shutdown(void)
{
    //DPMI_XMSPageHandle are stored in protected mode memory, free them before switching to real mode
    for(int i = 0; i < DPMI_MappedPages; ++i)
        XMS_Free(DPMI_XMSPageHandle[i]);

    DPMI_SwitchRealMode();
    DPMI_LOADER_Shutdown();

    if(DPMI_XMSHimemHandle)
    {
        XMS_Free(DPMI_XMSHimemHandle);
        DPMI_XMSHimemHandle = 0;
    }
    if(DPMI_XMSBelow4MHandle)
    {
        XMS_Free(DPMI_XMSBelow4MHandle);
        DPMI_XMSBelow4MHandle = 0;
    }
    if(DPMI_RmcbMemory)
    {
        DPMI_HighFree(DPMI_RmcbMemory);
        DPMI_RmcbMemory = 0;
    }
    STI();
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

        PDE pde;
        if(DPMI_PM)
            pde = DPMI_LoadPDE(DPMI_Rmcb->CR3, i);
        else //seems WC DO copy far structs, but there's a warning. (still on some cases it ignore the segment)
            _fmemcpy(&pde, &DPMI_Temp->PageDir[i], sizeof(PDE));

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
            PTE far* pt = (PTE far*)DPMI_L2PTR16R(physical);
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
    assert((short)ptr != 2);
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
    assert((short)ptr != 2);
    uint32_t addr = DPMI_PTR2L(ptr);
    uint16_t offset = (uint16_t)(align(addr, alignment) - addr);
    //_LOG("%lx %lx %lx", DPMI_HimemData, addr, align(addr,alignment));

    uint32_t extra = 0;
    while(((align(addr,alignment)+extra)&~0xFFFUL) != ((align(addr,alignment)+extra+size-1)&~0xFFFUL))
    {
        //_LOG("*** PAGE BOUNDARY *** ");
        //_LOG("%lx %d %d ", align(addr,alignment), size, alignment);
        free(ptr-2);
        extra += align(size,alignment);
        ptr = (uint8_t*)malloc((size_t)(size + extra + alignment + 2)) + 2;
        assert((short)ptr != 2);
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

//the DPMI_HWIRQHandler is not directly called in IDT but with a dummy wrapper( DPMI_IRQHANDLER(xx) ).
//the return addr on stack will be poped & discarded and IRET directly by DPMI_HWIRQHandler.
#pragma option -k- //BC
static void __NAKED far DPMI_ISRWrapper()
{
    _ASM_BEGIN
        _ASM(call DPMI_HWIRQHandler)
    _ASM_END
}
#pragma option -k

uint16_t DPMI_InstallISR(uint8_t i, void(*ISR)(void), DPMI_ISR_HANDLE* outputp handle)
{
    int ds = FP_SEG(handle); //record input ds

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
    handle->cs = FP_SEG(DPMI_UserINTHandler[i]);
    handle->offset = FP_OFF(DPMI_UserINTHandler[i]);
    handle->n = i;
    handle->extra = RmcbIndex;

    DPMI_UserINTHandler[i] = (void(far *)(void))ISR;
    DPMI_UserINTHandlerDS[i] = ds;
    DPMI_Rmcb->Table[RmcbIndex].Target = &DPMI_ISRWrapper; //resue PM handler. temporary implementation for IRQs. DPMI_HWIRQHandler needs a wrapper
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
    if(handle == NULL || handle->rm_cs == 0 || handle->rm_offset == 0
        || DPMI_Rmcb->Table[handle->extra].Target == NULL)
    {
        assert(FALSE);
        return -1;
    }
    CLIS();
    uint8_t vec = handle->n;
    DPMI_UserINTHandler[vec] = (void(far *)(void))MK_FP(handle->cs,handle->offset); //TODO: support chained call? for now restore to the first uninstall call.
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
    DPMI_Rmcb->Table[RmcbIndex].UserReg = reg;
    DPMI_Rmcb->Table[RmcbIndex].Target = (DPMI_RMCB_ENTRY)Fn;
    //patch return code to RETF (default is IRET)
    DPMI_Rmcb->Table[RmcbIndex].Code[DPMI_RMCBEntrySize-1] = 0xCB;

    //_LOG("RMCB Index:%d, addr: %lx\n", RmcbIndex, (((uint32_t)DPMI_Rmcb->RM_SEG)<<4) + offsetof(DPMI_RMCB, Table) +  (offsetof(DPMI_RMCBTable, Code) + sizeof(DPMI_RMCBTable)*RmcbIndex));
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
    DPMI_Rmcb->Table[RmcbIndex].UserReg = reg;
    DPMI_Rmcb->Table[RmcbIndex].Target = (DPMI_RMCB_ENTRY)Fn;
    //patch return code to IRET. not needed for now but if we support uninstall later, slot may be reused and the code need update each time on installation
    DPMI_Rmcb->Table[RmcbIndex].Code[DPMI_RMCBEntrySize-1] = 0xCF;
    STIL();
    return (((uint32_t)DPMI_Rmcb->RM_SEG) << 16) | (offsetof(DPMI_RMCB, Table) + offsetof(DPMI_RMCBTable, Code) + sizeof(DPMI_RMCBTable)*RmcbIndex);
}

void DPMI_GetPhysicalSpace(DPMI_SPACE* outputp spc)
{
    //NOTE: not working for MEDIUM/LARGE model
    //but currently not used. used only for RetroWave port trapping (emm.c)
    spc->baseds = DPMI_HimemData;
    spc->limitds = 64L*1024L - 1;
    spc->basecs = DPMI_HimemCode;
    spc->limitcs = 64L*1024L - 1;
    spc->stackpointer = _STACK_PTR;
    return;
}

BOOL DPMI_TSR(void)
{
    if(!DPMI_PM)
        return FALSE;

    //assert(!DPMI_Rmcb->Interrupt);
    CLIS();
    DPMI_Rmcb->PM_SP = _STACK_PTR;
    DPMI_TSRed = TRUE;
    DPMI_SwitchRealMode();
    //_LOG("FLAGS: %04x\n", CPU_FLAGS());
    STIL();
    //printf("HimemCS: %08lx, HimemDS: %08lx\n", DPMI_HimemCode, DPMI_HimemData);
    //printf("CR3: %08lx\n", DPMI_Rmcb->CR3);

    DPMI_LOADER_Shutdown();

    if(DPMI_XMSBelow4MHandle)
    {
        XMS_Free(DPMI_XMSBelow4MHandle);
        DPMI_XMSBelow4MHandle = 0;
    }
    
    _ASM_BEGIN
        _ASM2(mov ax, 0x3100)
        _ASM2(mov dx, 0)//all stay in himem, RMCB uses INT 21h 48h that won't be released.
        _ASM(int 0x21)
    _ASM_END
    DPMI_TSRed = FALSE;
    return FALSE;
}

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
        #define BUFSIZE 1024
        char buff[BUFSIZE];
        va_list aptr;
        va_start(aptr, fmt);
        #if defined(__BC__)
        int len = vsprintf(buff, fmt, aptr);
        #else
        int len = vsnprintf(buff, BUFSIZE, fmt, aptr);
        #endif
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
        recursion_gard = 0;
        return len;
    }
    return 0;
}

void __LIBCALL delay(unsigned millisec)
{
    uint32_t usec = ((uint32_t)millisec)*1000UL;
    DPMI_REG r = {0};
    r.w.ax = 0x8600; //bios delay function, 976 usec resolution
    r.w.dx = (uint16_t)(usec&0xFFFF);
    r.w.cx = (uint16_t)(usec>>16);
    DPMI_CallRealModeINT(0x15,&r);
}

#if !defined(NDEBUG)

#if defined(__BC__)
void __LIBCALL __assertfail(char* __msg, char* __cond, char* __file, int __line)
{ //BC's __assertfail will cause #GP, probably caused by INTn (abort)
    #if DEBUG
    STI();
    _LOG(__msg, __cond, __file, __line);
    #else
    printf(__msg, __cond, __file, __line);
    //fflush(stdout);
    #endif
#else //defined(__BC__)
void __LIBCALL _assert99(char* expr, char* func, char* file, int line)
{
    #if DEBUG
    STI();
    _LOG("%s(%d): in function `%s`\nassertion failed: %s\n", file, line, func, expr);
    #else
    printf("%s(%d): in function `%s`\nassertion failed: %s\n", file, line, func, expr);
    //fflush(stdout);
    #endif
#endif //defined(__BC__)

    #if DEBUG && 0
    if(DPMI_PM)
        _LOG("CR3: %08lx, DS: %04x, HimemCS: %08lx, HimemDS: %08lx\n", DPMI_Rmcb->CR3, _DS, DPMI_HimemCode, DPMI_HimemData);
    #endif
    
    if(!DPMI_TSRed)
        exit(1);
    else
        while(1);
}

#endif //!defined(NDEBUG)

//internal used, for debug
extern "C" BOOL DPMI_IsInProtectedMode()
{
    return DPMI_PM;
}

#if defined(__BC__)
//static inline function not optimized out (if not used) in BC
//put it in .c/.cpp
uint32_t PLTFM_BSF(uint32_t x)
{
    uint32_t i; __asm {push eax; bsf eax, x; mov i, eax; pop eax} return i;
}
#endif
