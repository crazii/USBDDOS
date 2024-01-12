#ifndef _DPMI_I21_H_
#define _DPMI_I21_H_
#include <string.h>
#include "USBDDOS/DPMI/dpmi.h"
#include "USBDDOS/dbgutil.h"

#ifdef __cplusplus
extern "C" {
#endif

static void (near *DPMI_I21H_pfnTerminate)(void) = NULL;
static uint32_t (near *DPMI_I21H_PM_FP2L)(uint16_t ds, uint16_t off) = NULL;

void __CDECL near DPMI_INT21H_Translation(DPMI_REG near* reg)
{
    DPMI_REG backup = *reg;
    uint32_t translsationBuffer = 0;
    _LOG("INT 21H, AH=%04x\n", reg->h.ah);

    if(reg->h.ah == 0x3D //open existing file (used by std in/out functions)
    || reg->h.ah == 0x09 //output string
    ) 
    {
        const char far* name = (const char far*)MK_FP(reg->w.ds, reg->w.dx);
        size_t size = 0;
        if(reg->h.ah == 0x09)
            while(name[size++] != '$');
        else
            size = _fstrlen(name)+1;
        translsationBuffer = DPMI_HighMalloc((size+15)>>4, FALSE);
        uint32_t seg = translsationBuffer&0xFFFF;
        DPMI_CopyLinear(seg<<4, DPMI_I21H_PM_FP2L(reg->w.ds, reg->w.dx), size);
        reg->w.ds = (uint16_t)seg;
        reg->w.dx = 0;
    }
    else if(reg->h.ah == 0x40// write to file (std in/out)
    || reg->h.ah == 0x3F //read from file
    )
    {
        translsationBuffer = DPMI_HighMalloc((reg->w.cx+15)>>4, FALSE);
        uint32_t seg = translsationBuffer&0xFFFF;
        if(reg->h.ah == 0x40)
            DPMI_CopyLinear(seg<<4, DPMI_I21H_PM_FP2L(reg->w.ds, reg->w.dx), reg->w.cx);
        reg->w.ds = (uint16_t)seg;
        reg->w.dx = 0;
    }
    else
        reg->w.ds = DPMI_GetDataSegment(reg->w.ds);
    reg->w.es = DPMI_GetDataSegment(reg->w.es);
    reg->w.fs = DPMI_GetDataSegment(reg->w.fs);
    reg->w.gs = DPMI_GetDataSegment(reg->w.gs);
    //_LOG("ds: %04x, es: %04x, fs: %04x, gs: %04x\n", reg->w.ds, reg->w.es, reg->w.fs, reg->w.gs);

    if(reg->h.ah == 0x4C)
    {
        if(DPMI_I21H_pfnTerminate)
            (*DPMI_I21H_pfnTerminate)();
		DPMI_ExceptionPatch = FALSE;
    }
    if(reg->h.ah == 0x31)
    {
        DPMI_ExceptionPatch = FALSE;
    }

    DPMI_CallRealModeINT(0x21, reg);

    uint32_t seg = translsationBuffer&0xFFFF;
    if(reg->h.ah == 0x3D || reg->h.ah == 0x3F || reg->h.ah == 0x40 || reg->h.ah == 0x09)
    {
        //reg->w.ds = backup.w.ds;
        reg->w.dx = backup.w.dx;
        if(reg->h.ah == 0x3F)
            DPMI_CopyLinear(DPMI_I21H_PM_FP2L(backup.w.ds, backup.w.dx), seg<<4, reg->w.ax);
    }
    reg->w.ds = backup.w.ds;
    reg->w.es = backup.w.es;
    reg->w.fs = backup.w.fs;
    reg->w.gs = backup.w.gs;
    //reg->w.flags |= CPU_TFLAG; //single step debug

    if(translsationBuffer)
        DPMI_HighFree(translsationBuffer);
    _LOG("INT 21H END\n");
}

#pragma option -k- //BC
static void __NAKED DPMI_INT21H()
{
    _ASM_BEGIN
        //make a DPMI_REG struct on the stack
        _ASM(push ss)
        _ASM(push sp) //ss: sp
        _ASM(push cs)
        _ASM(push ax)
        _ASM(push gs)
        _ASM(push fs)
        _ASM(push ds)
        _ASM(push es)
        _ASM(pushf)
        _ASM(pushad)

        _ASM2(mov ax, SEL_INTR_DS*8)
        _ASM2(mov ds, ax)

        _ASM(push sp)
        _ASM(call DPMI_INT21H_Translation)
        _ASM(pop ax)

        //load DPMI_REG struct into registers
        _ASM2(mov bp, sp)
        _ASM2(mov ax, word ptr [bp+DPMI_REG_OFF_FLAGS]); //flags
        _ASM2(mov word ptr [bp+DPMI_REG_SIZE+4], ax) //IRET flags
        _ASM(popad)
        _ASM2(add sp, 2) //skip flags
        _ASM(pop es)
        _ASM(pop ds)
        _ASM(pop fs)
        _ASM(pop gs)
        _ASM2(add sp, 8) //skip cs:ip, ss:sp
        _ASM(iret)
    _ASM_END
}

#if DEBUG
static void __NAKED DPMI_INT16H()
{
    _ASM_BEGIN
        //make a DPMI_REG struct on the stack
        _ASM(push ss)
        _ASM(push sp) //ss: sp
        _ASM(push cs)
        _ASM(push ax)
        _ASM(push gs)
        _ASM(push fs)
        _ASM(push ds)
        _ASM(push es)
        _ASM(pushf)
        _ASM(pushad)

        _ASM2(mov ax, SEL_INTR_DS*8)
        _ASM2(mov ds, ax)

        _ASM(push sp)
        _ASM(push 0x16)
        _ASM(call DPMI_CallRealModeINT)
        _ASM(pop ax)
        _ASM(pop ax)

        //load DPMI_REG struct into registers
        _ASM2(mov bp, sp)
        _ASM2(mov ax, word ptr [bp+DPMI_REG_OFF_FLAGS]); //flags
        _ASM2(mov word ptr [bp+DPMI_REG_SIZE+4], ax) //IRET flags
        _ASM(popad)
        _ASM2(add sp, 2) //skip flags
        _ASM(pop es)
        _ASM(pop ds)
        _ASM(pop fs)
        _ASM(pop gs)
        _ASM2(add sp, 8) //skip cs:ip, ss:sp
        _ASM(iret)
    _ASM_END
}
#endif
#pragma option -k //BC

#ifdef __cplusplus
}
#endif

#endif//_DPMI_I21_H_