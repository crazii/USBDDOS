#ifndef _DPMI_I21_H_
#define _DPMI_I21_H_
#include <string.h>
#include "USBDDOS/DPMI/dpmi.h"
#include "USBDDOS/dbgutil.h"

#ifdef __cplusplus
extern "C" {
#endif

static void (*DPMI_I21H_pfnTerminate)(void) = NULL;
static uint32_t (*DPMI_I21H_PM_FP2L)(uint16_t ds, uint16_t off) = NULL;

void __CDECL DPMI_INT21H_Translation(DPMI_REG* reg)
{
    DPMI_REG backup = *reg;
    uint32_t translsationBuffer = 0;
    _LOG("INT 21h, AH=%04x\n", reg->h.ah);

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

    if(reg->h.ah == 0x4C)
    {
        STI();
        if(DPMI_I21H_pfnTerminate)
            (*DPMI_I21H_pfnTerminate)();
    }

    uint32_t seg = translsationBuffer&0xFFFF;
    if(reg->h.ah == 0x3D || reg->h.ah == 0x3F || reg->h.ah == 0x40 || reg->h.ah == 0x09)
    {
        reg->w.ds = backup.w.ds;
        reg->w.dx = backup.w.dx;
        if(reg->h.ah == 0x3F)
            DPMI_CopyLinear(DPMI_I21H_PM_FP2L(backup.w.ds, backup.w.dx), seg<<4, reg->w.ax);
    }

    if(translsationBuffer)
        DPMI_HighFree(translsationBuffer);
    DPMI_CallRealModeINT(0x21, reg);
}

#pragma option -k-
static void __NAKED DPMI_INT21H()
{
    _ASM_BEGIN
        _ASM(push bp)
        _ASM2(mov bp, sp)
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

        _ASM2(mov ax, sp)
        _ASM(push ax)
        _ASM(call DPMI_INT21H_Translation)

        //load DPMI_REG struct into registers
        _ASM2(mov ax, word ptr [bp-18]); //flags
        _ASM2(mov word ptr [bp+6], ax) //IRET flags
        _ASM(popad)
        _ASM2(add sp, 2) //skip flags
        _ASM(pop es)
        _ASM(pop ds)
        _ASM(pop fs)
        _ASM(pop gs)
        _ASM2(add sp, 10) //skip cs:ip, ss:sp, bp
        _ASM(iret)
    _ASM_END
}
#pragma option -k

#ifdef __cplusplus
}
#endif

#endif//_DPMI_I21_H_