
#include <stdio.h>
#include <stdlib.h>
#include <dos.h>
#include <assert.h>
#include "USBDDOS/USB.H"
#include "USBDDOS/CLASS/MSC.H"
#include "RetroWav/RetroWav.h"
#include "RetroWav/Platform/DOS_CDC.h"
#include "RetroWav/Board/OPL3.h"
#include "EMM.h"
#include "HDPMIPT.H"
#include "USBDDOS/DBGUTIL.h"

#define OPL_PRIMARY 0
#define OPL_SECONDARY 1
#if defined(__BC__)
#define MAIN_ENABLE_TIMER 0 //BC build uses VCPI. it has compatibility issues with Miles Sound (DOS/4GW Extender protected mode, no keyboard input)
#else
#define MAIN_ENABLE_TIMER 1
#endif

RetroWaveContext MAIN_RWContext = {0};
static uint32_t MAIN_OPLTimerCtrlReg[2]; //if start 1 and 2 seperately we will miss one, so use 2 cache
static uint8_t MAIN_OPLIndexReg[2];

//primary index read
#define OPL_TIMER_REG_INDEX 4
#define OPL_TIMER1_MASK 0xC0
#define OPL_TIMER2_MASK 0xA0
#define OPL_TIMER1_START 0x01
#define OPL_TIMER2_START 0x02
#define OPL_TIMER1_TIMEOUT OPL_TIMER1_MASK
#define OPL_TIMER2_TIMEOUT OPL_TIMER2_MASK

//secondary index read (Adlib Gold). reference: AIL2.0 source code, dosbox
#define ADLG_IOBUSY 0x40UL
#define ADLG_VOLL_REG_INDEX 9
#define ADLG_VOLR_REG_INDEX 10

//data
#define KEY_ON 0x10 //channel on bit
static uint32_t ADLG_CtrlEnable = 0;    //seems not working for Miles Sound, don't use it
static uint32_t ADLG_Volume[2] = {0x08,0x08};
static uint32_t MAIN_OPL_WriteCount = 0;
static uint32_t MAIN_OPL_DelayCount = 0;

#define MAIN_OPL_MAX_INSTANT_WRITE (71/4) //USB1.1 maxium bulk:71(*8bytes), 19(*64 bytes). TODO: read EP config

#if MAIN_ENABLE_TIMER
DPMI_ISR_HANDLE MAIN_INT08Handle;
DPMI_REG MAIN_RealModeINT08 = {0};
static void MAIN_Timer_Interrupt(void) //may need a higher flush frequency to solve some missing notes and timing issue.
{ //clients (game) audio usually have high freqeuncy but invoke the chain as BIOS frequency (18.2Hz)
    MAIN_OPL_WriteCount = 0;
    MAIN_OPL_DelayCount = 0;
    retrowave_flush(&MAIN_RWContext);
    DPMI_CallRealModeIRET(&MAIN_RealModeINT08);
}
#endif

static uint32_t MAIN_OPL_Primary_Index(uint32_t port, uint32_t reg, uint32_t out)
{
    unused(port);
    if(out)
        MAIN_OPLIndexReg[OPL_PRIMARY] = (uint8_t)reg;
    else
    { //in status reg
        reg = reg & ~0xFFUL;
        if ((MAIN_OPLTimerCtrlReg[0] & (OPL_TIMER1_MASK|OPL_TIMER1_START)) == OPL_TIMER1_START)
            reg |= OPL_TIMER1_TIMEOUT;
        if ((MAIN_OPLTimerCtrlReg[1] & (OPL_TIMER2_MASK|OPL_TIMER2_START)) == OPL_TIMER2_START)
            reg |= OPL_TIMER2_TIMEOUT;
        inp((uint16_t)port); //instant delay
#if !MAIN_ENABLE_TIMER
        if(MAIN_OPL_WriteCount >= MAIN_OPL_MAX_INSTANT_WRITE)
        {
            if(++MAIN_OPL_DelayCount == 100)
            {
                //retrowave_opl3_queue_delay(&MAIN_RWContext); //buffer delay
                retrowave_flush(&MAIN_RWContext);
                ++MAIN_OPL_WriteCount;
                MAIN_OPL_DelayCount = 0;
            }
        }
#endif
    }
    return reg;
}

static uint32_t MAIN_OPL_Primary_Data(uint32_t port, uint32_t val, uint32_t out)
{
    unused(port);
    if(out)
    {
        if(MAIN_OPLIndexReg[OPL_PRIMARY] == OPL_TIMER_REG_INDEX)
        {
            if(val&(OPL_TIMER1_START|OPL_TIMER1_MASK))
                MAIN_OPLTimerCtrlReg[0] = val;
            if(val&(OPL_TIMER2_START|OPL_TIMER2_MASK))
                MAIN_OPLTimerCtrlReg[1] = val;
        }
        if(MAIN_OPL_WriteCount < MAIN_OPL_MAX_INSTANT_WRITE)
            retrowave_opl3_emit_port0(&MAIN_RWContext, MAIN_OPLIndexReg[OPL_PRIMARY], (uint8_t)val);
        else
            retrowave_opl3_queue_port0(&MAIN_RWContext, MAIN_OPLIndexReg[OPL_PRIMARY], (uint8_t)val);
        ++MAIN_OPL_WriteCount;
        return val;
    }
    return MAIN_OPL_Primary_Index(port, val, out);
}

static uint32_t MAIN_OPL_Secondary_Index(uint32_t port, uint32_t reg, uint32_t out)
{
    unused(port);
    if(out)
    {
        if(reg == 0xFF)
            ADLG_CtrlEnable = TRUE;
        else if(reg == 0xFE)
            ADLG_CtrlEnable = FALSE;

        MAIN_OPLIndexReg[OPL_SECONDARY] = (uint8_t)reg;
        return reg;
    }
    return ADLG_CtrlEnable ? ~ADLG_IOBUSY : MAIN_OPL_Primary_Index(port, reg, out);
}    

static uint32_t MAIN_OPL_Secondary_Data(uint32_t port, uint32_t val, uint32_t out)
{
    unused(port);
    if(out)
    {
        if(/*ADLG_CtrlEnable && */(MAIN_OPLIndexReg[OPL_SECONDARY] == ADLG_VOLL_REG_INDEX || MAIN_OPLIndexReg[OPL_SECONDARY] == ADLG_VOLR_REG_INDEX))
            ADLG_Volume[MAIN_OPLIndexReg[OPL_SECONDARY]-ADLG_VOLL_REG_INDEX] = val;
        if(MAIN_OPL_WriteCount < MAIN_OPL_MAX_INSTANT_WRITE)
            retrowave_opl3_emit_port1(&MAIN_RWContext, MAIN_OPLIndexReg[OPL_SECONDARY], (uint8_t)val);
        else
            retrowave_opl3_queue_port1(&MAIN_RWContext, MAIN_OPLIndexReg[OPL_SECONDARY], (uint8_t)val);
        ++MAIN_OPL_WriteCount;
        return val;
    }
    //in
    //if(ADLG_CtrlEnable) //adlib gold
    {
        if(MAIN_OPLIndexReg[OPL_SECONDARY] == ADLG_VOLL_REG_INDEX || MAIN_OPLIndexReg[OPL_SECONDARY] == ADLG_VOLR_REG_INDEX)
            return ADLG_Volume[MAIN_OPLIndexReg[OPL_SECONDARY]-ADLG_VOLL_REG_INDEX];
    }
    return MAIN_OPL_Primary_Index(port, val, out);
}

static EMM_IODT MAIN_IODT[4] =
{
    0x388, &MAIN_OPL_Primary_Index,
    0x389, &MAIN_OPL_Primary_Data,
    0x38A, &MAIN_OPL_Secondary_Index,
    0x38B, &MAIN_OPL_Secondary_Data,
};
static EMM_IOPT MAIN_IOPT;
#if defined(__DJ2__)
static EMM_IOPT MAIN_HDPMI_IOPT;
BOOL MAIN_HDPMI_PortTrapped;
#endif

int main(int argc, char* argv[])
{
    BOOL EnableRW = FALSE;
    if(argc != 1)
    {
        for(int i = 1; i < argc; ++i)
        {
            if(stricmp(argv[i], "/RW") == 0)
            {
                EnableRW = TRUE;
                break;
            }
        }
    }

    if(EnableRW)
    {
        unsigned short EMMVer = EMM_GetVersion();
        _LOG("EMM386 version: %d.%02d\n", (EMMVer&0xFF), (EMMVer>>8));
        if((EMMVer&0xFF) < 4 || (EMMVer&0xFF) == 4 && (EMMVer&0xFF00) < 46)
        {
            printf("EMM386 not installed or version not supported: %d.%02d\n", (EMMVer&0xFF), (EMMVer>>8));
            return -1;
        }
    }

    DPMI_Init();
    USB_Init();

    if(EnableRW)
    {
        if(retrowave_init_dos_cdc(&MAIN_RWContext) != 0)
            return -1;
        retrowave_io_init(&MAIN_RWContext);
        retrowave_opl3_reset(&MAIN_RWContext);

        #if MAIN_ENABLE_TIMER
        if( DPMI_InstallISR(0x08, &MAIN_Timer_Interrupt, &MAIN_INT08Handle) != 0)
        {
            puts("Failed to install interrupt handler 0x08.\n");
            return 1;
        }
        MAIN_RealModeINT08.w.cs = MAIN_INT08Handle.rm_cs;
        MAIN_RealModeINT08.w.ip = MAIN_INT08Handle.rm_offset;
        #endif

        if(!EMM_Install_IOPortTrap(0x388, 0x38B, MAIN_IODT, sizeof(MAIN_IODT)/sizeof(EMM_IODT), &MAIN_IOPT))
        {
            puts("IO trap installation failed.\n");
            return 1;
        }

        #if defined(__DJ2__)
        if(!(MAIN_HDPMI_PortTrapped=HDPMIPT_Install_IOPortTrap(0x388, 0x38B, MAIN_IODT, sizeof(MAIN_IODT)/sizeof(EMM_IODT), &MAIN_HDPMI_IOPT)))
            puts("Protected mode IO trap installation failed, make sure an HDPMI that supporting port trap is used.\n");
        #endif
    }

    if(USB_MSC_DOS_Install())
    {
        if(!DPMI_TSR())
            puts("TSR Installation failed.\n");
    }
    
    if(EnableRW)
    {
        BOOL uninstalled;
        #if MAIN_ENABLE_TIMER
        uninstalled = DPMI_UninstallISR(&MAIN_INT08Handle) == 0;
        assert(uninstalled);
        #endif

        uninstalled = EMM_Uninstall_IOPortTrap(&MAIN_IOPT);
        assert(uninstalled);

        #if defined(__DJ2__)
        if(MAIN_HDPMI_PortTrapped)
        {
            uninstalled = HDPMIPT_Uninstall_IOPortTrap(&MAIN_IOPT);
            assert(uninstalled);
        }
        #endif
    }
    return 0;
}
