#include <stdio.h>
#include <stdlib.h>
#include <dos.h>
#include <assert.h>
#include <string.h>
#include "USBDDOS/DPMI/dpmi.h"
#include "USBDDOS/usb.h"
#include "USBDDOS/CLASS/msc.h"
#include "USBDDOS/CLASS/hid.h"
#include "USBDDOS/dbgutil.h"

#define USBDDOS_VERSION 0x0200 //BCD
#ifndef USBDDOS_BUILD
#define USBDDOS_BUILD "Local build"
#endif

#if defined(__BC__) || defined(__WC__)
#define ENABLE_RETROWAVE 0 //code exceed 64K
#else
#define ENABLE_RETROWAVE 1
#endif

#if ENABLE_RETROWAVE
#include "RetroWav/retrowav.h"
#include "RetroWav/Platform/dos_cdc.h"
#include "RetroWav/Board/opl3.h"
#include "emm.h"
#include "hdpmipt.h"

#define OPL_PRIMARY 0
#define OPL_SECONDARY 1

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

        retrowave_opl3_emit_port0(&MAIN_RWContext, MAIN_OPLIndexReg[OPL_PRIMARY], (uint8_t)val);
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

        retrowave_opl3_emit_port1(&MAIN_RWContext, MAIN_OPLIndexReg[OPL_SECONDARY], (uint8_t)val);
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
#endif

struct 
{
    const char* option;
    const char* desc;
    BOOL enable;
}MAIN_Options[] =
{
    "/?", "Show help.", FALSE,
#if ENABLE_RETROWAVE
    "/RW", "Enable RetroWave OPL3 supprt. EMM386 v4.46+ / HDPMI32i needed.", FALSE,
#endif    
    "/disk", "Enable USB-disk support", FALSE,
    "/hid", "Enable USB keyboard & mouse support", FALSE,

    #if DEBUG
    "/test", "debug tests", FALSE,
    #endif
    NULL, NULL, 0,
};
enum EOption
{
    OPT_Help,
#if ENABLE_RETROWAVE
    OPT_RetroWave,
#endif    
    OPT_Disk,
    OPT_HID,

    #if DEBUG
    OPT_TEST,
    #endif
    OPT_COUNT,
};

int main(int argc, char* argv[])
{
    if(argc == 1 || (argc == 2 && stricmp(argv[1],"/?") == 0))
    {
        printf("USBDDOS: USB driver for DOS. https://github.com/crazii/USBDDOS\n");
        printf("Version: %d.%02d, Build: %s. Usage:\n", USBDDOS_VERSION>>8, USBDDOS_VERSION&0xFF, USBDDOS_BUILD);
        int i = 0;
        while(MAIN_Options[i].option)
        {
            printf(" %-8s: %s\n", MAIN_Options[i].option, MAIN_Options[i].desc);
            ++i;
        }
        return 0;
    }
    else
    {
        for(int i = 1; i < argc; ++i)
        {
            for(int j = 0; j < OPT_COUNT; ++j)
            {
                if(stricmp(argv[i], MAIN_Options[j].option) == 0)
                {
                    MAIN_Options[j].enable = TRUE;
                    break;
                }
            }
        }
    }

#if DEBUG && 0
    if(MAIN_Options[OPT_TEST].enable)
    {
        extern void DPMI_InitFlat();
        extern void USB_MSC_Test();
        //DPMI_InitFlat();
        //USB_MSC_Test();
        return 0;
    }
#endif


    DPMI_Init();


#if ENABLE_RETROWAVE
    if(MAIN_Options[OPT_RetroWave].enable)
    {
        unsigned short EMMVer = EMM_GetVersion();
        _LOG("EMM386 version: %d.%02d\n", (EMMVer&0xFF), (EMMVer>>8));
        if((EMMVer&0xFF) < 4 || (EMMVer&0xFF) == 4 && (EMMVer&0xFF00) < 46)
        {
            printf("EMM386 not installed or version not supported: %d.%02d\n", (EMMVer&0xFF), (EMMVer>>8));
            return -1;
        }
    }
#endif


    USB_Init();


#if ENABLE_RETROWAVE
    if(MAIN_Options[OPT_RetroWave].enable)
    {
        if(retrowave_init_dos_cdc(&MAIN_RWContext) != 0)
            return -1;
        retrowave_io_init(&MAIN_RWContext);
        retrowave_opl3_reset(&MAIN_RWContext);

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
    BOOL TSR = MAIN_Options[OPT_RetroWave].enable;
#else
    BOOL TSR = FALSE;
#endif
    TSR = (MAIN_Options[OPT_Disk].enable && USB_MSC_DOS_Install()) || TSR; //note: TSR must be put in the back
    TSR = (MAIN_Options[OPT_HID].enable && USB_HID_DOS_Install()) || TSR;
#if DEBUG && 0
    TSR = MAIN_Options[OPT_TEST].enable || TSR;
#endif
    if(TSR) 
    {
        if(!DPMI_TSR())
            puts("TSR Installation failed.\n");
    }
    else
        puts("No USB device found, exit.\n");
    
    //TSR failure
#if ENABLE_RETROWAVE
    if(MAIN_Options[OPT_RetroWave].enable)
    {
        BOOL uninstalled;

        uninstalled = EMM_Uninstall_IOPortTrap(&MAIN_IOPT);
        assert(uninstalled);

        #if defined(__DJ2__)
        if(MAIN_HDPMI_PortTrapped)
        {
            uninstalled = HDPMIPT_Uninstall_IOPortTrap(&MAIN_IOPT);
            assert(uninstalled);
        }
        #endif
        unused(uninstalled);
    }
#endif
    return DPMI_Exit(0);
}
