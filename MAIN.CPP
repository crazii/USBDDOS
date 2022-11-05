#include <stdio.h>
#include <stdlib.h>
#include <dos.h>
#include <assert.h>
#include "USBDDOS/USB.H"
#include "USBDDOS/DBGUTIL.h"
#include "RetroWav/RetroWav.h"
#include "RetroWav/Platform/DOS_CDC.h"
#include "RetroWav/Board/OPL3.h"
#include "EMM.h"

#define RETROWAVE_VENDOR 0x04D8
#define RETROWAVE_DEVID 0x000A

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
#define ADLG_IOBUSY 0x40
#define ADLG_VOLL_REG_INDEX 9
#define ADLG_VOLR_REG_INDEX 10

//data
#define KEY_ON 0x10 //channel on bit
static uint32_t ADLG_CtrlEnable = 0;    //seems not working for Miles Sound, don't use it
static uint32_t ADLG_Volume[2] = {0x08,0x08};
static uint32_t MIAN_OPL_WriteCount = 0;
#define MAIN_OPL_MAX_INSTANT_WRITE (71/4) //USB1.1 maxium bulk:71(*8bytes)

DPMI_REG MAIN_RealModeINT08 = {0};
static void MAIN_Timer_Interrupt(void) //don't care the frequency, use it as the client(game)'s timer setup for its audio
{
    MIAN_OPL_WriteCount = 0;
    retrowave_flush(&MAIN_RWContext);
    DPMI_CallRealModeIRET(&MAIN_RealModeINT08);
}

static uint32_t MAIN_OPL_Primary_Index(uint32_t port, uint32_t reg, uint32_t out)
{
    unused(port);
    if(out)
        MAIN_OPLIndexReg[OPL_PRIMARY] = (uint8_t)reg;
    else
    { //in status reg
        reg = reg & ~0xFF;
        if ((MAIN_OPLTimerCtrlReg[0] & (OPL_TIMER1_MASK|OPL_TIMER1_START)) == OPL_TIMER1_START)
            reg |= OPL_TIMER1_TIMEOUT;
        if ((MAIN_OPLTimerCtrlReg[1] & (OPL_TIMER2_MASK|OPL_TIMER2_START)) == OPL_TIMER2_START)
            reg |= OPL_TIMER2_TIMEOUT;
        inp(port); //instant delay
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
        if(MIAN_OPL_WriteCount < MAIN_OPL_MAX_INSTANT_WRITE)
            retrowave_opl3_emit_port0(&MAIN_RWContext, MAIN_OPLIndexReg[OPL_PRIMARY], (uint8_t)val);
        else
            retrowave_opl3_queue_port0(&MAIN_RWContext, MAIN_OPLIndexReg[OPL_PRIMARY], (uint8_t)val);
        ++MIAN_OPL_WriteCount;
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
        if(MIAN_OPL_WriteCount < MAIN_OPL_MAX_INSTANT_WRITE)
            retrowave_opl3_emit_port1(&MAIN_RWContext, MAIN_OPLIndexReg[OPL_SECONDARY], (uint8_t)val);
        else
            retrowave_opl3_queue_port1(&MAIN_RWContext, MAIN_OPLIndexReg[OPL_SECONDARY], (uint8_t)val);
        ++MIAN_OPL_WriteCount;
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

int main()
{
    unsigned short EMMVer = EMM_GetVersion();
    _LOG("EMM386 version: %d.%02d\n", (EMMVer&0xFF), (EMMVer>>8));
    if((EMMVer&0xFF) < 4 || (EMMVer&0xFF) == 4 && (EMMVer&0xFF00) < 46)
    {
        printf("EMM386 not installed or version not supported: %d.%02d\n", (EMMVer&0xFF), (EMMVer>>8));
        return -1;
    }

    DPMI_Init();
    USB_Init();

    USB_Device* device = NULL;
    for(int j = 0; j < USBT.HC_Count; ++j)
    {
        HCD_Interface* pHCI = USBT.HC_List+j;

        for(int i = 0; i < HCD_MAX_DEVICE_COUNT; ++i)
        {
            if(!HCD_IS_DEVICE_VALID(pHCI->DeviceList[i]))
                continue;
            USB_Device* dev = HC2USB(pHCI->DeviceList[i]);
            if(dev->Desc.widVendor == RETROWAVE_VENDOR)
            {
                device = dev;
                break;
            }
            //USB_ShowDeviceInfo(dev);
        }
    }

    if(device != NULL)
        printf("Vendor: %s, Name: %s\n", device->sManufacture, device->sProduct);
    else
    {
        printf("Retro wave device not found.\n");
        return 1;
    }

    retrowave_init_dos_cdc(&MAIN_RWContext, device);
    retrowave_io_init(&MAIN_RWContext);
    retrowave_opl3_reset(&MAIN_RWContext);

    printf("Installing...\n");
    if( DPMI_InstallISR(0x08, &MAIN_Timer_Interrupt, &MAIN_RealModeINT08.w.cs, &MAIN_RealModeINT08.w.ip) != 0)
    {
        printf("Failed to insteall interrupt handler.\n");
        return 1;
    }

    if(!EMM_IOPortTrap(0x388, 0x38B, MAIN_IODT, sizeof(MAIN_IODT)/sizeof(EMM_IODT)))
    {
        printf("IO trap installation failed.\n");
        return 1;
    }

    if(!DPMI_TSR())
    {
        printf("TSR Installation failed.\n");
        return 1;
    }

    return 0;
}
