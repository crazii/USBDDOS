#include <memory.h>
#include <stdio.h>
#include <dos.h>
#include <conio.h>
#include <string.h>
#include <assert.h>
#include "USBDDOS/HCD/EHCI.H"
#include "USBDDOS/DPMI/DPMI.H"
#include "USBDDOS/DPMI/XMS.H"
#include "USBDDOS/DBGUTIL.H"

static uint16_t EHCI_GetPortStatus(HCD_Interface* pHCI, uint8_t port);
static BOOL EHCI_SetPortStatus(HCD_Interface* pHCI, uint8_t port, uint16_t status);
static BOOL EHCI_InitDevice(HCD_Device* pDevice);
static BOOL EHCI_RemoveDevice(HCD_Device* pDevice);
static void* EHCI_CreateEndpoint(HCD_Device* pDevice, uint8_t EPAddr, HCD_TxDir dir, uint8_t bTransferType, uint16_t MaxPacketSize, uint8_t bInterval);
static BOOL EHCI_RemoveEndpoint(HCD_Device* pDevice, void* pEndpoint);

HCD_Method EHCI_AccessMethod =
{
    &EHCI_ControlTransfer,
    &EHCI_IsoTransfer,
    &EHCI_DataTransfer,
    &EHCI_DataTransfer,
    &EHCI_GetPortStatus,
    &EHCI_SetPortStatus,
    &EHCI_InitDevice,
    &EHCI_RemoveDevice,
    &EHCI_CreateEndpoint,
    &EHCI_RemoveEndpoint,
};

static void EHCI_ResetHC(HCD_Interface* pHCI);
static void EHCI_RunStop(HCD_Interface* pHCI, BOOL Run);

BOOL EHCI_InitController(HCD_Interface * pHCI, PCI_DEVICE* pPCIDev)
{
    //configure PCI
    {
        PCI_CMD cmd;
        cmd.reg16 = pPCIDev->Header.Command;
        cmd.bits.BusMaster = 1;
        cmd.bits.IOSpace = 0;
        cmd.bits.MemorySpace = 1;
        cmd.bits.InterruptDisable = 0;
        PCI_WriteWord(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, pHCI->PCIAddr.Function, PCI_REGISTER_CMD, cmd.reg16);
    }

    pHCI->dwPhysicalAddress = (*(uint32_t*)&pPCIDev->Offset[USBBASE]) & 0xFFFFFFE0L;
    pHCI->dwBaseAddress = DPMI_MapMemory(pHCI->dwPhysicalAddress, 4096);
    _LOG("EHCI USBBASE: %04x %04x\n", pHCI->dwPhysicalAddress, pHCI->dwBaseAddress);
    pHCI->pHCDMethod = &EHCI_AccessMethod;

    const EHCI_CAPS* caps = (EHCI_CAPS*)pHCI->dwBaseAddress;
    uint32_t OperationalBase = pHCI->dwBaseAddress + caps->Size;
    pHCI->bNumPorts = caps->N_PORTS;

    //take ownership & disable SMI
    if(caps->EECP)
    {
        uint32_t legsup = READ_USBLEGSUP(pPCIDev, caps);
        //uint32_t legctlsts = READ_USBLEGCTLSTS(pPCIDev, caps);
        if((legsup&CAPID) == CAPID_LEGSUP)
        {
            const int TRYC = 500;
            int tryc = 0;
            while((!(legsup&HC_OS_Owned_Semaphore) || (legsup&HC_BIOS_Owned_Semaphore)) && tryc++ < TRYC)
            {
                legsup |= HC_OS_Owned_Semaphore;
                legsup &= ~HC_BIOS_Owned_Semaphore;
                //PCI_WriteDWord(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, pHCI->PCIAddr.Function, USBLEGSUP(caps), legsup);
                WRITE_USBLEGSUP(pHCI->PCIAddr, caps, legsup);
                delay(1);
                legsup = PCI_ReadDWord(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, pHCI->PCIAddr.Function, USBLEGSUP(caps));
            }
            if(!(legsup&HC_OS_Owned_Semaphore) || (legsup&HC_BIOS_Owned_Semaphore))
            {
                printf("EHCI: Unable to take ownership from BIOS.\n");
                return FALSE;
            }
            WRITE_USBLEGCTLSTS(pHCI->PCIAddr, caps, 0); //disable all SMIs
        }
    }

    DPMI_StoreD(OperationalBase+CTRLDSSEGMENT, 0);

    //setup frame list
    uint16_t handle = 0;
    uint32_t FrameList = DPMI_LoadD(OperationalBase + PERIODICLISTBASE) & 0xFFFFF000L;
    //if(FrameList == 0) //if not 0, probably previous inited by BIOS
    { // use xms directly instead of DPMI_DMAMalloc to save memory for driver, especially BC (64K data only).
        handle = XMS_Alloc(4, &FrameList);
        if(handle == 0 || FrameList == 0)
            return FALSE;
        if((FrameList&0xFFF) != 0)
        {
            if(!XMS_Realloc(handle, 8, &FrameList)) // 4k data + 4k alignement
            {
                XMS_Free(handle);
                return FALSE;
            }
        }
        FrameList = align(FrameList, 4096);
        DPMI_StoreD(OperationalBase + PERIODICLISTBASE, FrameList);
    }

    FrameList = FrameList < 0x100000L ? FrameList : DPMI_MapMemory(FrameList, 4096);
    _LOG("EHCI frame list base address: %08lx\n", FrameList);
    //TODO: 1: build frame list entries
    pHCI->pHCDData = DPMI_DMAMalloc(sizeof(EHCI_HCData), EHCI_ALIGN);
    EHCI_HCData* pHCData = (EHCI_HCData*)pHCI->pHCDData;
    memset(pHCData, 0, sizeof(EHCI_HCData));

    //setup async list
    DPMI_StoreD(OperationalBase+ASYNCLISTADDR, 0); //TODO: 2:

    //TODO: 3:
    pHCData->Caps = caps;
    pHCData->OPRegBase = OperationalBase;
    pHCData->dwPeroidicListBase = FrameList;
    pHCData->wPeroidicListHandle = handle;

    //setup cmd
    EHCI_USBCMD cmd = {0};
    cmd.IntThreshold = 0x8;
    cmd.AsyncScheduleParkEN = 0;
    cmd.AsyncScheduleParkCNT = 0;
    cmd.LightHCRESET = 0;
    cmd.INTonAsyncAdv = 0;
    cmd.AynscScheduleEN = 1; //enable async process
    cmd.PeroidicScheduleEN = 1; //enable peroidic process
    cmd.FLSize = FLS_4096;
    cmd.HCReset = 0;
    cmd.RunStop = 0;
    DPMI_StoreD(OperationalBase+USBCMD, cmd.Val);
    EHCI_ResetHC(pHCI);
    DPMI_StoreD(OperationalBase+FRINDEX, 0);

    //clear all sts
    DPMI_StoreD(OperationalBase+USBSTS, ~0);

    //enable interrups
    DPMI_StoreD(OperationalBase+USBINTR, (/*INTonAsyncAdvanceEN|*/INTHostSysErrorEN|INTPortChangeEN|USBERRINTEN|USBINTEN));

    DPMI_StoreD(OperationalBase+CONFIGFLAG, ConfigureFlag); //CF: last action
    EHCI_RunStop(pHCI, TRUE);
    return TRUE;
}

BOOL EHCI_DeinitController(HCD_Interface* pHCI)
{
    #error not implemented.
    return FALSE;
}

BOOL EHCI_ISR(HCD_Interface* pHCI)
{
    #error not implemented.
    return FALSE;
}

uint8_t EHCI_ControlTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t inputp setup8[8],
    void* nullable pSetupData, uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData)
{
    #error not implemented.
    return 0;
}

uint8_t EHCI_IsoTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t* inoutp pBuffer,
    uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData)
{
    #error not implemented.
    return 0;
}

uint8_t EHCI_DataTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t* inoutp pBuffer,
    uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData)
{
    #error not implemented.
    return 0;
}

uint16_t EHCI_GetPortStatus(HCD_Interface* pHCI, uint8_t port)
{
    #error not implemented.
    return 0;
}

BOOL EHCI_SetPortStatus(HCD_Interface* pHCI, uint8_t port, uint16_t status)
{
    #error not implemented.
    return FALSE;
}

BOOL EHCI_InitDevice(HCD_Device* pDevice)
{
    #error not implemented.
    return FALSE;
}

BOOL EHCI_RemoveDevice(HCD_Device* pDevice)
{
    #error not implemented.
    return FALSE;
}

void* EHCI_CreateEndpoint(HCD_Device* pDevice, uint8_t EPAddr, HCD_TxDir dir, uint8_t bTransferType, uint16_t MaxPacketSize, uint8_t bInterval)
{
    #error not implemented.
    return NULL;
}

BOOL EHCI_RemoveEndpoint(HCD_Device* pDevice, void* pEndpoint)
{
    #error not implemented.
    return FALSE;
}

void EHCI_ResetHC(HCD_Interface* pHCI)
{
    EHCI_HCData* pHCData = (EHCI_HCData*)pHCI->pHCDData;
    DPMI_StoreD(pHCData->OPRegBase+USBCMD, DPMI_LoadD(pHCData->OPRegBase+USBCMD) | HCRESET);
    delay(50);
    DPMI_StoreD(pHCData->OPRegBase+USBCMD, DPMI_LoadD(pHCData->OPRegBase+USBCMD) & ~HCRESET & ~RS);
}

void EHCI_RunStop(HCD_Interface* pHCI, BOOL Run)
{
    EHCI_HCData* pHCData = (EHCI_HCData*)pHCI->pHCDData;
    DPMI_StoreD(pHCData->OPRegBase+USBCMD, DPMI_LoadD(pHCData->OPRegBase+USBCMD) | (Run ? RS : 0));
}
