#include <memory.h>
#include <stdio.h>
#include <dos.h>
#include <conio.h>
#include <string.h>
#include <assert.h>
#define _EHCI_IMPL
#include "USBDDOS/HCD/ehci.h"
#include "USBDDOS/DPMI/dpmi.h"
#include "USBDDOS/DPMI/xms.h"
#include "USBDDOS/usballoc.h"
#include "USBDDOS/dbgutil.h"

#define EHCI_INTERRUPTS (/*INTonAsyncAdvanceEN|*/INTHostSysErrorEN|INTPortChangeEN|USBERRINTEN|USBINTEN)

#define EHCI_EP_MAKE(pQH, epaddr) (void*)(((uintptr_t)(pQH))|(epaddr))
#define EHCI_EP_GETQH(pVoid) (EHCI_QH*)(((uintptr_t)(pVoid))&~0xFUL)
#define EHCI_EP_GETADDR(pVoid) (((uintptr_t)(pVoid))&0xFUL)

static uint8_t EHCI_HighSpeedSMask1to8[4] = //1,2,4,8
{
    0xFFU, 0x55U, 0x11U, 0x01U
};

//EHCI spec (charpter 3) require structures NOT to span across page boundary
#define DPMI_DMAMalloc DPMI_DMAMallocNCPB
#ifdef USB_TAlloc
#undef USB_TAlloc
#endif
#define USB_TAlloc DPMI_DMAMallocNCPB

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
static BOOL EHCI_SetupPeriodicList(HCD_Interface* pHCI);
static void EHCI_RunStop(HCD_Interface* pHCI, BOOL Run);
static void EHCI_InitQH(EHCI_QH* pInit, EHCI_QH* pLinkto);
static BOOL EHCI_DetachQH(EHCI_QH* pQH, EHCI_QH* pLinkHead, EHCI_QH** ppLinkEnd);
static void EHCI_BuildqTD(EHCI_qTD* pTD, EHCI_qTD* pNext, EHCI_QToken token, HCD_Request* pRequest, void* pBuffer);
static void EHCI_ISR_QH(HCD_Interface* pHCI, EHCI_QH* pHead, EHCI_QH* pTail);
static void EHCI_ISR_qTD(HCD_Interface* pHCI, EHCI_QH* pQH);


BOOL EHCI_InitController(HCD_Interface * pHCI, PCI_DEVICE* pPCIDev)
{
    //configure PCI: must enable MMIO first to read registers
    PCI_CMD pcicmd;
    pcicmd.reg16 = pPCIDev->Header.Command;
    pcicmd.bits.BusMaster = 1;
    pcicmd.bits.IOSpace = 0;
    pcicmd.bits.MemorySpace = 1;
    pcicmd.bits.InterruptDisable = 0;
    PCI_WriteWord(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, pHCI->PCIAddr.Function, PCI_REGISTER_CMD, pcicmd.reg16);
    pPCIDev->Header.Command = pcicmd.reg16;

    uint32_t usbbase = (*(uint32_t*)&pPCIDev->Offset[USBBASE]) & 0xFFFFFFE0L;
    pHCI->dwPhysicalAddress = usbbase;
    pHCI->dwBaseAddress = DPMI_MapMemory(pHCI->dwPhysicalAddress, 4096);
    _LOG("EHCI USBBASE: %08lx %08lx\n", pHCI->dwPhysicalAddress, pHCI->dwBaseAddress);
    pHCI->pHCDMethod = &EHCI_AccessMethod;

    EHCI_CAPS caps;
    DPMI_CopyLinear(DPMI_PTR2L(&caps), pHCI->dwBaseAddress, sizeof(caps));
    uint32_t OperationalBase = pHCI->dwBaseAddress + caps.Size;
    pHCI->bNumPorts = caps.hcsParamsBm.N_PORTS;
    _LOG("EHCI version: %x.%02x, PPC: %x, EECP: %x, N_CC: %d\n", caps.hciVersion>>8, caps.hciVersion&0xFF, caps.hcsParamsBm.PPC, caps.hccParamsBm.EECP,caps.hcsParamsBm.N_CC);

    //first things to do is to take ownership from BIOS and reset HC.
    //take ownership & disable SMI
    if(caps.hciVersion > 0x95 && caps.hccParamsBm.EECP)
    {
        EHCI_LEGSUP legsup; legsup.Val = READ_USBLEGSUP2(pPCIDev, caps);
        //uint32_t legctlsts = READ_USBLEGCTLSTS2(pPCIDev, caps);
        if(legsup.Bm.CapabilityID == CAPID_LEGSUP)
        {
            _LOG("EHCI: take ownership from BIOS...\n");
            const int TRYC = 50;
            int tryc = 0;
            while((!legsup.Bm.OS_Owned || legsup.Bm.BIOS_Owned) && tryc++ < TRYC)
            {
                legsup.Bm.OS_Owned = 1;
                WRITE_USBLEGSUP(pHCI->PCIAddr, caps, legsup);
                delay(10);
                legsup.Val = READ_USBLEGSUP(pHCI->PCIAddr, caps);
            }
            if(!legsup.Bm.OS_Owned || legsup.Bm.BIOS_Owned)
            {
                printf("EHCI: Unable to take ownership from BIOS.\n");
                DPMI_UnmapMemory(pHCI->dwBaseAddress);
                return FALSE;
            }
            WRITE_USBLEGCTLSTS(pHCI->PCIAddr, caps, 0); //disable all SMIs
        }
    }

    pHCI->pHCDData = DPMI_DMAMalloc(sizeof(EHCI_HCData), EHCI_ALIGN);
    EHCI_HCData* pHCData = (EHCI_HCData*)pHCI->pHCDData;
    memset(pHCData, 0, sizeof(EHCI_HCData));

    //reset
    pHCData->Caps = caps;
    pHCData->OPBase = OperationalBase;
    EHCI_ResetHC(pHCI);
    _LOG("EHCI operational base: %08lx\n", OperationalBase);

    //setup frame list
    if(!EHCI_SetupPeriodicList(pHCI))
    {
        pHCI->pHCDData = NULL;
        DPMI_DMAFree(pHCData);
        return FALSE;
    }

    //setup async list
    pHCData->ControlTail = &pHCData->ControlQH;
    pHCData->BulkTail = &pHCData->BulkQH;
    pHCData->ControlQH.HorizLink.Ptr = DPMI_PTR2P(&pHCData->BulkQH) | (Typ_QH<<Typ_Shift);
    #if 0
    pHCData->BulkQH.HorizLink.Bm.T = 1;
    #else //only circle queue + 'H' bit (EHCI spec 4.8.3) works on tested hardware (EHCI rev 0.95), although the above works well in VirtualBox. this saves mem bandwidth anyways
    pHCData->BulkQH.HorizLink.Ptr = DPMI_PTR2P(&pHCData->ControlQH) | (Typ_QH<<Typ_Shift);
    pHCData->ControlQH.Caps.HeadofReclamation = 1;
    #endif
    pHCData->ControlQH.Token.StatusBm.Halted = pHCData->BulkQH.Token.StatusBm.Halted = 1; //dummy heads
    DPMI_StoreD(OperationalBase+ASYNCLISTADDR, DPMI_PTR2P(&pHCData->ControlQH));

    //setup cmd
    EHCI_USBCMD cmd = {0};
    cmd.Bm.IntThreshold = 0x8;
    cmd.Bm.AynscScheduleEN = 1; //enable async process
    cmd.Bm.PeroidicScheduleEN = 1; //enable peroidic process
    DPMI_StoreD(OperationalBase+USBCMD, cmd.Val);

    DPMI_StoreD(OperationalBase+USBSTS, ~0UL); //clear all sts
    DPMI_StoreD(OperationalBase+USBINTR, EHCI_INTERRUPTS); //enable interrups
    DPMI_StoreD(OperationalBase+CONFIGFLAG, ConfigureFlag); //CF: last action
    EHCI_RunStop(pHCI, TRUE);

    //power up ports.
    if(caps.hcsParamsBm.PPC)
    {
        for(uint8_t i = 0; i < caps.hcsParamsBm.N_PORTS; ++i)
            DPMI_StoreD(OperationalBase+PORTSC+i*4U, PortPower);
    }
    delay(50);

    //pre-detect low/full speed devices, release to companion HC. MUST do it after CONFIGFLAG to get ownership of all ports
    if(caps.hcsParamsBm.N_CC > 0) //we got companion
    {
        uint8_t i;
        for(i = 0; i < caps.hcsParamsBm.N_PORTS; ++i)
            DPMI_StoreD(OperationalBase+PORTSC+i*4U, PortPower|PortReset);
        delay(55);
        for(i = 0; i < caps.hcsParamsBm.N_PORTS; ++i)
        {
            uint32_t pa = OperationalBase+PORTSC+i*4U;
            DPMI_StoreD(pa, PortEnable|PortPower); //PortEnable won't enable port but avoid disabling it
            delay(22); //spec require 2ms to enable ports with high speed devices after reset
            uint32_t status = DPMI_LoadD(pa);
            if((status&ConnectStatus) && !(status&PortEnable)) //not enabled: low/full speed
                DPMI_StoreD(pa, PortOwner); //handoff to companion HC
        }
    }
    //delay(50); //UBSTS not updated immediately
    _LOG("USBCMD: %08lx USBSTS: %08lx\n", DPMI_LoadD(OperationalBase+USBCMD), DPMI_LoadD(OperationalBase+USBSTS));
    return TRUE;
}

BOOL EHCI_DeinitController(HCD_Interface* pHCI)
{
    if(pHCI->pHCDData)
    {
        EHCI_HCData* pHCData = (EHCI_HCData*)pHCI->pHCDData;
        EHCI_RunStop(pHCI, FALSE);
        EHCI_ResetHC(pHCI);

        DPMI_StoreD(pHCData->OPBase + PERIODICLISTBASE, 0);
        DPMI_StoreD(pHCData->OPBase + ASYNCLISTADDR, 0);
        DPMI_StoreD(pHCData->OPBase + CONFIGFLAG, ~ConfigureFlag);

        if(pHCData->wPeroidicListHandle)
        {
            XMS_Free(pHCData->wPeroidicListHandle);
            if(pHCData->dwPeroidicListBase >= 0x100000L)
                DPMI_UnmapMemory(pHCData->dwPeroidicListBase);
        }
        DPMI_DMAFree(pHCI->pHCDData);
    }
    pHCI->pHCDData = NULL;
    pHCI->dwBaseAddress = 0;
    return TRUE;
}

BOOL EHCI_ISR(HCD_Interface* pHCI)
{
    EHCI_HCData* pHCData = (EHCI_HCData*)pHCI->pHCDData;
    uint32_t sts = DPMI_LoadD(pHCData->OPBase+USBSTS);
    if(!(sts&USBINTMASK))
        return FALSE;
    //_LOG("EHCI ISR %lx %lx  ", sts, DPMI_LoadD(pHCData->OPBase+USBCMD));
    DPMI_StoreD(pHCData->OPBase+USBINTR, 0); //disable interrupts

    if(sts&HostSysError)
    {
        DPMI_StoreD(pHCData->OPBase+USBSTS, HostSysError);//ACK & clear
        sts &= ~HostSysError;
        //_LOG("EHCI PCI sts: %x", PCI_ReadWord(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, pHCI->PCIAddr.Function, PCI_REGISTER_STS));
        //TODO: need a HCRest
        EHCI_RunStop(pHCI, TRUE);
    }

    //process QH & qTDs
    EHCI_ISR_QH(pHCI, &pHCData->ControlQH, pHCData->ControlTail);
    EHCI_ISR_QH(pHCI, &pHCData->BulkQH, pHCData->BulkTail);
    for(int i = 0 ; i < EHCI_INTR_COUNT; ++i)
        EHCI_ISR_QH(pHCI, &pHCData->InterruptQH[i], pHCData->InterruptTail[i]);

    //process iTDs & siTDs TODO:

    DPMI_StoreD(pHCData->OPBase+USBSTS, sts); //ACK all interrupts
    DPMI_StoreD(pHCData->OPBase+USBINTR, EHCI_INTERRUPTS); //enable interrupts
    return TRUE;
}

uint8_t EHCI_ControlTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t inputp setup8[8],
    void* nullable pSetupData, uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData)
{
   EHCI_QH* pQH = EHCI_EP_GETQH(pEndpoint);
    if(pCB == NULL || pQH == NULL || pQH->EXT.TransferType != USB_ENDPOINT_TRANSFER_TYPE_CTRL)
        return 0xFF;

    uint8_t DataPID = (dir == HCD_TXW) ? PID_OUT : PID_IN;
    uint8_t StatusPID = (dir == HCD_TXW) ? PID_IN : PID_OUT; //usb1.1 spec, status inverts direction of data
    EHCI_QToken token = {0};
    token.StatusBm.Active = 1;
    token.CERR = 3;
    token.C_Page = 0;
    //uint16_t MaxLength = (uint16_t)(EHCI_qTD_MaxSize - (length ? (uint32_t)(pSetupData) - alignup(pSetupData, 4096) : 0)); //pQH->Caps.MaxPacketSize;
    uint16_t MaxLength = EHCI_qTD_MaxSize - 4096; //not working when using maximum buffer (0...5 when 5th page finishes at page boundary), why?
    HCD_Request* pRequest = HCD_AddRequest(pDevice, pEndpoint, dir, pSetupData, length, EHCI_EP_GETADDR(pEndpoint), pCB, pCBData);

    EHCI_qTD* pEnd = (EHCI_qTD*)USB_TAlloc(sizeof(EHCI_qTD), EHCI_ALIGN); //dummy end (tail)
    memset(pEnd, 0, sizeof(EHCI_qTD));

    // build status TD
    EHCI_qTD* pStatusTD = (EHCI_qTD*)USB_TAlloc(sizeof(EHCI_qTD), EHCI_ALIGN);
    memset(pStatusTD, 0, sizeof(EHCI_qTD));
    EHCI_QToken st = token;
    st.IOC = 1;
    st.DataToggle = 1; //status data toggle always 1
    st.PID = StatusPID&0x3U;
    EHCI_BuildqTD(pStatusTD, pEnd, st, pRequest, 0);

    // build data TD
    EHCI_qTD* pDataTD = NULL;
    if(length)
    {
        uint16_t Transferred = 0;
        uint8_t DataToggle = 1;
        assert(pSetupData);
        EHCI_qTD* pNext = (EHCI_qTD*)USB_TAlloc(sizeof(EHCI_qTD), EHCI_ALIGN);
        memset(pNext, 0, sizeof(EHCI_qTD));
        pDataTD = pNext;

        while(Transferred < length)
        {
            uint16_t DataLength = min(MaxLength, (uint16_t)(length - Transferred));
            EHCI_qTD* pTD = pNext;
            pNext = (Transferred + DataLength == length) ? pStatusTD : (EHCI_qTD*)USB_TAlloc(sizeof(EHCI_qTD), EHCI_ALIGN);
            if(pNext != pStatusTD) memset(pNext, 0, sizeof(EHCI_qTD));
            EHCI_QToken dt = token;
            dt.Length = DataLength&0x7FFFU;
            dt.DataToggle = DataToggle&0x1U;
            dt.PID = DataPID&0x3U;
            EHCI_BuildqTD(pTD, pNext, dt, pRequest, (char*)pSetupData + Transferred);
            DataToggle ^= 1;

            Transferred = (uint16_t)(Transferred + DataLength);
            //assert(Transferred == length || align(DPMI_PTR2L(pSetupData) + Transferred, 4096) == DPMI_PTR2L(pSetupData) + Transferred);
            //MaxLength = EHCI_qTD_MaxSize;
        }
    }

    // build setup TD
    CLIS();
    EHCI_qTD* pSetupTD = pQH->EXT.Tail;
    assert(pSetupTD->EXT.Prev == NULL);
    assert((pQH->qTDNext.Ptr&~0x1FUL) == DPMI_PTR2P(pSetupTD) && pQH->Token.StatusBm.Active == 0); //currently control transfers are synced (from USBD), and there's no pending transfers
    EHCI_QToken stt = token;
    stt.PID = PID_SETUP;
    stt.Length = 8;
    stt.DataToggle = 0; //setup data toggle always 0
    stt.StatusBm.Active = 0;
    EHCI_BuildqTD(pSetupTD, length ? pDataTD : pStatusTD, stt, pRequest, setup8);
    pQH->EXT.Tail = pEnd;
    // start transfer
    pQH->BufferPointers[2].Page2.SBytes = 0; //spec required
    pSetupTD->Token.StatusBm.Active = 1;
    STIL();
    uint8_t error = 0;
    return error;
}

uint8_t EHCI_IsoTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t* inoutp pBuffer,
    uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData)
{
    //since we already record it in the TODO, skip compiler warning for reminding.
    unused(pDevice);unused(pEndpoint);unused(dir);unused(pBuffer);
    unused(length);unused(pCB);unused(pCBData);

    assert(FALSE && "not impelemented"); //not implemented. TODO:
    return 0xFF;
}

uint8_t EHCI_DataTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t* inoutp pBuffer,
    uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData)
{
    EHCI_QH* pQH = EHCI_EP_GETQH(pEndpoint);
    if(pCB == NULL || pQH == NULL || pQH->Token.PID != dir || pQH->EXT.TransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL || pBuffer == NULL || length == 0)
    {
        assert(FALSE);
        return 0xFF;
    }
    assert(pQH->Caps.Endpoint == EHCI_EP_GETADDR(pEndpoint));

    //uint16_t MaxLen = (uint16_t)(EHCI_qTD_MaxSize - ((uint32_t)DPMI_PTR2L(pBuffer) - alignup(DPMI_PTR2L(pBuffer), 4096))); //pQH->Caps.MaxPacketSize;
    uint16_t MaxLen = EHCI_qTD_MaxSize - 4096; //not working when using maximum buffer (0...5 when 5th page finishes at page boundary), why?
    HCD_Request* pRequest = HCD_AddRequest(pDevice, pEndpoint, dir, pBuffer, length, EHCI_EP_GETADDR(pEndpoint), pCB, pCBData);
    //_LOG("data xfer, ep: %d, size:%u, MaxLen:%u\n", pQH->Caps.Endpoint, length, MaxLen);

    EHCI_QToken token = {0};
    token.CERR = 3;
    token.PID = (dir == HCD_TXW) ? PID_OUT : PID_IN;
    token.C_Page = 0;
    assert((pQH->qTDNext.Ptr&~0x1FUL)==DPMI_PTR2P(pQH->EXT.Tail));
    EHCI_qTD* pHead = pQH->EXT.Tail;

    uint16_t transferred = 0;
    CLIS();
    while(transferred < length)
    {
        uint16_t PacketLength = min((uint16_t)(length - transferred), MaxLen);
        EHCI_qTD* pNewTail = (EHCI_qTD*)USB_TAlloc(sizeof(EHCI_qTD), EHCI_ALIGN);
        memset(pNewTail, 0, sizeof(EHCI_qTD));
        token.IOC = (transferred + PacketLength == length) ? 1 : 0; //enable interrupt for last TD
        token.Length = PacketLength&0x7FFFU;
        token.StatusBm.Active = (transferred == 0) ? 0 : 1;
        EHCI_BuildqTD(pQH->EXT.Tail, pNewTail, token, pRequest, (char*)pBuffer + transferred);
        transferred = (uint16_t)(transferred + PacketLength);
        //assert(transferred == length || align((char*)pBuffer + transferred, 4096) == (uint32_t)pBuffer + transferred);
        //MaxLen = EHCI_qTD_MaxSize;
        pQH->EXT.Tail = pNewTail;
    }
    //start transfer
    pQH->BufferPointers[2].Page2.SBytes = 0; //spec required
    pHead->Token.StatusBm.Active = 1;
    STIL();
    uint8_t error = 0;
    return error;
}

uint16_t EHCI_GetPortStatus(HCD_Interface* pHCI, uint8_t port)
{
    assert(port < pHCI->bNumPorts);
    EHCI_HCData* pHCData = (EHCI_HCData*)pHCI->pHCDData;
    uint32_t addr = pHCData->OPBase + PORTSC + port*4U;
    uint32_t status = DPMI_LoadD(addr);
    uint16_t result = 0;
    //_LOG("EHCI port %d status: %lx\n", port, status);

    if(status & ConnectStatus) //low/full speed devices already released to companion on HC init.
        result |= USB_PORT_ATTACHED|USB_PORT_High_Speed_Device;

    if(status & PortEnable)
        result |= USB_PORT_ENABLE;
    else
        result |= USB_PORT_DISABLE;

    if(status & PortSuspend)
        result |= USB_PORT_SUSPEND;
    if(status & PortReset) //in reset
        result |= USB_PORT_RESET;
    if(status & ConnectStatusChange)
        result |= USB_PORT_CONNECT_CHANGE;
    return result;
}

BOOL EHCI_SetPortStatus(HCD_Interface* pHCI, uint8_t port, uint16_t status)
{
    if(!pHCI || (port > pHCI->bNumPorts))
        return FALSE;
    EHCI_HCData* pHCData = (EHCI_HCData*)pHCI->pHCDData;
    uint32_t addr = pHCData->OPBase + PORTSC + port*4U;
    uint32_t current = DPMI_LoadD(addr);
    uint32_t forcebits = PortPower|PortEnable;

    if((status&USB_PORT_RESET)) //reset will auto enable this port if high speed device attached
    {
        const int timeout = 100; //5sec timeout
        DPMI_StoreD(addr, PortPower|PortReset); //PortReset won't use PortEnable by spec

        delay(55); //spec require at least 10ms, 50+ms get more compatibility

        DPMI_StoreD(addr, forcebits); //release reset signal, and don't disable port

        int i = 0;
        do { delay(55); ++i; } while((DPMI_LoadD(addr)&PortReset) && i < timeout); //spec require 2ms to enable high speed ports
        if(i == timeout)
            return FALSE;
        current = DPMI_LoadD(addr); //reload enable/disable state
        _LOG("EHCI port reset: %x\n",current);
    }

    if((status&USB_PORT_ENABLE) && !(current&PortEnable))
    {
        //driver cannot directly enable ports by the spec.
        //it must be RESET to perform 'reset and enable'
        //USBD will issue a port RESET to enable the port
    }

    if((status&USB_PORT_DISABLE) && (current&PortEnable))
    {
        forcebits &= ~PortEnable;
        DPMI_StoreD(addr, forcebits);
        do { delay(5); } while((DPMI_LoadD(addr)&PortEnable));
        DPMI_StoreD(addr, forcebits|PortEnableChange); //clear states
    }

    if((status&USB_PORT_SUSPEND) && !(current&PortSuspend))
    {
        do { DPMI_StoreD(addr, forcebits|PortSuspend); delay(5); } while(!(DPMI_LoadD(addr)&PortSuspend));
    }
    else if(!(status&USB_PORT_SUSPEND) && (current&PortSuspend))
        while((DPMI_LoadD(addr)&PortSuspend)) { DPMI_StoreD(addr, forcebits); delay(5); };

    if((status&USB_PORT_CONNECT_CHANGE))
    {
        DPMI_StoreD(addr, forcebits|ConnectStatusChange); //clear connect status change
        do { delay(10); } while(DPMI_LoadD(addr)&ConnectStatusChange);
        delay(150);
    }
    return TRUE;
}

BOOL EHCI_InitDevice(HCD_Device* pDevice)
{
    pDevice->pHCData = DPMI_DMAMalloc(sizeof(EHCI_HCDeviceData), EHCI_ALIGN);
    EHCI_HCDeviceData* pDeviceData = (EHCI_HCDeviceData*)pDevice->pHCData;
    memset(pDeviceData, 0, sizeof(EHCI_HCDeviceData));
    //EHCI_HCData* pHCData = (EHCI_HCData*)pDevice->pHCI->pHCDData;
    //memset(&pDeviceData->ControlQH, 0, sizeof(pDeviceData->ControlQH));
    //EHCI_InitQH(&pDeviceData->ControlQH, pHCData->ControlTail);
    //pHCData->ControlTail = &pDeviceData->ControlQH;
    return TRUE;
}

BOOL EHCI_RemoveDevice(HCD_Device* pDevice)
{
    if(!HCD_IS_DEVICE_VALID(pDevice))
        return FALSE;
    EHCI_HCDeviceData* pDeviceData = (EHCI_HCDeviceData*)pDevice->pHCData;
    //EHCI_HCData* pHCData = (EHCI_HCData*)pDevice->pHCI->pHCDData;
    //EHCI_DetachQH(&pDeviceData->ControlQH, &pHCData->ControlQH, &pHCData->ControlTail);
    //USB_TFree(pDeviceData->ControlQH.EXT.Tail);
    DPMI_DMAFree(pDeviceData);
    return TRUE;
}

void* EHCI_CreateEndpoint(HCD_Device* pDevice, uint8_t EPAddr, HCD_TxDir dir, uint8_t bTransferType, uint16_t MaxPacketSize, uint8_t bInterval)
{
    EHCI_HCDeviceData* pDeviceData = (EHCI_HCDeviceData*)pDevice->pHCData;
    EHCI_HCData* pHCData = (EHCI_HCData*)pDevice->pHCI->pHCDData;

    if(EPAddr == 0) //default control pipe
    {
        if(pDeviceData->ControlEPCreated)
            return EHCI_EP_MAKE(&pDeviceData->ControlQH, 0);
        else
        {
            pDeviceData->ControlEPCreated = TRUE;
            _LOG("EHCI: create default control pipe, addr:%d, maxpacket:%d\n", pDevice->bAddress, MaxPacketSize);
        }
    }
    assert(EPAddr <= 0xF);
    assert(bTransferType != USB_ENDPOINT_TRANSFER_TYPE_INTR || bInterval >= 1);
    bInterval = max(bInterval, 1); //just for safety
    uint16_t EPS = (pDevice->bSpeed==USB_PORT_Low_Speed_Device) ? EPS_LOW : (pDevice->bSpeed==USB_PORT_Full_Speed_Device) ? EPS_FULL : EPS_HIGH;

    EHCI_QH* pQH = EPAddr == 0 ? &pDeviceData->ControlQH : (EHCI_QH*)DPMI_DMAMalloc(sizeof(EHCI_QH), EHCI_ALIGN);
    memset(pQH, 0, sizeof(*pQH));
    pQH->Caps.DeviceAddr = pDevice->bAddress&0x7FU;
    pQH->Caps.Endpoint = EPAddr&0xFU;
    pQH->Caps.EndpointSpd = EPS&0x3U;
    pQH->Caps.MaxPacketSize = MaxPacketSize&0x7FFU;
    pQH->Caps.DTC = (bTransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL) ? 1 : 0; //always uses data toggle from qTD
    pQH->Caps.NakCounterRL = (bTransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL || bTransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK) ? 0xF : 0;

    pQH->Caps2.Mult = 1;
    pQH->Caps2.uFrameSMask = (bTransferType == USB_ENDPOINT_TRANSFER_TYPE_INTR || bTransferType == USB_ENDPOINT_TRANSFER_TYPE_ISOC) ? (bInterval<=4?EHCI_HighSpeedSMask1to8[bInterval-1]:0x1) : 0;
    if(EPS != EPS_HIGH)
    {
        pQH->Caps2.PortNum_1x = (pDevice->bHubPort+1U)&0x7FU; //1 based for hub device
        pQH->Caps2.HubAddr_1x = pDevice->pHub->bHubAddress&0x7FU;
        if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_INTR || bTransferType == USB_ENDPOINT_TRANSFER_TYPE_ISOC)
        {
            pQH->Caps2.uFrameSMask = 0x1; //0th (bit0) micro frame: start-split
            pQH->Caps2.uFrameCMask_1x = 0xE; //1,2,3 micro frames for INTR: complete-split
        }
        else if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL)
            pQH->Caps.ControlEP_1x = 1U;
    }
    pQH->Token.PID = (bTransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL) ? PID_SETUP : dir&0x1U; //TODO: is this necessary?

    pQH->EXT.TransferType = bTransferType&0x3U;

    if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL)
    {
        EHCI_InitQH(pQH, pHCData->ControlTail);
        pHCData->ControlTail = pQH;
    }
    else if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK)
    {
        EHCI_InitQH(pQH, pHCData->BulkTail);
        pHCData->BulkTail = pQH;
    }
    else if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_INTR || bTransferType == USB_ENDPOINT_TRANSFER_TYPE_ISOC)
    {
        if(EPS == EPS_HIGH)
            bInterval = (uint8_t)min(bInterval<=4?1:bInterval-4, EHCI_INTR_COUNT); //pack [1~4] to [1] and clamp [1~16] to [1~11]
        else
            bInterval = min(bInterval, EHCI_INTR_COUNT);
        pQH->EXT.Interval = bInterval&0xFU;
        //EHCI_QH* head = &pHCData->InterruptQH[bInterval-1];
        EHCI_QH** tail = &pHCData->InterruptTail[bInterval-1];
        EHCI_InitQH(pQH, *tail);
        *tail = pQH;
    }
    return EHCI_EP_MAKE(pQH, EPAddr);
}

BOOL EHCI_RemoveEndpoint(HCD_Device* pDevice, void* pEndpoint)
{
    EHCI_QH* pQH = EHCI_EP_GETQH(pEndpoint);
    if(pDevice == NULL || pQH == NULL)
        return FALSE;
    EHCI_HCDeviceData* pDeviceData = (EHCI_HCDeviceData*)pDevice->pHCData;
    if(pDeviceData == NULL)
        return FALSE;
    if(&pDeviceData->ControlQH == pQH) //default control pipe
    {
        pDeviceData->ControlEPCreated = FALSE;
        _LOG("EHCI remove default control pipe\n");
    }

    BOOL result = FALSE;
    EHCI_qTD* pTail = pQH->EXT.Tail;
    EHCI_HCData* pHCData = (EHCI_HCData*)pDevice->pHCI->pHCDData;
    uint8_t bTransferType = pQH->EXT.TransferType;

    if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL)
        result = EHCI_DetachQH(pQH, &pHCData->ControlQH, &pHCData->ControlTail);
    else if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK)
        result = EHCI_DetachQH(pQH, &pHCData->BulkQH, &pHCData->BulkTail);
    else if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_INTR || bTransferType == USB_ENDPOINT_TRANSFER_TYPE_ISOC)
    {
        assert(pQH->EXT.Interval <= 11);
        EHCI_QH* head = &pHCData->InterruptQH[pQH->EXT.Interval-1];
        EHCI_QH** tail = &pHCData->InterruptTail[pQH->EXT.Interval-1];
        result = EHCI_DetachQH(pQH, head, tail);
    }

    while(pTail != NULL) //remove unfinished TD
    {
        EHCI_qTD* pPrev = pTail->EXT.Prev;
        USB_TFree(pTail);
        pTail = pPrev;
    }
    if(&pDeviceData->ControlQH != pEndpoint)
        DPMI_DMAFree(pQH);
    return result;
}

void EHCI_ResetHC(HCD_Interface* pHCI)
{
    EHCI_HCData* pHCData = (EHCI_HCData*)pHCI->pHCDData;
    DPMI_StoreD(pHCData->OPBase+USBCMD, DPMI_LoadD(pHCData->OPBase+USBCMD) | HCRESET);
    delay(55); //spec require 50ms
    DPMI_StoreD(pHCData->OPBase+USBCMD, DPMI_LoadD(pHCData->OPBase+USBCMD) & ~HCRESET & ~RS);
    delay(55);
}

BOOL EHCI_SetupPeriodicList(HCD_Interface* pHCI)
{
    EHCI_HCData* pHCData = (EHCI_HCData*)pHCI->pHCDData;
    uint16_t handle = 0;
    uint32_t FrameList = DPMI_LoadD(pHCData->OPBase + PERIODICLISTBASE) & 0xFFFFF000L;
    //if(FrameList == 0) //if not 0, probably previous inited by BIOS
    { // use xms directly instead of DPMI_DMAMalloc to save memory for driver, especially BC (64K data only).
        handle = XMS_Alloc(4, &FrameList);
        if(handle == 0 || FrameList == 0)
            return FALSE;
        if((FrameList&0xFFF) != 0) //spec require frame list aligned to 4k
        {
            if(!XMS_Realloc(handle, 8, &FrameList)) //4k data + 4k alignement
            {
                XMS_Free(handle);
                return FALSE;
            }
        }
        FrameList = align(FrameList, 4096);
        assert((FrameList&0xFFF) == 0);
    }

    pHCData->dwPeroidicListBase = FrameList < 0x100000L ? FrameList : DPMI_MapMemory(FrameList, 4096);
    _LOG("EHCI frame list base address: %08lx %08lx\n", FrameList, pHCData->dwPeroidicListBase);
    pHCData->wPeroidicListHandle = handle;

    // build frame list entries
    EHCI_FLEP* flep = (EHCI_FLEP*)malloc(4096);
    assert(flep);
    {//scope for BC
        for(int i = 0; i < 1024; ++i)
            flep[i].Ptr = (Typ_QH<<Typ_Shift) | Tbit;
    }
    {//scope for BC
        for(int i = 0; i < EHCI_INTR_COUNT; ++i)
        {
            pHCData->InterruptQH[i].Token.StatusBm.Halted = 1;
            pHCData->InterruptQH[i].HorizLink.Ptr = (Typ_QH<<Typ_Shift) | Tbit;
            pHCData->InterruptTail[i] = &pHCData->InterruptQH[i];
        }
    }
    int offset = 0;
    {//scope for BC
        for(int i = EHCI_INTR_COUNT-1; i >= 3; --i)
        {
            int step = 1<<i;
            for(int j = offset++; j < 1024; j+=step)
                flep[j].Ptr = DPMI_PTR2P(&pHCData->InterruptQH[i]) | (Typ_QH<<Typ_Shift);
        }
    }
    {//scope for BC
        for(int i = 2; i >= 0; --i)
        {
            int step = 1<<i;
            for(int j = 0; j < 1024; j+=step)
            {
                EHCI_FLEP* ptr = &flep[j];
                EHCI_QH* last = NULL;
                while((ptr->Ptr&~0x1FUL) != 0)
                {
                    last = (EHCI_QH*)DPMI_P2PTR(ptr->Ptr&~0x1FUL);
                    ptr = &(last->HorizLink);
                }
                if(last != &pHCData->InterruptQH[i])
                    ptr->Ptr = DPMI_PTR2P(&pHCData->InterruptQH[i]) | (Typ_QH<<Typ_Shift);
            }
        }
    }

    DPMI_CopyLinear(pHCData->dwPeroidicListBase, DPMI_PTR2L(flep), 4096);
    free(flep);
    DPMI_StoreD(pHCData->OPBase + PERIODICLISTBASE, FrameList); //physical addr
    return TRUE;
}

void EHCI_RunStop(HCD_Interface* pHCI, BOOL Run)
{
    EHCI_HCData* pHCData = (EHCI_HCData*)pHCI->pHCDData;
    DPMI_MaskD(pHCData->OPBase+USBCMD, Run ? (~0UL) : (~RS), Run ? RS : 0);
}

void EHCI_InitQH(EHCI_QH* pInit, EHCI_QH* pLinkto)
{
    //memset(pInit, 0, sizeof(EHCI_QH));
    pInit->HorizLink.Bm.T = 1;
    EHCI_qTD* pTD = (EHCI_qTD*)USB_TAlloc(sizeof(EHCI_qTD), EHCI_ALIGN);
    memset(pTD, 0, sizeof(EHCI_qTD)); //pTD->Token.Active = 0;
    pInit->EXT.Tail = pTD;
    pInit->qTDAltNext.Ptr = pInit->qTDNext.Ptr = DPMI_PTR2P(pTD);

    if(pLinkto)
    {
        pInit->HorizLink = pLinkto->HorizLink;
        EHCI_FLEP link; //make the link in one atomic write
        link.Ptr = DPMI_PTR2P(pInit);
        link.Bm.Typ = Typ_QH;
        link.Bm.T = 0;
        pLinkto->HorizLink = link;
    }
}

BOOL EHCI_DetachQH(EHCI_QH* pQH, EHCI_QH* pLinkHead, EHCI_QH** ppLinkEnd)
{
    if(!pQH || !pLinkHead || !ppLinkEnd) return FALSE;

    EHCI_QH* qh = pLinkHead;
    EHCI_QH* prevQH = NULL;
    while(qh != *ppLinkEnd && qh != pQH)
    {
        prevQH = qh;
        qh = (qh->HorizLink.Ptr&~0x1FUL) ? (EHCI_QH*)DPMI_P2PTR(qh->HorizLink.Ptr&~0x1FUL) : NULL;
        assert(qh != NULL);
    }
    if(qh != pQH)
    {
        assert(FALSE);
        return FALSE;
    }
    if(*ppLinkEnd == pQH)
        *ppLinkEnd = prevQH;
    prevQH->HorizLink = qh->HorizLink;
    return TRUE;
}

void EHCI_BuildqTD(EHCI_qTD* pTD, EHCI_qTD* pNext, EHCI_QToken token, HCD_Request* pRequest, void* pBuffer)
{
    //memset(pTD, 0, sizeof(*pTD)); //preserve EXT.Prev
    pTD->Token = token;
    pTD->Next.Ptr = pNext ? DPMI_PTR2P(pNext) : Tbit;
    pTD->AltNext = pTD->Next;

    pTD->EXT.Request = pRequest;
    pTD->EXT.Next = pNext;
    if(pNext) {assert(pNext->EXT.Prev == NULL);pNext->EXT.Prev = pTD;}

    if(!token.Length)
        return;
    assert(pBuffer != NULL && token.Length <= EHCI_qTD_MaxSize);
    uint16_t length = token.Length;
    uint32_t linear = DPMI_PTR2L(pBuffer); //although L:P is contiguous for DOS DPMI host, we just map for each page for safety
    uint16_t offset = linear - alignup(linear, 4096);
    linear = alignup(linear, 4096);

    int i = 0;
    while(length > 0 && i < 5)
    {
        uint16_t size = (uint16_t)min(4096U - offset, length);
        pTD->BufferPages[i].Ptr = DPMI_L2P(linear); //L2P for each page
        pTD->BufferPages[i].Bm.CurrentOffset = offset&0xFFFU;

        ++i;
        offset = 0;
        linear += 4096;
        length = (uint16_t)(length-size);
    }
    for(; i < 5; ++i) assert(pTD->BufferPages[i].Ptr == 0);
    assert(length == 0); //length is pre-calc from outside that it must fit in the 5 buffer page.
}

void EHCI_ISR_QH(HCD_Interface* pHCI, EHCI_QH* pHead, EHCI_QH* pTail)
{
    assert(pHead != NULL && pTail != NULL);
    while(pHead != pTail)
    {
        EHCI_ISR_qTD(pHCI, pHead);
        EHCI_FLEP p = pHead->HorizLink;
        while(p.Bm.Typ != Typ_QH) p = *(EHCI_FLEP*)DPMI_P2PTR(p.Ptr&~0x1FUL); //iTD,siTD,QH,FSTN all start with a link ptr
        pHead = (EHCI_QH*)DPMI_P2PTR(p.Ptr&~0x1FUL);
        assert(pHead != NULL);
    }
    EHCI_ISR_qTD(pHCI, pHead); //[pHead, pTail]
}

void EHCI_ISR_qTD(HCD_Interface* pHCI, EHCI_QH* pQH)
{
    unused(pHCI);
    // detach all inactive TD first, then issue callback, to prevent extra transferr happen in callback and break the in-processing list
    EHCI_qTD* pTD = (pQH->qTDNext.Ptr&~0x1FUL) ? (EHCI_qTD*)DPMI_P2PTR(pQH->qTDNext.Ptr&~0x1FUL) : NULL;
    EHCI_qTD* pList = NULL;
    HCD_Request* pReq = NULL;
    uint8_t error = 0;
    while(pTD && pTD->EXT.Prev) pTD = pTD->EXT.Prev; //find head TD

    while(pTD && pTD != pQH->EXT.Tail)
    {
        assert(pTD->EXT.Request);
        EHCI_qTD* pNext = pTD->EXT.Next;
        uint8_t errmask = (uint8_t)(pTD->Token.Status&EHCI_QTK_ERRORS); //workaround -Wconversion bug
        if(pQH->Caps.EndpointSpd == EPS_HIGH) //P_ERROR is ping state for high speed ep
            errmask &= (uint8_t)~1U;

        error |= errmask;
        if(!pTD->Token.StatusBm.Active || (error && (pTD->EXT.Request == pReq))) //stop after first inactive request
        {
            assert(pTD->EXT.Prev == NULL); //prev should be inactive in queue
            //remove from list
            if(pNext)
                pNext->EXT.Prev = pTD->EXT.Prev;
            pTD->EXT.Next = pList;
            pList = pTD;
            pReq = pTD->EXT.Request;
            pTD = pNext;
        }
        else
            /*pTD = pNext;*/break;
    }

    if(pTD && error)
    {
        assert(!pTD->Token.StatusBm.Active && pTD == pQH->EXT.Tail || pTD->Token.StatusBm.Active);
        pQH->qTDNext.Ptr = DPMI_PTR2P(pTD);
    }

    while(pList) //release TD in reversed order
    {
        pTD = pList;
        pList = pList->EXT.Next;
        uint8_t error = (pTD->Token.Status&EHCI_QTK_ERRORS);
        if(pQH->Caps.EndpointSpd == EPS_HIGH) //P_ERROR is ping state for high speed ep
            error &= (uint8_t)~1U;

        if((pTD->Token.IOC || error) && !pTD->Token.StatusBm.Active)
        {
            ///*if(error)*/_LOG("CB %x %d CBE ", error, pTD->EXT.Request->size);
            //_LOG("CB %x ", HC2USB(pTD->EXT.Request->pDevice)->Desc.bDeviceClass);
            HCD_InvokeCallBack(pTD->EXT.Request, (uint16_t)(pTD->EXT.Request->size - pTD->Token.Length), error);
        }
        //_LOG("Free TD: %lx ", pTD);
        assert(pTD != pQH->EXT.Tail);
        USB_TFree(pTD);
    }
}
