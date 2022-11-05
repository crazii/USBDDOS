#include <memory.h>
#include <stdio.h>
#include <dos.h>
#include <conio.h>
#include <string.h>
#include <assert.h>
#include "USBDDOS/HCD/UHCI.H"
#include "USBDDOS/DPMI/DPMI.H"
#include "USBDDOS/USBALLOC.H"
#include "USBDDOS/DBGUTIL.H"

// reference spec: Universal Host Controller Interface (UHCI)
// Universal Serial Bus Specification 2.0

#define UHCI_USE_INTERRUPT 0

#define UHCI_ED_GETQH(p) (UHCI_QH*)(((uintptr_t)p)&~0xFU)
#define UHCI_ED_GETADDR(p) (((uintptr_t)p)&0xF)

static void UHCI_ISR_ProcessQH(HCD_Interface* pHCI, UHCI_QH* pQH, UHCI_QH* pEnd);
static void UHCI_ISR_ProcessTD(HCD_Interface* pHCI, UHCI_QH* pQH);
static uint16_t UHCI_GetPortStatus(HCD_Interface* pHCI, uint8_t port);
static BOOL UHCI_SetPortStatus(HCD_Interface* pHCI, uint8_t port, uint16_t status);
static BOOL UHCI_InitDevice(HCD_Device* pDevice);
static BOOL UHCI_RemoveDevice(HCD_Device* pDevice);
static void* UHCI_CreateEndpoint(HCD_Device* pDevice, uint8_t EPAddr, HCD_TxDir dir, uint8_t bTransferType, uint16_t MaxPacketSize, uint8_t bInterval);
static BOOL UHCI_RemoveEndpoint(HCD_Device* pDevice, void* pEndpoint);

static void UHCI_InitQH(UHCI_QH* pQH);
static  void UHCI_BuildTD(UHCI_TD* pTD, UHCI_TD* pNext, uint32_t  ControlStatus,uint8_t PID, uint8_t DevAddr, uint8_t EndPt, uint8_t DataTog, uint16_t MaxLen, uint32_t buffer);
static  void UHCI_QHTDSchedule(HCD_Interface* pHCI);
static  void UHCI_InsertQHintoQH(UHCI_QH* pToQH, UHCI_QH* pQH);
static BOOL UHCI_RemoveQHfromQH(UHCI_QH* pFromQH, UHCI_QH** pEndQH, UHCI_QH* pQH);
static  void UHCI_ResetHC(HCD_Interface* pHCI);
static  void UHCI_StartHC(HCD_Interface* pHCI);
static  void UHCI_StopHC(HCD_Interface* pHCI);
static  int  UHCI_WaitTDDone(UHCI_TD* pTD);
static UHCI_QH* UHCI_GetQHFromInterval(UHCI_HCData* pHCData, uint8_t interval);

HCD_Method UHCIAccessMethod =
{
    &UHCI_ControlTransfer,
    &UHCI_DataTransfer,
    &UHCI_DataTransfer,
    &UHCI_DataTransfer,
    &UHCI_GetPortStatus,
    &UHCI_SetPortStatus,
    &UHCI_InitDevice,
    &UHCI_RemoveDevice,
    &UHCI_CreateEndpoint,
    &UHCI_RemoveEndpoint,
};

BOOL UHCI_InitController(HCD_Interface * pHCI, PCI_DEVICE* pPCIDev)
{
    // stop USB SMI
    uint32_t pciValue = PCI_ReadByte(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, 2, LEGSUP);
    pciValue &= ~USBSMIEN;
    PCI_WriteByte(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, 2, LEGSUP, (uint8_t)pciValue);

    pHCI->dwPhysicalAddress = *(uint32_t*)&pPCIDev->offset[USBBASE] & 0xffffffe0L;
    pHCI->dwBaseAddress = pHCI->dwPhysicalAddress;    //IO address
    _LOG("UHCI base IO address: %04x\n", pHCI->dwBaseAddress);
    pHCI->pHCDMethod = &UHCIAccessMethod;

    pHCI->pHCDData = DPMI_DMAMalloc(sizeof(UHCI_HCData), 16);
    UHCI_HCData* pHCData = (UHCI_HCData*)pHCI->pHCDData;
    memset(pHCData, 0, sizeof(UHCI_HCData));

    // get host controller info
    pHCI->bNumPorts= 2;
    pHCData->dwHcdDataArea = inpd((uint16_t)(pHCI->dwBaseAddress + FLBASEADD)) & 0xfffff000L;
    pHCData->dwHcdDataArea = pHCData->dwHcdDataArea < 0x100000L ? pHCData->dwHcdDataArea : DPMI_MapMemory(pHCData->dwHcdDataArea, 4096);    //1024 entry, 4 bytes per entry.

    outpw((uint16_t)(pHCI->dwBaseAddress + USBINTR), UHCI_USE_INTERRUPT ? 0xF : 0); //USBINTR is 16 bit but only 4 bit is used

    // setup framelist
    UHCI_StopHC(pHCI);
    UHCI_QHTDSchedule(pHCI);
    UHCI_StartHC(pHCI);
    return TRUE;
}

BOOL UHCI_DeinitController(HCD_Interface* pHCI)
{
    uint32_t dwBase = pHCI->dwBaseAddress;
    if(dwBase)
    {
        UHCI_ResetHC(pHCI);
        delay(1);
    }
    if(pHCI->pHCDData)
        DPMI_DMAFree(pHCI->pHCDData);
    pHCI->pHCDData = NULL;
    pHCI->dwBaseAddress = 0;
    return TRUE;
}

BOOL UHCI_ISR(HCD_Interface* pHCI)
{
#if !UHCI_USE_INTERRUPT
    return TRUE;
#endif
    uint32_t iobase = pHCI->dwBaseAddress;
    uint16_t status = inpw((uint16_t)(iobase + USBSTS));
    if(!(status&USBINTMASK))
        return FALSE;
    
    if(status&(USBHSERROR|USBHCERROR))
    {
        UHCI_ResetHC(pHCI);
        return TRUE;
    }
    if(status&USBRESUMEDETECT)//TODO
    {
    }

    if(status&(USBINT|USBERRORINT))
    {
        //No DPC in DOS, process immediately
        UHCI_HCData* pHCData = (UHCI_HCData*)pHCI->pHCDData;
        UHCI_ISR_ProcessQH(pHCI, &pHCData->QH1ms, pHCData->InteruptTail[0]); //porcess ISO queue & 1ms interrupt queue
        UHCI_ISR_ProcessQH(pHCI, &pHCData->QH2ms, pHCData->InteruptTail[1]); //porcess 2ms interrupt queue
        UHCI_ISR_ProcessQH(pHCI, &pHCData->QH8ms, pHCData->InteruptTail[2]); //porcess 8ms interrupt queue
        //UHCI_ISR_ProcessQH(pHCI, &pHCData->ControlQH, pHCData->ControlTail); //porcess control queue. control QH and transferred immediately and TD pre-allocated and not queued, don't process.
        UHCI_ISR_ProcessQH(pHCI, &pHCData->BulkQH, pHCData->BulkTail); //porcess bulk queue
    }
    return TRUE;
}

uint8_t UHCI_ControlTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t outputp setup8[8],
    void* nullable pSetupData, uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData)
{
    UHCI_QH* pQH = UHCI_ED_GETQH(pEndpoint);
    if(pCB == NULL || pQH == NULL || pQH->Flags.Type != USB_ENDPOINT_TRANSFER_TYPE_CTRL)
    {
        return 0xFF;
    }
    //UHCI_HCData* pHCData = (UHCI_HCData*)pDevice->pHCI->pHCDData;
    //UHCI_HCDeviceData* pDD = (UHCI_HCDeviceData*)pDevice->pHCData;
    uint8_t bAddress = pDevice->bAddress;
    uint8_t bEndpoint = UHCI_ED_GETADDR(pEndpoint);
    uint16_t MaxLength = pQH->Flags.wMaxPacketSize;
    HCD_Request* pRequest = HCD_AddRequest(pDevice, pEndpoint, dir, pSetupData, length, bEndpoint, pCB, pCBData);
    uint32_t CS = CS_ActiveStatus | CS_C_ERR1 | (USB_PORT_Low_Speed_Device ? 0 : CS_LowSpeed);
    uint8_t DataPID = (dir == HCD_TXW) ? OUTPID : INPID;
    uint8_t StatusPID = (dir == HCD_TXW) ? INPID : OUTPID; //usb1.1 spec, status inverts direction of data
    UHCI_TD* pEnd = (UHCI_TD*)USB_TAlloc32(sizeof(UHCI_TD));
    memset(pEnd, 0, sizeof(UHCI_TD));

    // build status TD
    UHCI_TD* pStatusTD = (UHCI_TD*)USB_TAlloc32(sizeof(UHCI_TD));
    UHCI_BuildTD(pStatusTD, pEnd, CS, StatusPID, bAddress, bEndpoint, 1, 0x7ff, 0);
    pStatusTD->pRequest = pRequest;

    // build data TD
    UHCI_TD* pDataTD = NULL;
    if(length)
    {
        uint16_t Transferred = 0;
        uint8_t DataToggle = 1;
        uint32_t SetupDataAddr = DPMI_PTR2P(pSetupData);
        UHCI_TD* pNext = pStatusTD;

        while(Transferred < length)
        {
            uint16_t DataLength = min(MaxLength, (uint16_t)(length - Transferred));
            UHCI_TD* pTD = (UHCI_TD*)USB_TAlloc32(sizeof(UHCI_TD));
            UHCI_BuildTD(pTD, pNext, CS, DataPID, bAddress, bEndpoint, DataToggle, (uint16_t)(DataLength-1), SetupDataAddr + Transferred);
            pTD->pRequest = pRequest;
            DataToggle ^= 1;
            
            Transferred = (uint16_t)(Transferred+DataLength);
            pNext = pTD;
            if(!pDataTD)
                pDataTD = pTD;
        }
    }

    // build setup TD
    CLIS();
    UHCI_TD* pSetupTD = pQH->pTail;
    UHCI_BuildTD(pSetupTD, length ? pDataTD : pStatusTD, CS, SETUPPID, bAddress, bEndpoint, 0, 8-1, DPMI_PTR2P(&setup8));
    pSetupTD->pRequest = pRequest;
    // start transfer
    pSetupTD->ControlStatusBits.Active = 1;
    #if !UHCI_USE_INTERRUPT
    if(!UHCI_WaitTDDone(pSetupTD) || !UHCI_WaitTDDone(pStatusTD))
        _LOG("setup not ready\n");
    #if DEBUG && 0
    DBG_DumpB((uint8_t*)pSetupTD, 16);
    #endif
    uint16_t len = 0;
    uint8_t error = 0;
    while(pDataTD != NULL && pDataTD != pStatusTD)
    {
        len = (uint16_t)(len + pDataTD->ControlStatusBits.ActualLen);
        error |= (uint8_t)((pDataTD->ControlStatus&CS_ErrorMask)>>CS_ErrorShift);
        UHCI_TD* pTD = pDataTD->pNext;
        USB_TFree32(pDataTD);
        pDataTD = pTD;
    }
    USB_TFree32(pStatusTD);
    USB_TFree32(pSetupTD);
    HCD_InvokeCallBack(pRequest, len, error | (uint8_t)((pStatusTD->ControlStatus&CS_ErrorMask)>>CS_ErrorShift));
    #endif
    STIL();
    return (pStatusTD->ControlStatus >> 16) & 0xF;
}

uint8_t UHCI_DataTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t* inoutp pBuffer,
    uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData)
{
    UHCI_QH* pQH = UHCI_ED_GETQH(pEndpoint);
    if(pCB == NULL || pQH == NULL || pQH->Flags.Dir != dir || pQH->Flags.Type == USB_ENDPOINT_TRANSFER_TYPE_CTRL || pBuffer == NULL || length == 0)
        return 0xFF;
    
    //UHCI_HCData* pHCData = (UHCI_HCData*)pDevice->pHCI->pHCDData;
    //UHCI_HCDeviceData* pDD = (UHCI_HCDeviceData*)pDevice->pHCData;
    uint8_t USBAddress = pDevice->bAddress;
    uint8_t bEndpoint = UHCI_ED_GETADDR(pEndpoint);
    uint8_t PID = (dir == HCD_TXW) ? OUTPID : INPID;
    uint16_t MaxLen = pQH->Flags.wMaxPacketSize;
    uint8_t Toggle = pQH->Flags.DataToggle;
    HCD_Request* pRequest = HCD_AddRequest(pDevice, pEndpoint, dir, pBuffer, length, bEndpoint, pCB, pCBData);
    uint32_t pbuffer = DPMI_PTR2P(pBuffer);
    UHCI_TD* pHead = pQH->pTail;
    UHCI_TD* pEnd = NULL;

    uint32_t CS = CS_ActiveStatus | CS_C_ERR1 | (USB_PORT_Low_Speed_Device ? 0 : CS_LowSpeed) | ((pQH->Flags.Type == USB_ENDPOINT_TRANSFER_TYPE_ISOC) ? CS_IOS : 0);
    uint16_t transferred = 0;
    CLIS();
    while(transferred < length)
    {
        uint16_t PacketLength = min((uint16_t)(length - transferred), MaxLen);
        //Build TD with active state to start transfer
        UHCI_TD* pTD = (UHCI_TD*)USB_TAlloc32(sizeof(UHCI_TD));
        memset(pTD, 0, sizeof(UHCI_TD));
        uint32_t ICS = CS | ((transferred + PacketLength == length) ? CS_IOC : 0);
        UHCI_BuildTD(pQH->pTail, pTD, ICS,  PID, USBAddress, bEndpoint, Toggle, (uint16_t)(PacketLength - 1), pbuffer + transferred);
        pTD->pRequest = pRequest;
        #if DEBUG && 0
        DBG_DumpB((uint8_t*)pTD, 16);
        #endif
        Toggle ^=1;
        transferred = (uint16_t)(transferred + PacketLength);
        pEnd = pQH->pTail;
        pQH->pTail = pTD;
    }
    //start transfer
    pHead->ControlStatusBits.Active = 1;
    #if !UHCI_USE_INTERRUPT
    if(!UHCI_WaitTDDone(pEnd))
        _LOG("data transfer failed\n");
    uint16_t len = 0;
    uint8_t error = 0;
    while(pHead != pQH->pTail)
    {
        len = (uint16_t)(len + pHead->ControlStatusBits.ActualLen);
        error |= (uint8_t)((pHead->ControlStatus&CS_ErrorMask)>>CS_ErrorShift);
        UHCI_TD* pTD = pHead->pNext;
        USB_TFree32(pHead);
        pHead = pTD;
    }
    HCD_InvokeCallBack(pRequest, len, error);
    #endif
    STIL();

    pQH->Flags.DataToggle = Toggle&0x1U;
    return 0;
}

void UHCI_ISR_ProcessQH(HCD_Interface* pHCI, UHCI_QH* pQH, UHCI_QH* pEnd)
{
    do
    {
        UHCI_ISR_ProcessTD(pHCI, pQH);
        pQH = (UHCI_QH*)DPMI_P2PTR(pQH->HeadLink&~0xFU);
    }while(pQH != pEnd);
    UHCI_ISR_ProcessTD(pHCI, pQH); //[pQH, pEnd]
}

void UHCI_ISR_ProcessTD(HCD_Interface* pHCI, UHCI_QH* pQH)
{
#if UHCI_USE_INTERRUPT
    //detach all inactive TD first, then issue callback, to prevent extra transferr happen in callback and break the in-processing list
    UHCI_TD* pTD = pQH->pTail;
    UHCI_TD* pList = NULL;
    while(pTD)
    {
        if(!pTD->ControlStatusBits.Active)
        {
            //remove from list
            if(pTD->pPrev)
            {
                pTD->pPrev->pNext = pTD->pNext;
                pTD->pPrev->LinkPointer = pTD->pNext ? (pTD->pNext->PAddr | DepthSelect) : 0;
            }
            if(pTD->pNext)
                pTD->pNext->pPrev = pTD->pPrev;
            pTD->pPrev = NULL;
            pTD->pNext = pList;
            pList = pTD;
        }
        pTD = pTD->pPrev;
    }

    while(pList)
    {
        pTD = pList;
        pList = pList->pNext;
        if(pTD->ControlStatusBits.Interrupt || (pTD->ControlStatus&CS_ErrorMask))
        {
            uint8_t error = (pTD->ControlStatus&CS_ErrorMask)>>CS_ErrorShit;
            HCD_InvokeCallBack(pTD->pRequest, pTD->BufferPointer + pTD->ControlStatusBits.ActualLen - DPMI_PTR2P(pTD->pRequest->pBuffer), error);
        }
        pTD->pNext = NULL;
        USB_TFree32(pTD);
    }
#endif// UHCI_USE_INTERRUPT
}

uint16_t UHCI_GetPortStatus(HCD_Interface* pHCI, uint8_t port)
{
    uint16_t status = 0;
    uint16_t portbase = (uint16_t)(pHCI->dwBaseAddress + PORTBASE + port * 2U);
    uint16_t PORTSC = inpw(portbase);

    if(PORTSC & CCS)
        status |= USB_PORT_ATTACHED;

    if(PORTSC & LSDA)
        status |= USB_PORT_Low_Speed_Device ; // 01 = low speed device.
    else
        status |= USB_PORT_Full_Speed_Device; // 2 = full speed

    if(PORTSC & PED)
        status |= USB_PORT_ENABLE;
    else
        status |= USB_PORT_DISABLE;

    if(PORTSC & SUSPEND)
        status |= USB_PORT_SUSPEND;

    if(PORTSC & PR)
        status |= USB_PORT_RESET;   //resetting

    if(PORTSC & CSC)
        status |= USB_PORT_CONNECT_CHANGE;
    return status;
}

BOOL UHCI_SetPortStatus(HCD_Interface* pHCI, uint8_t port, uint16_t status)
{
    uint16_t portbase = (uint16_t)(pHCI->dwBaseAddress + PORTBASE + port * 2U);
    uint16_t cur = inpw(portbase);

    if((status&USB_PORT_RESET))
    {
        outpw(portbase, cur | PR);
        const int timeout = 500;
        int i = 0;
        while((inpw(portbase)&PR) && i < timeout)
        {
            delay(10);
            ++i;
        }
        if(i == timeout)
            return FALSE;
    }

    if((status&USB_PORT_ENABLE) && !(cur&PED))
    {
        outpw(portbase,  cur | PED);
        while(!(inpw(portbase)&PEDC))
            delay(10);
    }

    if((status&USB_PORT_DISABLE) && (cur&PED))
    {
        outpw(portbase,  cur & (uint16_t)~PED);
        while(!(inpw(portbase)&PEDC))
            delay(10);
    }

    if((status&USB_PORT_SUSPEND) && !(cur&SUSPEND))
    {
        while(!(inpw(portbase)&SUSPEND))
            outpw(portbase,  cur | SUSPEND);
    }
    return TRUE;
}

BOOL UHCI_InitDevice(HCD_Device* pDevice)
{
    pDevice->pHCData = DPMI_DMAMalloc(sizeof(UHCI_HCDeviceData), 16);
    UHCI_HCDeviceData* pDD = (UHCI_HCDeviceData*)pDevice->pHCData;
    memset(pDD, 0, sizeof(UHCI_HCDeviceData));
    
    UHCI_HCData* pHCData = (UHCI_HCData*)pDevice->pHCI->pHCDData;
    UHCI_InitQH(&pDD->ControlQH);
    for(int i = 0; i < 3; ++i)
    {
        pDD->ControlTD[i].ControlStatusBits.Active = 0;
        pDD->ControlTD[i].LinkPointer = 0;
    }
    pDD->ControlQH.pTail = pDD->ControlTD;
    UHCI_InsertQHintoQH(pHCData->ControlTail, &pDD->ControlQH);
    pHCData->ControlTail = &pDD->ControlQH;
    return TRUE;
}

BOOL UHCI_RemoveDevice(HCD_Device* pDevice)
{
    UHCI_HCDeviceData* pDD = (UHCI_HCDeviceData*)pDevice->pHCData;
    DPMI_DMAFree(pDD);
    return TRUE;
}

void* UHCI_CreateEndpoint(HCD_Device* pDevice, uint8_t EPAddr, HCD_TxDir dir, uint8_t bTransferType, uint16_t MaxPacketSize, uint8_t bInterval)
{
    UHCI_HCDeviceData* pDD = (UHCI_HCDeviceData*)pDevice->pHCData;
    UHCI_HCData* pHCData = (UHCI_HCData*)pDevice->pHCI->pHCDData;
    if(EPAddr == 0) //default control pipe
    {
        pDD->ControlQH.Flags.wMaxPacketSize = MaxPacketSize&0x7FFU;
        for(int i = 0; i < 3; ++i)
            pDD->ControlTD[i].TokenBits.DeviceAddress = pDevice->bAddress&0x7FU;
        return &pDD->ControlQH;
    }
    assert(EPAddr <= 0xF);
    assert(bInterval >= 1);

    UHCI_QH* pQH = (UHCI_QH*)DPMI_DMAMalloc(sizeof(UHCI_QH), 16);
    memset(pQH, 0, sizeof(pQH));
    UHCI_InitQH(&pDD->ControlQH);
    pQH->Flags.Dir = dir&0x1;
    pQH->Flags.Type = bTransferType&0x3U;
    if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL)
    {
        UHCI_InsertQHintoQH(pHCData->ControlTail, pQH);
        pHCData->ControlTail = pQH;
    }
    else if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_ISOC)
    {
        UHCI_InsertQHintoQH(pHCData->InteruptTail[0], pQH); //add to 1ms head
        pHCData->InteruptTail[0] = pQH;
    }
    else if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK )
    {
        UHCI_InsertQHintoQH(pHCData->BulkTail, pQH);
        pHCData->BulkTail = pQH;
    }
    else if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_INTR )
    {
        UHCI_QH* head = UHCI_GetQHFromInterval(pHCData, bInterval);
        pQH->Flags.Interval = (uint16_t)(head - &pHCData->QH1ms)&0x3U; //ptr to index
        assert(pQH->Flags.Interval <= 2);
        UHCI_InsertQHintoQH(pHCData->InteruptTail[pQH->Flags.Interval], pQH);
        pHCData->InteruptTail[pQH->Flags.Interval] = pQH;
    }
    pQH->pTail = (UHCI_TD*)USB_TAlloc32(sizeof(UHCI_TD));
    memset(pQH->pTail, 0, sizeof(UHCI_TD));
    return (void*)(((uintptr_t)pQH) | EPAddr);
}

BOOL UHCI_RemoveEndpoint(HCD_Device* pDevice, void* pEndpoint)
{
    UHCI_QH* pQH = UHCI_ED_GETQH(pEndpoint);
    if(pDevice == NULL || pQH == NULL)
        return FALSE;
    UHCI_StopHC(pDevice->pHCI);
    BOOL result = FALSE;
    UHCI_TD* pTail = pQH->pTail;
    UHCI_HCData* pHCData = (UHCI_HCData*)pDevice->pHCI->pHCDData;
    uint8_t bTransferType = pQH->Flags.Type;
    
    if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL)
    {
        pTail = NULL;
        result = UHCI_RemoveQHfromQH(&pHCData->ControlQH, &pHCData->ControlTail, pQH);
    }
    else
    {
        while(pTail != NULL) //remove unnifhsed TD
        {
            UHCI_TD* pPrev = pTail->pPrev;
            USB_TFree32(pTail);
            pTail = pPrev;
        }
    }

    if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_ISOC)
        result = UHCI_RemoveQHfromQH(&pHCData->QH1ms, &pHCData->InteruptTail[0], pQH);
    else if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK )
        result = UHCI_RemoveQHfromQH(&pHCData->BulkQH, &pHCData->BulkTail, pQH);
    else if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_INTR )
    {
        UHCI_QH* IntQH = UHCI_GetQHFromInterval(pHCData, pQH->Flags.Interval);
        UHCI_QH** IntQHTail = &pHCData->InteruptTail[pQH->Flags.Interval];
        result = UHCI_RemoveQHfromQH(IntQH, IntQHTail, pQH);
    }
    UHCI_StartHC(pDevice->pHCI);
    return result;
}

void UHCI_QHTDSchedule(HCD_Interface* pHCI)
{
    UHCI_HCData* pHCData = (UHCI_HCData*)pHCI->pHCDData;
    uint32_t pHCDAREA = pHCData->dwHcdDataArea;

    UHCI_QH* pQH = &pHCData->QH1ms;
    //set QH invalid accord to UHCIDescriptor definition.
    unsigned int i;
    for(i = 0; i < 3; i++)
    {
        pQH[i].HeadLink = TerminateFlag | QHFlag;
        pQH[i].ElementLink = TerminateFlag;
    }
    //clear frame list
    for(i = 0; i < 1024; i++)
        DPMI_StoreD(pHCDAREA + i * 4U, TerminateFlag);
    //link 8ms qh
    for(i = 0; i < 1024; i = i+8)
        DPMI_StoreD(pHCDAREA + i * 4U, DPMI_PTR2P(&pHCData->QH8ms) | QHFlag);
    //link 2ms qh
    for(i = 1; i < 1024; i = i+2)
        DPMI_StoreD(pHCDAREA + i * 4U, DPMI_PTR2P(&pHCData->QH2ms) | QHFlag);

    //link 1ms qh. UHCI frame time is 1ms and 1ms QH should apear on all entries in frame list
    for(i = 0; i < 1024; i++)
    {
        if(DPMI_LoadD(pHCDAREA + i * 4U) == TerminateFlag)
        {
            DPMI_StoreD(pHCDAREA + i * 4U, DPMI_PTR2P(&pHCData->QH1ms) | QHFlag);
        }
        else
        {
            UHCI_InsertQHintoQH(&pHCData->QH8ms, &pHCData->QH1ms);
            UHCI_InsertQHintoQH(&pHCData->QH2ms, &pHCData->QH1ms);
        }
    }

    UHCI_InitQH(&pHCData->BulkQH);
    UHCI_InitQH(&pHCData->ControlQH);

    UHCI_InsertQHintoQH(&pHCData->QH2ms, &pHCData->ControlQH); //control has higher priorty than bulk according to the spec
    UHCI_InsertQHintoQH(&pHCData->ControlQH, &pHCData->BulkQH);
    pHCData->ControlTail = &pHCData->ControlQH;
    pHCData->BulkTail = &pHCData->BulkQH;
    pHCData->InteruptTail[0] = &pHCData->QH1ms;
    pHCData->InteruptTail[1] = &pHCData->QH2ms;
    pHCData->InteruptTail[2] = &pHCData->QH8ms;
    return;
}

void UHCI_InitQH(UHCI_QH* pQH)
{
    pQH->HeadLink = QHFlag | TerminateFlag;
    pQH->ElementLink = 0;
}

void  UHCI_BuildTD(UHCI_TD* pTD, UHCI_TD* pNext, uint32_t  ControlStatus, uint8_t PID, uint8_t DevAddr, uint8_t EndPt, uint8_t DataToggle, uint16_t MaxLen, uint32_t buffer)
{
    memset(pTD, 0, sizeof(pTD));
    pTD->LinkPointer = pNext ? ((pNext->PAddr ? pNext->PAddr : DPMI_PTR2P(pNext)) | DepthSelect) : 0;
    pTD->ControlStatus = ControlStatus;
    pTD->TokenBits.MaxLen = MaxLen&0x3FFU;
    pTD->TokenBits.Endpoint = EndPt&0xFU;
    pTD->TokenBits.DeviceAddress = DevAddr&0x7FU;
    pTD->TokenBits.PID = PID;
    pTD->TokenBits.DataToggle = DataToggle&0x1U;
    pTD->BufferPointer = buffer;
    pTD->pNext = pNext;
    pTD->PAddr = DPMI_PTR2P(pTD);

    if(pNext) pNext->pPrev = pTD;
    return;
}

void UHCI_InsertQHintoQH(UHCI_QH* pToQH, UHCI_QH* pQH)
{
    assert(pQH);
    uint32_t next = pToQH->HeadLink;
    pToQH->HeadLink = DPMI_PTR2P(pQH) | QHFlag;
    pQH->HeadLink = next;
    return;
}

BOOL UHCI_RemoveQHfromQH(UHCI_QH* pFromQH, UHCI_QH** pEndQH, UHCI_QH* pQH)
{
    UHCI_QH* qh = pFromQH;
    UHCI_QH* pqh = NULL;
    while(qh != *pEndQH && qh != pQH)
    {
        pqh = qh;
        qh = (qh->HeadLink&~0xFU) ? (UHCI_QH*)DPMI_P2PTR(qh->HeadLink&~0xFU) : NULL;
    }
    if(qh != pQH)
    {
        assert(FALSE);
        return FALSE;
    }
    pqh->HeadLink = qh->HeadLink;
    if(*pEndQH == pQH)
        *pEndQH = pqh;
    return TRUE;
}

int UHCI_WaitTDDone(UHCI_TD* pTD)
{
    int  i = 0;
    int  WaitTime = 500;
    for(;i < WaitTime; i++)
    {
        if((pTD->ControlStatus&CS_ActiveStatus)) delay(1);
        else break;
    }
    if(i >= WaitTime) return 0;  //error.
    else return 1;
}

void UHCI_ResetHC(HCD_Interface* pHCI)
{
    outpw((uint16_t)(pHCI->dwBaseAddress + USBCMD), HCRESET | GRESET);
    delay(100);     //spec required
    outpw((uint16_t)(pHCI->dwBaseAddress + USBCMD), 0);
    return;
}

void UHCI_StopHC(HCD_Interface* pHCI)
{
    outpw((uint16_t)(pHCI->dwBaseAddress + USBCMD), 0x00);
    return;
}

void UHCI_StartHC(HCD_Interface* pHCI)
{
    outpw((uint16_t)(pHCI->dwBaseAddress + USBCMD), 0xc1);
    return;
}

UHCI_QH* UHCI_GetQHFromInterval(UHCI_HCData* pHCData, uint8_t interval)
{
    interval = interval < 1 ? 1 : interval;
    if(interval == 1)
        return &pHCData->QH1ms;
    else if(interval <= 4)
        return &pHCData->QH2ms;
    else
        return &pHCData->QH8ms;
}
