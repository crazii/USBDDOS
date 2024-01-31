#include <memory.h>
#include <stdio.h>
#include <dos.h>
#include <conio.h>
#include <string.h>
#include <assert.h>
#include "USBDDOS/HCD/uhci.h"
#include "USBDDOS/DPMI/dpmi.h"
#include "USBDDOS/DPMI/xms.h"
#include "USBDDOS/usballoc.h"
#include "USBDDOS/dbgutil.h"

// reference spec: Universal Host Controller Interface (UHCI)
// Universal Serial Bus Specification 2.0

#define UHCI_USE_INTERRUPT 1 //debug use, do not change to 0

#define UHCI_ED_GETQH(p) (UHCI_QH*)(((uintptr_t)p)&~0xFU)
#define UHCI_ED_GETADDR(p) (((uintptr_t)p)&0xFU)

static void UHCI_ISR_ProcessQH(HCD_Interface* pHCI, UHCI_QH* pQH, UHCI_QH* pEnd);
static void UHCI_ISR_ProcessTD(HCD_Interface* pHCI, UHCI_QH* pQH);
static uint16_t UHCI_GetPortStatus(HCD_Interface* pHCI, uint8_t port);
static BOOL UHCI_SetPortStatus(HCD_Interface* pHCI, uint8_t port, uint16_t status);
static BOOL UHCI_InitDevice(HCD_Device* pDevice);
static BOOL UHCI_RemoveDevice(HCD_Device* pDevice);
static void* UHCI_CreateEndpoint(HCD_Device* pDevice, uint8_t EPAddr, HCD_TxDir dir, uint8_t bTransferType, uint16_t MaxPacketSize, uint8_t bInterval);
static BOOL UHCI_RemoveEndpoint(HCD_Device* pDevice, void* pEndpoint);

static void UHCI_EnableInterrupt(HCD_Interface* pHCI, BOOL enable);
static void UHCI_QHTDSchedule(HCD_Interface* pHCI);
static void UHCI_InitQH(UHCI_QH* pQH);
static void UHCI_BuildTD(UHCI_TD* pTD, UHCI_TD* pNext, uint32_t  ControlStatus,uint8_t PID, uint8_t DevAddr, uint8_t EndPt, uint8_t DataTog, uint16_t MaxLen, uint32_t buffer);
static void UHCI_InsertTDintoQH(UHCI_QH* pQH, UHCI_TD* pTD);
static void UHCI_InsertQHintoQH(UHCI_QH* pToQH, UHCI_QH* pQH);
static BOOL UHCI_RemoveQHfromQH(UHCI_QH* pFromQH, UHCI_QH** pEndQH, UHCI_QH* pQH);
static void UHCI_ResetHC(HCD_Interface* pHCI);
static void UHCI_StartHC(HCD_Interface* pHCI);
static void UHCI_StopHC(HCD_Interface* pHCI);
#if !UHCI_USE_INTERRUPT
static int  UHCI_WaitTDDone(UHCI_TD* pTD);
#endif
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
    uint8_t function = pHCI->PCIAddr.Function; //2; //specs says 2 but IRQ won't enable
    uint32_t pciValue = PCI_ReadWord(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, function, LEGSUP);
    pciValue &= ~USBSMIEN; // stop USB SMI
    #if UHCI_USE_INTERRUPT
    pciValue |= USBPIRQDEN; //enable IRQ
    #endif
    pciValue |= 0x8F00; //clear all legacy trap
    PCI_WriteWord(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, function, LEGSUP, (uint16_t)pciValue);

    PCI_CMD cmd;
    cmd.reg16 = pPCIDev->Header.Command;
    cmd.bits.BusMaster = 1;
    cmd.bits.IOSpace = 1;
    cmd.bits.MemorySpace = 1;
    cmd.bits.InterruptDisable = 0;
    PCI_WriteWord(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, pHCI->PCIAddr.Function, PCI_REGISTER_CMD, cmd.reg16);

    pHCI->dwPhysicalAddress = (*(uint32_t*)&pPCIDev->Offset[USBBASE]) & 0xFFFFFFE0L;
    pHCI->dwBaseAddress = pHCI->dwPhysicalAddress;    //PIO address
    _LOG("UHCI base IO address: %04x\n", pHCI->dwBaseAddress);
    pHCI->pHCDMethod = &UHCIAccessMethod;
    UHCI_ResetHC(pHCI);

    uint16_t handle = 0;
    uint32_t DataArea = inpd((uint16_t)(pHCI->dwBaseAddress + FLBASEADD)) & 0xFFFFF000L;

    //the DataArea is used by BIOS, there's error we directly use it
    //allocate new one
    //if(DataArea == 0) //if not 0, probably previous inited by BIOS
    {
        handle = XMS_Alloc(4, &DataArea); //4k data + 4k alignement. use xms directly instead of DPMI_DMAMalloc to save memory for driver, especially BC (64K data only).
        if(handle == 0 || DataArea == 0)
            return FALSE;
        if((DataArea&0xFFF) != 0)
        {
            if(!XMS_Realloc(handle, 8, &DataArea))
            {
                XMS_Free(handle);
                return FALSE;
            }
        }
        DataArea = align(DataArea, 4096);
        outpd((uint16_t)(pHCI->dwBaseAddress + FLBASEADD), DataArea);
    }

    DataArea = DataArea < 0x100000L ? DataArea : DPMI_MapMemory(DataArea, 4096);    //1024 entry, 4 bytes per entry.
    _LOG("UHCI frame list base address: %08lx\n", DataArea);
    pHCI->pHCDData = DPMI_DMAMalloc(sizeof(UHCI_HCData), 16);
    UHCI_HCData* pHCData = (UHCI_HCData*)pHCI->pHCDData;
    memset(pHCData, 0, sizeof(UHCI_HCData));

    pHCI->bNumPorts = 2;
    pHCData->dwFrameListBase = DataArea;
    pHCData->wFrameListHandle = handle;

#if UHCI_USE_INTERRUPT
    UHCI_EnableInterrupt(pHCI, TRUE);
#endif
    UHCI_QHTDSchedule(pHCI); // setup framelist
    outpw((uint16_t)(pHCI->dwBaseAddress + FRNUM), 0);
    outpw((uint16_t)(pHCI->dwBaseAddress + USBSTS), 0x1F); //clear status bits
    outp((uint16_t)(pHCI->dwBaseAddress + SOF), 0x40);
    outpw((uint16_t)(pHCI->dwBaseAddress + USBCMD), MAXP|CF); //CF as last action
    UHCI_StartHC(pHCI);
    return TRUE;
}

BOOL UHCI_DeinitController(HCD_Interface* pHCI)
{
    uint32_t dwBase = pHCI->dwBaseAddress;
    if(dwBase)
    {
        outpd((uint16_t)(pHCI->dwBaseAddress + FLBASEADD), 0);
        UHCI_StopHC(pHCI);
        UHCI_ResetHC(pHCI);
    }
    if(pHCI->pHCDData)
    {
        UHCI_HCData* pHCData = (UHCI_HCData*)pHCI->pHCDData;
        if(pHCData->wFrameListHandle)
            XMS_Free(pHCData->wFrameListHandle);
        if(pHCData->dwFrameListBase >= 0x100000L)
            DPMI_UnmapMemory(pHCData->dwFrameListBase);
        DPMI_DMAFree(pHCI->pHCDData);
    }
    pHCI->pHCDData = NULL;
    pHCI->dwBaseAddress = 0;
    return TRUE;
}

BOOL UHCI_ISR(HCD_Interface* pHCI)
{
#if !UHCI_USE_INTERRUPT
    unused(pHCI);
    return FALSE;
#endif
    //_LOG("ISR ");
    uint32_t iobase = pHCI->dwBaseAddress;
    uint16_t status = inpw((uint16_t)(iobase + USBSTS));
    if(!(status&USBINTMASK))
        return FALSE;
    UHCI_EnableInterrupt(pHCI, FALSE);

    if(status&USBRESUMEDETECT) //TODO:
    {
        outpw((uint16_t)(iobase + USBSTS), USBRESUMEDETECT);
    }
    #if DEBUG && 0
    _LOG("UHCI HC status: %x ", inpw((uint16_t)(iobase + USBSTS)));
    #endif
    if(status&(USBINT|USBERRORINT))
    {
        //UHCI spec recommend process FRNUM-1 in frame list (interrupt happend on SOF when last frame processed),
        //but here we just iterate all
        //No DPC in DOS, process immediately
        UHCI_HCData* pHCData = (UHCI_HCData*)pHCI->pHCDData;
        UHCI_ISR_ProcessQH(pHCI, &pHCData->QH1ms, pHCData->InteruptTail[0]); //porcess 1ms interrupt queue
        UHCI_ISR_ProcessQH(pHCI, &pHCData->QH2ms, pHCData->InteruptTail[1]); //porcess 2ms interrupt queue
        UHCI_ISR_ProcessQH(pHCI, &pHCData->QH8ms, pHCData->InteruptTail[2]); //porcess 8ms interrupt queue
        UHCI_ISR_ProcessQH(pHCI, &pHCData->ControlQH, pHCData->ControlTail); //porcess control queue.
        UHCI_ISR_ProcessQH(pHCI, &pHCData->BulkQH, pHCData->BulkTail); //porcess bulk queue
        outpw((uint16_t)(iobase + USBSTS), status&(USBINT|USBERRORINT));
    }

    if(status&(USBHSERROR|USBHCERROR))
    {
        //UHCI_ResetHC(pHCI);
        outpw((uint16_t)(iobase + USBSTS), status&(USBHSERROR|USBHCERROR));
        return TRUE;
    }
    //if(status&USBHCHALTED)
    //    UHCI_StartHC(pHCI);
    UHCI_EnableInterrupt(pHCI, TRUE);
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

    uint8_t bAddress = pDevice->bAddress;
    uint8_t bEndpoint = UHCI_ED_GETADDR(pEndpoint);
    uint16_t MaxLength = pQH->Flags.wMaxPacketSize;
    HCD_Request* pRequest = HCD_AddRequest(pDevice, pEndpoint, dir, pSetupData, length, bEndpoint, pCB, pCBData);
    uint32_t CS = CS_ActiveStatus | CS_C_ERR3 | (pDevice->bSpeed == USB_PORT_Low_Speed_Device ? CS_LowSpeed : 0);
    uint8_t DataPID = (dir == HCD_TXW) ? OUTPID : INPID;
    uint8_t StatusPID = (dir == HCD_TXW) ? INPID : OUTPID; //usb1.1 spec, status inverts direction of data
    assert(sizeof(UHCI_TD) <= 32);
    UHCI_TD* pEnd = (UHCI_TD*)USB_TAlloc32(sizeof(UHCI_TD));
    memset(pEnd, 0, sizeof(UHCI_TD));
    //pEnd->LinkPointer = TerminateFlag;
    pEnd->PAddr = DPMI_PTR2P(pEnd);

    // build status TD
    UHCI_TD* pStatusTD = (UHCI_TD*)USB_TAlloc32(sizeof(UHCI_TD));
    memset(pStatusTD,0,sizeof(UHCI_TD));
    UHCI_BuildTD(pStatusTD, pEnd, CS | CS_IOC, StatusPID, bAddress, bEndpoint, 1, 0, 0); //status data toggle always 1
    pStatusTD->pRequest = pRequest;

    // build data TD
    UHCI_TD* pDataTD = NULL;
    if(length)
    {
        uint16_t Transferred = 0;
        uint8_t DataToggle = 1;
        assert(pSetupData);
        uint32_t SetupDataAddr = DPMI_PTR2P(pSetupData);
        UHCI_TD* pNext = (UHCI_TD*)USB_TAlloc32(sizeof(UHCI_TD));
        memset(pNext,0,sizeof(UHCI_TD));

        while(Transferred < length)
        {
            uint16_t DataLength = min(MaxLength, (uint16_t)(length - Transferred));
            //_LOG("UHCI Control Data length: %d\n", DataLength);
            UHCI_TD* pTD = pNext;
            pNext = (Transferred + DataLength == length) ? pStatusTD : (UHCI_TD*)USB_TAlloc32(sizeof(UHCI_TD));
            if(pNext != pStatusTD) memset(pNext, 0, sizeof(UHCI_TD));
            UHCI_BuildTD(pTD, pNext, CS, DataPID, bAddress, bEndpoint, DataToggle, DataLength, SetupDataAddr + Transferred);
            pTD->pRequest = pRequest;
            DataToggle ^= 1;

            Transferred = (uint16_t)(Transferred + DataLength);
            if(!pDataTD)
                pDataTD = pTD;
        }
    }

    // build setup TD
    CLIS();
    UHCI_TD* pSetupTD = pQH->pTail;
    assert(pSetupTD->pPrev == NULL);
    UHCI_BuildTD(pSetupTD, length ? pDataTD : pStatusTD, CS&~CS_ActiveStatus, SETUPPID, bAddress, bEndpoint, 0, 8, DPMI_PTR2P(setup8)); //setup data toggle always 0
    pSetupTD->pRequest = pRequest;
    pQH->pTail = pEnd;
    #if DEBUG && 0//&& UHCI_USE_INTERRUPT
    uint16_t status = inpw((uint16_t)(pDevice->pHCI->dwBaseAddress + USBSTS));
    _LOG("UHCI HC status: %x\n", status);
    _LOG("UHCI port status: %04x\n", inpw((uint16_t)(pDevice->pHCI->dwBaseAddress + PORTSC + pDevice->bHubPort * 2U)));
    assert(!(status & USBHCHALTED));
    #endif
    // start transfer
    pSetupTD->ControlStatusBits.Active = 1;
    uint8_t error = 0;
    #if !UHCI_USE_INTERRUPT
    if(!UHCI_WaitTDDone(pSetupTD) || !UHCI_WaitTDDone(pStatusTD))
    {
        #if DEBUG
        _LOG("setup transfer failed.\n"); //1st get confiure may fail but it's OK.
        UHCI_TD* pTD = pSetupTD;
        while(pTD != pEnd)
        {
            printf("%08lx: ", DPMI_PTR2P(pTD)); DBG_DumpD((uint32_t*)pTD, 4, NULL);
            pTD = pTD->pNext;
        }
        uint16_t status = inpw((uint16_t)(pDevice->pHCI->dwBaseAddress + USBSTS));
        _LOG("UHCI HC status: %x\n", status);
        _LOG("UHCI port status: %04x\n", inpw((uint16_t)(pDevice->pHCI->dwBaseAddress + PORTSC + pDevice->bHubPort * 2U)));
        //_LOG("UHCI frame num: %d\n", inpw((uint16_t)(pDevice->pHCI->dwBaseAddress + FRNUM)));
        #endif
        //no need to do this. the only case is 1st get configure but followed by 2nd reset immediately
        //pQH->ElementLink = pStatusTD->PAddr; //finish status if requested buffer is larger than device responded
        //if(!UHCI_WaitTDDone(pStatusTD))
        {
            //_LOG("setup status failed\n");
            pQH->ElementLink = pEnd->PAddr | DepthSelect; //link to new tail if halt in the middle
        }
        error = 0x80;
    }
    uint16_t len = 0;
    _LOG("UHCI control transfer done.\n");
    while(pDataTD != NULL && pDataTD != pStatusTD)
    {
        len = (uint16_t)(len + pDataTD->ControlStatusBits.ActualLen + 1); //actual len is transferred len-1
        error |= (uint8_t)((pDataTD->ControlStatus&CS_ErrorMask)>>CS_ErrorShift);
        UHCI_TD* pTD = pDataTD->pNext;
        USB_TFree32(pDataTD);
        pDataTD = pTD;
    }
    error |= (uint8_t)((pStatusTD->ControlStatus&CS_ErrorMask)>>CS_ErrorShift);
    USB_TFree32(pStatusTD);
    USB_TFree32(pSetupTD);
    pEnd->pPrev = NULL;
    //_LOG("ERROR: %x\n",error);
    HCD_InvokeCallBack(pRequest, len, error);
    #endif
    STIL();
    return error;
}

//TODO: ISO transfer uses only TDs
uint8_t UHCI_DataTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t* inoutp pBuffer,
    uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData)
{
    UHCI_QH* pQH = UHCI_ED_GETQH(pEndpoint);
    if(pCB == NULL || pQH == NULL || pQH->Flags.Dir != dir || pQH->Flags.Type == USB_ENDPOINT_TRANSFER_TYPE_CTRL || pBuffer == NULL || length == 0)
        return 0xFF;

    uint8_t USBAddress = pDevice->bAddress;
    uint8_t bEndpoint = UHCI_ED_GETADDR(pEndpoint);
    uint8_t PID = (dir == HCD_TXW) ? OUTPID : INPID;
    assert(pQH->Flags.wMaxPacketSize);
    uint16_t MaxLen = (uint16_t)max(pQH->Flags.wMaxPacketSize, (1023/pQH->Flags.wMaxPacketSize-1)*pQH->Flags.wMaxPacketSize); //USB1.1 spec 0~1023 bytes data packet
    //_LOG("EPADDR: %x, MAXLEN: %d\n", bEndpoint, MaxLen);
    uint8_t Toggle = pQH->Flags.DataToggle;
    HCD_Request* pRequest = HCD_AddRequest(pDevice, pEndpoint, dir, pBuffer, length, bEndpoint, pCB, pCBData);
    uint32_t pbuffer = DPMI_PTR2P(pBuffer);
    UHCI_TD* pHead = pQH->pTail;
    UHCI_TD* pEnd = NULL;

    uint32_t CS = CS_C_ERR1 | (pDevice->bSpeed == USB_PORT_Low_Speed_Device ? CS_LowSpeed : 0) | ((pQH->Flags.Type == USB_ENDPOINT_TRANSFER_TYPE_ISOC) ? CS_IOS : 0);
    uint16_t transferred = 0;
    CLIS();
    while(transferred < length)
    {
        uint16_t PacketLength = min((uint16_t)(length - transferred), MaxLen);
        //Build TD with active state to start transfer
        UHCI_TD* pNewTail = (UHCI_TD*)USB_TAlloc32(sizeof(UHCI_TD));
        memset(pNewTail, 0, sizeof(UHCI_TD));
        uint32_t cs = CS | ((transferred + PacketLength == length) ? CS_IOC : 0) | (transferred != 0 ? CS_ActiveStatus : 0);
        cs = cs | ((transferred != 0) ? CS_ActiveStatus : 0);
        UHCI_BuildTD(pQH->pTail, pNewTail, cs, PID, USBAddress, bEndpoint, Toggle, PacketLength, pbuffer + transferred);
        pQH->pTail->pRequest = pRequest;
        #if DEBUG && 0
        DBG_DumpB((uint8_t*)pQH->pTail, 16, NULL);
        #endif
        Toggle ^=1;
        transferred = (uint16_t)(transferred + PacketLength);
        pEnd = pQH->pTail;
        pQH->pTail = pNewTail;
    }
    //start transfer
    pHead->ControlStatusBits.Active = 1;

    uint8_t error = 0;
    #if !UHCI_USE_INTERRUPT
    if(!UHCI_WaitTDDone(pEnd))
        _LOG("data transfer failed\n");
    uint16_t len = 0;
    while(pHead != pQH->pTail)
    {
        len = (uint16_t)(len + pHead->ControlStatusBits.ActualLen + 1);
        error |= (uint8_t)((pHead->ControlStatus&CS_ErrorMask)>>CS_ErrorShift);
        UHCI_TD* pTD = pHead->pNext;
        USB_TFree32(pHead);
        pHead = pTD;
    }
    HCD_InvokeCallBack(pRequest, len, error);
    #else
    unused(pEnd);
    #endif
    STIL();

    pQH->Flags.DataToggle = Toggle&0x1U;
    return error;
}

void UHCI_ISR_ProcessQH(HCD_Interface* pHCI, UHCI_QH* pQH, UHCI_QH* pEnd)
{
    assert(pQH != NULL && pEnd != NULL);
    while(pQH != pEnd)
    {
        UHCI_ISR_ProcessTD(pHCI, pQH);
        pQH = (UHCI_QH*)DPMI_P2PTR(pQH->HeadLink&~0xFUL);
        assert(pQH != NULL);
    }
    UHCI_ISR_ProcessTD(pHCI, pQH); //[pQH, pEnd]
}

void UHCI_ISR_ProcessTD(HCD_Interface* pHCI, UHCI_QH* pQH)
{
#if UHCI_USE_INTERRUPT
    // detach all inactive TD first, then issue callback, to prevent extra transferr happen in callback and break the in-processing list
    //assert(pQH->pTail);
    UHCI_TD* pTD = (pQH->ElementLink&~0xFUL) ? (UHCI_TD*)DPMI_P2PTR(pQH->ElementLink&~0xFUL) : NULL; //pQH->pTail;
    UHCI_TD* pList = NULL;
    HCD_Request* pReq = NULL;
    uint8_t error = 0;
    while(pTD && pTD->pPrev) pTD = pTD->pPrev; //find head TD

    while(pTD && pTD != pQH->pTail)
    {
        assert(pTD->pRequest);
        UHCI_TD* pNext = pTD->pNext;
        error |= (uint8_t)((pTD->ControlStatus&CS_ErrorMask)>>CS_ErrorShift);
        if(!pTD->ControlStatusBits.Active || (error && pTD->pRequest == pReq)) //stop after first inactive request
        {
            assert(pTD->pPrev == NULL); //prev should be inactive in queue
            //remove from list
            if(pNext)
                pNext->pPrev = pTD->pPrev;
            //pTD->pPrev = NULL;
            pTD->pNext = pList;
            pList = pTD;
            pReq = pTD->pRequest;
            pTD = pNext;
        }
        else
            /*pTD = pNext;*/break;
    }
    if(pTD && error)
    {
        assert(!pTD->ControlStatusBits.Active && pTD == pQH->pTail || pTD->ControlStatusBits.Active); //TODO:
        assert(pTD->PAddr);
        pQH->ElementLink = pTD->PAddr | DepthSelect;
    }
    //assert(pQH->pTail == NULL || pQH->pTail->pPrev == NULL); //TODO:

    while(pList) //release TD in reversed order
    {
        pTD = pList;
        pList = pList->pNext;
        if((pTD->ControlStatusBits.Interrupt || (pTD->ControlStatus&CS_ErrorMask))
            && !pTD->ControlStatusBits.Active)
        {
            uint8_t error = (pTD->ControlStatus&CS_ErrorMask)>>CS_ErrorShift;
            if(error)
                _LOG("UHCI CB with error: %02x ", error);
            uint32_t BufferBegin = pTD->pRequest->pBuffer ? DPMI_PTR2P(pTD->pRequest->pBuffer) : pTD->BufferPointer;
            HCD_InvokeCallBack(pTD->pRequest, (uint16_t)(pTD->BufferPointer - BufferBegin + pTD->ControlStatusBits.ActualLen + 1), error);
        }
        //_LOG("Free TD: %x ", pTD);
        assert(pTD != pQH->pTail);
        USB_TFree32(pTD);
    }
#else
    unused(pQH);
#endif// UHCI_USE_INTERRUPT
    unused(pHCI);
    //_LOG("END");
}

uint16_t UHCI_GetPortStatus(HCD_Interface* pHCI, uint8_t port)
{
    uint16_t status = 0;
    uint16_t portbase = (uint16_t)(pHCI->dwBaseAddress + PORTSC + port * 2U);
    uint16_t portsc = inpw(portbase);

    _LOG("UCHI port %d status: %04x\n", port, PORTSC);

    if(portsc & CCS)
        status |= USB_PORT_ATTACHED;

    if(portsc & LSDA)
        status |= USB_PORT_Low_Speed_Device ; // 01 = low speed device.
    else
        status |= USB_PORT_Full_Speed_Device; // 2 = full speed

    if(portsc & PED)
        status |= USB_PORT_ENABLE;
    else
        status |= USB_PORT_DISABLE;

    if(portsc & SUSPEND)
        status |= USB_PORT_SUSPEND;

    if(portsc & PR)
        status |= USB_PORT_RESET;   //resetting

    if(portsc & CSC)
        status |= USB_PORT_CONNECT_CHANGE;
    return status;
}

BOOL UHCI_SetPortStatus(HCD_Interface* pHCI, uint8_t port, uint16_t status)
{
    uint16_t portbase = (uint16_t)(pHCI->dwBaseAddress + PORTSC + port * 2U);
    uint16_t portsc = inpw(portbase);

    if((status&USB_PORT_RESET))
    {
        const int timeout = 100; //5sec timeout
        outpw(portbase, PR);
        int i = 0;
        do { ++i; delay(5); } while(!(inpw(portbase)&PR) && i < timeout); //wait until reset signal actually applies
        if(i == timeout)
        {
            assert(FALSE);
            return FALSE;
        }

        delay(55);// spec require keeping reset signal at least 10ms, 50+ms get more compatibility

        outpw(portbase, 0);//inpw(portbase)&~PR); //release reset signal

        i = 0;
        do { delay(5); ++i; } while((inpw(portbase)&PR) && i < timeout*50); //wait until reset actually done.
        if(i == timeout)
        {
            assert(FALSE);
            return FALSE;
        }
        delay(15); //USB spec require device to be ready in 10ms after reset.
        outpw(portbase, PEDC|CSC);
        portsc = 0; //initially disabled after reset. clear enable in case reset & enable in one call
    }

    if((status&USB_PORT_ENABLE) && !(portsc&PED))
    {
        outpw(portbase, PED);
        do { delay(1); } while(!(inpw(portbase)&PED));
        _LOG("UHCI port %d status: %04x\n", port, inpw(portbase));
        outpw(portbase, CSC|PEDC|PED);
        delay(55);
    }

    if((status&USB_PORT_DISABLE) && (portsc&PED))
    {
        outpw(portbase, 0);//portsc & ((uint16_t)~PED));
        do { delay(1); } while((inpw(portbase)&PED));
    }

    if((status&USB_PORT_SUSPEND) && !(portsc&SUSPEND))
    {
        do { outpw(portbase, SUSPEND); delay(1); } while(!(inpw(portbase)&SUSPEND));
    }

    if((status&USB_PORT_CONNECT_CHANGE))
    {
        outpw(portbase, CSC); //clear connect status change
        do { delay(10); } while(inpw(portbase)&CSC);
        delay(150);
    }
    return TRUE;
}

BOOL UHCI_InitDevice(HCD_Device* pDevice)
{
    pDevice->pHCData = DPMI_DMAMalloc(sizeof(UHCI_HCDeviceData), 16);
    UHCI_HCDeviceData* pDeviceData = (UHCI_HCDeviceData*)pDevice->pHCData;
    memset(pDeviceData, 0, sizeof(UHCI_HCDeviceData));
    
    UHCI_HCData* pHCData = (UHCI_HCData*)pDevice->pHCI->pHCDData;
    UHCI_InitQH(&pDeviceData->ControlQH);
    //_LOG("%d\n",sizeof(UHCI_TD));
    //assert(sizeof(UHCI_TD) <= 32);
    UHCI_TD* pTD = (UHCI_TD*)USB_TAlloc32(sizeof(UHCI_TD));
    memset(pTD, 0, sizeof(UHCI_TD));
    UHCI_InsertTDintoQH(&pDeviceData->ControlQH, pTD);
    UHCI_InsertQHintoQH(pHCData->ControlTail, &pDeviceData->ControlQH);
    pHCData->ControlTail = &pDeviceData->ControlQH;
    return TRUE;
}

BOOL UHCI_RemoveDevice(HCD_Device* pDevice)
{
    if(!HCD_IS_DEVICE_VALID(pDevice))
        return FALSE;
    UHCI_HCDeviceData* pDeviceData = (UHCI_HCDeviceData*)pDevice->pHCData;
    UHCI_HCData* pHCData = (UHCI_HCData*)pDevice->pHCI->pHCDData;
    UHCI_RemoveQHfromQH(&pHCData->ControlQH, &pHCData->ControlTail, &pDeviceData->ControlQH);
    USB_TFree32(pDeviceData->ControlQH.pTail);
    DPMI_DMAFree(pDeviceData);
    return TRUE;
}

void* UHCI_CreateEndpoint(HCD_Device* pDevice, uint8_t EPAddr, HCD_TxDir dir, uint8_t bTransferType, uint16_t MaxPacketSize, uint8_t bInterval)
{
    UHCI_HCDeviceData* pDeviceData = (UHCI_HCDeviceData*)pDevice->pHCData;
    UHCI_HCData* pHCData = (UHCI_HCData*)pDevice->pHCI->pHCDData;
    if(EPAddr == 0) //default control pipe
    {
        pDeviceData->ControlQH.Flags.wMaxPacketSize = MaxPacketSize&0x7FFU;
        return &pDeviceData->ControlQH;
    }
    assert(EPAddr <= 0xF);
    assert(bTransferType != USB_ENDPOINT_TRANSFER_TYPE_INTR || bInterval >= 1);
    //UHCI_StopHC(pDevice->pHCI);
    UHCI_EnableInterrupt(pDevice->pHCI, FALSE);

    UHCI_QH* pQH = (UHCI_QH*)DPMI_DMAMalloc(sizeof(UHCI_QH), 16);
    UHCI_InitQH(pQH);
    pQH->Flags.Dir = dir&0x1;
    pQH->Flags.Type = bTransferType&0x3U;
    pQH->Flags.wMaxPacketSize = MaxPacketSize&0x7FFU;

    UHCI_TD* pTD = (UHCI_TD*)USB_TAlloc32(sizeof(UHCI_TD));
    memset(pTD, 0, sizeof(UHCI_TD));
    //pTD->LinkPointer = TerminateFlag;
    UHCI_InsertTDintoQH(pQH, pTD);

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
    //UHCI_StartHC(pDevice->pHCI);
    UHCI_EnableInterrupt(pDevice->pHCI, TRUE);
    return (void*)(((uintptr_t)pQH) | EPAddr);
}

BOOL UHCI_RemoveEndpoint(HCD_Device* pDevice, void* pEndpoint)
{
    UHCI_QH* pQH = UHCI_ED_GETQH(pEndpoint);
    if(pDevice == NULL || pQH == NULL)
        return FALSE;
    UHCI_HCDeviceData* pDeviceData = (UHCI_HCDeviceData*)pDevice->pHCData;
    if(pDeviceData == NULL)
        return FALSE;
    if(&pDeviceData->ControlQH == pEndpoint) //default control pipe
        return TRUE;
    //UHCI_StopHC(pDevice->pHCI);
    UHCI_EnableInterrupt(pDevice->pHCI, FALSE);

    BOOL result = FALSE;
    UHCI_TD* pTail = pQH->pTail;
    UHCI_HCData* pHCData = (UHCI_HCData*)pDevice->pHCI->pHCDData;
    uint8_t bTransferType = pQH->Flags.Type;

    _LOG("UHCI remove endpoint QH\n");
    if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL)
        result = UHCI_RemoveQHfromQH(&pHCData->ControlQH, &pHCData->ControlTail, pQH);
    if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_ISOC)
        result = UHCI_RemoveQHfromQH(&pHCData->QH1ms, &pHCData->InteruptTail[0], pQH);
    else if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK)
        result = UHCI_RemoveQHfromQH(&pHCData->BulkQH, &pHCData->BulkTail, pQH);
    else if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_INTR)
    {
        assert(pQH->Flags.Interval <= 2);
        UHCI_QH* IntQH = &pHCData->QH1ms + pQH->Flags.Interval;
        UHCI_QH** IntQHTail = &pHCData->InteruptTail[pQH->Flags.Interval];
        result = UHCI_RemoveQHfromQH(IntQH, IntQHTail, pQH);
    }
    _LOG("UHCI remove endpoint TD\n");
    while(pTail != NULL) //remove unfinished TD
    {
        UHCI_TD* pPrev = pTail->pPrev;
        USB_TFree32(pTail);
        pTail = pPrev;
    }
    DPMI_DMAFree(pQH);
    //UHCI_StartHC(pDevice->pHCI);
    UHCI_EnableInterrupt(pDevice->pHCI, TRUE);
    return result;
}

void UHCI_EnableInterrupt(HCD_Interface* pHCI, BOOL enable)
{
    outpw((uint16_t)(pHCI->dwBaseAddress + USBINTR), UHCI_USE_INTERRUPT && enable ? 0xF : 0); //USBINTR is 16 bit but only 4 bit is used
}

void UHCI_QHTDSchedule(HCD_Interface* pHCI)
{
    UHCI_HCData* pHCData = (UHCI_HCData*)pHCI->pHCDData;
    uint32_t FramelistAddr = pHCData->dwFrameListBase;

    UHCI_QH* pQH = &pHCData->QH1ms;
    //set QH invalid accord to UHCIDescriptor definition.
    unsigned int i;
    for(i = 0; i < 3; i++)
        UHCI_InitQH(&pQH[i]);
    //clear frame list
    for(i = 0; i < 1024; i++)
        DPMI_StoreD(FramelistAddr + i * 4U, TerminateFlag);
    //link 8ms qh
    for(i = 0; i < 1024; i = i+8)
        DPMI_StoreD(FramelistAddr + i * 4U, DPMI_PTR2P(&pHCData->QH8ms) | QHFlag);
    //link 2ms qh
    for(i = 1; i < 1024; i = i+2)
        DPMI_StoreD(FramelistAddr + i * 4U, DPMI_PTR2P(&pHCData->QH2ms) | QHFlag);

    //link 1ms qh. UHCI frame time is 1ms and 1ms QH should apear on all entries in frame list
    UHCI_InsertQHintoQH(&pHCData->QH8ms, &pHCData->QH1ms);
    UHCI_InsertQHintoQH(&pHCData->QH2ms, &pHCData->QH1ms);
    for(i = 0; i < 1024; i++)
    {
        if(DPMI_LoadD(FramelistAddr + i * 4U) == TerminateFlag)
            DPMI_StoreD(FramelistAddr + i * 4U, DPMI_PTR2P(&pHCData->QH1ms) | QHFlag);
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
    assert(pQH);
    memset(pQH, 0, sizeof(UHCI_QH));
    pQH->HeadLink = QHFlag | TerminateFlag;
    pQH->ElementLink = TerminateFlag;
}

void UHCI_BuildTD(UHCI_TD* pTD, UHCI_TD* pNext, uint32_t  ControlStatus, uint8_t PID, uint8_t DevAddr, uint8_t EndPt, uint8_t DataToggle, uint16_t MaxLen, uint32_t buffer)
{
    //memset(pTD, 0, sizeof(pTD));
    pTD->LinkPointer = pNext ? ((pNext->PAddr ? pNext->PAddr : DPMI_PTR2P(pNext)) | DepthSelect) : TerminateFlag;
    pTD->ControlStatus = ControlStatus;
    pTD->TokenBits.MaxLen = (uint16_t)(MaxLen-1)&0x7FFU;
    assert(MaxLen != 0 || pTD->TokenBits.MaxLen == TK_NullLength);
    assert(MaxLen == 0 || pTD->TokenBits.MaxLen < 0x500);    //500~7FE invalid (max 1280 bytes)
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

void UHCI_InsertTDintoQH(UHCI_QH* pQH, UHCI_TD* pTD)
{
    //_LOG("%x %x %x %x", pQH, pTD, pQH->pTail, (pQH->ElementLink&~0xFUL));
    assert(pQH && pTD && pQH->pTail == NULL && (pQH->ElementLink&~0xFUL) == 0); //only used by init
    pQH->pTail = pTD;
    pQH->ElementLink = pTD ? (DPMI_PTR2P(pTD) | DepthSelect) : TerminateFlag;
}

void UHCI_InsertQHintoQH(UHCI_QH* pToQH, UHCI_QH* pQH)
{
    assert(pQH && pToQH && pQH != pToQH);
    uint32_t paddr = DPMI_PTR2P(pQH);
    uint32_t next = pToQH->HeadLink;
    assert((next&~0xFUL) != paddr);
    pQH->HeadLink = next;
    pToQH->HeadLink = paddr | QHFlag;
    return;
}

BOOL UHCI_RemoveQHfromQH(UHCI_QH* pFromQH, UHCI_QH** pEndQH, UHCI_QH* pQH)
{
    assert(pFromQH && pEndQH && pQH);
    UHCI_QH* qh = pFromQH;
    UHCI_QH* prevQH = NULL;
    while(qh != *pEndQH && qh != pQH)
    {
        prevQH = qh;
        qh = (qh->HeadLink&~0xFUL) ? (UHCI_QH*)DPMI_P2PTR(qh->HeadLink&~0xFUL) : NULL;
        assert(qh != NULL);
    }
    if(qh != pQH)
    {
        assert(FALSE);
        return FALSE;
    }
    prevQH->HeadLink = qh->HeadLink;
    if(*pEndQH == pQH)
        *pEndQH = prevQH;
    return TRUE;
}

#if !UHCI_USE_INTERRUPT
int UHCI_WaitTDDone(UHCI_TD* pTD)
{
    int i = 0;
    int WaitTime = 500;
    for(;i < WaitTime; i++)
    {
        if(pTD->ControlStatusBits.Active) delay(1);
        else break;
    }
    _LOG("UHCI_WaitTDDone: %d\n",i);
    if(i >= WaitTime) return 0;  //error.
    else return 1;
}
#endif

void UHCI_ResetHC(HCD_Interface* pHCI)
{
    outpw((uint16_t)(pHCI->dwBaseAddress + USBCMD), HCRESET | GRESET);
    delay(100);     //spec required
    outpw((uint16_t)(pHCI->dwBaseAddress + USBCMD), 0); //stopped
    delay(100);
    return;
}

void UHCI_StopHC(HCD_Interface* pHCI)
{
    uint16_t cmd = inpw((uint16_t)(pHCI->dwBaseAddress + USBCMD));
    outpw((uint16_t)(pHCI->dwBaseAddress + USBCMD), (uint16_t)(cmd&~RS));
    delay(10);
    return;
}

void UHCI_StartHC(HCD_Interface* pHCI)
{
    uint16_t cmd = inpw((uint16_t)(pHCI->dwBaseAddress + USBCMD));
    _LOG("UHCI start HC cmd: %x\n", cmd);
    outpw((uint16_t)(pHCI->dwBaseAddress + USBCMD), (uint16_t)(cmd&~(EGSM|GRESET)));
    delay(10);
    outpw((uint16_t)(pHCI->dwBaseAddress + USBCMD), RS); //run
    uint16_t status = inpw((uint16_t)(pHCI->dwBaseAddress + USBSTS));
    _LOG("UHCI start HC status: %x\n", status); unused(status);
    assert(!(status & USBHCHALTED));
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
