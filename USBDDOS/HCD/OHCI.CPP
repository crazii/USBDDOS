#include <dos.h>
#include <stdio.h>
#include <memory.h>
#include <assert.h>
#include <string.h>
#include "USBDDOS/HCD/OHCI.H"
#include "USBDDOS/DPMI/DPMI.H"
#include "USBDDOS/DBGUTIL.H"

//ref: OHCI_Specification_Rev.1.0a

#define TIME_OUT 500L

OHCI_TD* OHCI_TD::NULLTD = NULL;

static uint8_t* OHCI_GetHCDBuffer(HCD_Device* pDevice);
static uint16_t OHCI_GetPortStatus(HCD_Interface* pHCI, uint8_t port);
static BOOL OHCI_SetPortStatus(HCD_Interface* pHCI, uint8_t port, uint16_t status);
static BOOL OHCI_InitDevice(HCD_Device* pDevice);
static BOOL OHCI_CreateEndpoint(HCD_Device* pDevice, uint8_t EPAddr, HCD_TxDir dir, uint8_t bTransferType, uint16_t MaxPacketSize, uint8_t bInterval);
static BOOL OHCI_RemoveDevice(HCD_Device* pDevice);

static OHCI_ED* OHCI_FindEnpoint(HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir);
static BOOL OHCI_RemoveEndpoints(HCD_Device* pDevice);

static void OHCI_AddToTail(OHCI_ED*& pTail, OHCI_ED* pED);

static void OHCI_BuildControl(HCD_Device* pDevice, OHCI_ED* pControlED, OHCI_TD* pSetup, OHCI_TD* pData, OHCI_TD* pStatus);

static void OHCI_BuildTD(OHCI_TD* pTD, uint8_t PID, uint8_t DelayInterrupt, uint8_t DataToggle, uint32_t BufferAddr, uint32_t length,
    OHCI_TD* pNextTD);
static void OHCI_UpdateTD(OHCI_TD* pTD, uint32_t BufferAddr, uint32_t length, OHCI_TD* pNext = NULL);

static void OHCI_BuildIsoTD(OHCI_ISO_TD* pTD, uint16_t StartFrame, uint8_t FrameCount, uint8_t DelayInterrupt, uint16_t PacketSize, uint32_t BufferAddr,
    uint32_t length, OHCI_ISO_TD* pNextTD);
static void OHCI_UpdateIsoTD(OHCI_ISO_TD* pTD, uint16_t StartFrame, uint8_t FrameCount, uint16_t PacketSize, uint32_t BufferAddr, uint32_t length);

static void OHCI_BuildED(OHCI_ED* pED, uint8_t FunctionAddress, uint8_t EndpointNo, uint8_t Direction, uint8_t LowSpeed, uint16_t MaxPackSize,
    OHCI_TD* TailP, OHCI_TD* HeadP);
static void OHCI_BuildIsoED(OHCI_ED* pED, uint8_t FunctionAddress, uint8_t EndpointNo, uint8_t Direction, uint8_t LowSpeed, uint16_t MaxPackSize,
    OHCI_ISO_TD* TailP, OHCI_ISO_TD* HeadP);

static void OHCI_BuildHCCA(OHCI_HCData * pHCDD);

static OHCI_ED** OHCI_GetEDFromInterval(OHCI_HCData* pHCDD, uint8_t interval);

HCD_Method OHCI_Method =
{
    &OHCI_ControlTransfer,
    &OHCI_IsochronousTransfer,
    &OHCI_DataTransfer,
    &OHCI_DataTransfer,
    &OHCI_GetHCDBuffer,
    &OHCI_GetPortStatus,
    &OHCI_SetPortStatus,
    &OHCI_InitDevice,
    &OHCI_CreateEndpoint,
    &OHCI_RemoveDevice,
};

BOOL OHCI_InitController(HCD_Interface* pHCI)
{
#if defined(__BC__) || defined(__WC__)
    assert(sizeof(OHCI_TD)%16 == 0);
    assert(sizeof(OHCI_ISO_TD)%16 == 0);
    assert(sizeof(OHCI_ED)%16 == 0);
#endif
    //static
    if(OHCI_TD::NULLTD == NULL)
    {
        OHCI_TD::NULLTD = (OHCI_TD*)DPMI_MappedMalloc(sizeof(OHCI_TD), 16);
        memset(OHCI_TD::NULLTD, 0, sizeof(sizeof(OHCI_TD)));
        OHCI_TD::NULLTD->PhysicalAddr = DPMI_PTR2P(OHCI_TD::NULLTD);
    }

    //enable MMIO
    PCI_Write_Word(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, pHCI->PCIAddr.Function, OCH_REGISTER_CMD, 0x0006); //bit 0: IO space - disabled, bit 2: memory space - enabled
    pHCI->dwPhysicalAddress = PCI_Read_DWord(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, pHCI->PCIAddr.Function, OHCI_REGISTER_BAR) & 0xffffffe0;
    //map to linear
    uint32_t size = PCI_Sizing(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, pHCI->PCIAddr.Function, OHCI_REGISTER_BAR);// should be 4K according to the spec
    pHCI->dwBaseAddress = DPMI_MapMemory(pHCI->dwPhysicalAddress, size);
    _LOG("OHCI BAR address: %08lx, mapped address %08lx, size: %08lx\n", pHCI->dwPhysicalAddress, pHCI->dwBaseAddress, size);

    uint32_t dwBase = pHCI->dwBaseAddress;
    // ownership handoff
    DPMI_MaskD(dwBase + HcCommandStatus, ~0L, OwnershipChangeRequest);
    {
        uint32_t SMMDriver = DPMI_LoadD(dwBase + HcControl) & InterruptRouting;
        if(!SMMDriver)
        {
            uint32_t HCFS = (DPMI_LoadD(dwBase + HcCommandStatus) & HostControllerFunctionalState) >> HostControllerFunctionalState_SHIFT;
            if(HCFS != USBRESET && HCFS != USBOPERATIONAL)
            {
                DPMI_MaskD(dwBase + HcCommandStatus, ~HostControllerFunctionalState, USBRESUME << HostControllerFunctionalState_SHIFT);
                delay(50);
            }
        }
        else
        {
            while (DPMI_LoadD(dwBase + HcControl) & InterruptRouting) //wait SMM driver
                delay(50);
        }
    }
    uint32_t FrameInterval = DPMI_LoadD(dwBase + HcFmInterval);
    DPMI_MaskD(dwBase + HcCommandStatus, ~0L, HostControllerReset);
    delay(1);
    DPMI_StoreD(dwBase + HcFmInterval, FrameInterval);

    //get ports
    pHCI->bNumPorts = DPMI_LoadB(dwBase + HcRhDescriptorA);//low 8bit of a register
    //_LOG("Numports: %d\n", pHCI->bNumPorts);

    pHCI->pHCDMethod = &OHCI_Method;
    pHCI->pHCDData = DPMI_MappedMalloc(sizeof(OHCI_HCData), 256);
    OHCI_HCData* pHCDD = (OHCI_HCData*)pHCI->pHCDData;
    memset(pHCDD, 0, sizeof(OHCI_HCData));

    //port power on
    if(DPMI_LoadD(dwBase + HcRhDescriptorA) & PowerSwitchingMode)
    { //per-port power
        for(uint32_t i = 0; i < pHCI->bNumPorts; i++)
            DPMI_MaskD(dwBase + HcRhPort1Status + i * 4, ~0L, SetPortPower);
    }
    else
    { //all-port power
        DPMI_MaskD(dwBase + HcRhStatus, ~0L, SetGlobalPower);
    }
    //hcca & queue heads
    OHCI_BuildHCCA(pHCDD);
    DPMI_StoreD(dwBase + HcHCCA, DPMI_PTR2P(&pHCDD->HCCA));
    //_LOG("HCCA: %04x %08lx\n", &pHCDD->HCCA, DPMI_PTR2P(&pHCDD->HCCA));
    assert((DPMI_PTR2P(&pHCDD->HCCA)&0xFF) == 0);//ensure alignment

    pHCDD->ControlHead.Control.Skip = 1;
    pHCDD->ControlHead.PhysicalAddr = DPMI_PTR2P(&pHCDD->ControlHead);
    DPMI_StoreD(dwBase + HcControlHeadED, pHCDD->ControlHead.PhysicalAddr);

    pHCDD->BulkHead.Control.Skip = 1;
    pHCDD->BulkHead.PhysicalAddr = DPMI_PTR2P(&pHCDD->BulkHead);
    DPMI_StoreD(dwBase + HcBulkHeadED, pHCDD->BulkHead.PhysicalAddr);

    //enable interrupt & lists
    DPMI_MaskD(dwBase + HcInterruptStatus, ~0L, 0x4000003FL);
    DPMI_StoreD(dwBase + HcInterruptEnable, ~StartofFrame);

    DPMI_MaskD(dwBase + HcControl, ~0L, ControlListEnable);
    DPMI_MaskD(dwBase + HcControl, ~0L, PeriodicListEnable);
    DPMI_MaskD(dwBase + HcControl, ~0L, IsochronousEnable);
    DPMI_MaskD(dwBase + HcControl, ~0L, BulkListEnable);

    DPMI_StoreD(dwBase + HcPeriodicStart, (FrameInterval&0x3FFF)*9/10); //spec requires 90% for interrupt/iso.

    //controller goto the USBOPERATIONAL state.
    DPMI_MaskD(dwBase + HcControl, ~HostControllerFunctionalState, (USBOPERATIONAL << HostControllerFunctionalState_SHIFT));
    DPMI_MaskD(dwBase + HcInterruptEnable, ~0L, StartofFrame); //we don't have ISR yet.

    pHCDD->ControlTail = &pHCDD->ControlHead;
    pHCDD->BulkTail = &pHCDD->BulkHead;
    pHCDD->ED32msTail = &pHCDD->ED32ms;
    pHCDD->ED16msTail = &pHCDD->ED16ms;
    pHCDD->ED8msTail = &pHCDD->ED8ms;
    pHCDD->ED4msTail = &pHCDD->ED4ms;
    pHCDD->ED2msTail = &pHCDD->ED2ms;
    pHCDD->ED1msTail = &pHCDD->ED1ms;
    return TRUE;
}

BOOL OHCI_DeinitController(HCD_Interface * pHCI)
{
    uint32_t dwBase = pHCI->dwBaseAddress;
    if(dwBase)
    {
        DPMI_MaskD(dwBase + HcCommandStatus, ~0L, HostControllerReset);
        delay(1);
    }
    if(pHCI->pHCDData)
        DPMI_MappedFree(pHCI->pHCDData);
    pHCI->pHCDData = NULL;
    return TRUE;
}

uint8_t OHCI_ControlTransfer(HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir, uint8_t outputp pSetup[8], void* nullable pSetupData, uint16_t length)
{
    OHCI_ED* pED = OHCI_FindEnpoint(pDevice, Endpoint, dir);
    if(!pED || pED->TransferType != USB_ENDPOINT_TRANSFER_TYPE_CTRL)
    {
        if(pED) printf("%d\n", pED->TransferType);
        return 0xFF;
    }

    OHCI_HCData* pHCDD = (OHCI_HCData*)pDevice->pHCI->pHCDData;
    OHCI_HCDeviceData* pDD = (OHCI_HCDeviceData*)pDevice->pHCData;

#if DEBUG
    //DBG_DumpReq(pReq);
    //_LOG("Control: Address:%d, Endpoint:%d, size:%d\n", pED->Control.FunctionAddress, pED->Control.EndPoint, pED->Control.MaxPacketSize);
    //printf("%08lx, %08lx\n", pED->pPrev->NextED, pED->PhysicalAddr&~0xF);
    //printf("%08lx\n",pED);
#endif
    assert(pED->Control.FunctionAddress == 0
    || (pDevice->bAddress&0x7F) == pED->Control.FunctionAddress);
    assert(pED->pPrev->NextED == (pED->PhysicalAddr&~0xF));
    memcpy(pDD->Setup, pSetup, 8);     //copy to data with physical mapping
    uint32_t dwIOBase = pDevice->pHCI->dwBaseAddress;

    OHCI_TD* ControlSetup = pED->pTD;
    OHCI_TD* ControlData = ControlSetup->pNext;
    OHCI_TD* ControlStatus = ControlData->pNext;
    assert(ControlSetup != ControlData && ControlSetup != ControlStatus && ControlData != ControlStatus);
    //TD queue: controlsetup => [contrldata =>] contrlstatus => null
    uint8_t pid = (dir == HCD_TXR) ? PIDIN : PIDOUT;
    //SETUP TD
    OHCI_UpdateTD(ControlSetup, DPMI_PTR2P(&pDD->Setup), 8, ControlData);
    //DATA TD
    OHCI_UpdateTD(ControlData, pSetupData ? DPMI_PTR2P(pSetupData) : 0, length, ControlStatus);
    //STATUS TD
    OHCI_UpdateTD(ControlStatus, 0, 0);
    ControlData->Control.PID = pid;
    ControlStatus->Control.PID = PIDINVERT(pid);    //status stage inverts direction of data stage according to the OHCI spec
    assert(ControlSetup->NextTD == ControlData->PhysicalAddr && ControlData->NextTD == ControlStatus->PhysicalAddr && ControlStatus->NextTD == OHCI_TD::NULLTD->PhysicalAddr);
    //start transfer
    assert(pED->TailP == OHCI_TD::NULLTD->PhysicalAddr && pED->HeadP == OHCI_TD::NULLTD->PhysicalAddr);
    assert(pED->pPrev->NextED == pED->PhysicalAddr);
    pED->HeadP = ControlSetup->PhysicalAddr;
#if DEBUG
    DBG_DBuff db = {1};
    DBG_Printf(&db, "ED TD before transation:\n");
    DBG_DumpPD(pED->PhysicalAddr, 4, &db);
    DBG_DumpPD(ControlSetup->PhysicalAddr, 4, &db);
    DBG_DumpPD(ControlData->PhysicalAddr, 4, &db);
    DBG_DumpPD(ControlStatus->PhysicalAddr, 4, &db);
#endif
    pED->Control.Skip = 0;
    DPMI_MaskD(dwIOBase + HcCommandStatus, ~0L, ControlListFilled);

    //wait to complete.
    uint8_t errorCode = 0;
    unsigned long int i;
    for(i = 0; i <TIME_OUT; ++i)
    {
        if((DPMI_LoadD(dwIOBase + HcInterruptStatus) & WriteBackDoneHead))
        {
            //DPMI_MaskD(dwIOBase + HcInterruptStatus, ~WriteBackDoneHead, 0);
            DPMI_MaskD(dwIOBase + HcInterruptStatus, ~0L, WriteBackDoneHead);
            uint32_t dwDoneHead = pHCDD->HCCA.dwDoneHead & ~0x1L;
            //assert(dwDoneHead != 0);

            if(dwDoneHead == ControlStatus->PhysicalAddr)
                errorCode = ControlStatus->Control.ConditionCode;
            else if(dwDoneHead != 0)
            {
                //_LOG("donehead: %08lx\n", dwDoneHead);
                OHCI_TD* pTD = (OHCI_TD*)DPMI_P2PTR(dwDoneHead);
                errorCode = pTD->Control.ConditionCode;
            }
            else
                errorCode = 0xFF;
            break;
        }
        delay(1);
    }
    pED->Control.Skip = 1;
    //while(DPMI_LoadD(dwIOBase+HcControlCurrentED) == pED->PhysicalAddr&~0xF);
#if DEBUG
    if(i == TIME_OUT || errorCode != 0)
    {
        //DBG_DumpReq(pReq);
        printf("Address:%d, Endpoint:%d, size:%d\n", pED->Control.FunctionAddress, pED->Control.EndPoint, pED->Control.MaxPacketSize);
        printf("Setup: ");DBG_DumpB(pDD->Setup, 8);
        printf("control transfer not complete, error: 0x%x\n", errorCode);
        DBG_Flush(&db);
        printf("ED TD after transation:\n");
        printf("PA: %08lx, %08lx, %08lx, %08lx\n", pED->PhysicalAddr, ControlSetup->PhysicalAddr, ControlData->PhysicalAddr, ControlStatus->PhysicalAddr);
        DBG_DumpPD(pED->PhysicalAddr, 4);
        DBG_DumpPD(ControlSetup->PhysicalAddr, 4);
        DBG_DumpPD(ControlData->PhysicalAddr, 4);
        DBG_DumpPD(ControlStatus->PhysicalAddr, 4);
        printf("HcControlHeadED: %08lx\n", DPMI_LoadD(dwIOBase + HcControlHeadED));
        printf("Control head PA: %08lx\n", pHCDD->ControlHead.PhysicalAddr);
        printf("Control head content:\n");
        printf("VA: %08x: ", &pHCDD->ControlHead);DBG_DumpD((uint32_t*)&pHCDD->ControlHead, 4);
        printf("PA: %08lx: ", pHCDD->ControlHead.PhysicalAddr);DBG_DumpPD(pHCDD->ControlHead.PhysicalAddr, 4);
        #if defined(__BC__)
        int flags;
        __asm{push ax; pushf; pop ax; mov flags,ax; pop ax;}
        printf("CPU FLAGS: %x\n", flags);
        #endif
    }
#endif
    if(i == TIME_OUT || errorCode != 0)
    {
        pED->HeadP = OHCI_TD::NULLTD->PhysicalAddr;
        errorCode = errorCode ? errorCode : 0xFF;
    }
    else
        assert(pED->HeadP == OHCI_TD::NULLTD->PhysicalAddr);

    return errorCode;
}

uint8_t OHCI_IsochronousTransfer(HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir, uint8_t* inoroutp pBuffer, uint16_t length, uint16_t* outputp txlen)
{
    OHCI_ED* pED = OHCI_FindEnpoint(pDevice, Endpoint, dir);
    //sanity check
    if(length == 0 || pBuffer == NULL || !pED || pED->TransferType != USB_ENDPOINT_TRANSFER_TYPE_ISOC)
    {
        *txlen = 0;
        return 0xFF;
    }
    OHCI_ISO_TD* pTD = pED->pIsoTD;
    
    //OHCI_HCData* pHCDD = (OHCI_HCData*) pDevice->pHCI->pHCDData;
    //OHCI_HCDeviceData* pDD = (OHCI_HCDeviceData*)pDevice->pHCData;

    uint32_t dwIOBase = pDevice->pHCI->dwBaseAddress;
    uint16_t MaxLen = pED->Control.MaxPacketSize;

    uint16_t FrameCount = (length + USB_ISO_FRAME_SIZE_LOWSPEED - 1) / USB_ISO_FRAME_SIZE_LOWSPEED;
    assert(FrameCount != 0);
    uint16_t Iteration = FrameCount / OHCI_MAX_ISO_FRAME; // [0,7]
    uint16_t LeftCount = FrameCount % OHCI_MAX_ISO_FRAME;
    uint16_t Leftbytes = length % MaxLen;
    if(LeftCount > 0)
        ++Iteration;
    if(Iteration == 1 && LeftCount != 0)
        FrameCount = LeftCount - 1;
    else
        FrameCount = OHCI_MAX_ISO_FRAME - 1;
    //printf("len:%d, pkt:%d, fc:%d, i:%d\n",length, MaxLen, FrameCount, Iteration);
#if !OHCI_TD_USE_CIRCLE_QUEUE
    uint32_t EDAddress = DPMI_PTR2P(pED);
    uint32_t TDAddress = DPMI_PTR2P(pTD);
    uint32_t NullTDAddress = DPMI_PTR2P(&pHCDD->NullTD);
#endif
    uint32_t BufferAddress = DPMI_PTR2P(pBuffer);
    uint16_t StartFrame = (DPMI_LoadW(dwIOBase + HcFmNumber) + 2); //advance 2 frame incase we miss the whole thing

    pTD->Control.StartFrame = StartFrame;
    pTD->Control.FrameCount = FrameCount;
    OHCI_UpdateIsoTD(pTD, StartFrame, (uint8_t)FrameCount, MaxLen, BufferAddress, length);

    uint16_t transferred = 0;

    //start transfer
    pED->HeadP = DPMI_PTR2P(pTD);
    //pED->Control.Skip = 0;

    int i;
    for(i = 0; i < Iteration; ++i)
    {
#if OHCI_TD_USE_CIRCLE_QUEUE
        while(pED->HeadP == pTD->NextTD)
            NOP;
        pED->TailP = pTD->NextTD;
        pTD = (OHCI_ISO_TD*)DPMI_P2PTR(pTD->NextTD);
        OHCI_UpdateIsoTD(pTD, StartFrame, (uint8_t)FrameCount, MaxLen, BufferAddress, length);
#else//OHCI_TD_USE_CIRCLE_QUEUE
        //refresh TD control uint16_t
        pTD->ControlFlags = (OHCI_CC_NOT_ACCESSED << 28L) | (FrameCount << 24L) | StartFrame;
        pTD->NextTD = NullTDAddress;
        //refresh ED element link
        pED->HeadP = TDAddress;
        pED->TailP == NullTDAddress;
#if DEBUG
        printf("Iso TD ED before transation:\n");
        DBG_DumpD((uint32_t*)pTD, 8);
#endif
        //wait to complete.
        //iso has no retire of TD due to error.
        uint32_t Try = 0;
        #define TRY_COUNT 0xFFFFFL
        while(!(DPMI_LoadD(dwIOBase + HcInterruptStatus) & WriteBackDoneHead))
        {
            NOP;//delay(1);
            if(++Try >= TRY_COUNT)  //just spin, don't delay
                break;
        }
        DPMI_StoreD(dwIOBase + HcInterruptStatus, WriteBackDoneHead);

#if DEBUG
        printf("Iso TD ED after transation:\n");
        DBG_DumpD((uint32_t*)pTD, 12);
#endif
        if((pTD->Control.ConditionCode) != 0)
            break;
#endif//OHCI_TD_USE_CIRCLE_QUEUE

        //advance to next iteration
        //uint16_t IterationSize = (FrameCount + 1) * MaxLen;
        uint16_t IterationSize = (i != Iteration - 1) ? (FrameCount + 1) * MaxLen : FrameCount * MaxLen + Leftbytes;
        transferred += IterationSize;
        BufferAddress += IterationSize;
        //StartFrame += (FrameCount + 1) + 1; //skip 1 frame incase CPU missed the controller frame
        StartFrame += FrameCount + 1; //won't get glitches if loop is optimized

        FrameCount = 7;
        if(i == Iteration - 2 && LeftCount != 0) //last iteration
            FrameCount = LeftCount;

        //update offset directly for optimization, skip setup the whole TD
        pTD->BufferPage = BufferAddress >> 12;
        int j;
        for(j = 0; j <= FrameCount; ++j)
            pTD->Offset[j] = (uint16_t)(BufferAddress&0xFFF) + j * MaxLen;
    }
    //pED->Control.Skip = 1;
    //while(DPMI_LoadD(dwIOBase+HcPeriodCurrentED) == pED->PhysicalAddr&~0xF);

#if DEBUG
    if(i >= TIME_OUT)
    {
        printf("Iso TD not complete. (%04x) \n", pTD->Control.ConditionCode);
    }
#endif    

    *txlen = transferred;
    //printf("TL:%d", *txlen);
    return pTD->Control.ConditionCode;
}

uint8_t OHCI_DataTransfer(HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir, uint8_t* inoroutp pBuffer, uint16_t length, uint16_t* outputp txlen)
{
    OHCI_ED* pED = OHCI_FindEnpoint(pDevice, Endpoint, dir);
    //sanity check
    if(length == 0 || pBuffer == NULL || !pED
    || (pED->TransferType != USB_ENDPOINT_TRANSFER_TYPE_BULK
     && pED->TransferType != USB_ENDPOINT_TRANSFER_TYPE_INTR))
    {
        _LOG("Error transfer endpoint: %d, pointer:%08p, type:%d\n", Endpoint, pED ? pED : 0, pED ? pED->TransferType : -1);
        *txlen = 0;
        return 0xFF;
    }
    uint32_t dwIOBase = pDevice->pHCI->dwBaseAddress;

#if OHCI_TD_USE_CIRCLE_QUEUE
    OHCI_TD* pTail = pED->pTD;
    assert(pED->TailP == pTail->PhysicalAddr);

    if(pTail->Buffer == NULL)
    {
        pTail->Buffer = DPMI_MappedMalloc(OHCI_TD_BUFFER_SIZE, 4);
        pTail->BufferPhysicalAddr = DPMI_PTR2P(pTail->Buffer);
    }
    uint16_t len = min(OHCI_TD_BUFFER_SIZE, length);
    memcpy(pTail->Buffer, pBuffer, len);
    //build TD
    OHCI_UpdateTD(pTail, pTail->BufferPhysicalAddr, len, pTail->pNext);
    //wait on full circle queue
    assert(pTail->pNext != NULL && pTail->pNext != pTail);
    while(pED->HeadP == pTail->pNext->PhysicalAddr) //{printf("NOP");}
        NOP;

    //start transfer
    pED->pTD = pTail->pNext;
    pED->TailP = pTail->pNext->PhysicalAddr;
    if(pED->TransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK) //endpoint already connected to bulk head, issue the command
        DPMI_MaskD(dwIOBase + HcCommandStatus , ~0L, BulkListFilled);

    *txlen = len;

    //peek previous TD error. TODO: handle error in interrupt
    /*
    if(DPMI_LoadD(dwIOBase + HcInterruptStatus) & WriteBackDoneHead)
    {
        DPMI_StoreD(dwIOBase + HcInterruptStatus, WriteBackDoneHead);
        uint32_t dwDoneHead = pHCDD->HCCA.dwDoneHead & ~0x1L;
        if(dwDoneHead != 0)
        {
            OHCI_TD* pTD = (OHCI_TD*)DPMI_P2PTR(dwDoneHead);
            return pTD->Control.ConditionCode;
        }
    }
    */
    return 0;
#else//OHCI_TD_USE_CIRCLE_QUEUE
    OHCI_HCData* pHCDD = (OHCI_HCData*)pDevice->pHCI->pHCDData;
    OHCI_HCDeviceData* pDD = (OHCI_HCDeviceData*)pDevice->pHCData;
    OHCI_TD* pTD = pED->pTD;
    OHCI_UpdateTD(pTD, DPMI_PTR2P(pBuffer), length);

    //start transfer
    assert(pED->HeadP == pED->TailP);
    //assert(pED->TailP == pHCDD->NullTD.PhysicalAdd);
    pED->HeadP = pTD->PhysicalAddr;
    pED->TailP = OHCI_T::NULLTD->PhysicalAddr;
    //pED->Control.Skip = 0;

#if DEBUG && 0
    DBG_DBuff db = {1};
    DBG_Printf(&db, "ED TD before transation:\n");
    DBG_Printf(&db, "%08lx: ", pED->PhysicalAddr&~0xFL);
    DBG_DumpD((uint32_t*)pED, 4, &db);
    DBG_Printf(&db, "%08lx: ", pTD->PhysicalAddr);
    DBG_DumpD((uint32_t*)pTD, 4, &db);
    DBG_Printf(&db, "donehead:%08lx\n", pHCDD->HCCA.dwDoneHead);
#endif
    if(pED->TransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK) //the endpoint is already connected to bulk head, just issue the command
        DPMI_MaskD(dwIOBase + HcCommandStatus , ~0L, BulkListFilled);

    //wait to complete.
    uint32_t i = 0;
    while(!(DPMI_LoadD(dwIOBase + HcInterruptStatus) & WriteBackDoneHead))
    {
        NOP;//delay(1); //dont use delay
        if(++i >= TIME_OUT*100)
        {
#if DEBUG && 0
            _LOG("TD not complete. \n");
#endif
            break;
        }
    }
    DPMI_StoreD(dwIOBase + HcInterruptStatus, WriteBackDoneHead);
    //pED->Control.Skip = 1;

    uint32_t dwDoneHead = pHCDD->HCCA.dwDoneHead & ~0x1L;
    if(pTD->Control.ConditionCode)
    {
        pED->HeadP = OHCI_T::NULLTD->PhysicalAddr;
#if DEBUG && 0
        printf("transfer not complete, error: 0x%x\n", pTD->Control.ConditionCode);
        printf("Address:%d, Endpoint:%d, size:%d\n", pED->Control.FunctionAddress, pED->Control.EndPoint, pED->Control.MaxPacketSize);
        printf("donehead:%08lx\n",dwDoneHead);
        DBG_Flush(&db);
        printf("ED TD after transation:\n");
        printf("%08lx: ", pED->PhysicalAddr);
        DBG_DumpD((uint32_t*)pED, 4);
        printf("%08lx: ", pTD->PhysicalAddr);
        DBG_DumpD((uint32_t*)pTD, 4);
        printf("\n");
#endif
    }
    else
    {
        assert(dwDoneHead == pTD->PhysicalAddr);
        assert((pED->HeadP&~0x1L) == OHCI_T::NULLTD->PhysicalAddr);
    }

    if(pTD->CurrentBufferP == 0) *txlen = length;//transfer complete
    else *txlen = pTD->CurrentBufferP - DPMI_PTR2P(pBuffer);
    
    return pTD->Control.ConditionCode;
    
#endif//OHCI_TD_USE_CIRCLE_QUEUE
}

uint8_t* OHCI_GetHCDBuffer(HCD_Device* pDevice)
{
    return ((OHCI_HCDeviceData*)pDevice->pHCData)->Buffer;
}

uint16_t OHCI_GetPortStatus(HCD_Interface* pHCI, uint8_t port)
{
    uint32_t dwBase = pHCI->dwBaseAddress;
    uint32_t status = DPMI_LoadD(dwBase + HcRhPort1Status + port * 4);

    uint16_t result = 0;
    if(status&PortEnableStatus)
        result |= USB_PORT_ENABLE;

    if(status&CurrentConnectStatus)
        result |= USB_PORT_ATTACHED;

    if(status&PortSuspendStatus)
        result |= USB_PORT_SUSPEND;

    if(status&PortResetStatus)
        result |= USB_PORT_RESET; //temporary state: resetting

    if(status&LowSpeedDeviceAttached)
        result |= USB_PORT_Low_Speed_Device;
    else
        result |= USB_PORT_Full_Speed_Device;
        
    if(status&ConnectStatusChange)
        result |= USB_PORT_CONNECT_CHANGE;

    return result;
}

BOOL OHCI_SetPortStatus(HCD_Interface* pHCI, uint8_t port, uint16_t status)
{
    uint32_t dwPortAddr = pHCI->dwBaseAddress + HcRhPort1Status + (uint32_t)port * 4;
    uint32_t cur = DPMI_LoadD(dwPortAddr);

    if(!(cur&CurrentConnectStatus))
    {
        assert(FALSE);
        return FALSE;
    }

    //setting bits to 0 always has no effect by OHCI spec, so we just set the effective bit
    if(status&USB_PORT_RESET)
    {
        DPMI_StoreD(dwPortAddr, PortResetStatusChange); //clear PortResetStatusChange (write 1 to clear)
        DPMI_StoreD(dwPortAddr, SetPortReset);
        int count = 0;
        while(!(DPMI_LoadD(dwPortAddr)&PortResetStatusChange) && count++ <= 500) //5 sec time out (Microsoft impl)
            delay(10);    //spec required
        DPMI_StoreD(dwPortAddr, PortResetStatusChange);
        //_LOG("port reset done.\n");
    }

    if((status&USB_PORT_ENABLE) && !(cur&PortEnableStatus))
    {
        DPMI_StoreD(dwPortAddr, SetPortEnable);
        while(!(DPMI_LoadD(dwPortAddr)&PortEnableStatus))
            delay(10);
        //_LOG("port endable done.\n");
    }

    if((status&USB_PORT_DISABLE) && (cur&PortEnableStatus))
    {
        //DPMI_MaskD(dwPortAddr, ~SetPortEnable, ClearPortEnable);
        DPMI_StoreD(dwPortAddr, ClearPortEnable);
        while((DPMI_LoadD(dwPortAddr)&PortEnableStatus))
            delay(10);
        //_LOG("port disable done.\n");
    }

    if(status&USB_PORT_SUSPEND)
    {
        DPMI_StoreD(dwPortAddr, PortSuspendStatusChange); //clear PortSuspendStatusChange
        DPMI_StoreD(dwPortAddr, SetPortSuspend);
        while(!(DPMI_LoadD(dwPortAddr)&PortSuspendStatusChange))
            delay(10);
        DPMI_StoreD(dwPortAddr, PortSuspendStatusChange);
    }

    if(status&USB_PORT_CONNECT_CHANGE)
    { //write to ConnectStatusChange will clear it
        DPMI_StoreD(dwPortAddr, ConnectStatusChange);
    }
    return TRUE;
}

BOOL OHCI_InitDevice(HCD_Device* pDevice)
{
    if(pDevice->pHCI->bDevCount >= HCD_MAX_DEVICE_COUNT)
        return FALSE;

    for(int i = 0; i < HCD_MAX_DEVICE_COUNT; ++i)
    {
        if(pDevice->pHCI->DeviceList[i] == NULL)
        {
            pDevice->pHCI->DeviceList[i] = pDevice;
            break;
        }
    }
    ++pDevice->pHCI->bDevCount;

    pDevice->pHCData = DPMI_MappedMalloc(sizeof(OHCI_HCDeviceData), 16);
    OHCI_HCDeviceData* pDD = (OHCI_HCDeviceData*)pDevice->pHCData;
    memset(pDD, 0, sizeof(OHCI_HCDeviceData));
    
    pDD->ControlED.Control.MaxPacketSize = 4;
    OHCI_BuildControl(pDevice, &pDD->ControlED, &pDD->ControlSetup, &pDD->ControlData, &pDD->ControlStatus);
    assert(pDD->ControlED.Control.FunctionAddress == pDevice->bAddress);
    OHCI_HCData* pHCDD = (OHCI_HCData*)pDevice->pHCI->pHCDData;
    OHCI_AddToTail(pHCDD->ControlTail, &pDD->ControlED);
    return TRUE;
}

BOOL OHCI_CreateEndpoint(HCD_Device* pDevice, uint8_t EPAddr, HCD_TxDir dir, uint8_t bTransferType, uint16_t MaxPacketSize, uint8_t bInterval)
{
    OHCI_HCDeviceData* pDD = (OHCI_HCDeviceData*)pDevice->pHCData;
    if(EPAddr == 0)
    {
        pDD->ControlED.Control.MaxPacketSize = MaxPacketSize;
        pDD->ControlED.Control.FunctionAddress = pDevice->bAddress;
        return TRUE;
    }

    uint8_t bNumEndpoints = pDevice->bNumEndpoints;
    assert(bNumEndpoints);
    if(bNumEndpoints == 0)
        return FALSE;
    if(bNumEndpoints != pDD->bNumEndpoints)
        OHCI_RemoveEndpoints(pDevice);

    assert(pDD->DataED == NULL && pDD->DataTD == NULL || pDD->DataED != NULL && pDD->DataTD != NULL);
    if(pDD->DataED == NULL && pDD->DataTD == NULL)
    {
        assert(pDD->bUsedEndpoint == 0);
        pDD->DataED = (OHCI_ED*)DPMI_MappedMalloc(sizeof(OHCI_ED)*bNumEndpoints, 16);
        pDD->DataTD = (OHCI_TD**)DPMI_MappedMalloc(sizeof(OHCI_TD*)*bNumEndpoints, 16);
        pDD->bNumEndpoints = bNumEndpoints;
    }
    if(pDD->DataED == NULL || pDD->DataTD == NULL)
        return FALSE;
    assert(pDD->bUsedEndpoint < pDD->bNumEndpoints);

    int EPIndex = -1;
    uint8_t pid = (dir == HCD_TXR) ? PIDIN : PIDOUT;
    //reuse the same addr
    for(int i = 0; i < pDD->bUsedEndpoint; ++i)
    {
        if(pDD->DataED[i].Control.FunctionAddress == EPAddr && pDD->DataED[i].Control.Direction == pid)
        {
            EPIndex = i;
            break;
        }
    }
    if(EPIndex == -1)
    {
        if(pDD->bUsedEndpoint >= pDD->bNumEndpoints)
            return FALSE;
        EPIndex = pDD->bUsedEndpoint++;
    }

    //build OHCI TD ED
    OHCI_HCData* pHCDD = (OHCI_HCData*)pDevice->pHCI->pHCDData;

    uint8_t address = pDevice->bAddress;
    uint8_t lowspeed = (pDevice->bEndpointSpeed == USB_PORT_Low_Speed_Device) ? 1 : 0;

    OHCI_ED& ed = pDD->DataED[EPIndex];
    OHCI_TD*& td = pDD->DataTD[EPIndex];

    int NumTDs = ((bTransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL) ? 3 : OHCI_TD_CIRCLE_QUEUE_SIZE);
    td = (OHCI_TD*)DPMI_MappedMalloc(sizeof(OHCI_TD)*NumTDs, 16);
    int DelayInterrupt = OHCI_TD_USE_CIRCLE_QUEUE ? OHCI_CW_NO_INTERRUPT : 0;

    uint8_t EndPoint = EPAddr;
    uint16_t wMaxSize = MaxPacketSize;
    uint8_t PID = dir == HCD_TXW ? PIDOUT : PIDIN;

    if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_ISOC)
    {
        for(int j = NumTDs - 1; j >= 0; --j)
            OHCI_BuildIsoTD((OHCI_ISO_TD*)&td[j], 0, 0, DelayInterrupt, wMaxSize, 0, 0,
            OHCI_TD_USE_CIRCLE_QUEUE ? (OHCI_ISO_TD*)&td[(j+1)%NumTDs] : (OHCI_ISO_TD*)OHCI_TD::NULLTD);
            
        OHCI_BuildIsoED(&ed, address, EndPoint, PID, lowspeed, wMaxSize, (OHCI_ISO_TD*)&td[0], (OHCI_ISO_TD*)&td[0]);
        ed.Control.Format = 1;
    }
    else if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL)
    {
        OHCI_BuildControl(pDevice, &ed, &td[0], &td[1], &td[2]);
        OHCI_BuildED(&ed, address, EndPoint, PIDSETUP, lowspeed, wMaxSize, OHCI_TD::NULLTD, &td[0]);

        assert(ed.Control.EndPoint == EndPoint);
        assert(ed.Control.FunctionAddress == address);
    }
    else
    {
        td[0].PhysicalAddr = ~0x0L; //skip condition check
        for(int j = NumTDs - 1; j >= 0; --j)
            OHCI_BuildTD(&td[j], PID, DelayInterrupt,
                //(j&0x1) ? OHCI_CW_DATATOGGLE_DATA1 : OHCI_CW_DATATOGGLE_DATA0, 0, 0,
                OHCI_CW_DATATOGGLE_CARRY, 0, 0,
                OHCI_TD_USE_CIRCLE_QUEUE ? &td[(j+1)%NumTDs]: OHCI_TD::NULLTD);
                
        OHCI_BuildED(&ed, address, EndPoint, PID, lowspeed, wMaxSize, &td[0], &td[0]);
    }
    ed.TransferType = bTransferType;

    OHCI_ED** ppED;
    if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_ISOC || bTransferType == USB_ENDPOINT_TRANSFER_TYPE_INTR)
            ppED = OHCI_GetEDFromInterval(pHCDD, bInterval);
    else if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK)
        ppED = &pHCDD->BulkTail;
    else
        ppED = &pHCDD->ControlTail;

    OHCI_AddToTail(*ppED, &ed);
    ed.Control.Skip = 0;

    return TRUE;
}

BOOL OHCI_RemoveDevice(HCD_Device* pDevice)
{
    if(!HCD_IS_DEVICE_VALID(pDevice))
        return FALSE;

    OHCI_SetPortStatus(pDevice->pHCI, pDevice->bHubPort, USB_PORT_DISABLE);
    //OHCI_HCData* pHCDD = (OHCI_HCData*)pDevice->pHCI->pHCDData;
    OHCI_HCDeviceData* pDD = (OHCI_HCDeviceData*)pDevice->pHCData;
    OHCI_RemoveEndpoints(pDevice);
    DPMI_MappedFree(pDD);

    for(int i = 0; i < HCD_MAX_DEVICE_COUNT; ++i)
    {
        if(pDevice->pHCI->DeviceList[i] == pDevice)
        {
            pDevice->pHCI->DeviceList[i] = NULL;
            ++pDevice->pHCI->bDevCount;
            return TRUE;
        }
    }
    pDevice->pHCData = NULL;
    pDevice->pHCI = NULL;
    return FALSE;
}

OHCI_ED* OHCI_FindEnpoint(HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir)
{
    OHCI_HCDeviceData* pDD = (OHCI_HCDeviceData*)pDevice->pHCData;
    if(Endpoint == 0)
        return &pDD->ControlED;
        
    uint8_t pid = (dir == HCD_TXR) ? PIDIN : PIDOUT;
    for(int i = 0; i < pDD->bUsedEndpoint; ++i)
    {
        if( pDD->DataED[i].Control.EndPoint == Endpoint && pDD->DataED[i].Control.Direction == pid)
            return &pDD->DataED[i];
    }
    return NULL;
}

BOOL OHCI_RemoveEndpoints(HCD_Device* pDevice)
{
    OHCI_HCData* pHCDD = (OHCI_HCData*)pDevice->pHCI->pHCDData;
    OHCI_HCDeviceData* pDD = (OHCI_HCDeviceData*)pDevice->pHCData;
    if(!pDD->DataED || !pDD->DataTD)
        return TRUE;

    uint32_t dwBase = pDevice->pHCI->dwBaseAddress;
    //disable lists before removing
    DPMI_MaskD(dwBase + HcControl, ~ControlListEnable, 0);
    DPMI_MaskD(dwBase + HcControl, ~PeriodicListEnable, 0);
    DPMI_MaskD(dwBase + HcControl, ~IsochronousEnable, 0);
    DPMI_MaskD(dwBase + HcControl, ~BulkListEnable, 0);

    for(int i = 0; i < pDevice->bNumEndpoints; ++i)
    {
        OHCI_ED& ed = pDD->DataED[i];
        OHCI_TD* td = pDD->DataTD[i];

        //remove ed
        ed.pPrev->pNext = ed.pNext;
        if(ed.pNext)
            ed.pNext->pPrev = ed.pPrev;
        ed.pPrev->NextED = ed.pNext ? ed.pNext->PhysicalAddr&~0xFL : 0;
        
        if(ed.TransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL)
        {
            while(DPMI_LoadD(dwBase+HcControlCurrentED) == ed.PhysicalAddr&~0xFL);

            if(pHCDD->ControlTail == &ed)
                pHCDD->ControlTail = ed.pPrev;
        }
        else if(ed.TransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK)
        {
            while(DPMI_LoadD(dwBase+HcBulkCurrentED) == ed.PhysicalAddr&~0xFL);

            if(pHCDD->BulkTail == &ed)
                pHCDD->BulkTail = ed.pPrev;
        }
        else
        {
            while(DPMI_LoadD(dwBase+HcPeriodCurrentED) == ed.TransferType); //TODO: RunningEDReclamation

            if(pHCDD->ED32msTail == &ed) pHCDD->ED32msTail = ed.pPrev;
            else if(pHCDD->ED16msTail == &ed) pHCDD->ED32msTail = ed.pPrev;
            else if(pHCDD->ED8msTail == &ed) pHCDD->ED32msTail = ed.pPrev;
            else if(pHCDD->ED4msTail == &ed) pHCDD->ED32msTail = ed.pPrev;
            else if(pHCDD->ED2msTail == &ed) pHCDD->ED32msTail = ed.pPrev;
            else if(pHCDD->ED1msTail == &ed) pHCDD->ED32msTail = ed.pPrev;
        }

        for(int j = 0; j < OHCI_TD_CIRCLE_QUEUE_SIZE; ++j)
        {
            if(td[j].Buffer)
                DPMI_MappedFree(td[j].Buffer);
        }
        DPMI_MappedFree(td);
    }

    //disable lists again
    DPMI_MaskD(dwBase + HcControl, ~0L, ControlListEnable);
    DPMI_MaskD(dwBase + HcControl, ~0L, PeriodicListEnable);
    DPMI_MaskD(dwBase + HcControl, ~0L, IsochronousEnable);
    DPMI_MaskD(dwBase + HcControl, ~0L, BulkListEnable);

    DPMI_MappedFree(pDD->DataED);
    DPMI_MappedFree(pDD->DataTD);

    pDD->DataED = NULL;
    pDD->DataTD = NULL;
    pDD->bNumEndpoints = 0;
    pDD->bUsedEndpoint = 0;

    return TRUE;
}

void OHCI_AddToTail(OHCI_ED*& pTail, OHCI_ED* pED)
{
    assert(pTail->pNext == NULL);
    assert((pED->PhysicalAddr&~0xFL) != 0);
    assert(pED->pPrev == NULL && pED->pNext == NULL);

    pTail->pNext = pED;
    pED->pPrev = pTail;

    pTail->NextED = pED->PhysicalAddr&~0xFL;
    pTail = pED;
}

void OHCI_BuildControl(HCD_Device* pDevice, OHCI_ED* pControlED, OHCI_TD* pSetup, OHCI_TD* pData, OHCI_TD* pStatus)
{
    uint8_t address = pDevice->bAddress;
    uint8_t lowspeed = (pDevice->bEndpointSpeed == USB_PORT_Low_Speed_Device) ? 1 : 0;
    uint16_t PacketSize = pControlED->Control.MaxPacketSize;
    //OHCI_HCData* pHCDD = (OHCI_HCData*)pDevice->pHCI->pHCDData;

    //build empty control list
    OHCI_BuildTD(pStatus, 0, 0/*interrupt and writeback done head immediately*/,
        OHCI_CW_DATATOGGLE_DATA1, 0, 0, OHCI_TD::NULLTD);

    OHCI_BuildTD(pData, 0, OHCI_CW_NO_INTERRUPT, OHCI_CW_DATATOGGLE_DATA1,
            0, 0, pStatus);

    OHCI_BuildTD(pSetup, PIDSETUP, OHCI_CW_NO_INTERRUPT, OHCI_CW_DATATOGGLE_DATA0,
        0, 0, pData);

    OHCI_BuildED(pControlED, address, 0, PIDFROMTD, lowspeed, PacketSize, OHCI_TD::NULLTD, pSetup);

    pControlED->TransferType = USB_ENDPOINT_TRANSFER_TYPE_CTRL;
    assert((pControlED->PhysicalAddr&~0xFL) != 0);
}

void OHCI_BuildTD(OHCI_TD* pTD, uint8_t PID, uint8_t DelayInterrupt, uint8_t DataToggle, uint32_t BufferAddr, uint32_t length,
    OHCI_TD* pNextTD)
{
    memset(pTD, 0, sizeof(OHCI_TD));
    //MSB LSB
    //CC:4|EC:2|T:2|DI:3|DP:2|R:1|-
    pTD->Control.ConditionCode = OHCI_CC_NOT_ACCESSED;
    pTD->Control.ErrorCount = 0;
    pTD->Control.DataToggle = DataToggle&0x3;
    pTD->Control.DelayInterrupt = DelayInterrupt&0x7;
    pTD->Control.PID = PID&0x3;
    pTD->Control.BufferRounding = 1;

    assert(pNextTD != NULL && pNextTD->PhysicalAddr != 0);
    pTD->NextTD = pNextTD->PhysicalAddr;
    pTD->CurrentBufferP = BufferAddr;
    pTD->BufferEnd = BufferAddr ? BufferAddr + length - 1 : 0;

    //extension
    pTD->PhysicalAddr = DPMI_PTR2P(pTD);
    pTD->pNext = pNextTD;
}

void OHCI_UpdateTD(OHCI_TD* pTD, uint32_t BufferAddr, uint32_t length, OHCI_TD* pNext/* == NULL*/)
{
    pTD->Control.ConditionCode = OHCI_CC_NOT_ACCESSED;
    pTD->Control.DataToggle = OHCI_CW_DATATOGGLE_CARRY;
    pTD->Control.ErrorCount = 0;
    pTD->CurrentBufferP = BufferAddr;
    pTD->BufferEnd = BufferAddr ? BufferAddr + length - 1 : 0;
    pTD->NextTD = pNext ? pNext->PhysicalAddr : OHCI_TD::NULLTD->PhysicalAddr;
}

void OHCI_BuildIsoTD(OHCI_ISO_TD* pTD, uint16_t StartFrame, uint8_t FrameCount, uint8_t DelayInterrupt, uint16_t PacketSize, uint32_t BufferAddr,
    uint32_t length, OHCI_ISO_TD* pNextTD)
{
    memset(pTD, 0, sizeof(OHCI_ISO_TD));
    pTD->Control.ConditionCode = OHCI_CC_NOT_ACCESSED&0xF;
    pTD->Control.FrameCount= FrameCount&0x7;
    pTD->Control.DelayInterrupt = DelayInterrupt&0x7;
    pTD->Control.StartFrame = StartFrame&0xFFFF;
    
    if(pNextTD) pTD->NextTD = DPMI_PTR2P(pNextTD);
    pTD->BufferPage = BufferAddr >> 12;    //align to 4K page
    pTD->BufferEnd = length ? BufferAddr + length - 1 : 0;

    for(int i = 0; i <= FrameCount; ++i)
        pTD->Offset[i] = (uint16_t)(BufferAddr&0xFFF) + i * PacketSize;
}

void OHCI_UpdateIsoTD(OHCI_ISO_TD* pTD, uint16_t StartFrame, uint8_t FrameCount, uint16_t PacketSize, uint32_t BufferAddr, uint32_t length)
{
    pTD->BufferPage = BufferAddr >> 12;    //align to 4K page
    pTD->BufferEnd = length ? BufferAddr + length - 1 : 0;

    pTD->Control.ConditionCode = OHCI_CC_NOT_ACCESSED&0xF;
    pTD->Control.FrameCount= FrameCount&0x7;
    pTD->Control.StartFrame = StartFrame&0xFFFF;

    for(int i = 0; i <= FrameCount; ++i)
        pTD->Offset[i] = (uint16_t)(BufferAddr&0xFFF) + i * PacketSize;
}

void OHCI_BuildED(OHCI_ED* pED, uint8_t FunctionAddress, uint8_t EndpointNo, uint8_t Direction, uint8_t LowSpeed, uint16_t MaxPackSize,
    OHCI_TD* TailP, OHCI_TD* HeadP)
{
    memset(pED, 0, sizeof(OHCI_ED));

    //MSB LSB
    //-|MPS:11|F:1|K:1|S:1|D:2|EN:4|FA:7
    pED->Control.FunctionAddress = FunctionAddress&0x7F;
    pED->Control.EndPoint = EndpointNo&0xF;
    pED->Control.Direction = Direction&0x3;
    pED->Control.LowSpeed = LowSpeed&0x1;
    pED->Control.Skip = 1&0x1;
    pED->Control.Format = 0&0x1;
    pED->Control.MaxPacketSize = MaxPackSize&0x7FF;

    //if(HeadP)
    //    pED->HeadP = HeadP->PhysicalAddr;
    if(TailP)
        pED->HeadP = pED->TailP = TailP->PhysicalAddr;

    pED->pTD = HeadP;
    pED->PhysicalAddr = DPMI_PTR2P(pED);
}

void OHCI_BuildIsoED(OHCI_ED* pED, uint8_t FunctionAddress, uint8_t EndpointNo, uint8_t Direction, uint8_t LowSpeed, uint16_t MaxPackSize,
    OHCI_ISO_TD* TailP, OHCI_ISO_TD* HeadP)
{
    memset(pED, 0, sizeof(OHCI_ED));

    //MSB LSB
    //-|MPS:11|F:1|K:1|S:1|D:2|EN:4|FA:7
    pED->Control.FunctionAddress = FunctionAddress&0x7F;
    pED->Control.EndPoint = EndpointNo&0xF;
    pED->Control.Direction = Direction&0x3;
    pED->Control.LowSpeed = LowSpeed&0x1;
    pED->Control.Skip = 1;
    pED->Control.Format = 1;
    pED->Control.MaxPacketSize = MaxPackSize&0x7FF;

    if(HeadP)
        pED->HeadP = DPMI_PTR2P(HeadP);
    if(TailP)
        pED->TailP = DPMI_PTR2P(TailP);

    pED->pIsoTD = HeadP;
    pED->PhysicalAddr = DPMI_PTR2P(pED);
}

void OHCI_BuildHCCA(OHCI_HCData * pHCDD)
{
    uint32_t* ptr = pHCDD->HCCA.InterruptTable;
    uint32_t dwAddress;

    //build 32 ms
    pHCDD->ED32ms.Control.Skip = 1;
    ptr[0] = DPMI_PTR2P(&pHCDD->ED32ms);

    //build 16 ms
    pHCDD->ED16ms.Control.Skip = 1;
    dwAddress = DPMI_PTR2P(&pHCDD->ED16ms);
    ptr[1] = dwAddress;
    ptr[1 + 16] = dwAddress;

    //build 8ms
    pHCDD->ED8ms.Control.Skip = 1;
    dwAddress = DPMI_PTR2P(&pHCDD->ED8ms);
    int i;
    for(i = 2; i< 32; i = i + 8)
        ptr[i] = dwAddress;

    //build 4ms
    pHCDD->ED4ms.Control.Skip = 1;
    dwAddress = DPMI_PTR2P(&pHCDD->ED4ms);
    for(i = 3; i < 32; i= i+4)
        ptr[i] = dwAddress;

    //build 2ms
    pHCDD->ED2ms.Control.Skip = 1;
    dwAddress = DPMI_PTR2P(&pHCDD->ED2ms);
    for(i = 0; i < 32; i = i + 2)
    {
        if(ptr[i] == 0)
        {
            ptr[i] = dwAddress;
            continue;
        }
        //get last ED
        uint32_t dwLast = ptr[i];
        uint32_t dwNext = dwLast;
        while((dwNext=DPMI_LoadD(DPMI_P2L(dwLast) + offsetof(OHCI_ED, NextED))) != 0) dwLast = dwNext;
        if(dwLast != dwAddress)//if found different ED.
            DPMI_StoreD(DPMI_P2L(dwLast) + offsetof(OHCI_ED, NextED), dwAddress);
    }
    //build 1ms
    pHCDD->ED1ms.Control.Skip = 1;
    dwAddress = DPMI_PTR2P(&pHCDD->ED1ms);
    for(i = 0; i < 32; i++)
    {
        if(ptr[i] == 0)
        {
            ptr[i]= dwAddress;
            continue;
        }
        //get last ED
        uint32_t dwLast = ptr[i];
        uint32_t dwNext = dwLast;
        while((dwNext=DPMI_LoadD(DPMI_P2L(dwLast) + offsetof(OHCI_ED, NextED))) != 0) dwLast = dwNext;
        if(dwLast != dwAddress)//if found different ED.
            DPMI_StoreD(DPMI_P2L(dwLast) + offsetof(OHCI_ED, NextED), dwAddress);
    }
    return;
}

OHCI_ED** OHCI_GetEDFromInterval(OHCI_HCData* pHCDD, uint8_t interval)
{
    interval = (interval <= 0) ? 1 : interval; //((interval >= 32) ? 32 : interval);

    if(interval <= 1)
        return &pHCDD->ED1msTail;
    else if(interval <= 2)
        return &pHCDD->ED2msTail;
    else if(interval >= 3 && interval <= 5)
        return &pHCDD->ED4msTail;
    else if(interval >= 6 && interval <= 11)
        return &pHCDD->ED8msTail;
    else if(interval >= 12 && interval <= 23)
        return &pHCDD->ED16msTail;
    else
        return &pHCDD->ED32msTail;
}
