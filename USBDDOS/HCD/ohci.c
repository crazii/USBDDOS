#include <dos.h>
#include <stdio.h>
#include <memory.h>
#include <assert.h>
#include <string.h>
#include "USBDDOS/HCD/ohci.h"
#include "USBDDOS/DPMI/dpmi.h"
#include "USBDDOS/usballoc.h"
#include "USBDDOS/dbgutil.h"
#if 0
// ref: OHCI_Specification_Rev.1.0a

// configs
#define TIME_OUT 500L

static void OHCI_ISR_ProcessDoneQueue(HCD_Interface* pHCI, uint32_t dwDoneHead);
static uint16_t OHCI_GetPortStatus(HCD_Interface* pHCI, uint8_t port);
static BOOL OHCI_SetPortStatus(HCD_Interface* pHCI, uint8_t port, uint16_t status);
static BOOL OHCI_InitDevice(HCD_Device* pDevice);
static BOOL OHCI_RemoveDevice(HCD_Device* pDevice);
static void* OHCI_CreateEndpoint(HCD_Device* pDevice, uint8_t EPAddr, HCD_TxDir dir, uint8_t bTransferType, uint16_t MaxPacketSize, uint8_t bInterval);
static BOOL OHCI_RemoveEndpoint(HCD_Device* pDevice, void* pEndpoint);

static void OHCI_AddEDToTail(OHCI_ED** pTail, OHCI_ED* pED);
static void OHCI_BuildTD(OHCI_TD* pTD, uint8_t PID, uint8_t DelayInterrupt, uint8_t DataToggle, uint32_t BufferAddr, uint32_t length, OHCI_TD* pNext);
static void OHCI_BuildISOTD(OHCI_ISO_TD* pTD, uint16_t StartFrame, uint8_t FrameCount, uint8_t DelayInterrupt, uint16_t PacketSize, uint32_t BufferAddr, uint32_t length, OHCI_ISO_TD* pNext);
static void OHCI_BuildED(OHCI_ED* pED, uint8_t FunctionAddress, uint8_t Endpoint, uint8_t Direction, uint8_t LowSpeed, uint16_t MaxPackSize);
static void OHCI_BuildISOED(OHCI_ED* pED, uint8_t FunctionAddress, uint8_t EndpointNo, uint8_t Direction, uint8_t LowSpeed, uint16_t MaxPackSize);
static void OHCI_BuildHCCA(OHCI_HCData* pHCDData);
static OHCI_ED** OHCI_GetEDFromInterval(OHCI_HCData* pHCDData, uint8_t interval);

HCD_Method OHCI_Method =
{
    &OHCI_ControlTransfer,
    &OHCI_IsochronousTransfer,
    &OHCI_DataTransfer,
    &OHCI_DataTransfer,
    &OHCI_GetPortStatus,
    &OHCI_SetPortStatus,
    &OHCI_InitDevice,
    &OHCI_RemoveDevice,
    &OHCI_CreateEndpoint,
    &OHCI_RemoveEndpoint,
};

BOOL OHCI_InitController(HCD_Interface* pHCI, PCI_DEVICE* pPCIDev)
{
#if defined(__BC__) || defined(__WC__)
    assert(sizeof(OHCI_TD) % 16 == 0);
    assert(sizeof(OHCI_ISO_TD) % 16 == 0);
    assert(sizeof(OHCI_ED) % 16 == 0);
#endif
    // enable MMIO
    PCI_CMD cmd;
    cmd.reg16 = pPCIDev->Header.Command;
    cmd.bits.BusMaster = 1;
    cmd.bits.IOSpace = 0;
    cmd.bits.MemorySpace = 1;
    cmd.bits.InterruptDisable = 0;
    PCI_WriteWord(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, pHCI->PCIAddr.Function, PCI_REGISTER_CMD, cmd.reg16);
    // map to linear
    pHCI->dwPhysicalAddress = pPCIDev->Header.DevHeader.Device.Base0 & 0xffffffe0;                                            // BAR0
    uint32_t size = PCI_Sizing(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, pHCI->PCIAddr.Function, OHCI_REGISTER_BAR); // should be 4K according to the spec
    pHCI->dwBaseAddress = DPMI_MapMemory(pHCI->dwPhysicalAddress, size);
    _LOG("OHCI BAR address: %08lx, mapped address %08lx, size: %08lx\n", pHCI->dwPhysicalAddress, pHCI->dwBaseAddress, size);

    uint32_t dwBase = pHCI->dwBaseAddress;
    // ownership handoff
    {
        uint32_t HCFS = (DPMI_LoadD(dwBase + HcControl) & HostControllerFunctionalState) >> HostControllerFunctionalState_SHIFT;
        _LOG("HCFS: %d\n", HCFS);
        if(!(DPMI_LoadD(dwBase + HcControl) & InterruptRouting))
        { // take ownership from BIOS driver
            if(HCFS != USBRESET)
            {
                _LOG("OHCI: take ownership from BIOS..\n");
                if(HCFS != USBOPERATIONAL)
                {
                    DPMI_MaskD(dwBase + HcControl, ~HostControllerFunctionalState, USBRESUME << HostControllerFunctionalState_SHIFT);
                    delay(50); //usb1.1 requires 20ms
                }
            }
            else //no BIOS driver nor SMM, do a reset
            {
                DPMI_MaskD(dwBase + HcControl, ~HostControllerFunctionalState, USBRESET << HostControllerFunctionalState_SHIFT);
                delay(50); //10~50ms by USB1.1 spec
            }
        }
        else if(HCFS == USBRESET)
        { // take ownership from SMM driver
            DPMI_MaskD(dwBase + HcCommandStatus, ~0UL, OwnershipChangeRequest);
            while(DPMI_LoadD(dwBase + HcControl) & InterruptRouting) // wait SMM driver
                delay(50);
        }
        //HCFS = (DPMI_LoadD(dwBase + HcControl) & HostControllerFunctionalState) >> HostControllerFunctionalState_SHIFT;
        //_LOG("HCFS: %d\n", HCFS);
    }
    uint32_t FrameInterval = DPMI_LoadD(dwBase + HcFmInterval);
    DPMI_MaskD(dwBase + HcCommandStatus, ~0UL, HostControllerReset);
    delay(1); //10us max
    DPMI_StoreD(dwBase + HcFmInterval, FrameInterval);

    // get ports
    pHCI->bNumPorts = DPMI_LoadB(dwBase + HcRhDescriptorA); // low 8bit of a register
    _LOG("Numports: %d\n", pHCI->bNumPorts);

    pHCI->pHCDMethod = &OHCI_Method;
    pHCI->pHCDData = DPMI_DMAMalloc(sizeof(OHCI_HCData), 256);
    OHCI_HCData* pHCDData = (OHCI_HCData*)pHCI->pHCDData;
    memset(pHCDData, 0, sizeof(OHCI_HCData));
    _LOG("HCDDData %08x, Initial memory usage: %d\n", pHCDData, sizeof(OHCI_HCData));

    // port power on
    if(DPMI_LoadD(dwBase + HcRhDescriptorA) & PowerSwitchingMode)
    { // per-port power
        for (uint32_t i = 0; i < pHCI->bNumPorts; i++)
            DPMI_MaskD(dwBase + HcRhPort1Status + i * 4, ~0UL, SetPortPower);
    }
    else
    { // all-port power
        DPMI_MaskD(dwBase + HcRhStatus, ~0UL, SetGlobalPower);
    }
    // hcca & queue heads
    OHCI_BuildHCCA(pHCDData);
    DPMI_StoreD(dwBase + HcHCCA, DPMI_PTR2P(&pHCDData->HCCA));
    //_LOG("HCCA: %04x %08lx\n", &pHCDData->HCCA, DPMI_PTR2P(&pHCDData->HCCA));
    assert((DPMI_PTR2P(&pHCDData->HCCA) & 0xFF) == 0); // ensure alignment
    assert((DPMI_PTR2P(&pHCDData->ControlHead) & 0xF) == 0);

    pHCDData->ControlHead.ControlBits.Skip = 1;
    DPMI_StoreD(dwBase + HcControlHeadED, DPMI_PTR2P(&pHCDData->ControlHead));

    pHCDData->BulkHead.ControlBits.Skip = 1;
    DPMI_StoreD(dwBase + HcBulkHeadED, DPMI_PTR2P(&pHCDData->BulkHead));

    // enable interrupt & lists
    DPMI_StoreD(dwBase + HcInterruptEnable, (0x4000003FL&~StartofFrame) | MasterInterruptEnable);

    DPMI_MaskD(dwBase + HcControl, ~0UL, ControlListEnable);
    DPMI_MaskD(dwBase + HcControl, ~0UL, PeriodicListEnable);
    DPMI_MaskD(dwBase + HcControl, ~0UL, IsochronousEnable);
    DPMI_MaskD(dwBase + HcControl, ~0UL, BulkListEnable);

    DPMI_StoreD(dwBase + HcPeriodicStart, (FrameInterval & 0x3FFF) * 9 / 10); // spec requires 90% for interrupt/iso.

    //DPMI_MaskD(dwBase + HcInterruptEnable, ~0UL, StartofFrame); //1ms interval. not needed for now.
    DPMI_MaskD(dwBase + HcControl, ~InterruptRouting, 0UL);
    // controller goto the USBOPERATIONAL state.
    DPMI_MaskD(dwBase + HcControl, ~HostControllerFunctionalState, (USBOPERATIONAL << HostControllerFunctionalState_SHIFT));

    pHCDData->ControlTail = &pHCDData->ControlHead;
    pHCDData->BulkTail = &pHCDData->BulkHead;
    pHCDData->ED32msTail = &pHCDData->ED32ms;
    pHCDData->ED16msTail = &pHCDData->ED16ms;
    pHCDData->ED8msTail = &pHCDData->ED8ms;
    pHCDData->ED4msTail = &pHCDData->ED4ms;
    pHCDData->ED2msTail = &pHCDData->ED2ms;
    pHCDData->ED1msTail = &pHCDData->ED1ms;
    return TRUE;
}

BOOL OHCI_DeinitController(HCD_Interface* pHCI)
{
    uint32_t dwBase = pHCI->dwBaseAddress;
    if(dwBase)
    {
        //DPMI_MaskD(dwBase + HcCommandStatus, ~0UL, HostControllerReset); //don't do reset on deinit, it won't init again on real hardware
        //delay(1);
        //disable interrupts and lists
        DPMI_MaskD(dwBase + HcInterruptEnable, ~(0x4000003FL|MasterInterruptEnable), 0);
        DPMI_MaskD(dwBase + HcControl, ~ControlListEnable, 0);
        DPMI_MaskD(dwBase + HcControl, ~PeriodicListEnable, 0);
        DPMI_MaskD(dwBase + HcControl, ~IsochronousEnable, 0);
        DPMI_MaskD(dwBase + HcControl, ~BulkListEnable, 0);

        PCI_CMD cmd;
        cmd.reg16 = 0;
        cmd.bits.BusMaster = 0;
        cmd.bits.IOSpace = 0;
        cmd.bits.MemorySpace = 0;
        cmd.bits.InterruptDisable = 1;
        PCI_WriteWord(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, pHCI->PCIAddr.Function, PCI_REGISTER_CMD, cmd.reg16);
        DPMI_UnmapMemory(dwBase);
    }
    if(pHCI->pHCDData)
        DPMI_DMAFree(pHCI->pHCDData);
    pHCI->pHCDData = NULL;
    pHCI->dwBaseAddress = 0;
    return TRUE;
}

BOOL OHCI_ISR(HCD_Interface* pHCI)
{
    if(!pHCI->dwBaseAddress)
        return FALSE;
    uint32_t dwIOBase = pHCI->dwBaseAddress;
    OHCI_HCData* pHCDData = (OHCI_HCData*)pHCI->pHCDData;
    uint32_t Interrupt = DPMI_LoadD(dwIOBase + HcInterruptStatus) & DPMI_LoadD(dwIOBase + HcInterruptEnable);
    //_LOG("OHCI ISR: interrupt: %x, donehead:%x\n", interrupt, pHCDData->HCCA.dwDoneHead);
    if(pHCDData->HCCA.dwDoneHead == 0 && Interrupt == 0)
        return FALSE;
    DPMI_StoreD(dwIOBase + HcInterruptDisable, MasterInterruptEnable); // disable all interrupt

    uint32_t context = pHCDData->HCCA.dwDoneHead ? (WriteBackDoneHead | ((pHCDData->HCCA.dwDoneHead & 0x1)*Interrupt)) : Interrupt;
    if(context & StartofFrame)
    {
        DPMI_StoreD(dwIOBase + HcInterruptStatus, StartofFrame);
        context &= ~StartofFrame;
    }
    if(context & UnrecoverableError) // need reset
    {
        //return TRUE;
    }
    if(context & ScheduleOverrun) // TODO:
    {
    }
    else if(context & MasterInterruptEnable)
        ;
    if(context & FrameNumberOverflow)
    {
        pHCDData->FrameNumberHigh = 0x10000 - ((pHCDData->HCCA.wFrameNumber ^ pHCDData->FrameNumberHigh) & 0x8000L);
        DPMI_StoreD(dwIOBase + HcInterruptStatus, FrameNumberOverflow);
        context &= ~FrameNumberOverflow;
    }
    if(context & ResumeDetected)
    {
        //delay(20); // spec required
        DPMI_MaskD(dwIOBase + HcControl, ~HostControllerFunctionalState, (USBOPERATIONAL << HostControllerFunctionalState_SHIFT));
        DPMI_StoreD(dwIOBase + HcInterruptStatus, ResumeDetected);
        context &= ~ResumeDetected;
    }
    if(context & WriteBackDoneHead) // note: no interupt for control transfer, and interrupt only for last TD of a single transfer request.
    {
        OHCI_ISR_ProcessDoneQueue(pHCI, pHCDData->HCCA.dwDoneHead & ~0x1UL);
        pHCDData->HCCA.dwDoneHead = 0;
        DPMI_StoreD(dwIOBase + HcInterruptStatus, WriteBackDoneHead);
        context &= ~WriteBackDoneHead;
    }
    if(context & RootHubStatusChange) // TODO: device attach/removed
    {
    }
    if(context & ~MasterInterruptEnable) // mask unprocessed interrupts, otherwise ISR will always be called for level triggered interrupt
        DPMI_StoreD(dwIOBase + HcInterruptDisable, context);
    DPMI_StoreD(dwIOBase + HcInterruptEnable, MasterInterruptEnable); // re-enable
    return TRUE;
}

uint8_t OHCI_ControlTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t outputp setup8[8],
    void* nullable pSetupData, uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData)
{
    OHCI_ED* pED = (OHCI_ED*)pEndpoint;
    if(!pCB || !pED || !pED->pTail || pED->ControlBits.TransferType != USB_ENDPOINT_TRANSFER_TYPE_CTRL)
    {
        //printf("Error transfer endpoint: %08p, pointer:%08p, type:%d\n", pEndpoint, pED ? pED : 0, pED ? pED->ControlBits.TransferType : -1);
        return 0xFF;
    }
    CLIS(); //interrupt handler may call transfer function, make sure no mess up of pTail & TailP
    
    //OHCI_HCData* pHCDData = (OHCI_HCData*)pDevice->pHCI->pHCDData;
    //OHCI_HCDeviceData* pDD = (OHCI_HCDeviceData*)pDevice->pHCData;
    assert(pED->ControlBits.FunctionAddress == 0 || (pDevice->bAddress & 0x7F) == pED->ControlBits.FunctionAddress);
    uint32_t dwIOBase = pDevice->pHCI->dwBaseAddress;
    uint8_t pid = (dir == HCD_TXR) ? PIDIN : PIDOUT; //note: status stage inverts direction of data stage according to the USB
    HCD_Request* pRequest = HCD_AddRequest(pDevice, pEndpoint, dir, pSetupData, length, pED->ControlBits.EndPoint, pCB, pCBData);
    OHCI_TD* pEmptyTail = (OHCI_TD*)USB_TAlloc32(sizeof(OHCI_TD));
    memset(pEmptyTail, 0, sizeof(OHCI_TD));
    
    // control queue: setup => [data =>] status => tail. build in reverse order.
    // STATUS TD
    OHCI_TD* pStatus = (OHCI_TD*)USB_TAlloc32(sizeof(OHCI_TD));
    OHCI_BuildTD(pStatus, (uint8_t)PIDINVERT(pid), 0, OHCI_CW_DATATOGGLE_DATA1, 0, 0, pEmptyTail); //interrupt and writeback done head without frame delay
    pStatus->pRequest = pRequest;
    // DATA TD
    OHCI_TD* pData = NULL;
    if(length > 0) //empty data packet won't work on real hardware
    {
        pData = (OHCI_TD*)USB_TAlloc32(sizeof(OHCI_TD));
        OHCI_BuildTD(pData, pid, OHCI_CW_NO_INTERRUPT, OHCI_CW_DATATOGGLE_DATA1, pSetupData ? DPMI_PTR2P(pSetupData) : 0, length, pStatus);
        pData->pRequest = pRequest;
    }
    // SETUP TD
    OHCI_TD* pSetup = pED->pTail;
    OHCI_BuildTD(pSetup, PIDSETUP, OHCI_CW_NO_INTERRUPT, OHCI_CW_DATATOGGLE_DATA0, DPMI_PTR2P(setup8), 8, length > 0 ? pData : pStatus);
    pSetup->pRequest = pRequest;
    // check status
    //_LOG("Setup: %lx, Data: %lx, Status: %lx\n", DPMI_PTR2P(pSetup), pData ? DPMI_PTR2P(pData) : 0, DPMI_PTR2P(pStatus));
    //_LOG("TailP: %lx, HeaP: %lx, EmptyTail: %lx\n", pED->TailP, pED->HeadP, DPMI_PTR2P(pEmptyTail));
    //assert(pED->TailP == pED->HeadP); //note: successive async call of control request may not finish, HeaP not processed to TailP.
    assert(pED->TailP == DPMI_PTR2P(pSetup)); // empty ED. (tailp==headp)
    if(length > 0) assert(pSetup->NextTD == DPMI_PTR2P(pData) && pData->NextTD == DPMI_PTR2P(pStatus) && pStatus->NextTD == DPMI_PTR2P(pEmptyTail));
    // start transfer
    pED->pTail = pEmptyTail;
    //pED->HeadP = pED->TailP;
    pED->TailP = DPMI_PTR2P(pEmptyTail);
    STIL();
#if DEBUG && 0
{
    CLIS();
    DPMI_StoreD(dwIOBase + HcInterruptDisable, MasterInterruptEnable);
    DBG_DBuff db = {1};
    DBG_Printf(&db, "ED TD before transation:\n");
    DBG_Printf(&db, "%08lx: ", DPMI_PTR2P(pED)); DBG_DumpPD(DPMI_PTR2P(pED), 4, &db);
    DBG_Printf(&db, "%08lx: ", DPMI_PTR2P(pSetup)); DBG_DumpPD(DPMI_PTR2P(pSetup), 4, &db);
    if(pData) {DBG_Printf(&db, "%08lx: ", DPMI_PTR2P(pData)); DBG_DumpPD(DPMI_PTR2P(pData), 4, &db);}
    DBG_Printf(&db, "%08lx: ", DPMI_PTR2P(pStatus)); DBG_DumpPD(DPMI_PTR2P(pStatus), 4, &db);

    DPMI_MaskD(dwIOBase + HcCommandStatus, ~0UL, ControlListFilled);
    
    // wait to complete.
    uint32_t i;
    for (i = 0; i < TIME_OUT; ++i)
    {
        if(pED->HeadP == pED->TailP) // finished
            break;
        delay(1);
    }
    delay(1);
    uint8_t errorCode = i == TIME_OUT ? 0xFF : pStatus->ControlBits.ConditionCode; // note: TDs will be deleted in ISR in unpredictable time, need CLI to prevent it.
    if(errorCode != 0)
    {
        printf("control transfer not complete, error: 0x%02x\n", errorCode);
        printf("Address: %d, Endpoint: %d, size: %d\n", pED->ControlBits.FunctionAddress, pED->ControlBits.EndPoint, pED->ControlBits.MaxPacketSize);
        printf("Setup: "); DBG_DumpB(setup8, 8, NULL);
        DBG_Flush(&db);
        printf("ED TD after transation:\n");
        printf("%08lx: ", DPMI_PTR2P(pED)); DBG_DumpPD(DPMI_PTR2P(pED), 4, NULL);
        printf("%08lx: ", DPMI_PTR2P(pSetup)); DBG_DumpPD(DPMI_PTR2P(pSetup), 4, NULL);
        if(pData) {printf("%08lx: ", DPMI_PTR2P(pData)); DBG_DumpPD(DPMI_PTR2P(pData), 4, NULL);}
        printf("%08lx: ", DPMI_PTR2P(pStatus)); DBG_DumpPD(DPMI_PTR2P(pStatus), 4, NULL);
    }
    STIL();
    DPMI_StoreD(dwIOBase + HcInterruptEnable, MasterInterruptEnable);
    return errorCode;
}
#endif
    DPMI_MaskD(dwIOBase + HcCommandStatus, ~0UL, ControlListFilled);
    return 0;
}

uint8_t OHCI_IsochronousTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t* inoutp pBuffer, uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData)
{ // Not tested yet.
    OHCI_ED* pED = (OHCI_ED*)pEndpoint;
    // sanity check
    if(!pCB || !length || !pBuffer || !pED || !pED->pISOTail || pED->ControlBits.TransferType != USB_ENDPOINT_TRANSFER_TYPE_ISOC || (pED->ControlBits.Direction != (dir ? PIDIN : PIDOUT)))
    {
        _LOG("Error transfer endpoint: %08p, pointer:%08p, type:%d\n", pEndpoint, pED ? pED : 0, pED ? pED->ControlBits.TransferType : -1);
        return 0xFF;
    }
    CLIS();    
    HCD_Request* pRequest = HCD_AddRequest(pDevice, pEndpoint, dir, pBuffer, length, pED->ControlBits.EndPoint, pCB, pCBData);
    uint32_t dwIOBase = pDevice->pHCI->dwBaseAddress;
    uint32_t BufferAddress = DPMI_PTR2P(pBuffer);
    uint16_t MaxLen = min(length, pED->ControlBits.MaxPacketSize); // used by Offset. OHCI require maxlen <= max parcket size. although it doesn't requre the HC to perform the check.
    uint16_t StartFrame = (uint16_t)(DPMI_LoadW(dwIOBase + HcFmNumber) + 2); // advance 2 frame incase miss the whole thing. TODO: use SOF?
    uint16_t transferred = 0;
    while(transferred < length)
    {
        uint16_t size = min((uint16_t)(length - transferred), (uint16_t)(MaxLen * OHCI_MAX_ISO_FRAME));
        uint8_t FrameCount = (uint8_t)((size + MaxLen - 1) / MaxLen);
        assert(FrameCount <= OHCI_MAX_ISO_FRAME);
        OHCI_ISO_TD* pNext = (OHCI_ISO_TD*)USB_TAlloc32(sizeof(OHCI_ISO_TD));
        memset(pNext, 0, sizeof(OHCI_ISO_TD));
        uint8_t DelayInterrupt = (transferred + size != length) ? OHCI_CW_NO_INTERRUPT : 0;
        OHCI_BuildISOTD(pED->pISOTail, StartFrame, FrameCount, DelayInterrupt, MaxLen, BufferAddress + transferred, size, pNext);
        pED->pISOTail->pRequest = pRequest; // add request ptr to everty TD incase error occors
        transferred = (uint16_t)(transferred + size);
        StartFrame = (uint16_t)(StartFrame + FrameCount + 1);
        pED->pISOTail = pNext;
    }
    // start transfer
    pED->TailP = DPMI_PTR2P(pED->pISOTail);
    STIL();
    return 0;
}

uint8_t OHCI_DataTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t* inoutp pBuffer, uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData)
{
    OHCI_ED* pED = (OHCI_ED*)pEndpoint;
    // sanity check
    if(!pCB || !length || !pBuffer || !pED || !pED->pTail || (pED->ControlBits.Direction != (dir ? PIDIN : PIDOUT)) || (((uintptr_t)pBuffer) & 0x3) != 0 || (pED->ControlBits.TransferType != USB_ENDPOINT_TRANSFER_TYPE_BULK && pED->ControlBits.TransferType != USB_ENDPOINT_TRANSFER_TYPE_INTR))
    {
        _LOG("Error transfer endpoint: %08p, pointer:%08p, type:%d\n", pEndpoint, pED ? pED : 0, pED ? pED->ControlBits.TransferType : -1);
        return 0xFF;
    }
    CLIS();
    HCD_Request* pRequest = HCD_AddRequest(pDevice, pEndpoint, dir, pBuffer, length, pED->ControlBits.EndPoint, pCB, pCBData);
    uint32_t BufferAddress = DPMI_PTR2P(pBuffer);
    uint16_t transferred = 0;
    while(transferred < length)
    {
        uint16_t len = min(OHCI_MAX_TD_BUFFER_SIZE, (uint16_t)(length - transferred));
        OHCI_TD* pNext = (OHCI_TD*)USB_TAlloc32(sizeof(OHCI_TD));
        memset(pNext, 0, sizeof(OHCI_TD));
        uint8_t DelayInterrupt = (transferred + len != length) ? OHCI_CW_NO_INTERRUPT : 0;
        OHCI_BuildTD(pED->pTail, pED->ControlBits.Direction, DelayInterrupt, OHCI_CW_DATATOGGLE_CARRY, BufferAddress + transferred, len, pNext);
        pED->pTail->pRequest = pRequest; // add request ptr to everty TD incase error occors
        transferred = (uint16_t)(transferred + len);
        pED->pTail = pNext;
    }
    // start transfer (advance TailP)
    pED->TailP = DPMI_PTR2P(pED->pTail);
    STIL();
    if(pED->ControlBits.TransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK) // endpoint already connected to bulk head, issue the command
        DPMI_MaskD(pDevice->pHCI->dwBaseAddress + HcCommandStatus, ~0UL, BulkListFilled);
    return 0;
}

void OHCI_ISR_ProcessDoneQueue(HCD_Interface* pHCI, uint32_t dwDoneHead)
{
    unused(pHCI);
    if(dwDoneHead == 0)
        return;
    //_LOG("PDQ %08lx ", dwDoneHead);
    OHCI_TD* pList = NULL;
    do //reverse done queue to processed order
    {
        OHCI_TD* pTD = (OHCI_TD*)DPMI_P2PTR(dwDoneHead); // could be an ISO_TD
        dwDoneHead = pTD->NextTD;
        pTD->pNext = pList;
        pList = pTD;
    } while(dwDoneHead);

    while(pList != NULL)
    {
        OHCI_TD* pTD = pList;
        pList = pList->pNext;
        uint8_t ccode = pTD->ControlBits.ConditionCode;
        BOOL Interrupt = pTD->ControlBits.DelayInterrupt == 0; // final TD of a transfer request (not tail of quueue)
        HCD_Request* pRequest = pTD->pRequest;
        OHCI_ED* pED = pRequest ? (OHCI_ED*)pRequest->pEndpoint : NULL;

        if(ccode) // error: enpoint halted, remove all pending TODO: could continue on some senarios
        {
            //_LOG("ERROR: %d\n", ccode);
            OHCI_TD* pNext = pTD->pNext;
            while(pNext && pNext->pRequest == pRequest) //reset finished entries with the same request from pList
            {
                pNext->pRequest = NULL;
                pNext = pNext->pNext;
            }
            if(pED)
            {
                pNext = (OHCI_TD*)DPMI_P2PTR(pED->HeadP&~0xFUL);
                while(pNext != pED->pTail) //remove pending entries with the same request from ED
                {
                    //_LOG("%x %x %x %x\n", pNext, pTD, pNext->pRequest, pRequest);
                    OHCI_TD* p = pNext->pNext;
                    if(pNext->pRequest == pRequest)
                    {
                        if(pNext != pTD)
                            USB_TFree32(pNext);
                    }
                    else
                        break; //request are consecutive
                    pNext = p;
                }
                //_LOG("%x %x\n",pNext, DPMI_PTR2P(pNext));
                pED->HeadP = DPMI_PTR2P(pNext);
            }
        }
        assert(!pED || pTD != pED->pTail);  //pTail always empty (un processed)
        if((Interrupt || ccode) && pRequest)
        {
            uint16_t actualLen = pTD->CurrentBufferP == 0 ? pRequest->size : (uint16_t)(pTD->CurrentBufferP - DPMI_PTR2P(pRequest->pBuffer));
            //_LOG("callback: %08lx, %08lx, error: 0x%02x\n", pTD->CurrentBufferP, DPMI_PTR2P(pRequest->pBuffer), ccode);
            HCD_InvokeCallBack(pRequest, actualLen, ccode);
        }
        //_LOG("Free TD: %08lx\n", pTD);
        USB_TFree32(pTD);
    }
}

uint16_t OHCI_GetPortStatus(HCD_Interface* pHCI, uint8_t port)
{
    uint32_t dwBase = pHCI->dwBaseAddress;
    uint32_t status = DPMI_LoadD(dwBase + HcRhPort1Status + port * 4U);

    uint16_t result = 0;
    if(status & PortEnableStatus)
        result |= USB_PORT_ENABLE;
    else
        result |= USB_PORT_DISABLE;

    if(status & CurrentConnectStatus)
        result |= USB_PORT_ATTACHED;

    if(status & PortSuspendStatus)
        result |= USB_PORT_SUSPEND;

    if(status & PortResetStatus)
        result |= USB_PORT_RESET; // temporary state: resetting

    if(status & LowSpeedDeviceAttached)
        result |= USB_PORT_Low_Speed_Device;
    else
        result |= USB_PORT_Full_Speed_Device;

    if(status & ConnectStatusChange)
        result |= USB_PORT_CONNECT_CHANGE;

    return result;
}

BOOL OHCI_SetPortStatus(HCD_Interface* pHCI, uint8_t port, uint16_t status)
{
    uint32_t dwPortAddr = pHCI->dwBaseAddress + HcRhPort1Status + (uint32_t)port * 4;
    uint32_t cur = DPMI_LoadD(dwPortAddr);

    if(!(cur & CurrentConnectStatus))
        return FALSE;

    // setting bits to 0 always has no effect by OHCI spec, so we just set the effective bit
    if(status & USB_PORT_RESET)
    {
        DPMI_StoreD(dwPortAddr, PortResetStatusChange); // clear PortResetStatusChange (write 1 to clear)
        DPMI_StoreD(dwPortAddr, SetPortReset);
        int timeout = 0;
        do
        {
            delay(10); // spec required 10
        } while(!(DPMI_LoadD(dwPortAddr) & PortResetStatusChange) && timeout++ <= 500); // 5 sec time out (Microsoft impl)
        if(timeout > 500)
            return FALSE;
        DPMI_StoreD(dwPortAddr, PortResetStatusChange);
        cur &= ~PortEnableStatus;
        _LOG("OHCI port reset done.\n");
    }

    if((status & USB_PORT_ENABLE) && !(cur & PortEnableStatus))
    {
        DPMI_StoreD(dwPortAddr, SetPortEnable);
        do
        {
            delay(10);
        } while(!(DPMI_LoadD(dwPortAddr) & PortEnableStatus));
        _LOG("OHCI port enable done.\n");
    }

    if((status & USB_PORT_DISABLE) && (cur & PortEnableStatus))
    {
        // DPMI_MaskD(dwPortAddr, ~SetPortEnable, ClearPortEnable);
        DPMI_StoreD(dwPortAddr, ClearPortEnable);
        while((DPMI_LoadD(dwPortAddr) & PortEnableStatus))
            delay(10);
        //_LOG("port disable done.\n");
    }

    if(status & USB_PORT_SUSPEND)
    {
        DPMI_StoreD(dwPortAddr, PortSuspendStatusChange); // clear PortSuspendStatusChange
        DPMI_StoreD(dwPortAddr, SetPortSuspend);
        while(!(DPMI_LoadD(dwPortAddr) & PortSuspendStatusChange))
            delay(10);
        DPMI_StoreD(dwPortAddr, PortSuspendStatusChange);
    }

    if(status & USB_PORT_CONNECT_CHANGE)
    { // write to ConnectStatusChange will clear it
        DPMI_StoreD(dwPortAddr, ConnectStatusChange);
    }
    return TRUE;
}

BOOL OHCI_InitDevice(HCD_Device* pDevice)
{
    pDevice->pHCData = DPMI_DMAMalloc(sizeof(OHCI_HCDeviceData), 16);
    OHCI_HCDeviceData* pDD = (OHCI_HCDeviceData*)pDevice->pHCData;
    memset(pDD, 0, sizeof(OHCI_HCDeviceData));
    _LOG("Device %08x, memory usage: %d\n", pDD, sizeof(OHCI_HCDeviceData));

    uint8_t address = pDevice->bAddress;
    uint8_t lowspeed = (pDevice->bSpeed == USB_PORT_Low_Speed_Device) ? 1 : 0;
    OHCI_BuildED(&pDD->ControlED, address, 0, PIDFROMTD, lowspeed, 4);
    assert(pDD->ControlED.ControlBits.FunctionAddress == pDevice->bAddress);
    OHCI_HCData* pHCDData = (OHCI_HCData*)pDevice->pHCI->pHCDData;
    OHCI_AddEDToTail(&pHCDData->ControlTail, &pDD->ControlED);
    return TRUE;
}

BOOL OHCI_RemoveDevice(HCD_Device* pDevice)
{
    OHCI_HCDeviceData* pDD = (OHCI_HCDeviceData*)pDevice->pHCData;
    USB_TFree32(pDD->ControlED.pTail);
    DPMI_DMAFree(pDD);
    return TRUE;
}

void* OHCI_CreateEndpoint(HCD_Device* pDevice, uint8_t EPAddr, HCD_TxDir dir, uint8_t bTransferType, uint16_t MaxPacketSize, uint8_t bInterval)
{
    OHCI_HCDeviceData* pDD = (OHCI_HCDeviceData*)pDevice->pHCData;
    if(EPAddr == 0)
    {
        assert(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL);
        pDD->ControlED.ControlBits.MaxPacketSize = MaxPacketSize&0x7FFU;
        pDD->ControlED.ControlBits.FunctionAddress = pDevice->bAddress&0x7FU;
        return &pDD->ControlED;
    }
    assert(EPAddr <= 0xF);

    OHCI_HCData* pHCDData = (OHCI_HCData*)pDevice->pHCI->pHCDData;
    uint8_t address = pDevice->bAddress;
    uint8_t lowspeed = (pDevice->bSpeed == USB_PORT_Low_Speed_Device) ? 1 : 0;

    OHCI_ED* pED = (OHCI_ED*)DPMI_DMAMalloc(sizeof(OHCI_ED), 16);
    memset(pED, 0, sizeof(OHCI_ED));

    uint8_t EndPoint = EPAddr;
    uint16_t wMaxSize = MaxPacketSize;
    uint8_t PID = dir == HCD_TXW ? PIDOUT : PIDIN;

    if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_ISOC)
        OHCI_BuildISOED(pED, address, EndPoint, PID, lowspeed, wMaxSize);
    else
        OHCI_BuildED(pED, address, EndPoint, PID, lowspeed, wMaxSize);

    OHCI_ED** ppED;
    if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_ISOC || bTransferType == USB_ENDPOINT_TRANSFER_TYPE_INTR)
        ppED = OHCI_GetEDFromInterval(pHCDData, bInterval);
    else if(bTransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK)
        ppED = &pHCDData->BulkTail;
    else
        ppED = &pHCDData->ControlTail;

    OHCI_AddEDToTail(ppED, pED);
    pED->ControlBits.Skip = 0;
    pED->ControlBits.TransferType = bTransferType&0x3U;
    return pED;
}

BOOL OHCI_RemoveEndpoint(HCD_Device* pDevice, void* pEndpoint)
{
    OHCI_HCData* pHCDData = (OHCI_HCData*)pDevice->pHCI->pHCDData;
    OHCI_HCDeviceData* pDD = (OHCI_HCDeviceData*)pDevice->pHCData;
    //_LOG("%p, %p, %p\n", pHCDData, pDD, pEndpoint);
    if(!pHCDData || !pDD || !pEndpoint)
        return FALSE;
    if(pEndpoint == &pDD->ControlED)
        return TRUE;

    uint32_t dwBase = pDevice->pHCI->dwBaseAddress;
    // disable lists before removing
    DPMI_MaskD(dwBase + HcControl, ~ControlListEnable, 0);
    DPMI_MaskD(dwBase + HcControl, ~PeriodicListEnable, 0);
    DPMI_MaskD(dwBase + HcControl, ~IsochronousEnable, 0);
    DPMI_MaskD(dwBase + HcControl, ~BulkListEnable, 0);
    delay(10);

    OHCI_ED* pED = (OHCI_ED*)pEndpoint;
    // remove ed
    pED->pPrev->NextED = pED->NextED;
    if(pED->ControlBits.TransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL)
    {
        while(DPMI_LoadD(dwBase + HcControlCurrentED) == DPMI_PTR2P(pED));
        if(pHCDData->ControlTail == pED)
            pHCDData->ControlTail = pED->pPrev;
    }
    else if(pED->ControlBits.TransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK)
    {
        while(DPMI_LoadD(dwBase + HcBulkCurrentED) == DPMI_PTR2P(pED));
        if(pHCDData->BulkTail == pED)
            pHCDData->BulkTail = pED->pPrev;
    }
    else
    {
        while(DPMI_LoadD(dwBase + HcPeriodCurrentED) == DPMI_PTR2P(pED)); // TODO: RunningEDReclamation

        if(pHCDData->ED32msTail == pED)
            pHCDData->ED32msTail = pED->pPrev;
        else if(pHCDData->ED16msTail == pED)
            pHCDData->ED32msTail = pED->pPrev;
        else if(pHCDData->ED8msTail == pED)
            pHCDData->ED32msTail = pED->pPrev;
        else if(pHCDData->ED4msTail == pED)
            pHCDData->ED32msTail = pED->pPrev;
        else if(pHCDData->ED2msTail == pED)
            pHCDData->ED32msTail = pED->pPrev;
        else if(pHCDData->ED1msTail == pED)
            pHCDData->ED32msTail = pED->pPrev;
    }
    OHCI_TD* pHead = pED->HeadP != OHCI_NULL_TD_PHYSICAL_ADDR ? (OHCI_TD*)DPMI_P2PTR(pED->HeadP) : NULL;
    while(pHead)
    {
        OHCI_TD* pNext = pHead->pNext;
        //_LOG("pHead: %08lx\n", pHead);
        USB_TFree32(pHead);
        pHead = pNext;
    }
    DPMI_DMAFree(pED);

    // enable lists again
    DPMI_MaskD(dwBase + HcControl, ~0UL, ControlListEnable);
    DPMI_MaskD(dwBase + HcControl, ~0UL, PeriodicListEnable);
    DPMI_MaskD(dwBase + HcControl, ~0UL, IsochronousEnable);
    DPMI_MaskD(dwBase + HcControl, ~0UL, BulkListEnable);
    delay(10);
    return TRUE;
}

void OHCI_AddEDToTail(OHCI_ED** pTail, OHCI_ED* pED)
{
    // assert(pTail->NextED == 0); //interrupt link
    assert(pED->pPrev == NULL && pED->NextED == 0);
    pED->NextED = (*pTail)->NextED;
    (*pTail)->NextED = DPMI_PTR2P(pED);
    pED->pPrev = *pTail;
    *pTail = pED;
}

void OHCI_BuildTD(OHCI_TD* pTD, uint8_t PID, uint8_t DelayInterrupt, uint8_t DataToggle, uint32_t BufferAddr, uint32_t length, OHCI_TD* pNext)
{
    memset(pTD, 0, sizeof(OHCI_TD));
    // MSB LSB
    // CC:4|EC:2|T:2|DI:3|DP:2|R:1|-
    pTD->ControlBits.ConditionCode = OHCI_CC_NOT_ACCESSED;
    pTD->ControlBits.ErrorCount = 0;
    pTD->ControlBits.DataToggle = DataToggle&0x3U;
    pTD->ControlBits.DelayInterrupt = DelayInterrupt&0x7U;
    pTD->ControlBits.PID = PID&0x3U;
    pTD->ControlBits.BufferRounding = 1;

    pTD->NextTD = pNext ? DPMI_PTR2P(pNext) : OHCI_NULL_TD_PHYSICAL_ADDR;
    pTD->CurrentBufferP = BufferAddr;
    pTD->BufferEnd = BufferAddr ? BufferAddr + length - 1 : 0;

    // extension
    pTD->pNext = pNext;
}

void OHCI_BuildISOTD(OHCI_ISO_TD* pTD, uint16_t StartFrame, uint8_t FrameCount, uint8_t DelayInterrupt, uint16_t PacketSize, uint32_t BufferAddr, uint32_t length, OHCI_ISO_TD* pNext)
{
    memset(pTD, 0, sizeof(OHCI_ISO_TD));
    pTD->ControlBits.ConditionCode = OHCI_CC_NOT_ACCESSED;
    pTD->ControlBits.FrameCount = FrameCount&0x7U;
    pTD->ControlBits.DelayInterrupt = DelayInterrupt&0x7U;
    pTD->ControlBits.StartFrame = StartFrame;

    if(pNext)
        pTD->NextTD = DPMI_PTR2P(pNext);
    pTD->BufferPage = BufferAddr&0xFFFFF000; // align to 4K page
    pTD->BufferEnd = length ? BufferAddr + length - 1 : 0;

    for (unsigned i = 0; i <= FrameCount; ++i)
        pTD->Offset[i] = (uint16_t)((BufferAddr & 0xFFF) + i * PacketSize);
}

void OHCI_BuildED(OHCI_ED* pED, uint8_t FunctionAddress, uint8_t Endpoint, uint8_t Direction, uint8_t LowSpeed, uint16_t MaxPackSize)
{
    memset(pED, 0, sizeof(OHCI_ED));
    // MSB LSB
    //-|MPS:11|F:1|K:1|S:1|D:2|EN:4|FA:7
    pED->ControlBits.FunctionAddress = FunctionAddress&0x7FU;
    pED->ControlBits.EndPoint = Endpoint&0xFU;
    pED->ControlBits.Direction = Direction&0x3U;
    pED->ControlBits.LowSpeed = LowSpeed&0x1U;
    pED->ControlBits.Skip = 0;
    pED->ControlBits.Format = 0;
    pED->ControlBits.MaxPacketSize = MaxPackSize&0x7FFU;
    pED->pTail = (OHCI_TD*)USB_TAlloc32(sizeof(OHCI_TD)); // non-transient allocation
    memset(pED->pTail, 0, sizeof(OHCI_TD));
    pED->HeadP = pED->TailP = DPMI_PTR2P(pED->pTail);
}

void OHCI_BuildISOED(OHCI_ED* pED, uint8_t FunctionAddress, uint8_t EndpointNo, uint8_t Direction, uint8_t LowSpeed, uint16_t MaxPackSize)
{
    assert(sizeof(OHCI_TD) == sizeof(OHCI_ISO_TD));
    OHCI_BuildED(pED, FunctionAddress, EndpointNo, Direction, LowSpeed, MaxPackSize);
    pED->ControlBits.Format = 1;
}

void OHCI_BuildHCCA(OHCI_HCData* pHCDData)
{
    uint32_t *ptr = pHCDData->HCCA.InterruptTable;
    uint32_t dwAddress;

    pHCDData->ED32ms.ControlBits.Skip = 1; // build 32 ms
    ptr[0] = DPMI_PTR2P(&pHCDData->ED32ms);

    pHCDData->ED16ms.ControlBits.Skip = 1; // build 16 ms
    dwAddress = DPMI_PTR2P(&pHCDData->ED16ms);
    ptr[1] = dwAddress;
    ptr[1 + 16] = dwAddress;

    pHCDData->ED8ms.ControlBits.Skip = 1; // build 8ms
    dwAddress = DPMI_PTR2P(&pHCDData->ED8ms);
    int i;
    for (i = 2; i < 32; i = i + 8)
        ptr[i] = dwAddress;

    pHCDData->ED4ms.ControlBits.Skip = 1; // build 4ms
    dwAddress = DPMI_PTR2P(&pHCDData->ED4ms);
    for (i = 3; i < 32; i = i + 4)
        ptr[i] = dwAddress;

    pHCDData->ED2ms.ControlBits.Skip = 1; // build 2ms
    dwAddress = DPMI_PTR2P(&pHCDData->ED2ms);
    for (i = 0; i < 32; i = i + 2)
    {
        if(ptr[i] == 0)
        {
            ptr[i] = dwAddress;
            continue;
        }
        // get last ED
        uint32_t dwLast = ptr[i];
        uint32_t dwNext = dwLast;
        while((dwNext = DPMI_LoadD(DPMI_P2L(dwLast) + offsetof(OHCI_ED, NextED))) != 0)
            dwLast = dwNext;
        if(dwLast != dwAddress) // if found different ED.
            DPMI_StoreD(DPMI_P2L(dwLast) + offsetof(OHCI_ED, NextED), dwAddress);
    }

    pHCDData->ED1ms.ControlBits.Skip = 1; // build 1ms
    dwAddress = DPMI_PTR2P(&pHCDData->ED1ms);
    for (i = 0; i < 32; i++)
    {
        if(ptr[i] == 0)
        {
            ptr[i] = dwAddress;
            continue;
        }
        // get last ED
        uint32_t dwLast = ptr[i];
        uint32_t dwNext = dwLast;
        while((dwNext = DPMI_LoadD(DPMI_P2L(dwLast) + offsetof(OHCI_ED, NextED))) != 0)
            dwLast = dwNext;
        if(dwLast != dwAddress) // if found different ED.
            DPMI_StoreD(DPMI_P2L(dwLast) + offsetof(OHCI_ED, NextED), dwAddress);
    }
    return;
}

OHCI_ED** OHCI_GetEDFromInterval(OHCI_HCData* pHCDData, uint8_t interval)
{
    interval = (interval <= 0) ? 1 : interval; //((interval >= 32) ? 32 : interval);

    if(interval <= 1)
        return &pHCDData->ED1msTail;
    else if(interval <= 2)
        return &pHCDData->ED2msTail;
    else if(interval >= 3 && interval <= 5)
        return &pHCDData->ED4msTail;
    else if(interval >= 6 && interval <= 11)
        return &pHCDData->ED8msTail;
    else if(interval >= 12 && interval <= 23)
        return &pHCDData->ED16msTail;
    else
        return &pHCDData->ED32msTail;
}
#endif
