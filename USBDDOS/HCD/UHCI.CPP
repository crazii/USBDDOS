#include <memory.h>
#include <stdio.h>
#include <dos.h>
#include <conio.h>
#include <string.h>
#include <assert.h>
#include "USBDDOS/HCD/UHCI.H"
#include "USBDDOS/DPMI/DPMI.H"
#include "USBDDOS/DBGUTIL.H"

// reference spec:
//                      Universal Host Controller Interface (UHCI)
//                      Universal Serial Bus Specification 2.0
static uint8_t* UHCI_GetHCDBuffer(HCD_Device* pDevice);
static uint16_t UHCI_GetPortStatus(HCD_Interface* pHCI, uint8_t port);
static BOOL UHCI_SetPortStatus(HCD_Interface* pHCI, uint8_t port, uint16_t status);
static BOOL UHCI_InitDevice(HCD_Device* pDevice);
static BOOL UHCI_CreateEndpoint(HCD_Device* pDevice, uint8_t EPAddr, HCD_TxDir dir, uint8_t bTransferType, uint16_t MaxPacketSize, uint8_t bInterval);
static BOOL UHCI_RemoveDevice(HCD_Device* pDevice);


void    UHCI_InitPort(HCD_Device* ptr,uint8_t portIndex);

static  void UHCI_BuildTD(UHCI_TD* pTD, uint32_t LinkPointer, uint32_t  ControlStatus,uint8_t PID, uint8_t DevAddr, uint8_t EndPt, uint8_t DataTog, uint16_t MaxLen, uint32_t buffer);
static  void UHCI_BuildQH(UHCI_QH* pQH, uint32_t HeadLink, uint32_t ElementLink);
static  void UHCI_QHTDSchedule(HCD_Interface * pHCI);
static  void UHCI_InsertTDintoQH(UHCI_QH* pQH, UHCI_TD* pTD);
static  void UHCI_InsertQHintoQH(UHCI_QH* pToQH, UHCI_QH* pQH);
static  void UHCI_ResetHC(uint16_t base);
static  void UHCI_StartHC(uint16_t base);
static  void UHCI_StopHC(uint16_t base);
static  void UHCI_ResetPort(uint16_t base,uint8_t portnum);
static  void UHCI_EnablePort(uint16_t base,uint8_t portnum);
static  int  UHCI_WaitTDDone(UHCI_TD* pTD);
static UHCI_QH* UHCI_GetQHFromInterval(UHCI_HCDD* pHCDD, uint8_t interval);

HCD_Method UHCIAccessMethod =
{
    &UHCI_ControlTransfer,
    &UHCI_IsochronousTransfer,
    &UHCI_BulkTransfer,
    &UHCI_InterruptTransfer,
    &UHCI_GetHCDBuffer,
    &UHCI_GetPortStatus,
    &UHCI_SetPortStatus,
    &UHCI_InitDevice,
    &UHCI_CreateEndpoint,
    &UHCI_RemoveDevice,
};

BOOL UHCI_InitController(HCD_Interface * pHCI)
{
    // stop USB SMI
    uint8_t pciValue = PCI_Read_Byte(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, 0, 0xc0);
    pciValue &= 0xe0; // clear  bit4 =  usb smi enable
    PCI_Write_Byte(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, 0, 0xc0, pciValue);

    pHCI->dwBaseAddress = PCI_Read_DWord(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, pHCI->PCIAddr.Function, 0x20) & 0xffffffe0;
    pHCI->pHCDMethod = &UHCIAccessMethod;

    pHCI->pHCDData = DPMI_MappedMalloc(sizeof(UHCI_HCDD), 16);
    UHCI_HCDD* pHCDD = (UHCI_HCDD*)pHCI->pHCDData;
    memset(pHCDD, 0, sizeof(UHCI_HCDD));

    //2 get host controller info
    pHCI->bNumPorts= 2;
    pHCDD->dHcdDataArea = inpd(pHCI->dwBaseAddress + FLBASEADD ) & 0xfffff000;
    pHCDD->dHcdDataArea = DPMI_MapMemory(pHCDD->dHcdDataArea, 4096);    //1024 entry, 4 bytes per entry.
    UHCI_BuildQH(&pHCDD->QH1ms, 0, 0);
    UHCI_BuildQH(&pHCDD->QH2ms, 0, 0);
    UHCI_BuildQH(&pHCDD->QH8ms, 0, 0);

    //2. setup framelist
    UHCI_StopHC(pHCI->dwBaseAddress);
    UHCI_QHTDSchedule(pHCI);
    UHCI_StartHC(pHCI->dwBaseAddress);
    return TRUE;
}

BOOL    UHCI_DeinitController(HCD_Interface* pHCI)
{
    assert(FALSE);//not implemented
    return FALSE;
}

uint8_t  UHCI_ControlTransfer(HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir, uint8_t outputp pSetup[8], void* nullable pSetupData, uint16_t length)
{
    UHCI_HCDD* pHCDD = (UHCI_HCDD*)pDevice->pHCI->pHCDData;
    UHCI_DD* pDD = (UHCI_DD*)pDevice->pHCData;
    memcpy(pDD->Setup, pSetup, 8);

    UHCI_TD* pSetupTD = &pDD->TD[0];
    UHCI_TD* pStatusTD = &pDD->TD[1];
    UHCI_TD* pDataTD = &pDD->TD[2];

    //2. build setup UHCI_TD
    uint8_t bAddress = pDevice->bAddress;
    uint16_t MaxLength = 0;
    uint32_t CS = ActiveStatus | C_ERR;
    if(pDevice->bEndpointSpeed == 01) CS = CS | LowSpeed;

    UHCI_BuildTD(pSetupTD, 0, CS, SETUPPID, bAddress, Endpoint, 0, sizeof(USB_Request)-1, DPMI_PTR2P(&pDD->Setup));
    UHCI_InsertTDintoQH(&(pHCDD->QH2ms), pSetupTD);
    if(!UHCI_WaitTDDone(pSetupTD))
    {
    #if DEBUG && 0
        printf("setup not ready\n");
    #endif
    }
    #if DEBUG
    DBG_DumpB((uint8_t*)pSetupTD, 16);
    #endif

    //2. build DATA UHCI_TD
    uint8_t DataPID = (dir == HCD_TXW) ? OUTPID : INPID;
    int  DataLengthBegin = length;

    if(DataLengthBegin)
    {
        int DataLengthTransferred = 0;
        int DataLength;
        uint8_t DataTogg = 1;

        if(DataLengthBegin >= MaxLength) DataLength = MaxLength;
        else DataLength = DataLengthBegin;

        while(DataLengthBegin > 0)
        {
            UHCI_BuildTD(pDataTD, 0, CS, DataPID, bAddress, Endpoint, DataTogg, DataLength-1, DPMI_PTR2P(pSetupData) + DataLengthTransferred);
            UHCI_InsertTDintoQH(&(pHCDD->QH2ms), pDataTD);

            if(!UHCI_WaitTDDone(pDataTD))
            {
            #if DEBUG
                printf("data phase not ready\n");
            #endif
            }
            #if DEBUG
            DBG_DumpB((uint8_t*)pDataTD, 16);
            #endif
            DataTogg ^= 1;

            uint32_t tempValue1 = pDataTD->ControlStatus & 0x7ff;
            if(tempValue1 !=  DataLength-1) break; // transferred data is int16_t packet.

            DataLengthBegin -= DataLength;
            DataLengthTransferred += DataLength;

            if(DataLengthBegin<MaxLength) DataLength = DataLengthBegin;
        }
    }

    //2. build status UHCI_TD
    UHCI_BuildTD(pStatusTD, 0, CS, DataPID, bAddress, Endpoint, 1, 0x7ff, 0);
    UHCI_InsertTDintoQH(&(pHCDD->QH2ms), pStatusTD);
    if(!UHCI_WaitTDDone(pStatusTD))
    {
    #if DEBUG && 0
        printf("status phase not ready\n");
    #endif
    }
    UHCI_InsertTDintoQH(&(pHCDD->QH2ms), 0);

    return (pStatusTD->ControlStatus >> 16) & 0xF;
}

uint8_t UHCI_IsochronousTransfer(HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir, uint8_t* inoroutp pBuffer, uint16_t length, uint16_t* outputp txlen)
{
    UHCI_HCDD* pHCDD = (UHCI_HCDD*)pDevice->pHCI->pHCDData;
    UHCI_DD* pDD = (UHCI_DD*)pDevice->pHCData;
    uint8_t USBAddress = pDevice->bAddress;

    uint8_t PID;
    uint8_t EndPoint;
    uint16_t MaxLen;
    uint8_t Toggle;

    if(MaxLen == 0)    //no bulk support
        return 0xFF;

    MaxLen = MaxLen <= USB_FRAME_SIZE_LOWSPEED ? MaxLen : USB_FRAME_SIZE_LOWSPEED;    //spec max 1280 bytes in one frame (1ms)

    //send UHCI_TD one by one; or send all UHCI_TD one time
    uint8_t Cycles= length / MaxLen;
    if(length % MaxLen) Cycles++;

    UHCI_TD* pTD = &pDD->TD[0];

    uint32_t CS = ActiveStatus | C_ERR | ISO;
    if(pDevice->bEndpointSpeed == 01) CS = CS | LowSpeed;

    uint16_t TfedLength = 0;
    uint16_t PacketLength;
    if(Cycles == 1) PacketLength = length;
    else PacketLength = MaxLen;

    for(int i = 0; i < Cycles; i++)
    {
        UHCI_BuildTD(pTD,0, CS,  PID,USBAddress, EndPoint, Toggle, PacketLength-1, DPMI_PTR2P(pBuffer) + TfedLength);
        //it's real time single task driver thus doesn't scheule TDs otherweise we need complicated setup for queues
        UHCI_InsertTDintoQH(&pHCDD->QH1ms, pTD);    //priortize over other transfer, use 1ms

        if(!UHCI_WaitTDDone(pTD))
        {
        #if DEBUG
            printf("bulk transferred error\n");
        #endif
        }

        #if DEBUG
        DBG_DumpB((uint8_t*)pTD, 16);
        #endif
        Toggle ^=1;

        uint32_t tempValue = pTD->ControlStatus & 0x7ff;
        TfedLength += (uint16_t)tempValue +1 ;
        *txlen = TfedLength;

        if(tempValue != PacketLength-1)    break;

        if(TfedLength + MaxLen > length) PacketLength = length - TfedLength; //last packet

        *txlen = TfedLength;

        if(pTD->ControlStatus & (DataBufferError | BabbleDetected | NAKReceived | BitstuffError))
            break;
    }

    UHCI_InsertTDintoQH(&pHCDD->QH1ms, 0);

    return (pTD->ControlStatus >> 16) & 0xF;
}

uint8_t UHCI_BulkTransfer(HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir, uint8_t* inoroutp pBuffer, uint16_t length, uint16_t* outputp txlen)
{
    UHCI_HCDD* pHCDD = (UHCI_HCDD*)pDevice->pHCI->pHCDData;
    UHCI_DD* pDD = (UHCI_DD*)pDevice->pHCData;
    uint8_t USBAddress = pDevice->bAddress;

    uint8_t PID;
    uint8_t EndPoint;
    uint16_t MaxLen;
    uint8_t Toggle;

    if(MaxLen == 0)    //no bulk support
        return 0xFF;

    MaxLen = MaxLen <= 1280 ? MaxLen : 1280;    //spec max 1280 bytes in one frame (1ms)

    //send UHCI_TD one by one; or send all UHCI_TD one time
    uint8_t Cycles= length / MaxLen;
    if(length % MaxLen) Cycles++;

    UHCI_TD* pTD = &pDD->TD[0];

    uint32_t CS = ActiveStatus | C_ERR;
    if(pDevice->bEndpointSpeed == 01) CS = CS | LowSpeed;

    uint16_t TfedLength = 0;
    uint16_t PacketLength;
    if(Cycles == 1) PacketLength = length;
    else PacketLength = MaxLen;

    for(int i = 0; i< Cycles; i++)
    {
        UHCI_BuildTD(pTD,0, CS,  PID,USBAddress, EndPoint, Toggle, PacketLength-1, DPMI_PTR2P(pBuffer) + TfedLength);
        UHCI_InsertTDintoQH(&pHCDD->QH2ms, pTD);

        if(!UHCI_WaitTDDone(pTD))
        {
        #if DEBUG
            printf("bulk transferred error\n");
        #endif
        }

        #if DEBUG
        DBG_DumpB((uint8_t*)pTD, 16);
        #endif
        Toggle ^=1;

        uint32_t tempValue = pTD->ControlStatus & 0x7ff;
        TfedLength += (uint16_t)tempValue +1 ;
        *txlen = TfedLength;

        if(tempValue != PacketLength-1)    break;

        if(TfedLength + MaxLen > length) PacketLength = length - TfedLength; //last packet

        *txlen = TfedLength;

        if(pTD->ControlStatus & (DataBufferError | BabbleDetected | NAKReceived | BitstuffError))
            break;
        delay(1);    //bulk uses 2ms interval and if one frame (1ms) complete, wait another
    }

    UHCI_InsertTDintoQH(&pHCDD->QH2ms, 0);

    return (pTD->ControlStatus >> 16) & 0xF;
}

uint8_t UHCI_InterruptTransfer(HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir, uint8_t* inoroutp pBuffer, uint16_t length, uint16_t* outputp txlen)
{
    UHCI_HCDD* pHCDD = (UHCI_HCDD*)pDevice->pHCI->pHCDData;
    UHCI_DD* pDD = (UHCI_DD*)pDevice->pHCData;
    uint8_t  USBAddress = pDevice->bAddress;

    uint8_t  PID;
    uint8_t  EndPoint;
    uint8_t  Toggle;
    uint8_t  status = 0;

    UHCI_TD* pTD = &pDD->TD[0];

    uint32_t CS = ActiveStatus | C_ERR | IOC;
    if(pDevice->bEndpointSpeed == 01) CS = CS | LowSpeed;

    uint16_t PacketLength;
    PacketLength = length;

    UHCI_QH* pTargetQH = UHCI_GetQHFromInterval(pHCDD, 1);

    UHCI_BuildTD(pTD, 0, CS,  PID,USBAddress, EndPoint, Toggle, PacketLength-1, DPMI_PTR2P(pBuffer));
    UHCI_InsertTDintoQH(pTargetQH, pTD);

    if(!UHCI_WaitTDDone(pTD))
    {
    #if DEBUG
        printf("bulk transferred error\n");
    #endif
    }

    #if DEBUG
    DBG_DumpB((uint8_t*)pTD, 16);
    #endif
    Toggle ^=1;

    *txlen = (uint16_t)(pTD->ControlStatus & 0x7ff) + 1;

    return (pTD->ControlStatus >> 16) & 0xF;
}

uint8_t* UHCI_GetHCDBuffer(HCD_Device* ptr)
{
    return ((UHCI_DD*)ptr->pHCData)->buffer1;
}

uint16_t UHCI_GetPortStatus(HCD_Interface* pHCI, uint8_t port)
{
    // if(inpw(pDevice->pHCI->dBaseAddress + PORTBASE + (portIndex -1)*2 ) & LSDA)
    //     pDevice->bEndpointSpeed = USB_PORT_Low_Speed_Device ; // 01 = low speed device.
      // else
    //       pDevice->bEndpointSpeed = USB_PORT_Full_Speed_Device; // 2 = full speed

    //2.. init device
    /*
    for(uint8_t i =1; i<=2; i++)//only 2 ports
    {
        if(inpw(pHCI->dBaseAddress + PORTBASE + (i-1) *2)   &  CCS) //device exist on port
        {
            UHCI_ResetPort(pHCI->dBaseAddress, i);
            UHCI_EnablePort(pHCI->dBaseAddress, i);
            USB_InitDevice(pHCI, i); //  init here.
        }
        else //port is
        {
            continue;
        }
    }*/


    return 0;
}
BOOL UHCI_SetPortStatus(HCD_Interface* pHCI, uint8_t port, uint16_t status)
{
    return TRUE;
}
BOOL UHCI_InitDevice(HCD_Device* pDevice)
{
    return TRUE;
}
BOOL UHCI_CreateEndpoint(HCD_Device* pDevice, uint8_t EPAddr, HCD_TxDir dir, uint8_t bTransferType, uint16_t MaxPacketSize, uint8_t bInterval)
{
    return TRUE;
}
BOOL UHCI_RemoveDevice(HCD_Device* pDevice)
{
    return TRUE;
}

void UHCI_QHTDSchedule(HCD_Interface* pHCI)
{
    UHCI_HCDD* pHCDD = (UHCI_HCDD*)pHCI->pHCDData;
    uint32_t pHCDAREA = pHCDD->dHcdDataArea;

    UHCI_QH* pQH = &pHCDD->QH1ms;
    //3set QH invalid accord to UHCIDescriptor definition.
    int i;
    for(i = 0; i < 3; i++)
    {
        pQH[i].HeadLink = TerminateFlag | QHFlag;
        pQH[i].ElementLink = TerminateFlag;
    }

    //3 clear frame list
    for(i = 0; i < 1024; i++)
    {
        DPMI_StoreD(pHCDAREA + i * 4, TerminateFlag);
    }

    //3 link 8ms qh
    for(i = 0; i < 1024; i = i+8)
    {
        DPMI_StoreD(pHCDAREA + i *4, DPMI_PTR2P(&pHCDD->QH8ms) | QHFlag);
    }

    //3 link 2ms qh
    for(i = 1; i < 1024; i = i+2)
    {
        DPMI_StoreD(pHCDAREA + i *4, DPMI_PTR2P(&pHCDD->QH2ms) | QHFlag);
    }

    //3 link 1ms qh. UHCI frame time is 1ms and 1ms QH should apear on all entries in frame list
    for(i = 0; i < 1024; i++)
    {
        if(DPMI_LoadD(pHCDAREA + i *4) == TerminateFlag )
        {
            DPMI_StoreD(pHCDAREA + i *4, DPMI_PTR2P(&pHCDD->QH1ms) | QHFlag);
        }
        else
        {
            UHCI_InsertQHintoQH(&pHCDD->QH8ms, &pHCDD->QH1ms);
            UHCI_InsertQHintoQH(&pHCDD->QH2ms, &pHCDD->QH1ms);
        }
    }

    return;
}

void  UHCI_BuildTD(UHCI_TD* pTD, uint32_t LinkPointer, uint32_t  ControlStatus,uint8_t PID, uint8_t DevAddr, uint8_t EndPt, uint8_t DataTog, uint16_t MaxLen, uint32_t buffer)
{
    UHCI_TD TDbuffer = {0};
    uint32_t temp = 0;
    if(LinkPointer) TDbuffer.LinkPointer = LinkPointer | DepthFirstFlag;
    else TDbuffer.LinkPointer = TerminateFlag;

    TDbuffer.ControlStatus = ControlStatus;

    temp =0;
    temp=MaxLen;
    temp <<=6;
    temp +=    EndPt;
    temp <<=7;
    temp += DevAddr;
    temp <<=8;
    temp +=  PID;
    if (DataTog) temp ^= DataToggle;
    TDbuffer.token = temp;

    TDbuffer.BufferPoint = buffer;
    memcpy(pTD, &TDbuffer, sizeof(UHCI_TD));
    return;
}

void UHCI_BuildQH(UHCI_QH* pQH, uint32_t HeadLink, uint32_t ElementLink)
{
    UHCI_QH QHbuffer = {0};
    QHbuffer.HeadLink = HeadLink;
    QHbuffer.ElementLink = ElementLink;

    //assume HeadLink is QH.
    if(HeadLink)  QHbuffer.HeadLink |= QHFlag;
    else QHbuffer.HeadLink |= TerminateFlag;
    //assume ElementLink is UHCI_TD.
    if(ElementLink == 0) QHbuffer.ElementLink |= TerminateFlag;

    memcpy(pQH, &QHbuffer, sizeof(UHCI_QH));
    return;
}

void UHCI_InsertTDintoQH(UHCI_QH* pQH, UHCI_TD* pTD)
{
    pQH->ElementLink = pTD ? DPMI_PTR2P(pTD) : TerminateFlag;
}

void UHCI_InsertQHintoQH(UHCI_QH* pToQH, UHCI_QH* pQH)
{
    pToQH->HeadLink = pQH ? (DPMI_PTR2P(pQH) | QHFlag) : TerminateFlag;
    return;
}

int UHCI_WaitTDDone(UHCI_TD* pTD)
{
    int  i = 0;
    int  WaitTime = 5000;
    for(;i < WaitTime; i++)
    {
        if((pTD->ControlStatus&ActiveStatus)) delay(1);
        else break;
    }
    if(i >= WaitTime) return 0;  //error.
    else return 1;
}

void UHCI_ResetHC(uint16_t base)
{
    outpw(base + USBCMD,HCRESET | GRESET);
    delay(100);     //spec require
    outpw(base + USBCMD, 0);
    return;
}

void UHCI_StopHC(uint16_t base)
{
    outpw(base + USBCMD, 0x00);
    return;
}

void UHCI_StartHC(uint16_t base)
{
    outpw(base + USBCMD, 0xc1);
    return;
}

void UHCI_ResetPort(uint16_t base,uint8_t portnum)
{
    uint16_t portIndex;
    if(portnum == 1) portIndex = PORT1;
    else portIndex = PORT2;

    outpw(base + portIndex, CCS | CSC | PED | PEDC | PR);
    delay(100);

    return;
}

void UHCI_EnablePort(uint16_t base,uint8_t portnum)
{
    uint16_t portIndex;
    if(portnum == 1) portIndex = PORT1;
    else portIndex = PORT2;

    outpw(base + portIndex, CCS | CSC | PED | PEDC);
    delay(1);

    outpw(base+ portIndex, CCS | CSC | PED | PEDC);  // twice ?????
    delay(200);

    return;
}


UHCI_QH* UHCI_GetQHFromInterval(UHCI_HCDD* pHCDD, uint8_t interval)
{
    interval = interval < 1 ? 1 : interval;
    if(interval == 1)
        return &pHCDD->QH1ms;
    else if(interval <= 4)
        return &pHCDD->QH2ms;
    else
        return &pHCDD->QH8ms;
}
