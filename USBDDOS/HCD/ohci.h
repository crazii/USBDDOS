#ifndef _OHCI_H_
#define _OHCI_H_
#include "USBDDOS/HCD/hcd.h"

//pci configuration space
#define OHCI_REGISTER_BAR       0x10    //BAR_OHCI (BAR0)

#define HcRevision              0L

#define HcControl               4L  //HcControl Register
#define ControlBulkServiceRatio (BIT0 | BIT1)   //HcControl bits
#define PeriodicListEnable      BIT2
#define IsochronousEnable       BIT3
#define ControlListEnable       BIT4
#define BulkListEnable          BIT5

#define HcCommandStatus         8L
#define HostControllerReset     BIT0
#define ControlListFilled       BIT1
#define BulkListFilled          BIT2
#define OwnershipChangeRequest  BIT3
#define InterruptRouting        BIT8
#define HostControllerFunctionalState (BIT6 | BIT7)
#define HostControllerFunctionalState_SHIFT 6
#define USBRESET                0UL
#define USBRESUME               1UL
#define USBOPERATIONAL          2UL
#define USBSUSPEND              3UL

#define HcInterruptStatus       0x0CL
#define ScheduleOverrun         BIT0
#define WriteBackDoneHead       BIT1
#define StartofFrame            BIT2
#define ResumeDetected          BIT3
#define UnrecoverableError      BIT4
#define FrameNumberOverflow     BIT5
#define RootHubStatusChange     BIT6
#define OwnershipChange         BIT30

#define HcInterruptEnable       0x10L   //bits as HcInterruptStatus, plus MasterInterruptEnable
#define HcInterruptDisable      0x14L   //bits as HcInterruptStatus, plus MasterInterruptEnable
#define MasterInterruptEnable   BIT31

#define HcHCCA                  0x18L
#define HcPeriodCurrentED       0x1CL
#define HcControlHeadED         0x20L
#define HcControlCurrentED      0x24L
#define HcBulkHeadED            0x28L
#define HcBulkCurrentED         0x2CL
#define HcDoneHead              0x30L
#define HcFmInterval            0x34L
#define HcFmRemaining           0x38L
#define HcFmNumber              0x3CL
#define HcPeriodicStart         0x40L
#define HcLSThreshold           0x44L

#define HcRhDescriptorA         0x48L
#define PowerSwitchingMode      BIT8

#define HcRhDescriptorB         0x4CL

#define HcRhStatus              0x50L
#define SetGlobalPower          BIT16

#define HcRhPort1Status         0x54L
#define CurrentConnectStatus    BIT0
#define ClearPortEnable         BIT0
#define PortEnableStatus        BIT1
#define SetPortEnable           BIT1
#define PortSuspendStatus       BIT2
#define SetPortSuspend          BIT2
#define PortOverCurrentIndicator BIT3
#define ClearSuspendStatus      BIT3
#define PortResetStatus         BIT4
#define SetPortReset            BIT4
#define PortPowerStatus         BIT8
#define SetPortPower            BIT8
#define LowSpeedDeviceAttached  BIT9
#define ClearPortPower          BIT9
#define ConnectStatusChange     BIT16
#define PortEnableStatusChange  BIT17
#define PortSuspendStatusChange BIT18
#define PortOverCurrentIndicatorChange BIT19
#define PortResetStatusChange   BIT20

#define PIDFROMTD               0 //for control ED PID
#define PIDSETUP                0
#define PIDOUT                  1
#define PIDIN                   2
#define PIDINVERT(pid)          (3-pid)

//control words
#define OHCI_CW_NO_INTERRUPT   7    //delay counter, 111 means disable donehead interrupt
#define OHCI_CW_DATATOGGLE_CARRY    0
#define OHCI_CW_DATATOGGLE_DATA0    2 //10b
#define OHCI_CW_DATATOGGLE_DATA1    3 //11b

//codition code
#define OHCI_CC_NO_ERROR        0L
#define OHCI_CC_ERROR_CRC       0x1L
#define OHCI_CC_ERROR_STALL     0x4L
#define OHCI_CC_ERROR_DEVICE_NOT_RESPONDING 0x5L
#define OHCI_CC_ERROR_PID_CHECK_FAILURE 0x6L
#define OHCI_CC_NOT_ACCESSED    0xEL    //'untouched'mark. both E,F are OK, inited by host driver

//0~8192 according to the spec (max 1 page crossing, need aligned to 4K if large than 4K) for each TD.
#define OHCI_MAX_TD_BUFFER_SIZE 4096
#define OHCI_NULL_TD_PHYSICAL_ADDR 0
//magic to distinguish TD from ISO_TD. make sure LSB 1 bits is 1, so that it won't conflict with Offset field of ISO_TD. 
//#define OHCI_TD_MAGIC (((uint32_t)('O')<<24) | ((uint32_t)('H' | 0x1)<<16) | (('C')<<8) | ('I') | 0x1)

#ifdef __cplusplus
extern "C"
{
#endif
struct OHCI_TransferDescriptor;
typedef struct OHCI_TransferDescriptor OHCI_TD;
struct OHCI_IsochronousTransferDescriptor;
typedef struct OHCI_IsochronousTransferDescriptor OHCI_ISO_TD;
struct OHCI_EndpointDescriptor;
typedef struct OHCI_EndpointDescriptor OHCI_ED;

typedef struct OHCI_TransferDescriptor
{
    union
    {
        uint32_t ControlFlags;
        struct //use 16bit bitfield for 16bit compiler compatibility i.e. Borland C
        {
            uint16_t Reserved1; //aviable to HCD (but not modifiable during processing)
            uint16_t Reserved2 : 2; //aviable to HCD (but not modifiable during processing)
            uint16_t BufferRounding : 1;
            uint16_t PID : 2;
            uint16_t DelayInterrupt : 3;
            uint16_t DataToggle : 2;
            uint16_t ErrorCount : 2;
            uint16_t ConditionCode : 4;
        }ControlBits;
    };
    uint32_t  CurrentBufferP;
    uint32_t  NextTD;
    uint32_t  BufferEnd;

    //extension. the OHCI spec only need the alignment as 16
    void* Unused[2];
#if defined(__BC__) || defined(__WC__)
    uint16_t padding[4];
#endif
    OHCI_TD* pNext; //Match the location of ISO TD
    HCD_Request* pRequest;
}OHCI_TD;

#define OHCI_MAX_ISO_FRAME 4    //max frame for a ISO TD. 8 by spec, reserve last 4 for custom information
typedef struct OHCI_IsochronousTransferDescriptor
{
    union
    {
        uint32_t ControlFlags; //condition code in it will be updated by hc
        struct
        {
            uint16_t StartFrame;
            uint16_t Reserved1 : 5;

            uint16_t DelayInterrupt : 3;
            uint16_t FrameCount : 3;
            uint16_t Reserved2 : 1;
            uint16_t ConditionCode : 4;
        }ControlBits;
    };
    uint32_t BufferPage;
    uint32_t NextTD;
    uint32_t BufferEnd;

    uint16_t Offset[OHCI_MAX_ISO_FRAME]; //max frames data by the spec
#if defined(__BC__) || defined(__WC__)
    uint16_t padding[2];
#endif
    OHCI_TD* pNext;
    HCD_Request* pRequest;
}OHCI_ISO_TD;

typedef struct OHCI_EndpointDescriptor
{
    union
    {
        uint32_t  ControlFlags;
        struct
        {
            uint16_t FunctionAddress : 7;
            uint16_t EndPoint : 4;
            uint16_t Direction : 2;
            uint16_t LowSpeed : 1;
            uint16_t Skip : 1;      //Skip: don't process
            uint16_t Format : 1;    //Format: Isochronous mode ED
            uint16_t MaxPacketSize : 11;
            //uint16_t Reserved : 5; //reserved for HCD
            uint16_t TransferType : 2; //HCD
            uint16_t Unsued : 3;    //HCD
        }ControlBits;
    };
    uint32_t  TailP;
    uint32_t  HeadP;
    uint32_t  NextED;

    //extension
    OHCI_ED* pPrev;
    union
    {
        OHCI_TD* pTail;         //tail TD
        OHCI_ISO_TD* pISOTail;  //tail ISO TD
    };
    void* Unused[2];
#if defined(__BC__) || defined(__WC__)
    uint16_t padding[4];
#endif
}OHCI_ED;

static_assert(sizeof(OHCI_ED) == 32, "size missmatch");
static_assert(sizeof(OHCI_TD) == 32, "size missmatch");
static_assert(sizeof(OHCI_ISO_TD) == 32, "size missmatch");

typedef struct
{
    uint32_t InterruptTable[32]; //HccaInterrruptTable
    uint16_t wFrameNumber;
    uint16_t wPadding;
    uint32_t dwDoneHead;
    uint32_t Reserved[29];
    uint32_t dwPadding; //extra custom padding to 256
}OHCI_HCCA_BLOCK;

typedef struct OHCI_HostControllerData //per hc data
{
    OHCI_HCCA_BLOCK HCCA; //hcca, 256-aligned
    OHCI_ED ControlHead;
    OHCI_ED BulkHead;

    //periodic data, isochronous and interrupt
    //interval frames, 1ms~32ms for full speed (1frame=1ms), 0.125ms~4ms for high speed (1frame=0.125ms)
    OHCI_ED ED32ms;
    OHCI_ED ED16ms;
    OHCI_ED ED8ms;
    OHCI_ED ED4ms;
    OHCI_ED ED2ms;
    OHCI_ED ED1ms;

    OHCI_ED* ControlTail;
    OHCI_ED* BulkTail;
    OHCI_ED* ED32msTail;
    OHCI_ED* ED16msTail;
    OHCI_ED* ED8msTail;
    OHCI_ED* ED4msTail;
    OHCI_ED* ED2msTail;
    OHCI_ED* ED1msTail;

    uint32_t FrameNumberHigh;
}OHCI_HCData;

typedef struct OHCI_HostControllerDeviceData //per device data
{
    //default control pipe (ed0), used before/after addressed.
    OHCI_ED ControlED;
    
}OHCI_HCDeviceData;

BOOL OHCI_InitController(HCD_Interface* pHCI, PCI_DEVICE* pPCIDev);
BOOL OHCI_DeinitController(HCD_Interface* pHCI);
BOOL OHCI_ISR(HCD_Interface* pHCI);

uint8_t OHCI_ControlTransfer(HCD_Device* pDevice, void* pEndPoint, HCD_TxDir dir, uint8_t inputp setup8[8],
    void* nullable pSetupData, uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData);

uint8_t OHCI_IsochronousTransfer(HCD_Device* pDevice, void* pEndPoint, HCD_TxDir dir, uint8_t* inoutp pBuffer,
    uint16_t length, HCD_COMPLETION_CB nullable pCB, void* nullable pCBData);

uint8_t OHCI_DataTransfer(HCD_Device* pDevice, void* pEndPoint, HCD_TxDir dir, uint8_t* inoutp pBuffer,
    uint16_t length, HCD_COMPLETION_CB nullable pCB, void* nullable pCBData);

#ifdef __cplusplus
}
#endif

#endif
