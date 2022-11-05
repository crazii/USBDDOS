#ifndef _OHCI_H_
#define _OHCI_H_
#include "USBDDOS/HCD/HCD.H"

//pci configuration space
#define OCH_REGISTER_CMD        0x04
#define OHCI_REGISTER_BAR       0x10    //BAR_OHCI

//BAR_OHCI
#define HcRevision              0L

#define HcControl               4L
#define ControlBulkServiceRatio (BIT0 | BIT1)
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
#define USBRESET                0L
#define USBRESUME               1L
#define USBOPERATIONAL          2L
#define USBSUSPEND              3L

#define HcInterruptStatus       0x0CL
#define WriteBackDoneHead       BIT1
#define StartofFrame            BIT2
#define RootHubStatusChange     BIT6

#define HcInterruptEnable       0x10L
#define HcInterruptDisable      0x14L
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

#define PIDFROMTD               0 //for ed PIN
#define PIDSETUP                0
#define PIDOUT                  1
#define PIDIN                   2
#define PIDINVERT(pid)          (3-pid)

//control uint32_t
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

//configs
#define OHCI_TD_CIRCLE_QUEUE_SIZE 16 //TODO: pre-allocation waste memory.
#define OHCI_TD_USE_CIRCLE_QUEUE (OHCI_TD_CIRCLE_QUEUE_SIZE > 1)
#define OHCI_TD_BUFFER_SIZE 128 //0~8192 according to the spec (max 1 page crossing, need aligned to 4K if large than 4K)

#if __cplusplus
extern "C"
{
#endif

typedef struct OHCI_TRANSFER_DESCRIPTOR
{
    union
    {
        uint32_t ControlFlags;
        //use 16bit bitfield for 16bit compiler compatibility i.e. Borland C
        struct
        {
            uint16_t Reserved1;
            uint16_t Reserved2 : 2;
            uint16_t BufferRounding : 1;
            uint16_t PID : 2;
            uint16_t DelayInterrupt : 3;
            uint16_t DataToggle : 2;
            uint16_t ErrorCount : 2;
            uint16_t ConditionCode : 4;
        }Control;
    };

    uint32_t  CurrentBufferP;
    uint32_t  NextTD;
    uint32_t  BufferEnd;

    //extension
    uint32_t PhysicalAddr;
    union
    {
        struct OHCI_TRANSFER_DESCRIPTOR* pNext;
        uint32_t Alignment16Compiler; //for BC
    };
    union
    {
        void* Buffer;
        uint32_t Alignment16Compiler2; //for BC
    };
    uint32_t BufferPhysicalAddr;

    static struct OHCI_TRANSFER_DESCRIPTOR* NULLTD;
}OHCI_TD;

#define OHCI_MAX_ISO_FRAME 8    //max frame for a iso TD
typedef struct OHCI_IsochronousTransferDescriptor
{
    union
    {
        uint32_t ControlFlags;      //condition code in it will be updated by hc
        struct
        {
            uint16_t StartFrame;
            uint16_t Reserved1 : 5;
            uint16_t DelayInterrupt : 3;
            uint16_t FrameCount : 3;
            uint16_t Reserved2 : 1;
            uint16_t ConditionCode : 4;
        }Control;
    };
    uint32_t BufferPage;
    uint32_t NextTD;
    uint32_t BufferEnd;

    uint16_t Offset[8]; //max 8 frames data by the spec
    
}OHCI_ISO_TD;

typedef struct OHCI_EndpointDescriptor
{
    union
    {
        uint32_t  ControlFlags;
        //use 16bit bitfield for 16bit compiler compatibility i.e. Borland C
        struct
        {
            uint16_t FunctionAddress : 7;
            uint16_t EndPoint : 4;
            uint16_t Direction : 2;
            uint16_t LowSpeed : 1;
            uint16_t Skip : 1;      //Skip: don't process
            uint16_t Format : 1;    //Format: Isochronous mode ED
            uint16_t MaxPacketSize : 11;
            uint16_t Reserved : 5;
        }Control;
    };

    uint32_t  TailP;
    uint32_t  HeadP;
    uint32_t  NextED;

    //extension. the OHCI spec doesn't need the size to be 16, but only alignment to 16
    //or we can think it as a virtual wrapper parent struct with additional fields
    struct OHCI_EndpointDescriptor* pPrev;
    struct OHCI_EndpointDescriptor* pNext;
    //head element link
    union
    {
        OHCI_TD* pTD;
        OHCI_ISO_TD* pIsoTD;
    };
#if defined(__BC__)
    uint16_t Unused[2];
    uint16_t TransferType;    //borland c will write whole uint16_t of single bitfield and doesn't support unamed struct
    uint32_t PhysicalAddr;
#else
    union
    {
        uint32_t PhysicalAddr;
        uint32_t TransferType : 2;
    };
#endif
}OHCI_ED;

#if defined(__DJ2__)
static_assert(sizeof(OHCI_ED) == 32);
static_assert(sizeof(OHCI_TD) == 32);
static_assert(sizeof(OHCI_ISO_TD) == 32);
#endif

typedef struct
{
    uint32_t        InterruptTable[32]; //HccaInterrruptTable
    uint16_t        wFrameNumber;
    uint16_t        wPad1;
    uint32_t        dwDoneHead;
    uint32_t        bufferTemp[30];//reserve
}OHCI_HCCA_BLOCK;

typedef struct OHCI_HostControllerData //per hc data
{
    OHCI_HCCA_BLOCK HCCA; //hcca, 256-aligned

    OHCI_ED ControlHead;
    OHCI_ED BulkHead;

    //periodic data, isochronous and interrupt
    //interval frames, 32ms for full speed (1frame=1ms), 4ms for high speed (1frame=0.125ms)
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
}OHCI_HCData;

typedef struct OHCI_HostControllerDeviceData //per device data
{
    //control ed, used before/after addressed. addres 0 not essentially point to this device, only guaranteed a reset opertation during enumeration.
    OHCI_ED ControlED;
    OHCI_TD ControlSetup;
    OHCI_TD ControlData;
    OHCI_TD ControlStatus;

    uint8_t Setup[8];
    //data
    OHCI_ED* DataED;
    OHCI_TD** DataTD;
    uint8_t bNumEndpoints;
    uint8_t bUsedEndpoint;

    uint8_t Buffer[HCDBUFFER_SIZE];
}OHCI_HCDeviceData;

BOOL OHCI_InitController(HCD_Interface* pHCI);
BOOL OHCI_DeinitController(HCD_Interface* pHCI);

uint8_t OHCI_ControlTransfer(HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir, uint8_t outputp pSetup[8], void* nullable pSetupData, uint16_t length);

uint8_t OHCI_IsochronousTransfer(HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir, uint8_t* inoroutp pBuffer, uint16_t length, uint16_t* outputp txlen);

uint8_t OHCI_DataTransfer(HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir, uint8_t* inoroutp pBuffer, uint16_t length, uint16_t* outputp txlen);

#if __cplusplus
}
#endif

#endif
