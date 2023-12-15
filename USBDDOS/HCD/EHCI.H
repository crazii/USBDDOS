#ifndef _EHCI_H_
#define _EHCI_H_
#include "USBDDOS/HCD/HCD.H"

//Enhanced Host Controller Interface Specification for Universal Serial Bus (ehci-specification-for-usb)

#define EHCI_PI 0x20    //PCI programming interface

//EHCI register offsets (USBBASE)
//capabilities (RO)
#define CAPLENGTH   0x00 //1
#define HCIVERSION  0x02 //2, interface version
#define HCSPARAMS   0x04 //4, structural params
#define HCCPARAMS   0x08 //4, cap params
#define HCSP_PORTROUTE 0x0C //8, companion port route description
typedef struct _EHCI_CAPS //helper structs
{
    uint8_t Size; //sizeof EHCI cap (USBBASE+size)=operational regs
    uint8_t Reserved;
    uint16_t hciVersion; //2 byte BCD
    union
    {
        struct
        {
            uint16_t N_PORTS : 4; //1~0xF
            uint16_t PPC : 1; //port power controll
            uint16_t hcsReserved3 : 2;
            uint16_t PortRoutingRules : 1;
            uint16_t N_PCC : 4; //ports per companion controller
            uint16_t N_CC : 4;  //number of companion controller
            uint16_t P_INDICATOR : 1; //port indicator
            uint16_t hcsReserved2 : 3;
            uint16_t debugPort : 4;
            uint16_t hcsReserved1 : 8;
        }hcsParamsBm;
        uint32_t hcsParams;
    };
    union
    {
        struct
        {
            uint16_t Address64 : 1;
            uint16_t ProgrammableFrameList : 1;
            uint16_t AsyncSchedulePark : 1;
            uint16_t hccReserved2 : 1;
            uint16_t IsochronousSchedulingThreshold : 4;
            uint16_t EECP : 8; //EHCI extended cap ptr, 0 or valid val, offset to PCI configuration space (>=0x40)
            uint16_t hccReserved1;
        }hccParamsBm;
        uint32_t hccParams;
    };
    uint64_t hcspPortRoute;
}EHCI_CAPS;
#if defined( __DJ2__)
_Static_assert(sizeof(EHCI_CAPS)==20, "incorrect size/alignment");
#endif

#ifdef _EHCI_IMPL //avoid defines polution

//PCI configuration space
#define USBBASE 0x10    //RW, 4, (MM) IO base
# define USBBASE_MM32 0x0 //bit1~2 of USBBASE,RO, 00
# define USBBASE_MM64 0x4 //bit1~2 of USBBASE,RO, 10
#define SBRN    0x60    //RO, 1, Serial bus release number
#define FLADJ   0x61    //RW, 1, Frame length adjustment
#define PORTWAKECAP 0x62 //RW,2, Port wake capability, optional

//USBLEGSUP: legacy support
#define HC_OS_Owned_Semaphore BIT24 //RW
#define HC_BIOS_Owned_Semaphore BIT16 //RW
#define NextEECP 0xFF00 //mask (RO)
#define GetNextEECP(legsup_val) (((legsup_val)&NextEECP)>>8)
#define CAPID_MASK 0x00FF //mask (RO)
#define CAPID_LEGSUP 0x01
typedef union _LEGSUP
{
    uint32_t Val;
    struct
    {
        uint8_t CapabilityID;
        uint8_t Next; //next EECP
        uint8_t BIOS_Owned : 1;
        uint8_t Reserved2 : 7;
        uint8_t OS_Owned : 1;
        uint8_t Reserved1 : 7;
    }Bm;
}ECHI_LEGSUP;
//USBLEGCTLSTS
#define SMIonBAR BIT31 //RWC
#define SMIonPCICMD BIT30 //RWC
#define SMIonOwnershipChange BIT29 //RWC
#define SMIonAsyncAdvance BIT21 //RO
#define SMIonHostSysError BIT20//RO
#define SMIonFrameListRollover BIT19 //RO
#define SMIonPortChangeDetect BIT18 //RO
#define SMIonUSBError BIT17 //RO
#define SMIonUSBComplete BIT16 //RO
#define SMIonBAREnable BIT15 //RW
#define SMIonPCICMDEnable BIT14 //RW
#define SMIonOSOwnershipEnable BIT13 //RW
#define SMIonAsyncAdvanceEnable BIT5 //RW
#define SMIonHostSysErrorEnable BIT4 //RW
#define SMIonFrameListRolloverEnable BIT3 //RW
#define SMIonPortChangeEnable BIT2 //RW
#define SMIonUSBErrorEnable BIT1 //RW
#define USBSMIEnable BIT0 //RW

//MUST check EECP != 0 before using it
#define USBLEGSUP(ehci_cap /*EHCI_CAPS*/) (uint8_t)(ehci_cap.hccParamsBm.EECP+0) //RO/RW, 4
#define USBLEGCTLSTS(ehci_cap /*EHCI_CAPS*/) (uint8_t)(ehci_cap.hccParamsBm.EECP+4) //RO/RW/RWC, 4

#define READ_USBLEGSUP(addr /*PCI_Addr*/, ehci_cap /*EHCI_CAPS*/) ((uint32_t)(ehci_cap.hccParamsBm.EECP ? PCI_ReadDWord((addr).Bus, (addr).Device, (addr).Function, USBLEGSUP(ehci_cap)) : 0))
#define READ_USBLEGCTLSTS(addr /*PCI_Addr*/, ehci_cap /*EHCI_CAPS*/) ((uint32_t)(ehci_cap.hccParamsBm.EECP ? PCI_ReadDWord((addr).Bus, (addr).Device, (addr).Function, USBLEGCTLSTS(ehci_cap)) : 0))
#define WRITE_USBLEGSUP(addr /*PCI_Addr*/, ehci_cap /*EHCI_CAPS*/, legsup) if(ehci_cap.hccParamsBm.EECP) PCI_WriteByte((addr).Bus, (addr).Device, (addr).Function, USBLEGSUP(ehci_cap)+3, legsup.Bm.OS_Owned)
#define WRITE_USBLEGCTLSTS(addr /*PCI_Addr*/, ehci_cap /*EHCI_CAPS*/, legctlsts) if(ehci_cap.hccParamsBm.EECP) PCI_WriteDWord((addr).Bus, (addr).Device, (addr).Function, USBLEGCTLSTS(ehci_cap), legctlsts)

#define READ_USBLEGSUP2(pcidev /*PCI_DEVICE*/, ehci_cap /*EHCI_CAPS*/) ((uint32_t)(ehci_cap.hccParamsBm.EECP ? *(uint32_t*)&pcidev->Offset[USBLEGSUP(ehci_cap)] : 0))
#define READ_USBLEGCTLSTS2(pcidev /*PCI_DEVICE*/, ehci_cap /*EHCI_CAPS*/) ((uint32_t)(ehci_cap.hccParamsBm.EECP ? *(uint32_t*)&pcidev->Offset[USBLEGCTLSTS(ehci_cap)] : 0))


//Operational Regsisters
#define USBCMD 0x00 //cmd
#define USBSTS 0x04 //status
#define USBINTR 0x08 //interrupt
#define FRINDEX 0x0C //frame index
#define CTRLDSSEGMENT 0x10 //64 bit (high dword)
#define PERIODICLISTBASE 0x14 //peroidic list base (frame list base)
#define ASYNCLISTADDR 0x18
#define CONFIGFLAG 0x40
#define PORTSC 0x44

//USBCMD
#define InterruptThreshold 0x00FF0000 //mask, RW, default 8 (1ms)
#define InterruptThreshold_Shift 16
#define AsyncScheduleParkEnable BIT11 //RO/RW, optional
#define AsyncScheduleParkCount 0x00300 //RO/RW, optional
#define AsyncScheduleParkCount_Shift 8
#define LightHCReset BIT7 //RW, optional
#define INTonAsyncAdvanceDoorbell BIT6 //RW
#define AsyncSheduleEnable BIT5 //RW
#define PeroidicScheduleEnable BIT4 //RW
#define FrameListSize 0x0C //mask, RW/RO, available when ProgrammableFrameList=1
# define FLS_4096 0x00 //1024 elements, 4k size, default.
# define FLS_2048 0x01
# define FLS_1024 0x02
# define FLS_Reserved 0x03
#define HCRESET BIT1 //RW
#define RS BIT0//Run/Stop, RW

typedef union _EHCI_USBCMD
{
    uint32_t Val;
    struct
    {
        uint8_t RunStop : 1;
        uint8_t HCReset : 1;
        uint8_t FLSize : 2;
        uint8_t PeroidicScheduleEN : 1;
        uint8_t AynscScheduleEN : 1;
        uint8_t INTonAsyncAdv : 1;
        uint8_t LightHCRESET : 1;
        uint8_t AsyncScheduleParkCNT : 2;
        uint8_t Reserved3 : 1;
        uint8_t AsyncScheduleParkEN : 1;
        uint8_t Reserved2 : 4;
        uint8_t IntThreshold; //default: 0x8 (8 micro Frames=1ms)
        uint8_t Reserved1;
    }Bm;
}EHCI_USBCMD;
#if defined(__DJ2__)
_Static_assert(sizeof(EHCI_USBCMD)==4, "incorrect size/alignment");
#endif


//USBSTS
#define AsyncSheduleSts BIT15 //RO
#define PeriodicSheduleSts BIT14 //RO
#define Reclamation BIT13 //RO
#define HCHalted BIT12 //RO
#define INTonAsyncAdvance BIT5 //RWC
#define HostSysError BIT4 //RWC
#define FrameListRollover BIT3 //RWC
#define PortChangeDetect BIT2 //RWC
#define USBERRINT BIT1 //RWC
#define USBINT BIT0 //RWC

//USBINTR
#define INTonAsyncAdvanceEN BIT5
#define INTHostSysErrorEN BIT4
#define INTFrameListRolloverEN BIT3
#define INTPortChangeEN BIT2
#define USBERRINTEN BIT1
#define USBINTEN BIT0

//CONFIGFLAG
#define ConfigureFlag BIT0 //RW, last action

//PORTSCs
#define WKOC_E BIT22 //RW
#define WKDSCNNT_E BIT21 //RW
#define WKCNNT_E BIT20 //RW
#define PortTests 0x0F0000 //mask, RW
#define PortIndicator 0x00C000 //mask, RW
# define PI_OFF 0x00
# define PI_Amber 0x01
# define PI_Green 0x02
# define PI_Undefined 0x03
#define PortOwner BIT13 //RW, 0=owned by EHCI, 1=owned by companion HC
#define PortPower BIT12 //RW/RO
#define LineStatus 0xC00 //mask, RO
#define LineStatusShift 10
# define LS_SE0 0x00 //
# define LS_JS 0x02
# define LS_KS 0x01
# define LS_Undefined 0x03
#define PortReset BIT8 //RW
#define PortSuspend BIT7 //RW
#define ForcePortResume BIT6 //RW
#define OvercurrentChange BIT5 //RWC
#define OvercurrentActive BIT4 //RO
#define PortEnableChange BIT3 //RWC
#define PortEnable BIT2 //RW
#define ConnectStatusChange BIT1 //RWC
#define ConnectStatus BIT0 //RO

#endif //_EHCI_IMPL

//structures

//Periodic frame list
#define Tbit 0x1 //terminate
#define Typ_iTD 0x0
#define Typ_QH 0x1
#define Typ_siTD 0x2
#define Typ_FSTN 0x3
#define Typ_Shift 1
typedef union FLElementPointer //frame list element pointer
{
    uint32_t Ptr;
    struct
    {
        uint16_t T : 1;
        uint16_t Typ : 2;
        uint16_t Zero : 2;
        uint16_t Lw : 11; //use uint16_t for Borland C 3.1
        uint16_t Hw : 16; //28 bit ptr by spec (but spec require it reference 32bytes aligned object, so we use 27bit)
    }Bm;
}EHCI_FLEP;
#if defined(__DJ2__)
_Static_assert(sizeof(EHCI_FLEP)==4, "incorrect size/alignment");
#endif

//iTD
typedef struct IsoTransferDescriptor
{
    EHCI_FLEP Next;

    struct StatusList
    {
        uint16_t Offset : 12;
        uint16_t PG : 3;
        uint16_t IOC : 1; //interrupt on compelete
        uint16_t Length : 12; //bytes to transfer. (updated if IO=in)
        //uint16_t Status : 4;
        uint16_t TransactionErr : 1;
        uint16_t BabbleDetected : 1;
        uint16_t DataBufferErr : 1;
        uint16_t Active : 1;
    }Status[8];

    union BufferPagePointerList
    {
        uint32_t Val;
        struct iBufferPage0
        {
            uint16_t DeviceAddr : 7;
            uint16_t Reserved0 : 1;
            uint16_t Endpt : 4; //endpoint
            uint16_t PtrLw : 4; //4k aligned
            uint16_t PtrHw : 16;
        }BP0;
        struct iBufferPage1
        {
            uint16_t MaxPacketSize : 11;
            uint16_t IO : 1; //direction. Out=0,In=1
            uint16_t PtrLw : 4;
            uint16_t PtrHw : 16;
        }BP1;
        struct iBufferPage2
        {
            uint16_t Multi : 2;
            uint16_t Reserved2 : 10;
            uint16_t PtrLw : 4;
            uint16_t PtrHw : 16;
        }BP2;
        /*struct
        {
            uint16_t Reserved3 : 12;
            uint16_t PtrLw : 4;
            uint16_t PtrHw : 16;
        }BP3to6;*/
    }BufferPages[7];

}EHCI_iTD;
#if defined(__DJ2__)
_Static_assert(sizeof(EHCI_iTD)==64, "incorrect size/alignment");
#endif

typedef struct SplitIsoTransferDescriptor
{
    EHCI_FLEP Next;

    //Endpoint Capabilities/Characteristics
    struct EPCC
    {
        uint16_t DeviceAddr : 7;
        uint16_t Reserved3 : 1;
        uint16_t Endpoint : 4;
        uint16_t Reserved2 : 4;
        uint16_t HudAddr : 7;
        uint16_t Reserved1 : 1;
        uint16_t PortNum : 7;
        uint16_t IO : 1; //direction. out=0,in=1
    }Caps;

    //Micro frame shcedule control
    struct uFrameSC
    {
        uint8_t uFrameSMask;
        uint8_t uFrameCMask;
        uint16_t Reserved4;
    }SheduleCtl;

    //Transfer status and control
    struct StatusControl
    {
        struct //Status
        {
            uint8_t Reserved5 : 1;
            uint8_t SplitXstate : 1;
            uint8_t MisseduFrame : 1;
            uint8_t TransactionErr : 1;
            uint8_t BabbleDetected : 1;
            uint8_t DataBufferErr : 1;
            uint8_t ERR : 1;
            uint8_t Active : 1;
        };

        uint8_t CProgMask;
        uint16_t Length : 10; //bytes to transfer (1023/3FF max)
        uint16_t Reserved6 : 4;
        uint16_t P : 1; //page select
        uint16_t IOC : 1; //interrupt on completion
    }StatusCtl;

    struct siBufferPage0
    {
        uint16_t CurrentOffset : 12; //written back by HC
        uint16_t BufferPointerLw : 4;
        uint16_t BufferPointerHw : 16;
    }BP0;

    struct siBufferPage1
    {
        uint16_t TCount : 2; //transaction count
        uint16_t TP : 2; //transaction position
        uint16_t ReservedBP1 : 7;
        uint16_t BufferPointerLw : 4;
        uint16_t BufferPointerHw : 16;
    }BP1;

    EHCI_FLEP BackLink; //Typ must be zero. Tbit as invalid bit

    uint32_t Padding;
}EHCI_siTD;
#if defined(__DJ2__)
_Static_assert(sizeof(EHCI_siTD)==32, "incorrect size/alignment");
#endif

#define PID_OUT 0
#define PID_IN  1
#define PID_SETUP 2
#define EHCI_qTD_MaxSize (5*4096) //5 pages=5*4096
#define EHCI_QTK_ERRORS (BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT0) //BIT0 low/full only

//Token (status and control) for qTD and QH overlay
typedef struct QueueToken
{
    //status 8bit
    union
    {
        uint8_t Status;
        struct
        {
            uint8_t P_ERROR : 1; //ping state for high speed transfer, error for low/full.
            uint8_t SplitXstate_1x : 1;
            uint8_t MisseduFrame_1x : 1; //low/full & periodic only
            uint8_t TransactionErr : 1;
            uint8_t BabbleDetected : 1;
            uint8_t DataBufferErr : 1;
            uint8_t Halted : 1;
            uint8_t Active : 1;
        }StatusBm;
    };

    uint8_t PID : 2;
    uint8_t CERR : 2; //err counter down
    uint8_t C_Page : 3;
    uint8_t IOC : 1;
    uint16_t Length : 15; //bytes to transfer, updated to actual transferred bytes if succeed. 5000h max (EHCI_qTD_MaxSize)
    uint16_t DataToggle : 1;
}EHCI_QToken;
#if defined(__DJ2__)
_Static_assert(sizeof(EHCI_QToken)==4, "incorrect size/alignment");
#endif

//qTD
typedef struct QueueElementTD
{
    EHCI_FLEP Next; //Typ must be zero. Tbit as invalid bit
    EHCI_FLEP AltNext; //alternate next

    EHCI_QToken Token;

    union BufferPage
    {
        uint32_t Ptr;
        struct 
        {
            uint16_t CurrentOffset : 12; //reserved for BufferPages 1~4, only valid for 0
            uint16_t PtrLw : 4;
            uint16_t PtrHw : 16;
        }Bm;
    }BufferPages[5];

    struct
    {
        struct QueueElementTD* Prev;
        struct QueueElementTD* Next;
        HCD_Request* Request;
#if defined(__BC__)
        uint16_t Unused[13];
#else
        uint32_t Unused[5];
#endif
    }EXT;
}EHCI_qTD;
#if defined(__DJ2__)
_Static_assert(sizeof(EHCI_qTD)==64, "incorrect size/alignment");
#endif

#define EPS_LOW  1
#define EPS_FULL 0
#define EPS_HIGH 2

//Queue Head
typedef struct QueueHead
{
    EHCI_FLEP HorizLink; //QHLP queue head horitontal link pointer

    //Endpoint capabilities/characteristics
    struct EndpointCC
    {
        uint16_t DeviceAddr : 7;
        uint16_t InactiveonNext_1x : 1; //only valid in perioidic for full/low endpoint
        uint16_t Endpoint : 4; //ep no.
        uint16_t EndpointSpd : 2; //EPS. 0=FullSpeed, 1=LowSpeed, 2=HighSpeed
        uint16_t DTC : 1; //DTC data oggle control
        uint16_t HeadofReclamation : 1;
        uint16_t MaxPacketSize : 11;
        uint16_t ControlEP_1x : 1; //only set when endpoint is full/low speed and a control ep
        uint16_t NakCounterRL : 4;
    }Caps;

    //Endpoint capabilities/characteristics 2
    struct EndpointCC2
    {
        uint16_t uFrameSMask : 8;
        uint16_t uFrameCMask_1x : 8; //periodic only
        uint16_t HubAddr_1x : 7;
        uint16_t PortNum_1x : 7;
        uint16_t Mult : 2; //high bandwith pipe multiplier (valid: 1~3)
    }Caps2;

    uint32_t qTDCurrent;
    EHCI_FLEP qTDNext; //Typ must be zero, Tbit as invalid bit

    union
    {
        uint32_t Ptr;
        struct
        {
            uint16_t T : 1;
            uint16_t NakCnt : 4;
            uint16_t PtrLw : 11;
            uint16_t PtrHw : 16;
        }Bm;
    }qTDAltNext; //alternate next

    EHCI_QToken Token;

    union qhBufferPointer
    {
        struct
        {
            uint16_t CurrentOffset : 12;
            uint16_t PtrLw : 4;
            uint16_t PtrHw : 16;
        }Page0;
        struct
        {
            uint16_t CProgMask : 8;
            uint16_t ReservedP1 : 4;
            uint16_t PtrLw : 4;
            uint16_t PtrHw : 16;
        }Page1;
        struct
        {
            uint16_t FrameTag : 5;
            uint16_t SBytes : 7;
            uint16_t PtrLw : 4;
            uint16_t PtrHw : 16;
        }Page2;
        //page 3&4 low 12 bits reserved
    }BufferPointers[5];

    //extra data & padding, used by driver
    struct
    {
        EHCI_qTD* Tail;
        uint8_t TransferType : 2;
        uint8_t Interval : 4;
        uint8_t PaddingBIT : 2;
#if defined(__BC__)
        uint8_t Padding8[13];
#else
        uint8_t Padding8[11];
#endif
    }EXT;
}EHCI_QH;
#if defined(__DJ2__)
_Static_assert(sizeof(EHCI_QH)==64, "incorrect size/alignment");
#endif

//FSTN: periodic frame span traversal node
typedef struct
{
    EHCI_FLEP NormalPathPointer; //normal path link pointer (NPLP)
    EHCI_FLEP BackPathPointer; //back path link pointer (BPLP), Typ must be Typ_QH
}FSTN;

#define EHCI_ALIGN 32

//USBDDOS specific

typedef struct EHCI_HostControllerDriverData
{
    //aligment
    EHCI_QH ControlQH;
    EHCI_QH BulkQH;
    EHCI_QH InterruptQH[11]; //2^(0~10), the bInterval of INTR(/ISO) endpoint for USB2.0 ranges from 1~16, in 2^(bInterval-1) micro frames (125us)

    //no aligment
    EHCI_CAPS Caps;
    uint32_t OPBase; //operational regs base
    EHCI_QH* ControlTail;
    EHCI_QH* BulkTail;
    EHCI_QH* InterruptTail[11];
    uint32_t dwPeroidicListBase;   //memory address.
    uint16_t wPeroidicListHandle;  //xms handle
}EHCI_HCData;

typedef struct EHCI_HostControllerDeviceData
{
    //TODO: no special handling of default control pipe
    EHCI_QH ControlQH; //the pre-created default control pipe is for code's historical reason and could be removed
    BOOL ControlEPCreated;
}EHCI_HCDeviceData;

BOOL EHCI_InitController(HCD_Interface* pHCI, PCI_DEVICE* pPCIDev);
BOOL EHCI_DeinitController(HCD_Interface* pHCI);
BOOL EHCI_ISR(HCD_Interface* pHCI);

uint8_t EHCI_ControlTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t inputp setup8[8],
    void* nullable pSetupData, uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData);

uint8_t EHCI_IsoTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t* inoutp pBuffer,
    uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData);

uint8_t EHCI_DataTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t* inoutp pBuffer,
    uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData);

#endif //_EHCI_H_