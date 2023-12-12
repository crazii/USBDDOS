#ifndef _EHCI_H_
#define _EHCI_H_
#include "USBDDOS/HCD/HCD.H"

//Enhanced Host Controller Interface Specification for Universal Serial Bus (ehci-specification-for-usb)

//PIC configuration space
#define USBBASE 0x10    //RW, 4, (MM) IO base
# define USBBASE_MM32 0x0 //bit1~2 of USBBASE,RO, 00
# define USBBASE_MM64 0x4 //bit1~2 of USBBASE,RO, 10
#define SBRN    0x60    //RO, 1, Serial bus release number
#define FLADJ   0x61    //RW, 1, Frame length adjustment
#define PORTWAKECAP 0x62 //RW,2, Port wake capability, optional

#define EHCI_PI 0x20    //PCI programming interface


//ECHI register offsets (USBBASE)
//capabilities (RO)
#define CAPLENGTH   0x00 //1
#define HCIVERSION  0x02 //2, interface version
#define HCSPARAMS   0x04 //4, structural params
#define HCCPARAMS   0x08 //4, cap params
#define HCSP_PORTROUTE 0x0C //8, companion port route description
typedef struct _ECHI_CAPS //helper structs
{
    uint8_t Size; //sizeof ECHI cap (USBBASE+size)=operational regs
    uint8_t Reserved;
    uint16_t hciVersion; //2 byte BCD
    union
    {
        struct
        {
            uint16_t hcsReserved1 : 8;
            uint16_t debugPort : 4;
            uint16_t hcsReserved2 : 3;
            uint16_t P_INDICATOR : 1; //port indicator
            uint16_t N_CC : 4;  //number of companion controller
            uint16_t N_PCC : 4; //ports per companion controller
            uint16_t PortRoutingRules : 1;
            uint16_t hcsReserved3 : 2;
            uint16_t PPC : 1; //port power controll
            uint16_t N_PORTS : 4; //1~0xF
        };
        uint32_t hcsParams;
    };
    union
    {
        struct
        {
            uint16_t hccReserved1;
            uint16_t EECP : 8; //ECHI extended cap ptr, 0 or valid val, offset to PCI configuration space (>=0x40)
            uint16_t IsochronousSchedulingThreshold : 4;
            uint16_t hccReserved2 : 1;
            uint16_t AsyncSchedulePark : 1;
            uint16_t ProgrammableFrameList : 1;
            uint16_t Address64 : 1;
        };
        uint32_t hccParams;
    };
    uint64_t hcspPortRoute;
}ECHI_CAPS;

//MUST check EECP != 0 before using it
#define USBLEGSUP(echi_cap /*ECHI_CAPS*/) (echi_cap->EECP+0) //RO/RW
#define USBLEGCTLSTS(echi_cap /*ECHI_CAPS*/) (echi_cap->EECP+4) //RO/RW/RWC

#define LOAD_USBLEGSUP(pcidev /*PCI_DEVICE*/, echi_cap /*ECHI_CAPS*/) ((uint32_t)(echi_cap->EECP ? *(uint32_t*)&pcidev->Offset[USBLEGSUP(echi_cap)] : 0))
#define LOAD_USBLEGCTLSTS(pcidev /*PCI_DEVICE*/, echi_cap /*ECHI_CAPS*/) ((uint32_t)(echi_cap->EECP ? *(uint32_t*)&pcidev->Offset[USBLEGCTLSTS(echi_cap)] : 0))
#define STORE_USBLEGSUP(addr /*PCI_Addr*/, echi_cap /*ECHI_CAPS*/, legsup) if(echi_cap->EECP) PCI_WriteDWord((addr).Bus, (addr).Device, (addr).Function, USBLEGSUP(echi_cap), legsup)
#define STORE_USBLEGCTLSTS(addr /*PCI_Addr*/, echi_cap /*ECHI_CAPS*/, legctlsts) if(echi_cap->EECP) PCI_WriteDWord((addr).Bus, (addr).Device, (addr).Function, USBLEGCTLSTS(echi_cap), legctlsts)

//USBLEGSUP: legacy support
#define HC_OS_Owned_Semaphore BIT24 //RW
#define HC_BIOS_Owned_Semaphore BIT16 //RW
#define NextEECP 0xFF00 //mask (RO)
#define GetNextEECP(legsup) (((legsup)&NextEECP)>>8)
#define CAPID 0x00FF //mask (RO)
#define CAPID_LEGSUP 0x01
typedef union _LEGSUP
{
    uint32_t Val;
    struct
    {
        uint8_t Reserved1 : 7;
        uint8_t OS_Owned : 1;
        uint8_t Reserved2 : 7;
        uint8_t BIOS_Owned : 1;
        uint8_t Next; //next EECP
        uint8_t CapabilityID;
    };
}LEGSUP;
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
#define InterruptThreshold 0x00FF0000 //mask, RW
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

typedef union _ECHI_USBCMD
{
    uint32_t Val;
    struct
    {
        uint8_t Reserved1;
        uint8_t IntThreshold;
        uint8_t Reserved2 : 4;
        uint8_t AsyncScheduleParkEN : 1;
        uint8_t Reserved3 : 1;
        uint8_t AsyncScheduleParkCNT : 2;
        uint8_t LightHCRESET : 1;
        uint8_t INTonAsyncAdv : 1;
        uint8_t AynscScheduleEN : 1;
        uint8_t PeroidicScheduleEN : 1;
        uint8_t FLSize : 2;
        uint8_t HCReset : 1;
        uint8_t RunStop : 1;
    };
}ECHI_USBCMD;
#if defined(__DJ2__)
_Static_assert(sizeof(ECHI_USBCMD)==4);
#endif


//USBSTS
#define AsyncSheduleSts BIT15 //RO
#define PeriodicSheduleSts BIT14 //RO
#define Reclamation BIT13 //RO
#define HCHalted BIT13 //RO
#define INTonAsyncAdvance BIT5 //RWC
#define HostSysError BIT4 //RWC
#define FrameListRollover BIT3 //RWC
#define PortChangeDetect BIT2 //RWC
#define USBERRINT BIT1 //RWC
#define USBINT BIT0

//USBINTR
#define INTonAsyncAdvanceEN BIT5
#define INTHostSysErrorEN BIT4
#define INTFrameListRolloverEN BIT3
#define INTPortChangeEN BIT2
#define USBERRINTEN BIT1
#define USBINTEN BIT0

//CONFIGFLAG
#define ConfigureFlag 0x1 //RW, last action

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
#define PortOwner BI13 //RW
#define PortPower BIT12 //RW/RO
#define LineStatus 0xC00 //mask, RO
# define LS_SE0 0x00
# define LS_JS 0x01
# define LS_KS 0x02
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

//Periodic frame list
#define Tbit 0x1 //terminate
#define Typ_iTD 0x0
#define Typ_QH 0x1
#define Typ_siTD 0x2
#define Typ_FSTN 0x3
typedef union FLElementPointer //frame list element pointer
{
    uint32_t Ptr;
    struct
    {
        uint16_t Hw : 16; //use uint16_t for Borland C 3.1
        uint16_t Lw : 11; //28 bit ptr by spec (but spec require it reference 32bytes aligned object, so we use 27bit)
        uint16_t Zero : 2;
        uint16_t Typ : 2;
        uint16_t T : 1;
    };
}FLEP;

//iTD
typedef struct IsoTransferDescriptor
{
    FLEP Next;

    struct StatusList
    {
        //uint16_t Status : 4;
        uint16_t Active : 1;
        uint16_t DataBufferErr : 1;
        uint16_t BabbleDetected : 1;
        uint16_t TransactionErr : 1;

        uint16_t Length : 12; //bytes to transfer. (updated if IO=in)
        uint16_t IOC : 1; //interrupt on compelete
        uint16_t PG : 3;
        uint16_t Offset : 12;
    }Status[8];

    union BufferPagePointerList
    {
        uint32_t Val;
        struct BufferPage0
        {
            uint16_t PtrHw : 16;
            uint16_t PtrLw : 4; //4k aligned
            uint16_t Endpt : 4; //endpoint
            uint16_t Reserved0 : 1;
            uint16_t DeviceAddr : 7;
        }BP0;
        struct BufferPage1
        {
            uint16_t PtrHw : 16;
            uint16_t PtrLw : 4;
            uint16_t IO : 1; //direction. Out=0,In=1
            uint16_t MaxPacketSize : 11;
        }BP1;
        struct BufferPage2
        {
            uint16_t PtrHw : 16;
            uint16_t PtrLw : 4;
            uint16_t Reserved2 : 10;
            uint16_t Multi : 2;
        }BP2;
        /*struct
        {
            uint16_t PtrHw : 16;
            uint16_t PtrLw : 4;
            uint16_t Reserved3 : 12;
        }BP3to6;*/
    }BufferPages[7];

}iTD;
#if defined(__DJ2__)
_Static_assert(sizeof(iTD)==64);
#endif

typedef struct SplitIsoTransferDescriptor
{
    FLEP Next;

    //Endpoint Capabilities/Characteristics
    struct EPCC
    {
        uint16_t IO : 1; //direction. out=0,in=1
        uint16_t PortNum : 7;
        uint16_t Reserved1 : 1;
        uint16_t HudAddr : 7;
        uint16_t Reserved2 : 4;
        uint16_t Endpoint : 4;
        uint16_t Reserved3 : 1;
        uint16_t DeviceAddr : 7;
    }Caps;

    //Micro frame shcedule control
    struct uFrameSC
    {
        uint16_t Reserved4;
        uint8_t uFrameCMask;
        uint8_t uFrameSMask;
    }SheduleCtl;

    //Transfer status and control
    struct StatusControl
    {
        uint16_t IOC : 1; //interrupt on completion
        uint16_t P : 1; //page select
        uint16_t Reserved5 : 4;
        uint16_t Length : 10; //bytes to transfer (1023/3FF max)
        uint8_t CProgMask;
        struct //Status
        {
            uint8_t Active : 1;
            uint8_t ERR : 1;
            uint8_t DataBufferErr : 1;
            uint8_t BabbleDetected : 1;
            uint8_t TransactionErr : 1;
            uint8_t MisseduFrame : 1;
            uint8_t SplitXstate : 1;
            uint8_t Reserved5 : 1;
        };
    }StatusCtl;

    struct BufferPage0
    {
        uint16_t BufferPointerHw : 16;
        uint16_t BufferPointerLw : 4;
        uint16_t CurrentOffset : 12; //written back by HC
    }BP0;

    struct BufferPage1
    {
        uint16_t BufferPointerHw : 16;
        uint16_t BufferPointerLw : 4;
        uint16_t ReservedBP1 : 7;
        uint16_t TP : 2; //transaction position
        uint16_t TCount : 2; //transaction count
    }BP1;

    FLEP BackLink; //Typ must be zero. Tbit as invalid bit

    uint32_t Padding;
}siTD;
#if defined(__DJ2__)
_Static_assert(sizeof(siTD)==32);
#endif



//qTD
typedef struct QueueElementTD
{
    FLEP Next; //Typ must be zero. Tbit as invalid bit
    FLEP AltNext; //alternate next

    struct _Token
    {
        uint16_t DataToggle : 1;
        uint16_t Length : 15; //bytes to transfer, updated to actual transferred bytes if succeed. 5000h max
        uint16_t IOC : 1;
        uint16_t C_Page : 3;
        uint16_t CERR : 2; //err counter
        uint16_t PID : 2;
        struct Status
        {
            uint8_t Active : 1;
            uint8_t Halted : 1;
            uint8_t DataBufferErr : 1;
            uint8_t BabbleDetected : 1;
            uint8_t TransactionErr : 1;
            uint8_t MisseduFrame : 1;
            uint8_t SplitXstate : 1;
            uint8_t PingState : 1;
        };
    }Token;

    struct BufferPage
    {
        uint16_t PtrHw : 16;
        uint16_t PtrLw : 4;
        uint16_t CurrentOffset : 12; //reserved for BufferPages 1~4, only valid for 0
    }BufferPages[5];

}qTD;


//Queue Head
typedef struct QueueHead
{
    FLEP HorizLink; //QHLP queue head horitontal link pointer

    //Endpoint capabilities/characteristics
    struct EndpointCC
    {
        uint16_t NakCounterRL : 4;
        uint16_t ControlEP : 1;
        uint16_t MaxPacketSize : 11;
        uint16_t HeadofReclamation : 1;
        uint16_t DataToggle : 1; //DTC data oggle control
        uint16_t EndpointSpd : 2; //EPS
        uint16_t Endpoint : 4; //ep no.
        uint16_t InactiveonNext : 1;
        uint16_t DeviceAddr : 7;
    }Caps;

    //Endpoint capabilities/characteristics 2
    struct EndpointCC2
    {
        uint16_t Mult : 2; //high bandwith pipe multiplier (valid: 1~3)
        uint16_t PortNum : 7;
        uint16_t HubAddr : 7;
        uint8_t uFrameCMask;
        uint8_t uFrameSMask;
    }Caps2;

    uint32_t qTDCurrent;
    FLEP qTDNext; //Typ must be zero, Tbit as invalid bit

    struct
    {
        uint16_t PtrHw : 16;
        uint16_t PtrLw : 11;
        uint16_t NakCnt : 4;
        uint16_t T : 1;
    }qTDAltNext; //alternate next

    struct StatusControl
    {
        uint16_t DataToggle : 1;
        uint16_t Length : 15; //bytes to transfer
        uint16_t IOC : 1; //interrupt on completion
        uint16_t C_Page : 3;
        uint16_t C_Err : 2;
        uint16_t Status : 9;
        uint16_t PingState : 1;
    }StatusCtl;

    union BufferPointer
    {
        struct Page0
        {
            uint16_t PtrHw : 16;
            uint16_t PtrLw : 4;
            uint16_t CurrentOffset : 12;
        };
        struct Page1
        {
            uint16_t PtrHw : 16;
            uint16_t PtrLw : 4;
            uint8_t ReservedP1;
            uint8_t CProgMask;
        };
        struct Page2
        {
            uint16_t PtrHw : 16;
            uint16_t PtrLw : 4;   
            uint16_t SBytes : 7;
            uint16_t FrameTag : 5;
        };
        //page 3&4 low 12 bits reserved
    }BufferPointers[5];

    //extra padding, used by driver
    uint32_t Padding[4];
    
}QH;
#if defined(__DJ2__)
_Static_assert(sizeof(QH)==64);
#endif

//FSTN: periodic frame span traversal node
typedef struct
{
    FLEP NormalPathPointer; //normal path link pointer (NPLP)
    FLEP BackPathPointer; //back path link pointer (BPLP), Typ must be Typ_QH
}FSTN;

#endif //_EHCI_H_