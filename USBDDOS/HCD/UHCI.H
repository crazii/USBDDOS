#ifndef _UHCI_H_
#define _UHCI_H_
#include "USBDDOS/HCD/HCD.H"

//PCI configuration sapce
#define USBBASE     0x20    //0x20~0x23, IO Space Base Address register (PCI BAR4)
#define LEGSUP      0xC0    //0xC0~0xC1, legacy support register
#define USBSMIEN    BIT4    //usb smi enable
#define USBPIRQDEN  BIT13

///PID
#define OUTPID   0xe1
#define INPID    0x69
#define SOFPID   0xa5
#define SETUPPID 0x2d

#define DATA0PID 0xc3
#define DATA1PID 0x4b
#define DATA2PID 0x87
#define MDATAPID 0x0f

#define ACKPID   0xd2
#define NAKPID   0x5a
#define STALLPID 0x1e
#define NYETPID  0x96

#define PREPID    0x3c
#define ERRPID    0x3c
#define SPLITPID  0x78
#define PINGPID   0xb4
#define RESERVEDPID  0xF0

//  invalid QH, TD; means last QH, TD
#define TerminateFlag   0x01
#define QHFlag          0x02
#define DepthSelect     0x04    //Depth/Breath Select, continue element link.

//control and status, matches ControlStatusBits
#define CS_C_ERR              (BIT27|BIT28) //error count 00b: no limit, ... 11b: interrupt after 3 error
#define CS_LowSpeed           BIT26
#define CS_IOS                BIT25    //isochronous select
#define CS_IOC                BIT24    //interrupt on complete
#define CS_ActiveStatus       BIT23
#define CS_StalledStatus      BIT22
#define CS_DataBufferError    BIT21
#define CS_BabbleDetected     BIT20
#define CS_NAKReceived        BIT19
#define CS_CRCTimeOutError    BIT18
#define CS_BitstuffError      BIT17
#define CS_ErrorMask          (BIT17|BIT18|BIT19|BIT20|BIT21|BIT22)
#define CS_ErrorShift         17L

#define CS_C_ERR_NOLIMIT      (0L<<27)
#define CS_C_ERR1             (1L<<27)     //error count 1
#define CS_C_ERR2             (2L<<27)
#define CS_C_ERR3             (3L<<27)

//token
#define TK_DataToggle       BIT19
#define TK_NullLength       0x7FF

// UHCI register
#define USBCMD              0x00    //command, 16bit
#define USBSTS              0x02    //status, 16bit
#define USBINTR             0x04    //interrupt enable,16bit
#define FRNUM               0x06    //frame number, 16bit (10bit used, 0~1024)
#define FLBASEADD           0x08    //frame list base addr, 32bit
#define SOF                 0x0c    //start of frame modify, 8bit
#define PORTSC              0x10
#define PORT1               0x10
#define PORT2               0x12

//USBCMD
#define RS                  BIT0    //run/stop
#define HCRESET             BIT1
#define GRESET              BIT2    //global reset
#define EGSM                BIT3    //enter global suspend
#define FGR                 BIT4    //force global resume
#define SWDBG               BIT5    //software debug
#define CF                  BIT6    //configure flag(HCD only, last action)
#define MAXP                BIT7    //max packet 0:32bytes, 1:64bytes

//USBSTS
#define USBINT              BIT0    //USBSTS bit 0
#define USBERRORINT         BIT1
#define USBRESUMEDETECT     BIT2
#define USBHSERROR          BIT3    //host system error
#define USBHCERROR          BIT4    //host controller error
#define USBHCHALTED         BIT5    //HCHalted
#define USBINTMASK          (BIT0|BIT1|BIT2|BIT3|BIT4)

//PORT1 PORT2

//Current Connect Status RO.
#define CCS                BIT0
//Connect Status Change R/WC.
#define CSC                BIT1
//Port Enabled/Disabled R/W.
#define PED                BIT2
//Port Enable/Disable Change R/WC.
#define PEDC               BIT3
//Low Speed Device Attached RO.
#define LSDA               BIT8
//Port Reset R/W.
#define PR                 BIT9
//Suspend R/W
#define SUSPEND            BIT12

#ifdef __cplusplus
extern "C" {
#endif

struct UHCI_TransferDescriptor;
typedef struct UHCI_TransferDescriptor UHCI_TD;

/*
                                                           3 2  1 0
        -----------------------------------------------------------
        |                   Link Pointer                  |0|Vf|Q|T|
        -----------------------------------------------------------

         3130          23    16 15     11 10                      0
        -----------------------------------------------------------
        |R  | |       | Status |    R    |     ActLen              |                           
        -----------------------------------------------------------        

        31          2120 1918     1514           8 7              0                                                            
        -----------------------------------------------------------
        |   MaxLen    |R|D| EndPt  | Dev Addr     |       PID      |
        -----------------------------------------------------------

        31                                                        0                                                           
        -----------------------------------------------------------
        |                  Buffer Pointer                          |
        -----------------------------------------------------------       
*/
typedef struct UHCI_TransferDescriptor
{
    uint32_t LinkPointer;
    union
    {
        uint32_t ControlStatus;
        struct
        {
            uint16_t ActualLen : 11;
            uint16_t Reserved0 : 5;
            uint16_t Reserved1 : 1;
            uint16_t E_Bitstuff : 1;    //error bits
            uint16_t E_CRC_Timeout : 1;
            uint16_t E_NAK : 1;
            uint16_t E_Babble : 1;
            uint16_t E_DataBuffer : 1;
            uint16_t E_Stalled : 1;
            uint16_t Active : 1;
            uint16_t Interrupt : 1;     //IOC
            uint16_t Isochronous : 1;   //IOS
            uint16_t LowSpeed : 1;      //CS_LowSpeed
            uint16_t ErrorLimit : 2;    //CS_C_ERR. 0: no limit
            uint16_t ShortPacketDetect : 1; //SPD
            uint16_t Reserved2 : 2;
        }ControlStatusBits;
    };
    union
    {
        uint32_t Token;
        struct
        {
            #if defined(__BC__)
            uint16_t PID : 8;
            uint16_t DeviceAddress : 7;
            uint16_t Endpoint : 4;
            uint16_t DataToggle : 1;
            uint16_t Reserved : 1;
            uint16_t MaxLen : 11; //0x7FF=NULL
            #else
            uint32_t PID : 8;
            uint32_t DeviceAddress : 7;
            uint32_t Endpoint : 4;
            uint32_t DataToggle : 1;
            uint32_t Reserved : 1;
            uint32_t MaxLen : 11; //0x7FF=NULL
            #endif
        }TokenBits; 
    };
    uint32_t BufferPointer;

    //extension
    uint32_t PAddr; //physical addr
    HCD_Request* pRequest;
    UHCI_TD* pNext;
    UHCI_TD* pPrev;
    #if defined(__BC__)
    uint16_t padding[3];
    #endif
}UHCI_TD;

/*
                                                                     3 2  1 0
        -----------------------------------------------------------
        |                   Queue Head Link Pointer       |0|0|Q|T|
        -----------------------------------------------------------

                                                                      3 2  1 0
        -----------------------------------------------------------
        |                   Queue Element Link Pointer    |0|R|Q|T|
        -----------------------------------------------------------
*/
typedef struct
{ // QH must keep 16-byte align.
    uint32_t HeadLink;
    uint32_t ElementLink;

    //externsion
    struct
    {
        uint16_t wMaxPacketSize : 11;
        uint16_t Type : 2;
        uint16_t Dir : 1;
        uint16_t Interval : 2;
        uint16_t DataToggle : 1;
        uint16_t Reserved0 : 15;
    }Flags;
    UHCI_TD* pTail;
    #if defined(__BC__)
    uint16_t padding[1];
    #endif
}UHCI_QH;

typedef struct UHCI_HostControllerDriverData
{
    UHCI_QH QH1ms;
    UHCI_QH QH2ms;
    UHCI_QH QH8ms;

    UHCI_QH ControlQH;
    UHCI_QH BulkQH;

    UHCI_QH* InteruptTail[3];
    UHCI_QH* ControlTail;
    UHCI_QH* BulkTail;
    uint32_t dwFrameListBase;// memory address.
    uint16_t wFrameListHandle; //xms handle
}UHCI_HCData;

typedef struct UHCI_HostControllerDeviceData
{
    UHCI_QH ControlQH;
}UHCI_HCDeviceData;

BOOL UHCI_InitController(HCD_Interface* pHCI, PCI_DEVICE* pPCIDev);
BOOL UHCI_DeinitController(HCD_Interface* pHCI);
BOOL UHCI_ISR(HCD_Interface* pHCI);

uint8_t UHCI_ControlTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t inputp setup8[8],
    void* nullable pSetupData, uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData);

uint8_t UHCI_DataTransfer(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, uint8_t* inoutp pBuffer,
    uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData);

#ifdef __cplusplus
}
#endif

#endif
