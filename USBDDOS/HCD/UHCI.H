#ifndef _UHCI_H_
#define _UHCI_H_
#include "USBDDOS/HCD/HCD.H"

//  invalid QH, TD; means last QH, TD
#define TerminateFlag      0x01
  
#define QHFlag             0x02
#define DepthFirstFlag     0x04

#define ActiveStatus       0x800000
#define StalledStatus      0x400000
#define DataBufferError    0x200000
#define BabbleDetected     0x100000
#define NAKReceived        0x80000
#define CRCTimeOutError    0x40000
#define BitstuffError      0x20000

#define C_ERR              0x8000000
#define LowSpeed           0x4000000
#define ISO                0x2000000    //isochronous transfer
#define IOC                0x1000000    //interrupt on complete
#define DataToggle         0x80000

// UHCI register
#define USBCMD              0x00
#define USBSTS              0x02
#define USBINTR             0x04
#define FRNUM               0x06
#define FLBASEADD           0x08    //frame list base addr
#define SOF                 0x0c
#define PORTBASE            0x10
#define PORT1               0x10
#define PORT2               0x12

//# USBCMD     
#define RS                  0x01
#define HCRESET             0x02
#define GRESET              0x04

//#PORT1 PORT2

//Current Connect Status RO.
#define  CCS                0x01
//Connect Status Change R/WC.
#define  CSC                0x02
//Port Enabled/Disabled R/W.
#define  PED                0x04
//Port Enable/Disable Change R/WC.
#define  PEDC               0x08
//Low Speed Device Attached RO.
#define  LSDA               0x100
//Port Reset R/W.
#define  PR                 0x200

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
typedef struct QH {
    uint32_t   HeadLink;
    uint32_t   ElementLink;
    uint32_t   uint16_t1;    // QH must keep 16-uint8_t align.
    uint32_t   uint16_t2;
}UHCI_QH;

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
typedef struct UHCI_TransferDescriptor {
      uint32_t  LinkPointer;
      uint32_t  ControlStatus;
      uint32_t  token;
      uint32_t  BufferPoint;
}UHCI_TD;

typedef struct UHCI_HostControllerDriverData {
    UHCI_QH     QH1ms;
    UHCI_QH     QH2ms;
    UHCI_QH     QH8ms;
    uint32_t    dHcdDataArea;// memory address.
}UHCI_HCDD;

typedef struct UHCI_HostControllerDeviceData {
    UHCI_TD     TD[3];
    uint8_t     Setup[8];
    uint8_t     buffer1[256];
} UHCI_DD;


BOOL    UHCI_InitController(HCD_Interface* pHCI);
BOOL    UHCI_DeinitController(HCD_Interface* pHCI);

uint8_t UHCI_ControlTransfer(HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir, uint8_t outputp pSetup[8], void* nullable pSetupData, uint16_t length);

uint8_t UHCI_IsochronousTransfer(HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir, uint8_t* inoroutp pBuffer, uint16_t length, uint16_t* outputp txlen);

uint8_t UHCI_BulkTransfer(HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir, uint8_t* inoroutp pBuffer, uint16_t length, uint16_t* outputp txlen);

uint8_t UHCI_InterruptTransfer(HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir, uint8_t* inoroutp pBuffer, uint16_t length, uint16_t* outputp txlen);

#endif
