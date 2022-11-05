#ifndef _HCD_H_
#define _HCD_H_ 1
#include "USBDDOS/USBCFG.H"
#include "USBDDOS/PCI.h"

//host controler driver generic

#define HCDBUFFER_SIZE 0x100

struct HCD_HostControllerIterface;
typedef struct HCD_HostControllerIterface HCD_Interface;

typedef struct  //device in HC's view
{
    uint8_t         bHubPort; //0 based port index
    uint8_t         bAddress; //device address assigned outside by enumeration process
    uint8_t         bEndpointSpeed;
    uint8_t         bNumEndpoints;  //ep alllocated from HC
    HCD_Interface*  pHCI;           //unique host controller data
    void*           pHCData;        //per device host controller data
}HCD_Device;
#define HCD_IS_DEVICE_VALID(device) ((device) != NULL && (device)->pHCData != NULL && (device)->pHCI != NULL)

typedef enum HCD_TransferDirection
{
    HCD_TXW = 0x0,    //host to device
    HCD_TXR = 0x1,    //device to host
}HCD_TxDir;

typedef uint8_t (*HCD_CONTROL_FUNCTION) (HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir, uint8_t outputp pSetup[8], void* nullable pSetupData, uint16_t length); //setup always 8B by the spec
typedef uint8_t (*HCD_TRANSFER_FUNCTION) (HCD_Device* pDevice, uint8_t Endpoint, HCD_TxDir dir, uint8_t* inoroutp pBuffer, uint16_t length, uint16_t* outputp txlen); //transfer dir are decided by endpoint address

typedef struct HCD_HostrControllerDriverMethod
{
    //note: pReqData must be allocated with DPMI, or from device buffer
    HCD_CONTROL_FUNCTION ControlTransfer;

    //note: bufferPtr must be allocated with DPMI, or from device buffer
    HCD_TRANSFER_FUNCTION IsochronousTransfer;

    //note: bufferPtr must be allocated with DPMI, or from device buffer
    HCD_TRANSFER_FUNCTION BulkTransfer;

    //note: bufferPtr must be allocated with DPMI, or from device buffer
    HCD_TRANSFER_FUNCTION InterruptTransfer;

    //get control buffer for this device (per instance), in HCDBUFFER_SIZE
    uint8_t* (*GetHCDBuffer)(HCD_Device* pDevice);

    uint16_t (*GetPortStatus)(HCD_Interface* pHCI, uint8_t port);

    BOOL (*SetPortStatus)(HCD_Interface* pHCI, uint8_t port, uint16_t status);

    BOOL (*InitDevice)(HCD_Device* pDevice);

    BOOL (*CreateEndpoint)(HCD_Device* pDevice, uint8_t EPAddr, HCD_TxDir dir, uint8_t bTransferType, uint16_t MaxPacketSize, uint8_t bInterval);

    BOOL (*RemoveDevice)(HCD_Device* pDevice);

}HCD_Method;

typedef struct HCD_HostControllerIterface
{    
    HCD_Method*     pHCDMethod; //host controller driver methods
    void*           pHCDData;   //host controller driver data
    PCI_Addr        PCIAddr;    //BDF,set by outside routine
    uint32_t        dwPhysicalAddress;   //physical memory address (not directly accessible)
    uint32_t        dwBaseAddress;       //MMIO linear address
    HCD_Device*     DeviceList[HCD_MAX_DEVICE_COUNT];
    uint8_t         bNumPorts;
    uint8_t         bDevCount;
}HCD_Interface;

typedef struct HCD_HostControllerType
{
    uint32_t dwID;    //program interface id
    BOOL (*InitController)(HCD_Interface* pHCI);
    BOOL (*DeinitController)(HCD_Interface* pHCI);
}HCD_Type;

#define USB_MAX_HC_TYPE 3   //uhci, ohci, ehci

#endif
