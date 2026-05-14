#ifndef _HCD_H_
#define _HCD_H_ 1
#include "USBDDOS/platform.h"
#include "USBDDOS/usbcfg.h"
#include "USBDDOS/pci.h"

//host controler driver generic

#define USB_MAX_HC_TYPE 3   //only to support uhci, ohci, ehci

#ifdef __cplusplus
extern "C" {
#endif

struct HCD_HostControllerInterface;
typedef struct HCD_HostControllerInterface HCD_Interface;
struct HCD_RequestBlock;
typedef struct HCD_RequestBlock HCD_Request;
typedef struct HCD_HUB HCD_HUB;

typedef struct //device in HC's view
{
    uint8_t         bHubPort; //0 based port index
    uint8_t         bAddress; //device address assigned outside by enumeration process
    uint8_t         bSpeed;
    uint8_t         bReserved;      //temp padding
    HCD_Request* volatile pRequest;
    HCD_Interface*  pHCI;           //unique host controller data
    void*           pHCData;        //per device host controller data
    HCD_HUB*        pHub;
}HCD_Device;

#define HCD_IS_DEVICE_VALID(device) ((device) != NULL && (device)->pHCData != NULL && (device)->pHCI != NULL)

typedef enum HCD_TransferDirection
{
    HCD_TXW = 0x0,    //host to device
    HCD_TXR = 0x1,    //device to host
}HCD_TxDir;

typedef void(*HCD_COMPLETION_CB) (HCD_Request* pRequest);

typedef struct HCD_RequestBlock //TODO: use request block as transfer parameter? (like URB in OSes)
{
    HCD_Device* pDevice;
    void* pBuffer;
    void* pEndpoint;
    HCD_COMPLETION_CB pFnCB; //user callback
    void* pCBData;//user specified data
    HCD_Request* pNext;
    uint16_t size;      //required size in bytse
    uint16_t transferred; //transferred size
    uint8_t endpoint;   //endpoint addr (num)
    uint8_t dir;        //HCD_TxDir
    uint8_t error;      //error code
    uint8_t padding1[1]; //pad to 32 bytes
    #if defined(__BC__) || defined(__WC__)
    #if defined(__MEDIUM__) || defined(__LARGE__)
    uint16_t padding2[4]; //TODO:
    #else
    uint16_t padding2[6]; //TODO:
    #endif
    #endif
}HCD_Request;

static_assert(sizeof(HCD_Request) <= 32, "size error");

//note: pSetupData & setup8 must be allocated with DPMI_DMAMalloc, or from device buffer
typedef uint8_t (*HCD_CONTROL_FUNCTION) (HCD_Device* pDevice, void* pEndPoint, HCD_TxDir dir, uint8_t inputp setup8[8],
    void* nullable pSetupData, uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData); //setup always 8B by the spec

//note: pBuffer must be allocated with DPMI_DMAMalloc, or from device buffer
typedef uint8_t (*HCD_TRANSFER_FUNCTION) (HCD_Device* pDevice, void* pEndPoint, HCD_TxDir dir, uint8_t* inoutp pBuffer,
    uint16_t length, HCD_COMPLETION_CB pCB, void* nullable pCBData);

typedef struct HCD_HostrControllerDriverMethod
{
    HCD_CONTROL_FUNCTION ControlTransfer;
    
    HCD_TRANSFER_FUNCTION IsochronousTransfer;

    HCD_TRANSFER_FUNCTION BulkTransfer;

    HCD_TRANSFER_FUNCTION InterruptTransfer;

    uint16_t (*GetPortStatus)(HCD_Interface* pHCI, uint8_t port);

    BOOL (*SetPortStatus)(HCD_Interface* pHCI, uint8_t port, uint16_t status);

    BOOL (*InitDevice)(HCD_Device* pDevice);

    BOOL (*RemoveDevice)(HCD_Device* pDevice);

    void* (*CreateEndpoint)(HCD_Device* pDevice, uint8_t EPAddr, HCD_TxDir dir, uint8_t bTransferType, uint16_t MaxPacketSize, uint8_t bInterval);

    BOOL (*RemoveEndPoint)(HCD_Device* pDevice, void* pEndpoint);

}HCD_Method;

typedef struct HCD_HostControllerType
{
    uint32_t dwPI;    //program interface
    const char* name;
    BOOL (*InitController)(HCD_Interface* pHCI, PCI_DEVICE* pPCIDev);
    BOOL (*DeinitController)(HCD_Interface* pHCI);
    BOOL (*ISR)(HCD_Interface* pHCI); //DON'T do IO, memory allocation in ISR.
}HCD_Type;

typedef struct HCD_HostControllerInterface
{
    PCI_Addr        PCIAddr;    //BDF,set by outside routine
    PCI_DEVICE      PCI;
    HCD_Type*       pType;
    HCD_Method*     pHCDMethod; //host controller driver methods
    void*           pHCDData;   //host controller driver data
    uint32_t        dwPhysicalAddress;   //physical memory address (not directly accessible)
    uint32_t        dwBaseAddress;       //IO port address or MMIO linear address
    HCD_Device*     DeviceList[HCD_MAX_DEVICE_COUNT];
    uint8_t         bNumPorts;
    uint8_t         bDevCount;
}HCD_Interface;

#define HCD_IS_CONTROLLER_VALID(phc) (phc != NULL && phc->pType != NULL && phc->pHCDMethod != NULL && phc->pHCDData != NULL)

BOOL HCD_InitController(HCD_Interface* pHCI, uint8_t bus, uint8_t dev, uint8_t func, HCD_Type* type, PCI_DEVICE* pPCIDev);

BOOL HCD_DeinitController(HCD_Interface* pHCI);

BOOL HCD_InitDevice(HCD_HUB* pHub, HCD_Device* pDevice, uint8_t port, uint16_t portStatus);

HCD_Device* HCD_FindDevice(HCD_Interface* pHCI, uint8_t address);

BOOL HCD_RemoveDevice(HCD_Device* pDevice);

//called internally by HC driver
HCD_Request* HCD_AddRequest(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, void* pBuffer, uint16_t size, uint8_t endpoint, HCD_COMPLETION_CB pFnCB, void* pCallbackData);

//invoke callback and remove the request entry
BOOL HCD_InvokeCallBack(HCD_Request* pRequest, uint16_t actuallen, uint8_t ecode);

//the initial state of all ports on a hub device should be disabled, @see USB_EnumerateDevices
typedef struct HCD_HUB
{
    const char* name;
    HCD_Interface* pHCI; //HC of this hub, always valid
    HCD_Device* pDevice; //optional, only valid if it is a hub device, instead of root hub
    uint8_t bHubAddress; //optional, only valid if it is a hub device, instead of root hub
    uint8_t bNumPorts;

    //hub functions
    uint16_t (*GetPortStatus)(struct HCD_HUB* pHCI, uint8_t port);
    BOOL (*SetPortStatus)(struct HCD_HUB* pHCI, uint8_t port, uint16_t status);
}HCD_HUB;

extern const HCD_HUB HCD_ROOT_HUB_Prototype; //root hub prototype, copy it and set 'pHCI & bNumPorts' and it's done.

#ifdef __cplusplus
}
#endif

#endif
