#ifndef _USBD_H_
#define _USBD_H_
#include "USBDDOS/HCD/HCD.H"

#ifdef __cplusplus
extern "C" {
#endif

//bmRequestType
#define USB_REQ_WRITE           0        //host to dev
#define USB_REQ_READ            0x80    //dev to host
#define USB_REQTYPE_STANDARD    0x00
#define USB_REQTYPE_CLASS       0x20    //device class
#define USB_REQTYPE_VENDOR      0x40    //vendor specific

//recipient mask
#define USB_REQREC_DEVICE       0
#define USB_REQREC_INTERFACE    1
#define USB_REQREC_ENDPOINT     2
#define USB_REQREC_OTHER        3

//bRequest
#define USB_REQ_GET_STATUS      0
#define USB_REQ_CLEAR_FEATURE   1
#define USB_REQ_SET_FEATURE     3
#define USB_REQ_SET_ADDRESS     5
#define USB_REQ_GET_DESCRIPTOR  6
#define USB_REQ_SET_DESCRIPTOR  7
#define USB_REQ_GET_CONFIGURATION 8
#define USB_REQ_SET_CONFIGURATION 9
#define USB_REQ_GET_INTERFACE   10
#define USB_REQ_SET_INTERFACE   11
#define USB_REQ_SYNCH_FRAME     12

#define ENDPOINT_HALT           0 //clear feature: halt 

//Descriptor Types , wValue
#define USB_DT_DEVICE          1
#define USB_DT_CONFIGURATION   2
#define USB_DT_STRING          3
#define USB_DT_INTERFACE       4
#define USB_DT_ENDPOINT        5
#define USB_DT_DEVICE_QUALIFIER  6
#define USB_DT_OTHER_SPEED_CONFIGURATION 7
#define USB_DT_INTERFACE_POWER 8
#define USB_DT_CSINTERFACE     0x24 //class-specific interface

//Descriptor Index , wIndex
#define USB_LANG_ID_ENG         0x0409    //for USB_DT_STRING

//device class, subclass, protocol
#define USBC_INTERFACE          0x0
#define USBC_AUDIO              0x1
#define USBC_CDC                0x2      //communication device class
#define USBC_HID                0x3
//no 0x4 from spec
#define USBC_PHYSICAL           0x5
#define USBC_STILLIMAGE         0x6
#define USBC_PRINTER            0x7
#define USBC_MASSSTORAGE        0x8
#define USBC_HUBCLASS           0x9
#define USBC_CDCDATA            0xA
#define USBC_SMARTCARD          0xB
//no 0xC from spec
#define USBC_SECURITYCONTENT    0xD
#define USBC_VEDIO              0xE    
#define USBC_PERSONHEALTHCARE   0xF
#define USBC_AV                 0x10 //aidio/video
#define USBC_BILLBOARD          0x11
#define USBC_TYPECBRIDGE        0x12
#define USBC_WIRELESS           0xE0

#define UHCITYPE    1
#define OHCITYPE    2
#define EHCITYPE    4

typedef struct USB_DeviceDescriptor
{
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint16_t    wbcdUSB;
    uint8_t     bDeviceClass;
    uint8_t     bDeviceSubClass;
    uint8_t     bDeviceProtocol;
    uint8_t     bMaxPacketSize;
    uint16_t    widVendor;
    uint16_t    widProduct;
    uint16_t    wbcdDevice;
    uint8_t     biManufacture;
    uint8_t     biProduct;
    uint8_t     biSerialNumber;
    uint8_t     bNumConfigurations;
}USB_DeviceDesc;

typedef struct USB_Endpoint_Descriptor
{
    uint8_t    bLength;
    uint8_t    bDescriptorType;
    union
    {
        uint8_t bEndpointAddress;
        struct
        {
            uint8_t Num : 4;
            uint8_t Reserved : 3;
            uint8_t Dir : 1;   //1:in, 0:out
        }bEndpointAddressBits;
    };

    union
    {
        uint8_t    bmAttributes;
        struct
        {
            uint8_t TransferType : 2;    //USB_ENDPOINT_TRANSFER_TYPE_*
            uint8_t SyncType : 2;
            uint8_t UsageType : 2;
            uint8_t Reserved : 2;
        }bmAttributesBits;
    };

    union
    {
        uint16_t wMaxPacketSize;
        struct
        {
            uint16_t Size : 11;
            uint16_t AddtionalTransactionPerFrame : 2;
            uint16_t Reserved : 3;
        }wMaxPacketSizeFlags;
    };

    uint8_t    bInterval;
}USB_EndpointDesc;

typedef struct USB_InterfaceDesc
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;

    //////////////////////
    //extened data. copy data from source not containing this data may cause page fualt but
    //usually there's extra data below the header so it is ok
    USB_EndpointDesc* pEndpoints;
    uint16_t offset; //offset in descriptor buffer, used for get class specific interface desc inside (right after) interface desc
}USB_InterfaceDesc;

typedef struct USB_ConfigurationDescriptor
{
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint16_t    wTotalLength;
    uint8_t     bNumInterfaces;
    uint8_t     bConfigValue;
    uint8_t     bConfigIndex;
    uint8_t     bmAttributes;
    uint8_t     bMaxPower;

    //extened data. copy data from source not containing this data may cause page fualt but
    //usually there's extra data below the header so it is ok
    USB_InterfaceDesc* pInterfaces;
}USB_ConfigDesc;

typedef struct
{
    uint8_t     bmRequestType;
    uint8_t     bRequest;
    uint16_t    wValue;
    uint16_t    wIndex;
    uint16_t    wLength;
}USB_Request;

typedef enum USB_DeviceState
{
    DS_UnAttached,
    DS_Default,
    DS_Addressed,
    DS_Configured,
    DS_Ready,   //driver installed
}USB_DevState;

//USB stack: hc => usb => class

//USB device struct
typedef struct
{
    HCD_Device      HCDDevice;  //HC device inheritance (C compatible aggregation)
    USB_DeviceDesc  Desc;
    char            sManufacture[32];
    char            sProduct[32];
    char            sSerialNumber[32];
    USB_ConfigDesc* pConfigList;
    void**          pEndpoints;         //pre-created endpoints
    USB_EndpointDesc** pEndpointDesc;   //1:1 map to pEndpoints
    void*           pDefaultControlEP;  //default control pipe (ep0)
    uint8_t*        pDeviceBuffer;      //USB_DEVBUFFER_SIZE
    uint8_t*        pSetup;             //8 byte setup buffer
    uint8_t         bNumEndpoints;
    uint8_t         bCurrentConfig;
    uint8_t         bStatus; //device status: default, addressed, configured

    void*           pDriverData; // device driver data    
}USB_Device;

//hc device to usb device
#define HC2USB(hcd) ((USB_Device*)(((uintptr_t)(hcd)) - offsetof(USB_Device, HCDDevice)))

//generic class driver
typedef struct USB_ClassDriver
{
    BOOL (*InitDevice)(USB_Device* pDevice);
    BOOL (*DeinitDevice)(USB_Device* pDevice);
}USBC_Driver;

//specific device driver, not use yet. driver should be added by upper stack user
typedef struct USB_VendorDriver
{
    uint32_t dVendorID;
    uint32_t dClassID;
    BOOL (*InitDevice)(USB_Device* pDevice);
    BOOL (*DeinitDevice)(USB_Device* pDevice);
}USB_Driver;

//custom functions before USB_Init & after USB_Deinit
typedef struct USB_PrePostRoutine
{
    void (*PreInit)(void);
    void (*PostDeInit)(void);
}USB_Routine;

typedef struct 
{
    HCD_Type HC_Types[USB_MAX_HC_TYPE];
    HCD_Interface HC_List[USB_MAX_HC_COUNT];
    USBC_Driver ClassDrivers[USBC_MAX_DRIVER];
    USB_Device Devices[USB_MAX_DEVICE_COUNT];
    USB_Routine InitFunctions[USB_MAX_DEVICE_COUNT]; //un-ordered function array
    uint16_t HC_Count;
    uint16_t DeviceCount;
}USB_Table;

extern USB_Table USBT;

void USB_Init(void);

//start requst and return immediately. pRequest & pBuffer are copied so is free to release
uint8_t USB_SendRequest(USB_Device* pDevice, USB_Request* pRequest, void* pBuffer, HCD_COMPLETION_CB pFnCallback, void* pCallbackData); //send request to endpoint 0

//wait on result. will enable interrupt, not aviable in interrupt handler
uint8_t USB_SyncSendRequest(USB_Device* pDevice, USB_Request* pRequest, void* pBuffer);

//start transfer and return immediately. pBuffer are copied so is free to release
uint8_t USB_Transfer(USB_Device* pDevice, void* pEndpoint, uint8_t* pBuffer, uint16_t length, HCD_COMPLETION_CB pFnCallback, void* pCallbackData);

//wait on result. will enable interrupt, not aviable in interrupt handler
uint8_t USB_SyncTransfer(USB_Device* pDevice, void* pEndpoint, uint8_t* pBuffer, uint16_t length, uint16_t* txlen);

void* USB_FindEndpoint(USB_Device* pDevice, USB_EndpointDesc* pDesc);

USB_EndpointDesc* USB_GetEndpointDesc(USB_Device* pDevice, void* pEndpoint);

uint8_t USB_GetConfigDescriptor(USB_Device* pDevice, uint8_t* buffer, uint16_t length);

BOOL USB_SetConfiguration(USB_Device* pDevice, uint8_t configuration);

BOOL USB_ParseConfiguration(uint8_t* pBuffer, uint16_t length, USB_Device* pDevice);

BOOL USB_GetDescriptorString(USB_Device* pDevice, uint8_t bID, char* pBuffer, uint16_t length);

void USB_ShowDeviceInfo(USB_Device* pDevice);

BOOL USB_ClearHalt(USB_Device* pDevice, uint8_t epAddr); //clear stall

void USB_IdleWait();

//ISR related
//the finalizer is a routine called after EOI, so that other IRQ can be handled.
//currently used by HID to simulate scancodes with multiple bytes (E0 prefix) 
typedef struct _USB_ISR_Finalizer
{
    void* data;
    void (*FinalizeISR)(void* data);
    struct _USB_ISR_Finalizer* next;
}USB_ISR_Finalizer;

//temporarily add finalizer. must be called in USB ISR handler (usually a data-callback from real HC interrupt, i.e.
//callback passed to USB_Transfer/USB_SendRequest)
//param finalizer will be released on ISR return, by calling free()
//FinalizeISR is responsible to manage data ptr i.e. if data in USB_ISRFinalizer is malloced, FinalizeISR is responsible to free it.
void USB_ISR_AddFinalizer(USB_ISR_Finalizer* finalizer);

#ifdef __cplusplus
}
#endif

#endif
