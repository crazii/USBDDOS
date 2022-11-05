#ifndef _USBD_H_
#define _USBD_H_
#include "USBDDOS/HCD/HCD.H"

#define BIT0    0x01L
#define BIT1    0x02L
#define BIT2    0x04L
#define BIT3    0x08L
#define BIT4    0x10L
#define BIT5    0x20L
#define BIT6    0x40L
#define BIT7    0x80L
#define BIT8    0x100L
#define BIT9    0x200L
#define BIT10   0x400L
#define BIT11   0x800L
#define BIT12   0x1000L
#define BIT13   0x2000L
#define BIT14   0x4000L
#define BIT15   0x8000L
#define BIT16   0x10000L
#define BIT17   0x20000L
#define BIT18   0x40000L
#define BIT19   0x80000L
#define BIT20   0x100000L
#define BIT21   0x200000L
#define BIT22   0x400000L
#define BIT23   0x800000L
#define BIT24   0x1000000L
#define BIT25   0x2000000L
#define BIT26   0x4000000L
#define BIT27   0x8000000L
#define BIT28   0x10000000L
#define BIT29   0x20000000L
#define BIT30   0x40000000L
#define BIT31   0x80000000L

#if __cplusplus
extern "C" {
#endif

///PID
#define    OUTPID   0xe1
#define    INPID    0x69
#define    SOFPID   0xa5
#define    SETUPPID 0x2d

#define    DATA0PID 0xc3
#define    DATA1PID 0x4b
#define    DATA2PID 0x87
#define    MDATAPID 0x0f

#define    ACKPID   0xd2
#define    NAKPID   0x5a
#define    STALLPID 0x1e
#define    NYETPID  0x96

#define    PREPID    0x3c
#define    ERRPID    0x3c
#define    SPLITPID  0x78
#define    PINGPID   0xb4
#define    RESERVEDPID   0xF0

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

//Descriptor Types , wValue
#define  USB_DT_DEVICE          1
#define  USB_DT_CONFIGURATION   2
#define  USB_DT_STRING          3
#define  USB_DT_INTERFACE       4
#define  USB_DT_ENDPOINT        5
#define  USB_DT_DEVICE_QUALIFIER  6
#define  USB_DT_OTHER_SPEED_CONFIGURATION 7
#define  USB_DT_INTERFACE_POWER 8

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

#define USB_ENDPOINT_TRANSFER_TYPE_CTRL 0
#define USB_ENDPOINT_TRANSFER_TYPE_ISOC 1
#define USB_ENDPOINT_TRANSFER_TYPE_BULK 2
#define USB_ENDPOINT_TRANSFER_TYPE_INTR 3

#define USB_FRAME_SIZE_LOWSPEED 1280    //max bandwith during 1 frame
#define USB_ISO_FRAME_SIZE_LOWSPEED 1152 //90%

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
        }bEndpointAddressFlags;
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
        }bmAttributesFlags;
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

#define USB_PORT_SPEEDMASK 0x0FL
#define USB_PORT_ATTACHED 0x10L    //readonly
#define USB_PORT_ENABLE   0x20L
#define USB_PORT_SUSPEND  0x40L
#define USB_PORT_RESET    0x80L
#define USB_PORT_DISABLE  0x100L
#define USB_PORT_CONNECT_CHANGE 0x200L   //write it will clear the bit
//read only
#define    USB_PORT_Low_Speed_Device   1
#define    USB_PORT_Full_Speed_Device  2
#define    USB_PORT_High_Speed_Device  4

//USB stack: hc => usb => class
typedef struct
{
    HCD_Device      HCDDevice;  //HC device inheritance (C compatible aggregation)
    USB_DeviceDesc  Desc;
    char            sManufacture[32];
    char            sProduct[32];
    char            sSerialNumber[32];
    USB_ConfigDesc* pConfigList;
    uint8_t         bCurrentConfig;
    uint8_t         bStatus;    //device status: default, addressed, configured

    void*           pDriverData;// device driver data    
}USB_Device;

//hc device to usb device (static_cast)
#define HC2USB(hcd) (USB_Device*)(((uintptr_t)(hcd)) - offsetof(USB_Device, HCDDevice))

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

typedef struct 
{
    HCD_Type HC_Types[USB_MAX_HC_TYPE];
    HCD_Interface HC_List[USB_MAX_HC_COUNT];
    USBC_Driver ClassDrivers[USBC_MAX_DRIVER];
    USB_Device Devices[USB_MAX_DEVICE_COUNT];
    uint16_t HC_Count;
    uint16_t DeviceCount;
}USB_Table;

extern USB_Table USBT;

void USB_Init(void);

BOOL USB_InitController(uint8_t bus, uint8_t dev, uint8_t func);

BOOL USB_InitDevice(HCD_Interface* pHCI, uint8_t portIndex);

uint8_t USB_SendRequest(USB_Device* pDevice, USB_Request* pRequest, void* pBuffer); //send request to endpoint 0

BOOL USB_SetConfiguration(USB_Device* pDevice, uint8_t configuration);

BOOL USB_ParseConfiguration(uint8_t* pBuffer, uint16_t length, USB_Device* pDevice);

void USB_ShowDeviceInfo(USB_Device* pDevice);

#if __cplusplus
}
#endif

#endif
