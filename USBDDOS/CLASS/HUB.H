#ifndef _HUB_H_
#define _HUB_H_
#include "USBDDOS/USB.H"

#define USB_REQ_TYPE_HUB      (USB_REQTYPE_CLASS)
#define USB_REQ_TYPE_HUBPORT  (USB_REQTYPE_CLASS |  USB_REQREC_ENDPOINT | USB_REQREC_INTERFACE)
#define USB_DT_HUB 0x29

typedef struct USB_HubDescriptor
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bNbrPorts;
    uint16_t wHubCharacteristics;
    uint8_t bPwrOn2PwrGood;
    uint8_t bHubContrCurrent;
    //DeviceRemovable, variable bit mask
    //PortPwrCtrlMask, variable bit mask
}USB_HubDesc;

//bRequest
//common request 0,1,3,6,7 in USB.H
//for USB_REQ_TYPE_HUBPORT
#define USB_REQ_CLEAR_TT_BUFFER 8
#define USB_REQ_RESET_TT 9
#define USB_REQ_GET_TT_STATE 10
#define USB_REQ_STOP_TT 11

//hub features
#define C_HUB_LOCAL_POWER 0
#define C_HUB_OVER_CURRENT 1

//port features
#define PORT_CONNECTION 0
#define PORT_ENABLE 1 //clear
#define PORT_SUSPEND 2 //set,clear
#define PORT_OVER_CURRENT 3
#define PORT_RESET 4 //set
#define PORT_POWER 8 //set,clear
#define PORT_LOW_SPEED 9
#define C_PORT_CONNECTION 16 //clear
#define C_PORT_ENABLE 17 //clear
#define C_PORT_SUSPEND 18 //clear
#define C_PORT_OVER_CURRENT 19 //clear
#define C_PORT_RESET 20 //clear
#define PORT_TEST 21
#define PORT_INDICATOR 22 //set,clear

//port status bits (get)
#define PS_CONNECTION BIT0
#define PS_ENABLE BIT1
#define PS_SUSPEND BIT2
#define PS_OVERCURRENT BIT3
#define PS_RESET BIT4
#define PS_POWER BIT8
#define PS_LOW_SPEED BIT9
#define PS_HIGH_SPEED BIT10
#define PS_TEST BIT11
#define PS_INDICATOR BIT12

//port status change bits (get)
#define PSC_CONNECTION BIT0
#define PSC_ENABLE BIT1
#define PSC_SUSPEND BIT2
#define PSC_OVERCURRENT BIT3
#define PSC_RESET BIT4

//indicattor, wIndex high byte
#define PORT_INDICATOR_AUTO 0
#define PORT_INDICATOR_AMBER 1
#define PORT_INDICATOR_GREEN 2
#define PORT_INDICATOR_OFF 3

typedef struct
{
    USB_HubDesc desc;
    uint8_t statusEP; //IN, INTR
}USB_HUB_DriverData;

BOOL USB_HUB_InitDevice(USB_Device* pDevice);

BOOL USB_HUB_DeinitDevice(USB_Device* pDevice);


#endif