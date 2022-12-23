#ifndef _HID_H_
#define _HID_H_
//Human Interface Class
//Device Class Definition for Human Interface Devices (HID) version 1.11:
//https://usb.org/sites/default/files/hid1_11.pdf
//
//the Universal Serial Bus HID Usage Tables (for key codes)
//https://www.usb.org/sites/default/files/documents/hut1_12v2.pdf
//
//the HID class uses an mandantory interrupt input and an optional interrupt output end point.
//the boot interface is required for bootable HID devices, i.e. used by BIOS to support simple input in BIOS setup menu, or in DOS
//the boot interface is more simple and here is to be supported and supported only.

#include "USBDDOS/USB.H"

#define USB_REQ_TYPE_HID    (USB_REQTYPE_CLASS | USB_REQREC_INTERFACE)

//descriptor type
#define USB_DT_HID    0x21
#define USB_DT_REPORT 0x22

//bInterfaceSubClass
#define USB_HIDSC_NO                0   //no subclass
#define USB_HIDSC_BOOT_INTERFACE    1   //boot interface

//bInterfaceProtocol
#define USB_HIDP_NONE           0
#define USB_HIDP_KEYBOARD       1
#define USB_HIDP_MOUSE          2

//bRequest. see Appendix G for requirments:     Boot mouse, Nonboot mouse, Boot keyboard, Nonboot keyboard (Y=required, O=optional)
#define USB_REQ_HID_GET_REPORT      0x01    //      Y           Y               Y               Y
#define USB_REQ_HID_GET_IDLE        0x02    //      O           O               O               O
#define USB_REQ_HID_GET_PROTOCOL    0x03    //      O           O               Y               Y
#define USB_REQ_HID_SET_REPORT      0x09    //      O           O               Y               Y
#define USB_REQ_HID_SET_IDLE        0x0A    //      Y           O               Y               O
#define USB_REQ_HID_SET_PROTOCOL    0x0B    //      Y           O               Y               O

//USB_REQ_HID_?ET_PROTOCOL
#define USB_HID_PROTOCOL_BOOT       0
#define USB_HID_PROTOCOL_REPORT     1

//USB_REQ_HID_SET_IDLE
#define USB_HID_MAKE_IDLE(duration, reportID) ((uint16_t)(((duration)<<8)|((reportID)&0xFF)))
#define USB_HID_IDLE_INDEFINITE 0
#define USB_HID_IDLE_REPORTALL 0

//bit masks for modifier, chapter 8.3
#define USB_HID_LCTRL       0x01
#define USB_HID_LSHIFT      0x02
#define USB_HID_LALT        0x04
#define USB_HID_LGUI        0x08
#define USB_HID_RCTRL       0x10
#define USB_HID_RSHIFT      0x20
#define USB_HID_RALT        0x40
#define USB_HID_RGUI        0x80

//keyboard LEDs (use output report to set)
#define USB_HID_LED_NUMLOCK     0x01
#define USB_HID_LED_CAPSLOCK    0x02
#define USB_HID_LED_SCROLLLOCK  0x04
#define USB_HID_LED_COMPOSE     0x08
#define USB_HID_LED_KANA        0x10

//BIOS modifier mask
#define USB_HID_BIOS_RSHIFT     0x0001
#define USB_HID_BIOS_LSHIFT     0x0002
#define USB_HID_BIOS_CTRL       0x0004
#define USB_HID_BIOS_ALT        0x0008
#define USB_HID_BIOS_SCRLLOCK_S 0x0010 //scroll locked staus
#define USB_HID_BIOS_NUMLOCK_S  0x0020 //num locked staus
#define USB_HID_BIOS_CAPSLOCK_S 0x0040 //caps locked staus
#define USB_HID_BIOS_INSLOCK_S  0x0080 //insert locked staus

#define USB_HID_BIOS_LCTRL      0x0100
#define USB_HID_BIOS_LALT       0x0200
#define USB_HID_BIOS_RCTRL      0x0400
#define USB_HID_BIOS_RALT       0x0800
#define USB_HID_BIOS_SCRLLOCK   0x1000 //scroll lock pressed
#define USB_HID_BIOS_NUMLOCK    0x2000 //num lock pressed
#define USB_HID_BIOS_CAPSLOCK   0x4000 //caps lock pressed
#define USB_HID_BIOS_SYSREQ     0x8000 //sys req pressed



#if defined(__DJ2__)
#pragma pack(1)
#endif

typedef struct USB_HID_SubDescriptor
{
    uint8_t bDescriptorType;
    uint16_t wDescriptorLength;
}USB_HID_SUBDESC;

typedef struct USB_HID_Descritor
{
    uint8_t bLength;
    uint8_t bDescriptorType; //USB_HID_DESCRIPTOR
    uint16_t bcdHID;
    uint8_t bCountryCode;
    uint8_t bNumberDescriptors;
    USB_HID_SUBDESC desc[1]; //could be more USB_HID_SUBDESC, decided by bNumberDescriptors
}USB_HID_DESC;
#if defined(__DJ2__)
_Static_assert(sizeof(USB_HID_DESC) == 9, "incorrect size"); //minimal size
#endif

//boot protocol keyboard packet. Appendix B.1
typedef struct USB_HID_KeyPacket_Boot
{
    uint8_t modifier;
    uint8_t reserved;   //not used in boot protocol
    uint8_t keycodes[6];
}USB_HID_KEY;

//boot protocol mouse packet. Appendix B.2
typedef struct USB_HIDE_MousePacket_Boot
{
    uint8_t Button1 : 1;
    uint8_t Button2 : 1;
    uint8_t Button3 : 1;
    uint8_t unused : 5;

    uint8_t X;
    uint8_t Y;
    
}USB_HID_MOUSE;

#if defined(__DJ2__)
#pragma pack()
#endif

typedef union 
{
    uint8_t Buffer[8];
    USB_HID_KEY Key;
    USB_HID_MOUSE Mouse;
}USB_HID_Data;

typedef struct
{
    USB_HID_DESC* Descirptors;
    void* pDataEP[2]; //interrupt in/out
    uint8_t bEPAddr[2];
    uint8_t bInterface;
    uint8_t Idle;
    uint8_t Interval; //interval for interrupt input endpoint
    uint8_t Index;    //index to Data, flipped each time

    USB_HID_Data Data[2];
    uint32_t DelayTimer;
    uint32_t RepeatingTimer;

}USB_HID_Interface;

typedef struct
{
    USB_HID_Interface Interface[2]; //USB_HIDP_KEYBOARD-1 or USB_HIDP_MOUSE-1 for keyboard/mouse. There're wireless kbd & mouse devices with one USB receiver.
}USB_HID_DriverData;

#ifdef __cplusplus
extern "C"
{
#endif

BOOL USB_HID_InitDevice(USB_Device* pDevice);

BOOL USB_HID_DeinitDevice(USB_Device* pDevice);

BOOL USB_HID_DOS_Install();
BOOL USB_HID_DOS_Uninstall();

void USB_HID_PreInit();
void USB_HID_PostDeInit();

#ifdef __cplusplus
}
#endif

#endif//_HID_H_