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

//Keyboard working in Virtualbox, DOS7.0 command line, but not working in EDIT.COM, neither some games.
//But wholly working in real machines, tested in DOS7.1(win98 real dos).
//tested with Skyroads, cannot jump(SPACE) when both UP+LEFT(or RIGHT) are pressed, debug shows the keyboard won't send 3 key combinations, it's not a driver's bug

//tested mouse with WarcraftII, movement go wild soon after in game. need more test & bugfix.

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
#define USB_REQ_HID_GET_PROTOCOL    0x03    //      Y           O               Y               Y
#define USB_REQ_HID_SET_REPORT      0x09    //      O           O               Y               Y
#define USB_REQ_HID_SET_IDLE        0x0A    //      O           O               Y               O
#define USB_REQ_HID_SET_PROTOCOL    0x0B    //      Y           O               Y               O

//USB_REQ_HID_?ET_PROTOCOL
#define USB_HID_PROTOCOL_BOOT       0
#define USB_HID_PROTOCOL_REPORT     1

//USB_REQ_HID_SET_IDLE
#define USB_HID_MAKE_IDLE(duration, ReportID) ((uint16_t)(((duration)<<8)|((ReportID)&0xFF)))
#define USB_HID_IDLE_INDEFINITE 0
#define USB_HID_IDLE_REPORTALL 0

//USB_REQ_HID_?ET_REPORT
#define USB_HID_REPORT_INPUT    0x1
#define USB_HID_REPORT_OUTPUT   0x2
#define USB_HID_REPORT_FEATURE  0x2
#define USB_HID_MAKE_REPORT(ReportType, ReportID) ((uint16_t)(((ReportType)<<8)|((ReportID)&0xFF)))

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
//B.1 Protocol 1(Keyboard)
#define USB_HID_LED_NUMLOCK     0x01
#define USB_HID_LED_CAPSLOCK    0x02
#define USB_HID_LED_SCROLLLOCK  0x04
#define USB_HID_LED_COMPOSE     0x08
#define USB_HID_LED_KANA        0x10

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
    uint8_t Modifier;
    uint8_t Reserved;   //not used in boot protocol
    uint8_t Keycodes[6];
}USB_HID_KEY;

//boot protocol mouse packet. Appendix B.2
typedef struct USB_HIDE_MousePacket_Boot
{
    //uint8_t Button1 : 1;
    //uint8_t Button2 : 1;
    //uint8_t Button3 : 1;
    //uint8_t unused : 5;
    uint8_t Button;

    int8_t DX;
    int8_t DY;
    
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
    uint8_t Idle : 1;
    uint8_t Index : 1;    //index to Data, flipped each time
    uint16_t BIOSModifier;
    uint8_t PrevCount;
    uint8_t Interval; //interval for interrupt input endpoint
    uint8_t RecordCount; //
    uint8_t Records[6]; //record key in pressed order. after one key up, deciding the effective one

    USB_HID_Data Data[2];
    uint16_t DelayTimer;
    uint16_t RepeatingTimer;

}USB_HID_Interface;

#define USB_HID_KEYBOARD 0
#define USB_HID_MOUSE 1

typedef struct
{
    USB_HID_Interface Interface[2]; //USB_HID_KEYBOARD or USB_HID_MOUSE. There're wireless kbd & mouse devices with one USB receiver.
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