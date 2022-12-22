//host controller interface table
//device driver tabe

//static bridging tables between USB layer and controler/deivces
//the basic USB layer should not care about those detail and theoritically the table can be dynamically inited & manipulated.
//but as generic hc and classes, they can be statically built, while vendor specific drivers should be added dynamically.

//for flat32 mode the code and data are loaded into himmem by the HDPMI loader/DOS Extender.
//in real/v86 mode, this table can be relocated to himem (through custom malloc) ready for TSRs
//TODO: another option is to write a custom (simple) loader in real mode.

#include <stdlib.h>
#include <string.h>
#include "USBDDOS/DPMI/DPMI.H"
#include "USBDDOS/USB.H"
#include "USBDDOS/HCD/OHCI.H"
#include "USBDDOS/HCD/UHCI.H"
#include "USBDDOS/CLASS/CDC.H"
#include "USBDDOS/CLASS/MSC.H"
#include "USBDDOS/CLASS/HID.H"

#define USE_CDC 1
#define USE_MSC 1
#define USE_HID 1

USB_Table USBT = 
{
    //HC_Types
    {
        //UHCI
        {
            0x00,
            UHCI_InitController,
            UHCI_DeinitController,
            UHCI_ISR,
        },
        //OHCI
        {
            0x10,
            OHCI_InitController,
            OHCI_DeinitController,
            OHCI_ISR,
        },
        //EHCI
        {
            0x20,
            NULL,
            NULL,
        },
    },

    //HC_List
    {0},

    //ClassDrivers
    {
        //USBC_INTERFACE        0x0
        0,0,
        //USBC_AUDIO            0x1
        0,0,
        //USBC_CDC              0x2      //communication device class
        #if USE_CDC
        USB_CDC_InitDevice, USB_CDC_DeinitDevice,
        #else
        0,0
        #endif
        //USBC_HID              0x3
        #if USE_HID
        USB_HID_InitDevice, USB_HID_DeinitDevice,
        #endif
        //no 0x4 from spec
        0,0,
        //USBC_PHYSICAL         0x5
        0,0,
        //USBC_STILLIMAGE       0x6
        0,0,
        //USBC_PRINTER          0x7
        0,0,
        //USBC_MASSSTORAGE      0x8
        #if USE_MSC
        USB_MSC_InitDevice, USB_MSC_DeinitDevice,
        #else
        0, 0,
        #endif
        //USBC_HUBCLASS         0x9
        0,0,
        //USBC_CDCDATA          0xA
        0,0,
        //USBC_SMARTCARD        0xB
        0,0,
        //no 0xC from spec
        0,0,
        //USBC_SECURITYCONTENT  0xD
        0,0,
        //USBC_VEDIO            0xE
        0,0,
        //USBC_PERSONHEALTHCARE 0xF
        0,0,
    },
    //Devices
    {0},
    //USB_Routine
    {
        USB_MSC_PreInit, USB_MSC_PostDeInit,
        USB_HID_PreInit, USB_HID_PostDeInit,
    },
    0,
    0,
};