#ifndef _USBCFG_H_
#define _USBCFG_H_

//configurables
#define USB_MAX_HC_COUNT        8
#define USBC_MAX_DRIVER         0x10        //class driver [0,USBC_PERSONHEALTHCARE]
#define USB_MAX_VENDOR_DRIVER   0x10        //not used yet
#define USB_DEVBUFFER_SIZE      0x100
#define USB_MAX_HUB_COUNT       0x10

//constants
#define HCD_MAX_DEVICE_COUNT 15     //device per root hub by spec
#if defined(__BC__) || defined(__WC__)
#define USB_MAX_DEVICE_COUNT 4      //save 16 bit memory
#else
#define USB_MAX_DEVICE_COUNT 127    //no need to change, max address 127
#endif

#define USB_ENDPOINT_TRANSFER_TYPE_CTRL 0
#define USB_ENDPOINT_TRANSFER_TYPE_ISOC 1
#define USB_ENDPOINT_TRANSFER_TYPE_BULK 2
#define USB_ENDPOINT_TRANSFER_TYPE_INTR 3

//for full speed & low speed
#define USB_FRAME_SIZE_FULLSPEED        1024    //max bandwith during 1 frame, theoretical 1280
#define USB_ISO_FRAME_SIZE_FULLSPEED    921 //90%

//port status
#define USB_PORT_SPEEDMASK 0x0FL    //(R)
#define USB_PORT_Low_Speed_Device   1
#define USB_PORT_Full_Speed_Device  2
#define USB_PORT_High_Speed_Device  4
#define USB_PORT_ATTACHED 0x10L    //(R)
#define USB_PORT_ENABLE   0x20L
#define USB_PORT_SUSPEND  0x40L
#define USB_PORT_RESET    0x80L
#define USB_PORT_DISABLE  0x100L
#define USB_PORT_CONNECT_CHANGE 0x200L  //write to it will clear the bit

//bits
#define BIT0    0x01UL
#define BIT1    0x02UL
#define BIT2    0x04UL
#define BIT3    0x08UL
#define BIT4    0x10UL
#define BIT5    0x20UL
#define BIT6    0x40UL
#define BIT7    0x80UL
#define BIT8    0x100UL
#define BIT9    0x200UL
#define BIT10   0x400UL
#define BIT11   0x800UL
#define BIT12   0x1000UL
#define BIT13   0x2000UL
#define BIT14   0x4000UL
#define BIT15   0x8000UL
#define BIT16   0x10000UL
#define BIT17   0x20000UL
#define BIT18   0x40000UL
#define BIT19   0x80000UL
#define BIT20   0x100000UL
#define BIT21   0x200000UL
#define BIT22   0x400000UL
#define BIT23   0x800000UL
#define BIT24   0x1000000UL
#define BIT25   0x2000000UL
#define BIT26   0x4000000UL
#define BIT27   0x8000000UL
#define BIT28   0x10000000UL
#define BIT29   0x20000000UL
#define BIT30   0x40000000UL
#define BIT31   0x80000000UL

#endif //_USBCFG_H_
