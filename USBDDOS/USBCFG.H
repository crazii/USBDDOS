#ifndef _USBCFG_H_
#define _USBCFG_H_

#define HCD_MAX_DEVICE_COUNT 15     //device per root hub by spec

#define USB_MAX_DEVICE_COUNT 127    //no need to change, max address 127

#define USB_MAX_HC_COUNT 8

#define USBC_MAX_DRIVER 0x10        //class driver [0,USBC_PERSONHEALTHCARE]

#define USB_MAX_VENDOR_DRIVER 0x10  //not used yet

#endif //_USBCFG_H_