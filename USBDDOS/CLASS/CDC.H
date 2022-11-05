#ifndef _CDC_H_
#define _CDC_H_
#include "USBDDOS/USB.H"

//USB Communication device class driver
//https://www.usb.org/document-library/class-definitions-communication-devices-12

#ifdef __cplusplus
extern "C"
{
#endif

#define USB_REQ_TYPE_CDC    (USB_REQTYPE_CLASS | USB_REQREC_INTERFACE)

#define USB_REQ_CDC_SET_LINE_CODING         0x20
#define USB_REQ_CDC_GET_LINE_CODING         0x21
#define USB_REQ_CDC_SET_LINE_CONTROL_STATE  0x22

//line coding
#define USB_CDC_LINE_CODING_1STOPBIT        0
#define USB_CDC_LINE_CODING_1p5STOPBIT      1
#define USB_CDC_LINE_CODING_2STOPBIT        0

#define USB_CDC_PARITY_NONE     0
#define USB_CDC_PARITY_ODD      1
#define USB_CDC_PARITY_EVEN     2
#define USB_CDC_PARITY_MARK     3
#define USB_CDC_PARITY_SPACE    4

typedef struct {
    uint32_t   dwDTERate;      //baud rate. bps
    uint8_t    bCharFormat;    //stop bits
    uint8_t    bParityType;    //parity
    uint8_t    bDataBits;      //5,6,7,8,16
}USB_CDC_LINE_CODING;

//line control state mask 0-15
#define USB_CDC_LINE_CONTROL_STATE_DTE      0x1
#define USB_CDC_LINE_CONTROL_STATE_CARRIER  0x2
//2:15 reserved

#define USB_CDC_MAX_PACKET_CONTROL      64
#define USB_CDC_MAX_PACKET_INTERRUPT    16
#define USB_CDC_MAX_PACKET_BULK         64

//subclasses
#define USBC_CDCSC_DIRECT_LINE_CONTROL_MODEL        0x01
#define USBC_CDCSC_ABSTRACT_CONTROL_MODEL           0x02
#define USBC_CDCSC_TELEPHONE_CONTROL_MODEL          0x03
#define USBC_CDCSC_MULTICHANNEL_CONTROL_MODEL       0x04
#define USBC_CDCSC_CAPI_CONTROL_MODEL               0x05
#define USBC_CDCSC_ETHERNET_CONTROL_MODEL           0x06
#define USBC_CDCSC_ATM_NETWORKING_CONTROL_MODEL     0x07
#define USBC_CDCSC_WIRELESS_HANDSET_CONTROL_MODEL   0x08
#define USBC_CDCSC_DEVICE_MANAGEMENT                0x09
#define USBC_CDCSC_MOBILE_DIRECT_LINE_MODEL         0x0A
#define USBC_CDCSC_OBEX                             0x0B
#define USBC_CDCSC_ETHERNET_EMULATION_MODEL         0x0C
#define USBC_CDCSC_NETWORK_CONTROL_MODEL            0x0D

//protocols
#define USBP_CDC_NONE                               0x00
#define USBP_CDC_ATCOMMAND                          0x01
#define USBP_CDC_ATCOMMAND_PCCA_101                 0x02
#define USBP_CDC_ATCOMMAND_PCCA_101_AND_ANNEXO      0x03
#define USBP_CDC_ATCOMMAND_GSM_707                  0x04
#define USBP_CDC_ATCOMMAND_3GPP_27007               0x05
#define USBP_CDC_ATCOMMAND_CDMA                     0x06
#define USBP_CDC_ETHERNET_EMULATION_MODEL           0x07

//data protocols
#define USBP_CDC_DATA_ISDN_BRI                      0x30
#define USBP_CDC_DATA_HDLC                          0x31
#define USBP_CDC_DATA_TRANSPARENT                   0x32
#define USBP_CDC_DATA_Q921_MANAGEMENT               0x50
#define USBP_CDC_DATA_Q921_DATA_LINK                0x51
#define USBP_CDC_DATA_Q921_TEI_MULTIPLEXOR          0x52
#define USBP_CDC_DATA_V42BIS_DATA_COMPRESSION       0x90
#define USBP_CDC_DATA_EURO_ISDN                     0x91
#define USBP_CDC_DATA_V24_RATE_ADAPTION_TO_ISDN     0x92
#define USBP_CDC_DATA_CAPI_COMMAND                  0x93
#define USBP_CDC_DATA_HOST_BASED_DRIVER             0xFD
#define USBP_CDC_DATA_IN_PROTOCOL_UNIT_FUNCTIONAL_DESCRIPTOR 0xFE

//caps
//ACM caps
#define USBCAP_CDC_ACM_COMM_FEATURE BIT0
#define USBCAP_CDC_ACM_LINE_CODING  BIT1 //line state & line coding
#define USBCAP_CDC_ACM_SEND_BREAK	BIT2
#define USBCAP_CDC_ACM_NETWORK_CONNECTION BIT3 //noification

//functional desc for bInterface (bDataInterface doesn't have desc by the spec)
typedef struct //ACM
{
	uint8_t bFunctionLength;
	uint8_t bDescriptorType; //USB_DT_CSINTERFACE
	uint8_t bDescriptorSubtype; //USBC_CDCSC_ABSTRACT_CONTROL_MODEL
	uint8_t bmCapabilities;
}USB_CDC_ACM_Desc;

typedef struct
{
	uint8_t bInterface;
	uint8_t bDataInterface;
	void* bDataEP[2];
	USB_CDC_ACM_Desc ACMDesc;
}USB_CDC_DriverData;

uint8_t USB_CDC_SetLineCoding(USB_Device* pDevice, USB_CDC_LINE_CODING* LineCoding);

uint8_t USB_CDC_GetLineCoding(USB_Device* pDevice, USB_CDC_LINE_CODING* LineCoding);

uint8_t USB_CDC_SetControlLineStatus(USB_Device* pDevice, uint16_t StatMask);

BOOL USB_CDC_InitDevice(USB_Device* pDevice);

BOOL USB_CDC_DeinitDevice(USB_Device* pDevice);

uint8_t USB_CDC_SyncTransfer(USB_Device* pDevice, HCD_TxDir dir, uint8_t* inoutp pBuffer, uint16_t length, uint16_t* outputp txlen);

uint8_t USB_CDC_Transfer(USB_Device* pDevice, HCD_TxDir dir, uint8_t* inoutp pBuffer, uint16_t length,HCD_COMPLETION_CB nullable pCallback, void* nullable pContext);

#ifdef __cplusplus
}
#endif

#endif
