#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <malloc.h>
#include <ctype.h>
#include <conio.h>
#include <dos.h>
#include "USBDDOS/CLASS/HUB.H"
#include "USBDDOS/DPMI/DPMI.H"
#include "USBDDOS/DBGUTIL.H"

static BOOL USB_HUB_SetPortFeature(USB_Device* pDevice, uint8_t port, uint16_t feature)
{
    USB_HUB_DriverData* pDriverData = (USB_HUB_DriverData*)pDevice->pDriverData;
    if(port > pDriverData->desc.bNbrPorts)
        return FALSE;

    USB_Request req = {USB_REQ_WRITE|USB_REQ_TYPE_HUBPORT, USB_REQ_SET_FEATURE, feature, port+1, 0}; //1 based port no
    return USB_SyncSendRequest(pDevice, &req, NULL) == 0;
}

static BOOL USB_HUB_ClearPortFeature(USB_Device* pDevice, uint8_t port, uint16_t feature)
{
    USB_HUB_DriverData* pDriverData = (USB_HUB_DriverData*)pDevice->pDriverData;
    if(port > pDriverData->desc.bNbrPorts)
        return FALSE;
    USB_Request req = {USB_REQ_WRITE|USB_REQ_TYPE_HUBPORT, USB_REQ_CLEAR_FEATURE, feature, port+1, 0}; //1 based port no
    return USB_SyncSendRequest(pDevice, &req, NULL) == 0;
}

//first word: status (PS_*), second word: status changes (PSC_*)
static uint32_t USB_HUB_GetPortStatus(USB_Device* pDevice, uint8_t port)
{
    USB_HUB_DriverData* pDriverData = (USB_HUB_DriverData*)pDevice->pDriverData;
    if(port > pDriverData->desc.bNbrPorts)
        return 0;

    USB_Request req = {USB_REQ_READ|USB_REQ_TYPE_HUBPORT, USB_REQ_GET_STATUS, 0, port+1, 4}; //1 based port no
    uint32_t* status = (uint32_t*)DPMI_DMAMalloc(4, 4);
    uint8_t e = USB_SyncSendRequest(pDevice, &req, status);
    uint32_t sts = *status;
    DPMI_DMAFree(status);
    return e == 0 ? sts : 0;
}

static BOOL HUB_SetPortStatus(HCD_HUB* pHub, uint8_t port, uint16_t status)
{
    if(!pHub || !pHub->pDevice || !HC2USB(pHub->pDevice)->pDriverData)
    {
        assert(FALSE);
        return FALSE;
    }
    uint32_t current = USB_HUB_GetPortStatus(HC2USB(pHub->pDevice), port);
    current = *(uint16_t*)&current; //no shift with endianness
    BOOL result = TRUE;

    if((status&USB_PORT_RESET))
    {
        _LOG("HUB port %d resetting\n", port);
        result = USB_HUB_SetPortFeature(HC2USB(pHub->pDevice), port, PORT_RESET) && result;
        delay(55);
        result = USB_HUB_ClearPortFeature(HC2USB(pHub->pDevice), port, PORT_RESET) && result;
        delay(5);

        //reload states after reset
        current = USB_HUB_GetPortStatus(HC2USB(pHub->pDevice), port);
        current = *(uint16_t*)&current;
        _LOG("HUB port %d reset %x\n", port, current);
    }

    if((status&USB_PORT_ENABLE) && !(current&PS_ENABLE))
    {
        //cannot directly enable ports according to the spec. (usb2.0 11.24.2.7.1.2)
        //it must be RESET to perform 'reset and enable'
        //USBD will issue a port RESET to enable the port
    }

    if((status&USB_PORT_DISABLE) && (current&PS_ENABLE))
    {
        _LOG("HUB ClearPortFeature: enable\n");
        result = USB_HUB_ClearPortFeature(HC2USB(pHub->pDevice), port, PORT_ENABLE) && result;
        delay(5);
        assert(!(USB_HUB_GetPortStatus(HC2USB(pHub->pDevice), port)&PS_ENABLE));
    }

    if((status&USB_PORT_SUSPEND) && !(current&PS_SUSPEND))
        result = USB_HUB_SetPortFeature(HC2USB(pHub->pDevice), port, PORT_SUSPEND) && result;
    else if(!(status&USB_PORT_SUSPEND) && (current&PS_SUSPEND))
        result = USB_HUB_ClearPortFeature(HC2USB(pHub->pDevice), port, PORT_SUSPEND) && result;

    if((status&USB_PORT_CONNECT_CHANGE))
        result = USB_HUB_ClearPortFeature(HC2USB(pHub->pDevice), port, C_PORT_CONNECTION) && result;
    return result;
}

static uint16_t HUB_GetPortStatus(HCD_HUB* pHub, uint8_t port)
{
    if(!pHub || !pHub->pDevice || !HC2USB(pHub->pDevice)->pDriverData)
    {
        assert(FALSE);
        return 0;
    }
    uint32_t status = USB_HUB_GetPortStatus(HC2USB(pHub->pDevice), port);
    uint32_t statuschange = ((uint16_t*)&status)[1];
    status = ((uint16_t*)&status)[0];;
    uint16_t result = 0;

    //by the spec
    if(!(status&PS_LOW_SPEED) && !(status&PS_HIGH_SPEED))
        result |= USB_PORT_Full_Speed_Device;
    else if((status&PS_LOW_SPEED) && !(status&PS_HIGH_SPEED))
        result |= USB_PORT_Low_Speed_Device;
    else if(!(status&PS_LOW_SPEED) && (status&PS_HIGH_SPEED))
        result |= USB_PORT_High_Speed_Device;

    if(status & PS_CONNECTION)
        result |= USB_PORT_ATTACHED;

    if(status & PS_ENABLE)
        result |= USB_PORT_ENABLE;
    else
        result |= USB_PORT_DISABLE;

    if(status & PS_SUSPEND)
        result |= USB_PORT_SUSPEND;
    if(status & PS_RESET) //in reset
        result |= USB_PORT_RESET;
    if(statuschange & PSC_CONNECTION)
        result |= USB_PORT_CONNECT_CHANGE;
    return result;
}

BOOL USB_HUB_InitDevice(USB_Device* pDevice)
{
    _LOG("HUB Endpoint count: %d\n",pDevice->bNumEndpoints);
    if(pDevice->bNumEndpoints != 1) //TODO: verify (usb2.0: 11.23.1)
        return FALSE;

    USB_HubDesc desc;
    USB_Request req = {USB_REQ_READ|USB_REQ_TYPE_HUB, USB_REQ_GET_DESCRIPTOR, USB_DT_HUB<<8, 0, sizeof(desc)};
    void* dma = DPMI_DMAMalloc(sizeof(desc), 4);
    uint8_t err = USB_SyncSendRequest(pDevice, &req, dma);
    memcpy(&desc, dma, sizeof(desc));
    if(err != 0)
    {
        _LOG("HUB Failed get hub descriptor. %x\n",err);
        return FALSE;
    }
    _LOG("HUB Port count: %d.\n", desc.bNbrPorts);

    USB_HUB_DriverData* pData = (USB_HUB_DriverData*)malloc(sizeof(USB_HUB_DriverData));
    memset(pData, 0, sizeof(*pData));
    pData->desc = desc;
    pData->statusEP = pDevice->pEndpointDesc[0]->bEndpointAddress;
    pDevice->pDriverData = pData;

    //powerup & disable all ports: prepare enumeration for USBD
    //powerup
    {for(uint8_t i = 0; i < desc.bNbrPorts; ++i)
        USB_HUB_SetPortFeature(pDevice, i, PORT_POWER);}
    delay(55);

    //only after RESEST the high-speed flags is properly set
    {for(uint8_t i = 0; i < desc.bNbrPorts; ++i)
        USB_HUB_SetPortFeature(pDevice, i, PORT_RESET);}

    delay(55);

    {for(uint8_t i = 0; i < desc.bNbrPorts; ++i)
        USB_HUB_ClearPortFeature(pDevice, i, PORT_RESET);}
    delay(55);

    //disable
    {for(uint8_t i = 0; i < desc.bNbrPorts; ++i)
        USB_HUB_ClearPortFeature(pDevice, i, PORT_ENABLE);}
    _LOG("HUB ports powered & disabled.\n");
    #if 0
    {for(uint8_t i = 0; i < desc.bNbrPorts; ++i)
        _LOG("HUB port %d status: %x\n",i, USB_HUB_GetPortStatus(pDevice,i));}
    #endif
    delay(55);

    //transfer status ? (we don't support PnP yet)
    //USB_Transfer(pDevice, pDevice->pEndpointDesc[0], )

    HCD_HUB hub =
    {
        "HUB", pDevice->HCDDevice.pHCI, &pDevice->HCDDevice, pDevice->HCDDevice.bAddress, desc.bNbrPorts,
        &HUB_GetPortStatus,
        &HUB_SetPortStatus,
    };
    return USB_AddHub(hub);
}

BOOL USB_HUB_DeinitDevice(USB_Device* pDevice)
{
    USB_HUB_DriverData* pData = (USB_HUB_DriverData*)pDevice->pDriverData;
    if(pData)
        free(pData);
    pDevice->pDriverData = NULL;
    //TODO: remove hub from usb?
    return TRUE;
}
