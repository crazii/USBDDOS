#include "USBDDOS/CLASS/CDC.H"
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <malloc.h>

static uint8_t USB_CDC_LineCoding(USB_Device* pDevice, USB_CDC_LINE_CODING* LineCoding, BOOL Read)
{
    USB_CDC_DD* pDD = (USB_CDC_DD*)pDevice->pDriverData;

    USB_Request Req =
    {
        (uint8_t)(Read ? USB_REQ_READ | USB_REQ_TYPE_CDC : USB_REQ_TYPE_CDC),
        (uint8_t)(Read ? USB_REQ_CDC_GET_LINE_CODING : USB_REQ_CDC_SET_LINE_CODING),
        0, pDD->bDataInterface, sizeof(USB_CDC_LINE_CODING)
    };

    USB_CDC_LINE_CODING* Data = (USB_CDC_LINE_CODING*)pDevice->pDeviceBuffer;
    assert(Data);

    if(!Read)
        *Data = *LineCoding;
    uint8_t result = USB_SyncSendRequest(pDevice, &Req, Data);
    if(result == 0 && Read)
        *LineCoding = *Data;
    return result;
}

uint8_t USB_CDC_SetLineCoding(USB_Device* pDevice, USB_CDC_LINE_CODING* LineCoding)
{
    return USB_CDC_LineCoding(pDevice, LineCoding, FALSE);
}

uint8_t USB_CDC_GetLineCoding(USB_Device* pDevice, USB_CDC_LINE_CODING* LineCoding)
{
    return USB_CDC_LineCoding(pDevice, LineCoding, TRUE);
}

uint8_t USB_CDC_SetControlLineStatus(USB_Device* pDevice, uint16_t StatMask)
{
    USB_CDC_DD* pDD = (USB_CDC_DD*)pDevice->pDriverData;

    USB_Request Req = 
    {
        USB_REQ_TYPE_CDC, USB_REQ_CDC_SET_LINE_CONTROL_STATE, StatMask, pDD->bInterface, 0,
    };
    return USB_SyncSendRequest(pDevice, &Req, NULL);
}

BOOL USB_CDC_InitDevice(USB_Device* pDevice)
{
    assert(pDevice->bStatus == DS_Configured);

    uint8_t bNumInterfaces = pDevice->pConfigList[pDevice->bCurrentConfig].bNumInterfaces;
    USB_InterfaceDesc* pIntfaceDesc0 = pDevice->pConfigList[pDevice->bCurrentConfig].pInterfaces;

    assert(pIntfaceDesc0->bInterfaceClass == USBC_CDC);  //must be cdc to be here;
    assert(pIntfaceDesc0->bInterfaceSubClass == USBC_CDCSC_DIRECT_LINE_CONTROL_MODEL
    || pIntfaceDesc0->bInterfaceSubClass == USBC_CDCSC_ABSTRACT_CONTROL_MODEL);

    USB_CDC_DD* pDD = (USB_CDC_DD*)malloc(sizeof(USB_CDC_DD));
    memset(pDD, 0, sizeof(USB_CDC_DD));
    pDevice->pDriverData = pDD;
    pDD->bInterface = pIntfaceDesc0->bInterfaceNumber;

    for(int j = 1; j < bNumInterfaces; ++j)
    {
        USB_InterfaceDesc* pIntfaceDesc = pIntfaceDesc0 + j;
        if(pIntfaceDesc->bInterfaceClass == USBC_CDCDATA)
        {
            pDD->bDataInterface = pIntfaceDesc->bInterfaceNumber;

            for(int i = 0; i < pIntfaceDesc->bNumEndpoints; ++i)
            {
                USB_EndpointDesc* pEndpointDesc = pIntfaceDesc->pEndpoints + i;
                assert(pEndpointDesc->bmAttributesBits.TransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK);
                pDD->bDataEP[pEndpointDesc->bEndpointAddressBits.Dir] = USB_FindEndpoint(pDevice, pEndpointDesc);
            }
            break;
        }
    }
    assert(pDD->bDataEP[0] != NULL && pDD->bDataEP[1] != NULL);
    return TRUE;
}

BOOL USB_CDC_DeinitDevice(USB_Device* pDevice)
{
    USB_CDC_DD* pDD = (USB_CDC_DD*)pDevice->pDriverData;
    free(pDD);
    pDevice->pDriverData = NULL;
    return TRUE;
}

uint8_t USB_CDC_SyncTransfer(USB_Device* pDevice, HCD_TxDir dir, uint8_t* inoutp pBuffer, uint16_t length, uint16_t* outputp txlen)
{
    assert(pDevice->bStatus == DS_Ready);
    USB_CDC_DD* pDD = (USB_CDC_DD*)pDevice->pDriverData;
    assert(pDD);
    void* pEndpoint = pDD->bDataEP[dir&0x1];
    assert(pEndpoint);
    return USB_SyncTransfer(pDevice, pEndpoint, pBuffer, length, txlen);
}

uint8_t USB_CDC_Transfer(USB_Device* pDevice, HCD_TxDir dir, uint8_t* inoutp pBuffer, uint16_t length, HCD_COMPLETION_CB nullable pCallback, void* nullable pContext)
{
    assert(pDevice->bStatus == DS_Ready);
    USB_CDC_DD* pDD = (USB_CDC_DD*)pDevice->pDriverData;
    assert(pDD);
    void* pEndpoint = pDD->bDataEP[dir&0x1];
    assert(pEndpoint);
    //return pDevice->HCDDevice.pHCI->pHCDMethod->BulkTransfer(&pDevice->HCDDevice, endpoint, dir, pBuffer, length, txlen, pCallback, pContext) == 0; //need prepare buffer, or user may free pBuffer
    return USB_Transfer(pDevice, pEndpoint, pBuffer, length, pCallback, pContext);
}