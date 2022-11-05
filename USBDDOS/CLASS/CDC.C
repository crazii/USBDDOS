#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <malloc.h>
#include "USBDDOS/CLASS/CDC.H"
#include "USBDDOS/DPMI/DPMI.H"
#include "USBDDOS/DBGUTIL.H"

static uint8_t USB_CDC_LineCoding(USB_Device* pDevice, USB_CDC_LINE_CODING* LineCoding, BOOL Read)
{
	USB_CDC_DriverData* pDriverData = (USB_CDC_DriverData*)pDevice->pDriverData;

	USB_InterfaceDesc* pIntfaceDesc0 = pDevice->pConfigList[pDevice->bCurrentConfig].pInterfaces;
	if(pIntfaceDesc0->bInterfaceSubClass != USBC_CDCSC_ABSTRACT_CONTROL_MODEL)
		return 0xFF; //only ACM support it
	if(!(pDriverData->ACMDesc.bmCapabilities&USBCAP_CDC_ACM_LINE_CODING))
		return 0xFF;

    USB_Request Req =
    {
        (uint8_t)(Read ? USB_REQ_READ | USB_REQ_TYPE_CDC : USB_REQ_TYPE_CDC),
        (uint8_t)(Read ? USB_REQ_CDC_GET_LINE_CODING : USB_REQ_CDC_SET_LINE_CODING),
		0, pDriverData->bDataInterface, sizeof(USB_CDC_LINE_CODING)
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
	USB_CDC_DriverData* pDriverData = (USB_CDC_DriverData*)pDevice->pDriverData;

	USB_InterfaceDesc* pIntfaceDesc0 = pDevice->pConfigList[pDevice->bCurrentConfig].pInterfaces;
	if(pIntfaceDesc0->bInterfaceSubClass != USBC_CDCSC_ABSTRACT_CONTROL_MODEL)
		return 0xFF; //only ACM support it
	if(!(pDriverData->ACMDesc.bmCapabilities&USBCAP_CDC_ACM_LINE_CODING))
		return 0xFF;

    USB_Request Req = 
    {
		USB_REQ_TYPE_CDC, USB_REQ_CDC_SET_LINE_CONTROL_STATE, StatMask, pDriverData->bInterface, 0,
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

	USB_CDC_DriverData* pDriverData = (USB_CDC_DriverData*)malloc(sizeof(USB_CDC_DriverData));
	memset(pDriverData, 0, sizeof(USB_CDC_DriverData));
	pDevice->pDriverData = pDriverData;
	pDriverData->bInterface = pIntfaceDesc0->bInterfaceNumber;

    for(int j = 1; j < bNumInterfaces; ++j)
    {
        USB_InterfaceDesc* pIntfaceDesc = pIntfaceDesc0 + j;
        if(pIntfaceDesc->bInterfaceClass == USBC_CDCDATA)
        {
			pDriverData->bDataInterface = pIntfaceDesc->bInterfaceNumber;

			for(int i = 0; i < pIntfaceDesc->bNumEndpoints; ++i)
			{
				USB_EndpointDesc* pEndpointDesc = pIntfaceDesc->pEndpoints + i;
				assert(pEndpointDesc->bmAttributesBits.TransferType == USB_ENDPOINT_TRANSFER_TYPE_ISOC
					|| pEndpointDesc->bmAttributesBits.TransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK); //ISO or BULK(async) by the spec
				pDriverData->bDataEP[pEndpointDesc->bEndpointAddressBits.Dir] = USB_FindEndpoint(pDevice, pEndpointDesc);
			}
			break;
		}
	}
	assert(pDriverData->bDataEP[0] != NULL && pDriverData->bDataEP[1] != NULL);

	uint16_t length = pDevice->pConfigList[pDevice->bCurrentConfig].wTotalLength;
	uint8_t* buffer = (uint8_t*)DPMI_DMAMalloc(length, 4);
	uint8_t result = USB_GetConfigDescriptor(pDevice, buffer, length);
	if(result == 0)
	{
		uint8_t* desc = buffer + pIntfaceDesc0->offset + pIntfaceDesc0->bLength;
		while(*(desc+1) == USB_DT_CSINTERFACE)
		{
			_LOG("CDC functional desc: %x\n", *(desc+2));
			assert(desc + *desc <= buffer + length);
			if(*(desc+2) == USBC_CDCSC_ABSTRACT_CONTROL_MODEL)
			{
				pDriverData->ACMDesc = *(USB_CDC_ACM_Desc*)desc;
				_LOG("CDC ACM: %x %x %x %x\n", pDriverData->ACMDesc.bFunctionLength, pDriverData->ACMDesc.bDescriptorType, pDriverData->ACMDesc.bDescriptorSubtype, pDriverData->ACMDesc.bmCapabilities);
				break;
			}
			desc += *desc;
		}
	}
	DPMI_DMAFree(buffer);
	return result == 0;
}

BOOL USB_CDC_DeinitDevice(USB_Device* pDevice)
{
	USB_CDC_DriverData* pDriverData = (USB_CDC_DriverData*)pDevice->pDriverData;
	free(pDriverData);
    pDevice->pDriverData = NULL;
    return TRUE;
}

uint8_t USB_CDC_SyncTransfer(USB_Device* pDevice, HCD_TxDir dir, uint8_t* inoutp pBuffer, uint16_t length, uint16_t* outputp txlen)
{
    assert(pDevice->bStatus == DS_Ready);
	USB_CDC_DriverData* pDriverData = (USB_CDC_DriverData*)pDevice->pDriverData;
	assert(pDriverData);
	void* pEndpoint = pDriverData->bDataEP[dir&0x1];
    assert(pEndpoint);
    return USB_SyncTransfer(pDevice, pEndpoint, pBuffer, length, txlen);
}

uint8_t USB_CDC_Transfer(USB_Device* pDevice, HCD_TxDir dir, uint8_t* inoutp pBuffer, uint16_t length, HCD_COMPLETION_CB nullable pCallback, void* nullable pContext)
{
    assert(pDevice->bStatus == DS_Ready);
	USB_CDC_DriverData* pDriverData = (USB_CDC_DriverData*)pDevice->pDriverData;
	assert(pDriverData);
	void* pEndpoint = pDriverData->bDataEP[dir&0x1];
	assert(pEndpoint);
	//return pDevice->HCDDevice.pHCI->pHCDMethod->BulkTransfer(&pDevice->HCDDevice, endpoint, dir, pBuffer, length, txlen, pCallback, pContext) == 0; //need prepare buffer, or user may free pBuffer
	return USB_Transfer(pDevice, pEndpoint, pBuffer, length, pCallback, pContext);
}