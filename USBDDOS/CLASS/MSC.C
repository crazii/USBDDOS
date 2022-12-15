#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <malloc.h>
#include "USBDDOS/CLASS/MSC.H"
#include "USBDDOS/DPMI/DPMI.H"
#include "USBDDOS/DBGUTIL.H"

BOOL USB_MSC_InitDevice(USB_Device* pDevice)
{
    assert(pDevice->bStatus == DS_Configured);

    uint8_t bNumInterfaces = pDevice->pConfigList[pDevice->bCurrentConfig].bNumInterfaces;
    USB_InterfaceDesc* pIntfaceDesc = pDevice->pConfigList[pDevice->bCurrentConfig].pInterfaces;

    assert(pIntfaceDesc->bInterfaceClass == USBC_MASSSTORAGE);  //must be cdc to be here;
    if(pIntfaceDesc->bInterfaceProtocol != USB_MSC_PROTOCOL_BBB) //currently only support BOT
        return FALSE;

    USB_MSC_DriverData* pDriverData = (USB_MSC_DriverData*)malloc(sizeof(USB_MSC_DriverData));
    memset(pDriverData, 0, sizeof(USB_MSC_DriverData));
    pDevice->pDriverData = pDriverData;
    pDriverData->bInterface = pIntfaceDesc->bInterfaceNumber;

    for(int i = 0; i < pIntfaceDesc->bNumEndpoints; ++i)
    {
        USB_EndpointDesc* pEndpointDesc = pIntfaceDesc->pEndpoints + i;
        assert(pEndpointDesc->bmAttributesBits.TransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK); //ISO or BULK(async) by the spec
        pDriverData->pDataEP[pEndpointDesc->bEndpointAddressBits.Dir] = USB_FindEndpoint(pDevice, pEndpointDesc);
    }

    //get max LUN
    USB_Request Req = { USB_REQ_READ | USB_REQ_TYPE_MSC, USB_REQ_MSC_GET_MAX_LUN, 0, pDriverData->bInterface, 1, };
    uint8_t* maxLUN = (uint8_t*)DPMI_DMAMalloc(1, 4);
    *maxLUN = 0;
    USB_SyncSendRequest(pDevice, &Req, maxLUN); //ignore return. device may not support multiple LUN.
    DPMI_DMAFree(maxLUN);
    pDriverData->MaxLUN = *maxLUN;
    maxLUN = NULL;
    
    USB_Request Req2 =  {USB_REQ_TYPE_MSC, USB_REQ_MSC_RESET, 0, pDriverData->bInterface, 0};
    USB_SyncSendRequest(pDevice, &Req, NULL);

    //perform other readings to do sanity check
    {
        USB_MSC_INQUIRY_CMD cmd;
        USB_MSC_INQUIRY_DATA data;
        memset(&cmd, 0, sizeof(cmd));
        cmd.opcode = USB_MSC_SBC_INQUIRY;
        cmd.LUN = 0;
        cmd.AllocationLength = sizeof(data);
        if(!USB_MSC_IssueCommand(&cmd, sizeof(cmd), NULL, &data))
            return FALSE;
        _LOG("PDT: %x\n", data.PDT);
    }
    {
        USB_MSC_READCAP_CMD cmd;
        USB_MSC_READCAP_DATA data;
        memset(&cmd, 0, sizeof(cmd));
        cmd.opcode = USB_MSC_SBC_READ_CAP;
        cmd.LUN = 0;
        if(!USB_MSC_IssueCommand(&cmd, sizeof(cmd), NULL, &data))
            return FALSE;
        uint32_t cap = data.LastLBA / 1024 / 1024 * data.BlockSize;
        _LOG("Capcacity: %d MB\n");
    }

    return TRUE;
}

BOOL USB_MSC_DeinitDevice(USB_Device* pDevice)
{
    return FALSE;
}


BOOL USB_MSC_DOS_Install()
{
    return FALSE;
}

BOOL USB_MSC_IssueCommand(const void* cmd, uint32_t CmdSize, const void* CmdData, void* outputp data)
{
    return FALSE;
}