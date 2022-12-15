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

    for(int j = 0; j < bNumInterfaces; ++j)
    {
        USB_InterfaceDesc* pIntfaceDescI = pIntfaceDesc + j;
        if(pIntfaceDescI->bInterfaceClass == USBC_MASSSTORAGE //must be MSC
            && pIntfaceDescI->bInterfaceProtocol == USB_MSC_PROTOCOL_BBB)//currently only support BOT
        {
            pIntfaceDesc = pIntfaceDescI;
            break;
        }
    }
    assert(pIntfaceDesc->bInterfaceClass == USBC_MASSSTORAGE); 
    if(pIntfaceDesc->bInterfaceProtocol != USB_MSC_PROTOCOL_BBB)
        return FALSE;
    uint8_t bInterface = pIntfaceDesc->bInterfaceNumber;

    //get max LUN
    USB_Request Req = { USB_REQ_READ | USB_REQ_TYPE_MSC, USB_REQ_MSC_GET_MAX_LUN, 0, bInterface, 1, };
    uint8_t* pMaxLUN = (uint8_t*)DPMI_DMAMalloc(1, 16);
    *pMaxLUN = 0;
    USB_SyncSendRequest(pDevice, &Req, pMaxLUN); //ignore return. device may not support multiple LUN.
    uint8_t maxLUN = *pMaxLUN;
    DPMI_DMAFree(pMaxLUN);
    pMaxLUN = NULL;
    
    USB_Request Req2 =  {USB_REQ_TYPE_MSC, USB_REQ_MSC_RESET, 0, bInterface, 0};
    if(USB_SyncSendRequest(pDevice, &Req2, NULL) != 0)
        return FALSE;

    USB_MSC_DriverData* pDriverData = (USB_MSC_DriverData*)malloc(sizeof(USB_MSC_DriverData));
    memset(pDriverData, 0, sizeof(USB_MSC_DriverData));
    pDevice->pDriverData = pDriverData;
    pDriverData->bInterface = bInterface;
    pDriverData->MaxLUN = maxLUN;
    for(int i = 0; i < pIntfaceDesc->bNumEndpoints; ++i)
    {
        USB_EndpointDesc* pEndpointDesc = pIntfaceDesc->pEndpoints + i;
        assert(pEndpointDesc->bmAttributesBits.TransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK); //BULK only
        pDriverData->pDataEP[pEndpointDesc->bEndpointAddressBits.Dir] = USB_FindEndpoint(pDevice, pEndpointDesc);
    }

    //perform other readings as sanity check
    {
        USB_MSC_INQUIRY_CMD cmd;
        USB_MSC_INQUIRY_DATA data;
        memset(&cmd, 0, sizeof(cmd));
        cmd.opcode = USB_MSC_SBC_INQUIRY;
        cmd.LUN = 0;
        cmd.AllocationLength = sizeof(data);
        if(!USB_MSC_IssueCommand(pDevice, &cmd, sizeof(cmd), &data, sizeof(data), HCD_TXR))
        {
            free(pDriverData);
            pDevice->pDriverData = NULL;
            return FALSE;
        }
        _LOG("PDT: %x\n", data.PDT);
    }
    {
        USB_MSC_READCAP_CMD cmd;
        USB_MSC_READCAP_DATA data;
        memset(&cmd, 0, sizeof(cmd));
        cmd.opcode = USB_MSC_SBC_READ_CAP;
        for(int i = 0; i <= pDriverData->MaxLUN; ++i)
        {
            cmd.LUN = ((uint8_t)i)&0x7U;
            if(!USB_MSC_IssueCommand(pDevice, &cmd, sizeof(cmd), &data, sizeof(data), HCD_TXR))
            {
                free(pDriverData);
                pDevice->pDriverData = NULL;
                return FALSE;
            }
            uint32_t maxLBA = EndianSwap32(data.LastLBA);
            uint32_t BlockSize = EndianSwap32(data.BlockSize);
            float cap = (float)maxLBA / 1024.0f / 1024.0f / 1024.0f * (float)BlockSize;
            _LOG("LUN %d Capcacity: %.1f GB\n", i, cap);
            pDriverData->SizeGB = (uint16_t)(pDriverData->SizeGB + (uint16_t)cap);
        }
    }
    return TRUE;
}

BOOL USB_MSC_DeinitDevice(USB_Device* pDevice)
{
    USB_MSC_DriverData* pDriverData = (USB_MSC_DriverData*)pDevice->pDriverData;
    free(pDriverData);
    pDevice->pDriverData = NULL;
    return TRUE;
}

BOOL USB_MSC_BulkReset(USB_Device* pDevice)
{
    if(pDevice == NULL || pDevice->pDriverData == NULL)
        return FALSE;
    USB_Request Req =  {USB_REQ_TYPE_MSC, USB_REQ_MSC_RESET, 0, ((USB_MSC_DriverData*)pDevice->pDriverData)->bInterface, 0};
    return USB_SyncSendRequest(pDevice, &Req, NULL) != 0;
}

BOOL USB_MSC_IssueCommand(USB_Device* pDevice, void* inputp cmd, uint32_t CmdSize, void* inoutp data, uint32_t DataSize, HCD_TxDir dir)
{
    if(pDevice == NULL || pDevice->pDriverData == NULL || cmd == NULL || CmdSize > 16 || CmdSize < 1)
        return FALSE;
    USB_MSC_DriverData* pDriverData = (USB_MSC_DriverData*)pDevice->pDriverData;

    //CBW
    USB_MSC_CBW cbw;
    cbw.dCBWSignature = USB_MSC_SIGNATURE;
    cbw.dCBWTag = (uintptr_t)&cbw;
    cbw.dCBWDataTransferLength = DataSize;
    cbw.bmCBWFlags = dir == HCD_TXR ? 0x80U : 0;
    cbw.bCBWLUN = 0;
    cbw.bCBWCBLength = ((uint8_t)CmdSize)&0x1FU;
    memcpy(cbw.CBWCB + 15 + CmdSize - 1, cmd, CmdSize); //copy to signaficant bytes
    void* dma = DPMI_DMAMalloc(sizeof(cbw), 16);
    memcpy(dma, &cbw, sizeof(cbw));
    uint16_t len = 0;
    USB_SyncTransfer(pDevice, pDriverData->pDataEP[0], dma, sizeof(cbw), &len);
    DPMI_DMAFree(dma);
    if(len != sizeof(cbw))
        return FALSE;
    
    //DATA
    if(DataSize && data != NULL)
    {
        dma = DPMI_DMAMalloc(DataSize, 16);
        if(dir == HCD_TXW)
            memcpy(dma, data, DataSize);
        len = 0;
        USB_SyncTransfer(pDevice, pDriverData->pDataEP[dir&0x1], dma, (uint16_t)DataSize, &len);
        if(dir == HCD_TXR)
            memcpy(data, dma, DataSize);
        DPMI_DMAFree(dma);
        if(len != DataSize)
            return FALSE;
    }

    //CSW
    len = 0;
    dma = DPMI_DMAMalloc(sizeof(USB_MSC_CSW), 16);
    USB_SyncTransfer(pDevice, pDriverData->pDataEP[1], dma, sizeof(USB_MSC_CSW), &len);
    USB_MSC_CSW csw = *(USB_MSC_CSW*)dma;
    DPMI_DMAFree(dma);
    if(len != sizeof(USB_MSC_CSW) || csw.dCSWSignature != USB_MSC_SIGNATURE || csw.dCSWTag != cbw.dCBWTag)
        return FALSE;

    if(csw.dCSWDataResidue != 0)
    {
        _LOG("MSC CBW length: %d, CSW DataResidue: %d\n", cbw.dCBWDataTransferLength, csw.dCSWDataResidue);
        return FALSE;   
    }

    if(csw.dCSWStatus != USB_MSC_CSW_STATUS_PASSED)
    {
        _LOG("MSC CSW Status: %d\n", csw.dCSWStatus);
        return FALSE;
    }
    return TRUE;
}

BOOL USB_MSC_DOS_Install()
{

    return FALSE;
}

BOOL USB_MSC_DOS_Uninstall()
{
    return FALSE;
}