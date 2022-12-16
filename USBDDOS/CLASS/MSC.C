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
        _LOG("MSC bInterfaceProtocol: %x\n", pIntfaceDescI->bInterfaceProtocol);
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
    _LOG("MSC MaxLUN: %d\n", maxLUN);
    
    USB_Request Req2 =  {USB_REQ_TYPE_MSC, USB_REQ_MSC_RESET, 0, bInterface, 0};
    if(USB_SyncSendRequest(pDevice, &Req2, NULL) != 0)
    {
        _LOG("MSC Error reset bulk.\n");
        return FALSE;
    }
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
        pDriverData->bEPAddr[pEndpointDesc->bEndpointAddressBits.Dir] = pEndpointDesc->bEndpointAddress;
    }
    assert(pDriverData->pDataEP[0] != NULL && pDriverData->pDataEP[1] != NULL);

    USB_ClearHalt(pDevice, pDriverData->bEPAddr[0]);
    USB_ClearHalt(pDevice, pDriverData->bEPAddr[1]);

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
            _LOG("MSC Failed INQUIRY.\n");
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
                _LOG("MSC Failed get capacity.\n");
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
    if(pDevice == NULL || pDevice->pDriverData == NULL)
        return FALSE;
    USB_MSC_DriverData* pDriverData = (USB_MSC_DriverData*)pDevice->pDriverData;
    if(pDriverData)
    {
        free(pDriverData);
        pDevice->pDriverData = NULL;
    }
    return TRUE;
}

BOOL USB_MSC_BulkReset(USB_Device* pDevice)
{
    if(pDevice == NULL || pDevice->pDriverData == NULL)
        return FALSE;
    USB_Request Req =  {USB_REQ_TYPE_MSC, USB_REQ_MSC_RESET, 0, ((USB_MSC_DriverData*)pDevice->pDriverData)->bInterface, 0};
    return USB_SyncSendRequest(pDevice, &Req, NULL) != 0;
}

BOOL USB_MSC_IssueCommand(USB_Device* pDevice, void* inputp cmd, uint32_t CmdSize, void* inoutp nullable data, uint32_t DataSize, HCD_TxDir dir)
{
    if(pDevice == NULL || pDevice->pDriverData == NULL || cmd == NULL || CmdSize > 16 || CmdSize < 1
        || (DataSize != 0 && data == NULL))
        return FALSE;
    USB_MSC_DriverData* pDriverData = (USB_MSC_DriverData*)pDevice->pDriverData;

    //CBW
    USB_MSC_CBW cbw;
    memset(&cbw, 0, sizeof(cbw));
    cbw.dCBWSignature = USB_MSC_CBW_SIGNATURE;
    cbw.dCBWTag = (uintptr_t)&cbw; //use addr as tag
    cbw.dCBWDataTransferLength = DataSize;
    cbw.bmCBWFlags = dir == HCD_TXR ? 0x80U : 0;
    cbw.bCBWLUN = 0;
    cbw.bCBWCBLength = ((uint8_t)CmdSize)&0x1FU;
    memcpy(cbw.CBWCB, cmd, CmdSize);
    uint8_t* dma = (uint8_t*)DPMI_DMAMalloc(sizeof(cbw), 16);
    memcpy(dma, &cbw, sizeof(cbw));
    uint16_t len = 0;
    uint16_t size = sizeof(cbw);
    uint8_t error = USB_SyncTransfer(pDevice, pDriverData->pDataEP[0], dma, size, &len); //TODO: error code now is HC specific, need abstraction (wrapper).
    DPMI_DMAFree(dma);
    if(len != size || error)
    {
        _LOG("MSC CBW failed: %x, %d, %d.\n", error, size, len);
        USB_ClearHalt(pDevice, pDriverData->bEPAddr[0]);
        return FALSE;
    }
    
    //DATA
    if(DataSize)
    {
        dma = (uint8_t*)DPMI_DMAMalloc(DataSize, 16);
        memset(dma, 0, DataSize);
        if(dir == HCD_TXW)
            memcpy(dma, data, DataSize);
        len = 0;
        error = USB_SyncTransfer(pDevice, pDriverData->pDataEP[dir&0x1], dma, (uint16_t)DataSize, &len);
        if(dir == HCD_TXR)
            memcpy(data, dma, DataSize);
        DPMI_DMAFree(dma);
        if(len != DataSize || error)
        {
            _LOG("MSC DATA Failed: %x, %d, %d, %d.\n", error, dir, DataSize, len);
            USB_ClearHalt(pDevice, pDriverData->bEPAddr[dir&0x1]);
            return FALSE;
        }
    }

    //CSW
    len = 0;
    dma = (uint8_t*)DPMI_DMAMalloc(sizeof(USB_MSC_CSW), 16);
    error = USB_SyncTransfer(pDevice, pDriverData->pDataEP[1], dma, sizeof(USB_MSC_CSW), &len);
    USB_MSC_CSW csw = *(USB_MSC_CSW*)dma;
    DPMI_DMAFree(dma);
    if(len != sizeof(USB_MSC_CSW) || csw.dCSWSignature != USB_MSC_CSW_SIGNATURE || csw.dCSWTag != cbw.dCBWTag
    || csw.dCSWDataResidue != 0 || error)
    {
        USB_ClearHalt(pDevice, pDriverData->bEPAddr[1]);
        _LOG("MSC CSW: %x, %x, %x, %x\n", csw.dCSWSignature, csw.dCSWTag, csw.dCSWDataResidue, csw.bCSWStatus);
        _LOG("MSC CSW Failed: %x, %d, %d\n", error, sizeof(USB_MSC_CSW), len);
        _LOG("MSC CBW length: %d\n", cbw.dCBWDataTransferLength);
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