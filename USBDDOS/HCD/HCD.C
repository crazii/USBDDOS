#include <string.h>
#include <assert.h>
#include "USBDDOS/HCD/HCD.H"
#include "USBDDOS/DPMI/DPMI.H"
#include "USBDDOS/USBALLOC.H"
#include "USBDDOS/DBGUTIL.H"

static uint16_t HCD_RootHub_GetPortStatus(struct HCD_HUB* pHub, uint8_t port)
{
    assert(pHub && pHub->pHCI && port < pHub->bNumPorts);
    return pHub->pHCI->pHCDMethod->GetPortStatus(pHub->pHCI, port);
}

static BOOL HCD_RootHub_SetPortStatus(struct HCD_HUB* pHub, uint8_t port, uint16_t status)
{
    assert(pHub && pHub->pHCI && port < pHub->bNumPorts);
    return pHub->pHCI->pHCDMethod->SetPortStatus(pHub->pHCI, port, status);
}

const HCD_HUB HCD_ROOT_HUB_Prototype =
{
    "ROOT HUB",
    NULL,
    NULL,
    0,
    0,
    HCD_RootHub_GetPortStatus,
    HCD_RootHub_SetPortStatus,
};

BOOL HCD_InitController(HCD_Interface* pHCI, uint8_t bus, uint8_t dev, uint8_t func, HCD_Type* type, PCI_DEVICE* pPCIDev)
{
    memset(pHCI, 0, sizeof(HCD_Interface));
    pHCI->PCIAddr.Bus = bus;
    pHCI->PCIAddr.Device = dev;
    pHCI->PCIAddr.Function = func;
    pHCI->PCI = *pPCIDev;
    pHCI->pType = type; //set prematurely
    return (type != NULL && type->dwPI == pPCIDev->Header.PInterface && type->InitController != NULL) ? type->InitController(pHCI, pPCIDev) : FALSE;
}

BOOL HCD_DeinitController(HCD_Interface* pHCI)
{
    BOOL result = HCD_IS_CONTROLLER_VALID(pHCI);
    result = result && pHCI->pType->DeinitController(pHCI);
    if(result)
        memset(pHCI, 0, sizeof(*pHCI));
    return result;
}

BOOL HCD_InitDevice(HCD_HUB* pHub, HCD_Device* pDevice, uint8_t port, uint16_t portStatus)
{
    if(pDevice == NULL || pHub->pHCI->bDevCount >= HCD_MAX_DEVICE_COUNT || port >= pHub->pHCI->bNumPorts || pHub->pHCI->DeviceList[port] != NULL)
        return FALSE;
    memset(pDevice, 0, sizeof(HCD_Device));
    pHub->pHCI->DeviceList[port] = pDevice;
    ++pHub->pHCI->bDevCount;

    pDevice->pHCI = pHub->pHCI;
    pDevice->pHub = pHub;
    pDevice->bHubPort = port;
    pDevice->bSpeed = portStatus&USB_PORT_SPEEDMASK;
    pDevice->pHCData = NULL;
    return pHub->pHCI->pHCDMethod->InitDevice(pDevice);
}

HCD_Device* HCD_FindDevice(HCD_Interface* pHCI, uint8_t address)
{
    for(int i = 0; i < pHCI->bDevCount; ++i)
    {
        if(pHCI->DeviceList[i]->bAddress == address)
            return pHCI->DeviceList[i];
    }
    return NULL;
}

BOOL HCD_RemoveDevice(HCD_Device* pDevice)
{
    if(!HCD_IS_DEVICE_VALID(pDevice))
        return FALSE;
    if(pDevice->pHCI->DeviceList[pDevice->bHubPort] != pDevice || pDevice->pHCI->bDevCount == 0)
    {
        assert(FALSE);
        return FALSE;
    }

    assert(pDevice->pRequest == NULL);

    BOOL result = pDevice->pHub->SetPortStatus(pDevice->pHub, pDevice->bHubPort, USB_PORT_DISABLE);
    assert(result);
    result = result && pDevice->pHCI->pHCDMethod->RemoveDevice(pDevice);
    assert(result);

    if(result)
    {
        pDevice->pHCI->DeviceList[pDevice->bHubPort] = NULL;
        --pDevice->pHCI->bDevCount;
        pDevice->pHCData = NULL;
        pDevice->pHCI = NULL;
    }
    return result;
}

HCD_Request* HCD_AddRequest(HCD_Device* pDevice, void* pEndpoint, HCD_TxDir dir, void* pBuffer, uint16_t size, uint8_t endpoint, HCD_COMPLETION_CB pFnCB, void* pCallbackData)
{
    if(!HCD_IS_DEVICE_VALID(pDevice) || pEndpoint == NULL || pFnCB == NULL)
        return NULL;
    //check buffer overlap. it's logical that transfer buffers won't overlap.
    #if DEBUG
    {
        CLIS();
        volatile HCD_Request* req = pDevice->pRequest;
        while(req != NULL)
        {
            if(pBuffer >= req->pBuffer && (uint8_t*)pBuffer <= (uint8_t*)req->pBuffer + req->size)
                return NULL;
            req = req->pNext;
        }
        STIL();
    }
    #endif
    assert( sizeof(HCD_Request) <= 32);
    HCD_Request* pRequest = (HCD_Request*)USB_TAlloc32(sizeof(HCD_Request));
    if(!pRequest)
        return pRequest;
    pRequest->pDevice = pDevice;
    pRequest->pBuffer = pBuffer;
    pRequest->pEndpoint = pEndpoint;
    pRequest->pFnCB = pFnCB;
    pRequest->pCBData = pCallbackData;
    pRequest->size = size;
    pRequest->dir = dir;
    pRequest->endpoint = endpoint;

    CLIS();
    pRequest->pNext = pDevice->pRequest;
    pDevice->pRequest = pRequest;
    STIL();
    return pRequest;
}

BOOL HCD_InvokeCallBack(HCD_Request* pRequest, uint16_t actuallen, uint8_t ecode) //called in interrupt handlers
{
    //_LOG("HCD Invoke CB: %08lx %08lx %08lx\n", pRequest, pRequest ? pRequest->pDevice : NULL, (pRequest && pRequest->pDevice) ? pRequest->pDevice->pRequest : NULL);
    if(pRequest == NULL || pRequest->pDevice == NULL)
    {
        assert(FALSE);
        return FALSE;
    }
    CLIS();
    HCD_Request* req = pRequest->pDevice->pRequest;
    HCD_Request* prev = NULL;
    while(req != NULL && req != pRequest)
    {
        prev = req;
        req = req->pNext;
    }
    if(!req)
    {
        STIL();
        assert(FALSE);
        return FALSE;
    }
    assert(req == pRequest);
    if(prev)
        prev->pNext = req->pNext;
    else
        pRequest->pDevice->pRequest = req->pNext;
    //_LOG("HCD request remaining: %08lx\n", pRequest->pDevice->pRequest);
    STIL();
    req->transferred = actuallen;
    req->error = ecode;
    req->pFnCB(req);
    //_LOG("Free req: %08lx\n", req);
    USB_TFree32(req);
    return TRUE;
}
