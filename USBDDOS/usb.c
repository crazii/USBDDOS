#include <stdio.h>
#include <stdlib.h>
#include <dos.h>
#include <string.h>
#include <assert.h>
#include "USBDDOS/usb.h"
#include "USBDDOS/DPMI/dpmi.h"
#include "USBDDOS/pic.h"
#include "USBDDOS/usballoc.h"
#include "USBDDOS/dbgutil.h"

#define USB_PCI_SBC_CLASS   0x0C  //PCI serial bus controller
#define USB_SUBCLASS        03    //PCI class ID 0Ch, sub class id 03h

//make idle wait can be interrupted by Ctrl+C
//HLT (halt) supported from 8086. but need privilege.
#if defined(__BC__)

#define USB_IDLE_WAIT() do {\
_ASM_BEGIN \
_ASM(pushf) \
_ASM(sti) \
_ASM(hlt)\
_ASM(popf) \
_ASM_END } while(0)

#elif defined(__WC__)

static void USB_IDLE_WAIT() { 
    __asm {
        pushf;
        sti;
        nop;
        popf;
    }
}

#else

#define USB_IDLE_WAIT() do {\
 int i = STI(); \
 NOP(); \
 if(!i) CLI(); \
 } while(0)

#endif

#define USB_MASK_IRQ_ON_IDLEWAIT 1

typedef struct
{
    volatile BOOL Finished;
    volatile uint16_t Length;   //actual length
    volatile uint8_t ErrorCode; //condition code, no error if == 0
}USB_SyncCallbackResult;

typedef struct
{
    HCD_COMPLETION_CB pCB;
    void* pCBData;
    void* pUserBuffer; //may be invalid
    uint8_t* pSetup8;
}USB_UserCallbackResult;

static USB_ISR_Finalizer USB_ISR_FinalizerHeader;
static USB_ISR_Finalizer* USB_ISR_FinalizerPtr;
static DPMI_ISR_HANDLE USB_ISRHandle[USB_MAX_HC_COUNT];
#define ISR_HANDLE_SHARED(i) USB_ISRHandle[i].user
static BOOL USB_InISR = FALSE;
static uint16_t USB_IRQMask = 0;

static void USB_Shutdown(void);
static BOOL USB_InitController(uint8_t bus, uint8_t dev, uint8_t func, PCI_DEVICE* pPCIDev);
static BOOL USB_InitDevice(HCD_HUB* pHub, uint8_t portIndex, uint16_t portStatus);
static BOOL USB_RemoveDevice(USB_Device* pDevice);
static void USB_EnumerateDevices();
static BOOL USB_ConfigDevice(USB_Device* pDevice, uint8_t address);
static void USB_ConfigEndpoints(USB_Device* pDevice);
static DPMI_ISR_HANDLE* USB_FindISRHandle(uint8_t irq);
static void USB_ISR_Wraper(void);
static __INLINE void USB_ISR(void);
static void USB_Completion_SyncCallback(HCD_Request* pRequest); //calledin hc driver interrupt handler
static void USB_Completion_UserCallback(HCD_Request* pRequest); //calledin hc driver interrupt handler

static int USB_CompareHUB_HC_PI(const void* l, const void* r) //compare PCI programming interface
{
    const HCD_Interface* pHCIL = ((const HCD_HUB*)l)->pHCI;
    const HCD_Interface* pHCIR = ((const HCD_HUB*)r)->pHCI;
    assert(pHCIL && pHCIR);
    return (long)pHCIR->pType->dwPI - (long)pHCIL->pType->dwPI; //large PI comes first
}

void USB_Init(void)
{
    atexit(&USB_Shutdown);
    USB_InitAlloc();

    for(int i = 0; i < USB_MAX_DEVICE_COUNT; ++i)
    {
        if(USBT.InitFunctions[i].PreInit)
            USBT.InitFunctions[i].PreInit();
        else
            break;
    }

    USBT.HC_Count = 0;
    memset(USBT.Devices, 0, sizeof(USBT.Devices));
    CLIS();
    USB_IRQMask = PIC_GetIRQMask();
    STIL();
    _LOG("IRQ mask: %04x\n", USB_IRQMask);

    //enumerate host controllers
    for(uint8_t bus = 0; bus < PCI_MAX_BUS; bus++)
    {
        for(uint8_t dev = 0; dev < PCI_MAX_DEV; dev++)
        {
            for(uint8_t func = 0; func < PCI_MAX_FUNC; ++func)
            {//some controller has multiple functions (i.e. 0 - UHCI,1 - UHCI,2 - UHCI,x - EHCI)
                PCI_DEVICE pcidev;
                PCI_ReadDevice(bus, dev, func, &pcidev);
                if(pcidev.Header.Class == USB_PCI_SBC_CLASS && pcidev.Header.SubClass == USB_SUBCLASS)
                {
                    uint8_t INTPIN = pcidev.Header.DevHeader.Device.INTPIN;
                    uint8_t IRQ = pcidev.Header.DevHeader.Device.IRQ;
                    //note: the PCI BIOS will setup the IRQ and write the IRQ register, the IRQ is setup as level triggered.
                    _LOG("Host Controller: Bus:%d, Dev:%d, Func:%d, pi:0x%02x, IRQ: %d, INT Pin: INT%c#\n", bus, dev, func, pcidev.Header.PInterface, IRQ, 'A'+INTPIN-1);

                    if(IRQ==0xFF) //0xFF: INTPIN not connected to an IRQ
                    {
                        IRQ = PCI_AssignIRQ(bus, dev, func, INTPIN);
                        if(IRQ != 0xFF)
                            PIC_SetLevelTriggered((uint8_t)IRQ, TRUE); //make sure it is level triggerred
                        else
                        {
                            _LOG("Invalid IRQ: %d, skip\n", IRQ);
                            continue;
                        }
                        _LOG("Set IRQ from %d to %d\n", pcidev.Header.DevHeader.Device.IRQ, IRQ);
                        pcidev.Header.DevHeader.Device.IRQ = IRQ;
                    }

                    USB_InitController(bus, dev, func, &pcidev);
                }
            }
        }
    }
    
    //sort by programing interface so that advanced HC comes first
    //so we can detect 2.0 devices first instead of its compation HC.
    //after sorting, USB_ISRHandle won't match the HC_List, but that won't hurt
    
    //note: on detection HC, we can reverse function search order that it iterates from high to low,
    //since EHCI specs require EHCI has higher function num than its companion HC.
    //but on some machine (VirtualBox sometimes) they are on a separated bus
    //so sorting would be a better solution
    //now we enumerate bus through HUB, we need sort on HUBs, not HCs
    qsort(USBT.HUB_List, USBT.HUB_Count, sizeof(HCD_HUB), USB_CompareHUB_HC_PI);

    USB_EnumerateDevices();
    return;
}

void USB_Shutdown(void)
{
    if(DPMI_TSRed)  //_dos_keep will call atexit, we just skip for that
        return;
    //clean up (if app exit normally without TSR)
    //remove device in reverse order (HUBs inited before its downstream devices, need HUB device to operate on ports)
    for(int address = USB_MAX_DEVICE_COUNT-1; address >= 0; --address)
    {
        USB_Device* pDevice = &USBT.Devices[address];
        if(!HCD_IS_DEVICE_VALID(&pDevice->HCDDevice))
            continue;
        while(pDevice->HCDDevice.pRequest != NULL) //waiting on interrupt to handle pending request
            USB_IDLE_WAIT();
        //_LOG("CPUFLAGS: %04x\n", CPU_FLAGS()); //make sure interrupt is on (IF set)
        assert(CPU_FLAGS()&CPU_IFLAG);
        _LOG("Removing device at address: %d\n", address+1);
        USB_RemoveDevice(pDevice);
    }

    CLIS();
    PIC_SetIRQMask(USB_IRQMask);
    for(int j = 0; j < USBT.HC_Count; ++j)
    {
        if(USB_ISRHandle[j].n && !ISR_HANDLE_SHARED(j))
        {
            _LOG("Unstalling ISR, IRQ: %d, Vector: 0x%02x\n", PIC_VEC2IRQ(USB_ISRHandle[j].n), USB_ISRHandle[j].n);
            DPMI_UninstallISR(&USB_ISRHandle[j]);
        }
    }
    STIL();

    for(int k = 0; k < USBT.HC_Count; ++k)
    {
        HCD_Interface* pHCI = &USBT.HC_List[k];
        if(!HCD_IS_CONTROLLER_VALID(pHCI))
            continue;
        for(int i = 0; i < USB_MAX_HC_TYPE; ++i)
        {
            if(HCD_DeinitController(pHCI))
                break;
        }
    }
    USBT.HC_Count = 0;
    memset(USBT.Devices, 0, sizeof(USBT.Devices));
    USB_ShutdownAlloc();

    for(int i = 0; i < USB_MAX_DEVICE_COUNT; ++i)
    {
        if(USBT.InitFunctions[i].PostDeInit)
            USBT.InitFunctions[i].PostDeInit();
        else
            break;
    }
}

BOOL USB_InitController(uint8_t bus, uint8_t dev, uint8_t func, PCI_DEVICE* pPCIDev)
{
    if(USBT.HC_Count >= USB_MAX_HC_COUNT)
        return FALSE;

    BOOL OK = FALSE;
    PCI_HEADER* header = &pPCIDev->Header;
    CLIS();
    for(int i = 0; i < USB_MAX_HC_TYPE; ++i)
    {
        if(USBT.HC_Types[i].InitController != NULL)
        {
            HCD_Interface* pHCI = &USBT.HC_List[USBT.HC_Count];
            uint16_t irqmask = PIC_GetIRQMask();
            PIC_MaskIRQ(header->DevHeader.Device.IRQ); //mask irq until controller init ready
            //_LOG("IRQ Mask change %04x %04x\n", irqmask, PIC_GetIRQMask());
            OK = HCD_InitController(pHCI, bus, dev, func, &USBT.HC_Types[i], pPCIDev);
            if(!OK)
            {
                memset(pHCI, 0, sizeof(*pHCI));
                PIC_SetIRQMask(irqmask);
                continue;
            }

            //add root hub
            HCD_HUB hub = HCD_ROOT_HUB_Prototype;
            hub.pHCI = pHCI;
            hub.bNumPorts = pHCI->bNumPorts;
            if(!USB_AddHub(hub))
            {
                printf("Error: max hub count exceeded %d.", USBT.HUB_Count);
                exit(1);
            }

            uint16_t index = USBT.HC_Count++;
            uint8_t iv = PIC_IRQ2VEC(header->DevHeader.Device.IRQ);
            _LOG("Install ISR 0x%02x\n", iv);
            DPMI_ISR_HANDLE* handle = USB_FindISRHandle(header->DevHeader.Device.IRQ);
            if(handle != NULL)
            {
                USB_ISRHandle[index] = *handle;
                ISR_HANDLE_SHARED(index) = 1;
            }
            else if(DPMI_InstallISR(iv, USB_ISR_Wraper, &USB_ISRHandle[index]) != 0)
            {
                PIC_SetIRQMask(irqmask);
                printf("Error: Install ISR failed.\n");
                STIL();
                exit(-1);
            }
            PIC_UnmaskIRQ(header->DevHeader.Device.IRQ);
            //_LOG("altered IRQ mask: %04x\n", PIC_GetIRQMask());
            break;
        }
    }
    STIL();
    return OK;
}

BOOL USB_InitDevice(HCD_HUB* pHub, uint8_t portIndex, uint16_t portStatus)
{
    //early return
    if(pHub->pHCI->bDevCount >= USB_MAX_HC_COUNT)
        return FALSE;
    if(USBT.DeviceCount >= USB_MAX_DEVICE_COUNT)
        return FALSE;

    USB_Device* pDevice = NULL;
    uint8_t address;
    for(address = 0; address < USB_MAX_DEVICE_COUNT; ++address)
    {
        if(!HCD_IS_DEVICE_VALID(&USBT.Devices[address].HCDDevice))
            break;
    }
    pDevice = &USBT.Devices[address];
    address = (uint8_t)(address + 1); //1 based
    memset(pDevice, 0, sizeof(USB_Device));
    //_LOG("HCD_InitDevice\n");
    if( !HCD_InitDevice(pHub, &pDevice->HCDDevice, portIndex, portStatus) )
    {
        _LOG("USB HCD_InitDevice Failed.\n");
        return FALSE;
    }
    pDevice->bStatus = DS_Default;
    pDevice->pDeviceBuffer = (uint8_t*)DPMI_DMAMalloc(USB_DEVBUFFER_SIZE, 4);
    pDevice->pSetup = (uint8_t*)DPMI_DMAMalloc(8,4);

    //predetermine packet size
    if (pDevice->HCDDevice.bSpeed == USB_PORT_Low_Speed_Device)
        pDevice->Desc.bMaxPacketSize = 8;
    else if (pDevice->HCDDevice.bSpeed == USB_PORT_High_Speed_Device)
        pDevice->Desc.bMaxPacketSize = 64;
    else if(pDevice->HCDDevice.bSpeed ==  USB_PORT_Full_Speed_Device)
        pDevice->Desc.bMaxPacketSize = 8; // 8~64
    else
        assert(FALSE);

    //_LOG("bSpeed: %d\n", pDevice->HCDDevice.bSpeed);

    int trycount = 0;
    while(trycount++ <= 3)
    {
        if(USB_ConfigDevice(pDevice, address))
        {
            assert(pDevice->bStatus == DS_Configured);
            break;
        }
        //pHub->SetPortStatus(pHub, portIndex, USB_PORT_DISABLE); //disable it to continue enumeration on next port - RemoveDevice will do it
    }
    // install device driver
    BOOL NoDriver = FALSE;
    if(pDevice->bStatus == DS_Configured)
    {
        uint8_t bClass = pDevice->Desc.bDeviceClass;
        #if DEBUG
        USB_ShowDeviceInfo(pDevice);
        #else
        _LOG("USB: Device class:%x, installing drivers\n", bClass);
        #endif
        //early skip configuring endpoints if we don't have driver for the class
        if(bClass < USBC_MAX_DRIVER && USBT.ClassDrivers[bClass].InitDevice != NULL
        /*|| TODO: check vendor specific driver*/)
        {
            _LOG("USB: config endpoints\n");
            USB_ConfigEndpoints(pDevice);

            BOOL DriverInstalled = FALSE;
            if(bClass < USBC_MAX_DRIVER && USBT.ClassDrivers[bClass].InitDevice != NULL)
                DriverInstalled = USBT.ClassDrivers[bClass].InitDevice(pDevice);

            if(!DriverInstalled)
                //try vendor specific driver
                //not implemented
                {}
            pDevice->bStatus = DriverInstalled ? DS_Ready : pDevice->bStatus;

            if(DriverInstalled)
                return TRUE;
        }
        else
            NoDriver = TRUE;
    }

    if(NoDriver)
        printf("No driver found at port: %d for device (class %02x), skip\n", portIndex, pDevice->Desc.bDeviceClass);
    else
        printf("Error initializing device at port: %d, address: %d.\n", portIndex, address);
    USB_RemoveDevice(pDevice); //this will disable the hub port
    return FALSE;
}

BOOL USB_RemoveDevice(USB_Device* pDevice)
{
    if(!HCD_IS_DEVICE_VALID(&pDevice->HCDDevice))
        return FALSE;
    assert(pDevice->HCDDevice.pRequest == NULL);

    uint8_t bClass = pDevice->Desc.bDeviceClass;
    if(bClass < USBC_MAX_DRIVER && USBT.ClassDrivers[bClass].InitDevice != NULL)
        USBT.ClassDrivers[bClass].DeinitDevice(pDevice);

    for(int e = 0; e < pDevice->bNumEndpoints; ++e)
        pDevice->HCDDevice.pHCI->pHCDMethod->RemoveEndPoint(&pDevice->HCDDevice, pDevice->pEndpoints[e]);
    pDevice->HCDDevice.pHCI->pHCDMethod->RemoveEndPoint(&pDevice->HCDDevice, pDevice->pDefaultControlEP); //HCD implementation probably do nothing but better do the call.
    _LOG("USB Removing HCD device\n");
    HCD_RemoveDevice(&pDevice->HCDDevice);

    _LOG("USB Free device buffers\n");
    if(pDevice->pDeviceBuffer)
        DPMI_DMAFree(pDevice->pDeviceBuffer);
    if(pDevice->pSetup)
        DPMI_DMAFree(pDevice->pSetup);

    _LOG("USB Free device configs\n");
    if(pDevice->pConfigList != NULL)
    {
        for(int i = 0; i < pDevice->Desc.bNumConfigurations; ++i)
        {
            for(int j = 0; j < pDevice->pConfigList[i].bNumInterfaces; ++j)
            {
                free(pDevice->pConfigList[i].pInterfaces[j].pEndpoints);
            }
            free(pDevice->pConfigList[i].pInterfaces);
        }
        free(pDevice->pConfigList);
    }
    _LOG("USB Free endpoints\n");
    free(pDevice->pEndpoints);
    free(pDevice->pEndpointDesc);
    memset(pDevice, 0, sizeof(USB_Device));
    return TRUE;
}

uint8_t USB_SendRequest(USB_Device* pDevice, USB_Request* pRequest, void* pBuffer, HCD_COMPLETION_CB pFnCallback, void* pCallbackData)
{
    if(pDevice == NULL || !HCD_IS_DEVICE_VALID(&pDevice->HCDDevice) 
        || !HCD_IS_CONTROLLER_VALID(pDevice->HCDDevice.pHCI))
        return FALSE;
    HCD_TxDir dir = (pRequest->bmRequestType&USB_REQ_READ) ? HCD_TXR : HCD_TXW;
    HCD_CONTROL_FUNCTION pFn = pDevice->HCDDevice.pHCI->pHCDMethod->ControlTransfer;
    memcpy(pDevice->pSetup, pRequest, 8);
    //_LOG("Request data length: %d\n", pRequest->wLength);
    
    if(pFnCallback == USB_Completion_SyncCallback) //sync call, no need to allocate buffer. assume pBuffer can be mapped to physical memory and accessible by HC
        return pFn(&pDevice->HCDDevice, pDevice->pDefaultControlEP, dir, pDevice->pSetup, pBuffer, pRequest->wLength, pFnCallback, pCallbackData);

    uint8_t* dma = (uint8_t*)USB_TAlloc(pRequest->wLength, 4);
    memcpy(dma, pBuffer, pRequest->wLength);
    uint8_t* request = (uint8_t*)USB_TAlloc(8, 4);  //allocate extra setup for non sync mode.
    memcpy(request, pRequest, 8);

    USB_UserCallbackResult* pUserData = (USB_UserCallbackResult*)USB_TAlloc32(sizeof(USB_UserCallbackResult));
    pUserData->pCB = pFnCallback;
    pUserData->pCBData = pCallbackData;
    pUserData->pUserBuffer = pBuffer;
    pUserData->pSetup8 = request;
    return pFn(&pDevice->HCDDevice, pDevice->pDefaultControlEP, dir, request, dma, pRequest->wLength, USB_Completion_UserCallback, pUserData);
}

uint8_t USB_SyncSendRequest(USB_Device* pDevice, USB_Request* pRequest, void* pBuffer)
{
    USB_SyncCallbackResult result;
    result.Finished = FALSE;
    uint8_t error = USB_SendRequest(pDevice, pRequest, pBuffer, USB_Completion_SyncCallback, &result);
    if(error != 0)
    {
        _LOG("USB_SendRequest error: %d\n", error);
        return error;
    }

#if USB_MASK_IRQ_ON_IDLEWAIT
    CLIS();
    uint16_t mask = PIC_GetIRQMask();
    PIC_SetIRQMask(PIC_IRQ_UNMASK(0xFFFF,pDevice->HCDDevice.pHCI->PCI.Header.DevHeader.Device.IRQ)); //only enable current controller IRQ
    STIL();
#endif

    while(!result.Finished)
        USB_IDLE_WAIT();

#if USB_MASK_IRQ_ON_IDLEWAIT
    {
        CLIS();
        PIC_SetIRQMask(mask);
        STIL();
    }
#endif
    return result.ErrorCode;
}

uint8_t USB_Transfer(USB_Device* pDevice, void* pEndpoint, uint8_t* pBuffer, uint16_t length, HCD_COMPLETION_CB pFnCallback, void* pCallbackData)
{
    if(pDevice == NULL || !HCD_IS_DEVICE_VALID(&pDevice->HCDDevice) 
        || !HCD_IS_CONTROLLER_VALID(pDevice->HCDDevice.pHCI) || pEndpoint == NULL || pBuffer == NULL || length == 0)
    {
        assert(FALSE);
        return 0xFFU;
    }
    USB_EndpointDesc* pDesc = USB_GetEndpointDesc(pDevice, pEndpoint);
    if(pDesc == NULL)
    {
        assert(FALSE);
        return 0xFFU;
    }
    HCD_TxDir dir = pDesc->bEndpointAddressBits.Dir ? HCD_TXR : HCD_TXW;
    if(pDesc->bmAttributesBits.TransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL /*|| pDesc->bmAttributesBits.TransferType > USB_ENDPOINT_TRANSFER_TYPE_INTR*/)
    {
        assert(FALSE);
        return 0xFFU;
    }
    
    HCD_TRANSFER_FUNCTION pFn = (&pDevice->HCDDevice.pHCI->pHCDMethod->IsochronousTransfer)[pDesc->bmAttributesBits.TransferType - USB_ENDPOINT_TRANSFER_TYPE_ISOC];

    if(pFnCallback == USB_Completion_SyncCallback) //sync call, no need to allocate buffer. assume pBuffer can be mapped to physical memory and accessible by HC
        return pFn(&pDevice->HCDDevice, pEndpoint, dir, pBuffer, length, pFnCallback, pCallbackData);

    uint8_t* dma = (uint8_t*)USB_TAlloc(length, 4);
    memcpy(dma, pBuffer, length);

    USB_UserCallbackResult* pUserData = (USB_UserCallbackResult*)USB_TAlloc32(sizeof(USB_UserCallbackResult));
    pUserData->pCB = pFnCallback;
    pUserData->pCBData = pCallbackData;
    pUserData->pUserBuffer = pBuffer;
    pUserData->pSetup8 = NULL;
    return pFn(&pDevice->HCDDevice, pEndpoint, dir, dma, length, USB_Completion_UserCallback, pUserData);
}

uint8_t USB_SyncTransfer(USB_Device* pDevice, void* pEndpoint, uint8_t* pBuffer, uint16_t length, uint16_t* txlen)
{
    USB_SyncCallbackResult result;
    result.Finished = FALSE;

    uint8_t error = USB_Transfer(pDevice, pEndpoint, pBuffer, length, USB_Completion_SyncCallback, &result);
    if(error != 0)
    {
        assert(FALSE);
        return error;
    }

#if USB_MASK_IRQ_ON_IDLEWAIT
    CLIS();
    uint16_t mask = PIC_GetIRQMask();
    PIC_SetIRQMask(PIC_IRQ_UNMASK(0xFFFF,pDevice->HCDDevice.pHCI->PCI.Header.DevHeader.Device.IRQ)); //only enable current controlelr IRQ
    STIL();
#endif

    //idle wait for interrupt (HW notifying finish event and hcd driver call USB_Completion_Callback)
    while(!result.Finished)
        USB_IDLE_WAIT();

#if USB_MASK_IRQ_ON_IDLEWAIT
    {
        CLIS();
        PIC_SetIRQMask(mask);
        STIL();
    }
#endif
    *txlen = result.Length;
    return result.ErrorCode;
}

void* USB_FindEndpoint(USB_Device* pDevice, USB_EndpointDesc* pDesc)
{
    for(int i = 0; i < pDevice->bNumEndpoints; ++i)
    {
        if(pDesc == pDevice->pEndpointDesc[i])
            return pDevice->pEndpoints[i];
    }
    assert(FALSE);
    return NULL;
}

USB_EndpointDesc* USB_GetEndpointDesc(USB_Device* pDevice, void* pEndpoint)
{
    for(int i = 0; i < pDevice->bNumEndpoints; ++i)
    {
        if(pEndpoint == pDevice->pEndpoints[i])
            return pDevice->pEndpointDesc[i];
    }
    assert(FALSE);
    return NULL;
}

uint8_t USB_GetConfigDescriptor(USB_Device* pDevice, uint8_t* buffer, uint16_t length)
{
    if(!HCD_IS_DEVICE_VALID(&pDevice->HCDDevice) || buffer == NULL || length < 9)
        return 0xFF;

    _LOG("get config desc length: %d\n", length);
    #if 0
    USB_Request Request1 = {USB_REQ_READ, USB_REQ_GET_DESCRIPTOR, USB_DT_CONFIGURATION << 8, 0, 9}; //get TotalLength first
    uint8_t result = USB_SyncSendRequest(pDevice, &Request1, buffer);
    assert(result == 0);
    uint16_t TotalLength = ((USB_ConfigDesc*)buffer)->wTotalLength;
    assert(length >= TotalLength);
    #endif

    USB_Request Request = {USB_REQ_READ, USB_REQ_GET_DESCRIPTOR, USB_DT_CONFIGURATION << 8, 0, 0};
    Request.wLength = min(pDevice->pConfigList[pDevice->bCurrentConfig].wTotalLength, length);
    uint8_t result = USB_SyncSendRequest(pDevice, &Request, buffer);
    assert(result == 0);
    return result;
}

BOOL USB_SetConfiguration(USB_Device* pDevice, uint8_t configuration)
{
    if(pDevice->bStatus < DS_Configured)   //spec requires configured. except configuration==0
        return FALSE;
    assert(pDevice->pConfigList != NULL);

    uint8_t bNumConfigurations = pDevice->Desc.bNumConfigurations;
    if(configuration >= bNumConfigurations)
        return FALSE;

    if(configuration == pDevice->bCurrentConfig)
        return TRUE;

    USB_Request request = {USB_REQ_WRITE|USB_REQTYPE_STANDARD, USB_REQ_SET_CONFIGURATION, 0, 0, 0};
    request.wValue = configuration;
    BOOL result = USB_SyncSendRequest(pDevice, &request, NULL);
    assert(result);
    if(!result)
        return FALSE;

    pDevice->bCurrentConfig = configuration;
    //reconfig ep
    if(pDevice->bStatus == DS_Ready)
    {
        USB_ConfigEndpoints(pDevice);
    }
    return TRUE;
}

BOOL USB_ParseConfiguration(uint8_t* pBuffer, uint16_t length, USB_Device* pDevice)
{
    assert(((USB_ConfigDesc*)pBuffer)->wTotalLength == length);
    pDevice->pConfigList = (USB_ConfigDesc*)malloc(sizeof(USB_ConfigDesc)*pDevice->Desc.bNumConfigurations);
    assert(pDevice->pConfigList);
    memset(pDevice->pConfigList, 0, sizeof(USB_ConfigDesc)*pDevice->Desc.bNumConfigurations);
    pDevice->bCurrentConfig = 0;

    int ConfigIndex = -1;
    int InterfaceIndex = -1;
    int EndpointIndex = -1;
    uint16_t i = 0;
    while(i < length)
    {
        uint8_t len = *(pBuffer + i);
        uint8_t descType = *(pBuffer + (i+1));
        if(descType == USB_DT_CONFIGURATION)
        {
            ++ConfigIndex;
            assert(ConfigIndex < pDevice->Desc.bNumConfigurations);
            //reset sub indices
            InterfaceIndex = -1;
            EndpointIndex = -1;

            USB_ConfigDesc* pConfigDesc = (USB_ConfigDesc*)(pBuffer + i);
            pDevice->pConfigList[ConfigIndex] = *pConfigDesc;
            USB_InterfaceDesc* pInterfaceDesc = (USB_InterfaceDesc*)malloc(sizeof(USB_InterfaceDesc)*pConfigDesc->bNumInterfaces);
            assert(pInterfaceDesc);
            memset(pInterfaceDesc, 0, sizeof(USB_InterfaceDesc)*pConfigDesc->bNumInterfaces);
            pDevice->pConfigList[ConfigIndex].pInterfaces = pInterfaceDesc;
        }
        else if(descType == USB_DT_INTERFACE)
        {
            ++InterfaceIndex;
            EndpointIndex = -1;
            //assert(InterfaceIndex < pDevice->pConfigList[ConfigIndex].bNumInterfaces); //why would it happen? - vendor specific descriptors, ignore
            if(InterfaceIndex >= pDevice->pConfigList[ConfigIndex].bNumInterfaces)
            {
                assert(ConfigIndex == pDevice->Desc.bNumConfigurations-1); //should be the last config
                _LOG("USB skip vendor specific config descriptors.\n");
                break;
            }

            USB_InterfaceDesc *pInterfaceDesc = (USB_InterfaceDesc*)(pBuffer + i);
            pDevice->pConfigList[ConfigIndex].pInterfaces[InterfaceIndex] = *(USB_InterfaceDesc*)(pBuffer + i);
            if (pDevice->Desc.bDeviceClass == 0)
            {
                pDevice->Desc.bDeviceClass = pInterfaceDesc->bInterfaceClass;
                pDevice->Desc.bDeviceSubClass = pInterfaceDesc->bInterfaceSubClass;
                pDevice->Desc.bDeviceProtocol = pInterfaceDesc->bInterfaceProtocol;
            }

            uint8_t EndPointNum = pInterfaceDesc->bNumEndpoints;
            USB_EndpointDesc* pEndPointDesc = (USB_EndpointDesc*)malloc(sizeof(USB_EndpointDesc)*EndPointNum);
            _LOG("interface %d endpoint num:%d\n",InterfaceIndex, EndPointNum);
            assert(EndPointNum == 0 || pEndPointDesc);
            memset(pEndPointDesc, 0, sizeof(USB_EndpointDesc)*EndPointNum);
            pDevice->pConfigList[ConfigIndex].pInterfaces[InterfaceIndex].pEndpoints = pEndPointDesc;
            pDevice->pConfigList[ConfigIndex].pInterfaces[InterfaceIndex].offset = i;
        }
        else if(descType == USB_DT_ENDPOINT)
        {
            ++EndpointIndex;
            assert(EndpointIndex < pDevice->pConfigList[ConfigIndex].pInterfaces[InterfaceIndex].bNumEndpoints);
            pDevice->pConfigList[ConfigIndex].pInterfaces[InterfaceIndex].pEndpoints[EndpointIndex] = *(USB_EndpointDesc*)(pBuffer+i);
        }
        i = (uint16_t)(i + len);
    }
    return TRUE;
}

BOOL USB_GetDescriptorString(USB_Device* pDevice, uint8_t bID, char* pBuffer, uint16_t length)
{
    if(!HCD_IS_DEVICE_VALID(&pDevice->HCDDevice) || pBuffer == NULL || length == 0)
        return FALSE;
    uint8_t* Buffer = pDevice->pDeviceBuffer;
    memset(Buffer, 0, USB_DEVBUFFER_SIZE);
    assert((void*)Buffer != (void*)pBuffer); //need a separate string buffer
    USB_Request Request = {USB_REQ_READ|USB_REQTYPE_STANDARD, USB_REQ_GET_DESCRIPTOR, 0, USB_LANG_ID_ENG, 1U}; //get length first
    Request.wValue = (uint16_t)((USB_DT_STRING << 8) + bID);
#if 0
    if (USB_SyncSendRequest(pDevice, &Request, Buffer) == 0)
    {
        Request.wLength = min(USB_DEVBUFFER_SIZE, Buffer[0]*2-2); //it's confusing that spec says Buffer[0] is descriptor in bytes but the unicode length is Buffer[0]-2
#else
    {
        Request.wLength = USB_DEVBUFFER_SIZE;
#endif
        if (USB_SyncSendRequest(pDevice, &Request, Buffer) == 0)
        {
            int count = min(length-1, Buffer[0]-2);
            wcstombs(pBuffer, (wchar_t*)(Buffer+2), (size_t)count);
            pBuffer[count] = '\0';
            count = (int)strlen(pBuffer);
            while(--count > 0 && pBuffer[count] == ' ') pBuffer[count] = '\0'; //trim
            return TRUE;
        }
    }
    return FALSE;
}

#if DEBUG
void USB_ShowDeviceInfo(USB_Device* pDevice)
{
    printf("%s,  %s,  %s\n", pDevice->sManufacture, pDevice->sProduct, pDevice->sSerialNumber);
    printf("bDeviceClass: %x, bDeviceSubClass: %x, bDeviceProtocol: %x\n", pDevice->Desc.bDeviceClass, pDevice->Desc.bDeviceSubClass, pDevice->Desc.bDeviceProtocol);
    printf("widVendor: %04X, widProduct: %04X, wbcdUSB: %04X\n", pDevice->Desc.widVendor, pDevice->Desc.widProduct, pDevice->Desc.wbcdUSB);
    printf("bMaxPacketSize: %d, bSpeed: %x\n", pDevice->Desc.bMaxPacketSize, pDevice->HCDDevice.bSpeed);
    printf("bHubPort: %x, bAddress: %x\n", pDevice->HCDDevice.bHubPort, pDevice->HCDDevice.bAddress);

    printf("bNumConfigurations: %d\n", pDevice->Desc.bNumConfigurations);
    for(int i = 0; i < pDevice->Desc.bNumConfigurations; ++i)
    {
        printf("Config %d: Interface Count %d\n", i, pDevice->pConfigList[i].bNumInterfaces);
        for(int j = 0; j < pDevice->pConfigList[i].bNumInterfaces; ++j)
        {
            printf("    Interface %d: Class %d, Subclass: %d, Endpoint Count: %d\n", j,
                pDevice->pConfigList[i].pInterfaces[j].bInterfaceClass,
                pDevice->pConfigList[i].pInterfaces[j].bInterfaceSubClass,
                pDevice->pConfigList[i].pInterfaces[j].bNumEndpoints);
            for(int k = 0; k < pDevice->pConfigList[i].pInterfaces[j].bNumEndpoints; ++k)
            {
                USB_EndpointDesc epd = pDevice->pConfigList[i].pInterfaces[j].pEndpoints[k];
                printf("        Endpoint %d: Address: %02xh, Type: %d, MaxPacketSize: %d, Interval: %d\n", k,
                    epd.bEndpointAddress, epd.bmAttributesBits.TransferType, epd.wMaxPacketSizeFlags.Size, epd.bInterval);
            }
        }
    }

    return;
}
#endif

BOOL USB_ClearHalt(USB_Device* pDevice, uint8_t epAddr)
{
    USB_Request req = {USB_REQ_WRITE|USB_REQTYPE_STANDARD|USB_REQREC_ENDPOINT, USB_REQ_CLEAR_FEATURE, ENDPOINT_HALT, 0, 0};
    req.wIndex = epAddr;
    return USB_SyncSendRequest(pDevice, &req, NULL) == 0; //TODO: reset data toggle on ep
}

void USB_IdleWait()
{
    USB_IDLE_WAIT();
}

BOOL USB_AddHub(HCD_HUB hub)
{
    if(USBT.HUB_Count < USB_MAX_HUB_COUNT)
    {
        USBT.HUB_List[USBT.HUB_Count++] = hub;
        return TRUE;
    }
    else
    {
        _LOG("Error: max hub count exceeded %d.", USB_MAX_DEVICE_COUNT);
        return FALSE;
    }
}

void USB_ISR_AddFinalizer(USB_ISR_Finalizer* finalizer)
{
    finalizer->next = NULL;
    USB_ISR_FinalizerPtr->next = finalizer;
    USB_ISR_FinalizerPtr = finalizer;
}

static void USB_EnumerateDevices()
{
    //initially all powered devices will be in default states (address 0), but ports disabled.
    //enable one port each time, and it will respond via device address (FA) 0, default pipe - endpoint 0.
    //if device inited, assign address to it (the address is used for further communication), and continue to next device.
    //otherwise disable the port (only one device can be enabled with address 0) and continue to next
    {for(int j = 0; j < USBT.HUB_Count; ++j) //disable all ports if they're previously enabled by default/by BIOS. only 1 can be enabled during enumearation.
    {
        HCD_HUB* pHub = &USBT.HUB_List[j];
        for(uint8_t i = 0; i < pHub->bNumPorts; ++i)
        {
            uint16_t status = pHub->GetPortStatus(pHub, i);

            if((status&USB_PORT_ATTACHED))
                pHub->SetPortStatus(pHub, i, USB_PORT_DISABLE);
        }
    }}

    for(int j = 0; j < USBT.HUB_Count; ++j)
    {
        HCD_HUB* pHub = &USBT.HUB_List[j];
        HCD_Interface* pHCI = pHub->pHCI;
        _LOG("Enumerate device for %s %s on Bus:Dev:Func %02d:%02d:%02d\n", pHCI->pType->name, pHub->name, pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, pHCI->PCIAddr.Function); unused(pHCI);

        for(uint8_t i = 0; i < pHub->bNumPorts; ++i)
        {
            uint16_t status = pHub->GetPortStatus(pHub, i);
            
            if(status&USB_PORT_ATTACHED)
            {
                //stablize
                pHub->SetPortStatus(pHub, i, USB_PORT_CONNECT_CHANGE); //clear state change
                delay(100);
                BOOL stablized = !(pHub->GetPortStatus(pHub, i)&USB_PORT_CONNECT_CHANGE);
                if(!stablized)
                    continue;
                delay(10);

                _LOG("Enumerate device at port %d.\n",i);
                USB_InitDevice(pHub, i, status);
            }
        }    
    }
}

static BOOL USB_ConfigDevice(USB_Device* pDevice, uint8_t address)
{
    //https://techcommunity.microsoft.com/t5/microsoft-usb-blog/how-does-usb-stack-enumerate-a-device/ba-p/270685

    HCD_Interface* pHCI = pDevice->HCDDevice.pHCI;
    HCD_HUB* pHub = pDevice->HCDDevice.pHub;
    // 1st reset
    BOOL reset = pHub->SetPortStatus(pHub, pDevice->HCDDevice.bHubPort, USB_PORT_RESET|USB_PORT_ENABLE);
    if(!reset || !(pHub->GetPortStatus(pHub, pDevice->HCDDevice.bHubPort)&USB_PORT_ENABLE)) //port enable failed: device detached, or 1.x device attached directly to EHCI root ports and EHCI has no companion HC to hand off
        return FALSE;

    uint8_t* Buffer = pDevice->pDeviceBuffer;
    memset(Buffer, 0, USB_DEVBUFFER_SIZE);

    // get device descriptor
    _LOG("USB: initial descriptor\n");
    pDevice->pDefaultControlEP = pHCI->pHCDMethod->CreateEndpoint(&pDevice->HCDDevice, 0, HCD_TXW, USB_ENDPOINT_TRANSFER_TYPE_CTRL, pDevice->Desc.bMaxPacketSize, 0);
    USB_Request Request1 = {USB_REQ_READ | USB_REQTYPE_STANDARD, USB_REQ_GET_DESCRIPTOR, USB_DT_DEVICE << 8, 0, 64};
    uint8_t result = USB_SyncSendRequest(pDevice, &Request1, Buffer);
    //first try may fail because incorrect max packet size
    USB_DeviceDesc* pDesc = (USB_DeviceDesc*)Buffer;
    _LOG("USB: ep0 max packet size: %d->%d\n", pDevice->Desc.bMaxPacketSize, pDesc->bMaxPacketSize);
    if(pDesc->bMaxPacketSize && pDesc->bMaxPacketSize != pDevice->Desc.bMaxPacketSize)
    {
        pDevice->Desc.bMaxPacketSize = pDesc->bMaxPacketSize;
        pDevice->HCDDevice.pHCI->pHCDMethod->RemoveEndPoint(&pDevice->HCDDevice, pDevice->pDefaultControlEP);
        pDevice->pDefaultControlEP = pHCI->pHCDMethod->CreateEndpoint(&pDevice->HCDDevice, 0, HCD_TXW, USB_ENDPOINT_TRANSFER_TYPE_CTRL, pDevice->Desc.bMaxPacketSize, 0);//update EP0 maxpacket
    }

    // 2nd reset
    reset = pHub->SetPortStatus(pHub, pDevice->HCDDevice.bHubPort, USB_PORT_RESET|USB_PORT_ENABLE);
    if(!reset)
        return FALSE;

    // set device address.
    // before set address, nothing can be done except get desc.
    // the device will be adressed state
    _LOG("USB: set device address: %d\n", address);
    assert(pDevice->HCDDevice.bAddress == 0);
    USB_Request Request2 = {USB_REQ_WRITE|USB_REQTYPE_STANDARD, USB_REQ_SET_ADDRESS, 0, 0, 0};
    Request2.wValue = address;
    result = USB_SyncSendRequest(pDevice, &Request2, NULL);
    delay(2); // spec required.
    if(result != 0)
    {
        assert(FALSE);
        return FALSE;
    }
    pDevice->bStatus = DS_Addressed;
    pDevice->HCDDevice.bAddress = address;
    pDevice->HCDDevice.pHCI->pHCDMethod->RemoveEndPoint(&pDevice->HCDDevice, pDevice->pDefaultControlEP);
    pDevice->pDefaultControlEP = pHCI->pHCDMethod->CreateEndpoint(&pDevice->HCDDevice, 0, HCD_TXW, USB_ENDPOINT_TRANSFER_TYPE_CTRL, pDevice->Desc.bMaxPacketSize, 0); //update EP0 device address

    // get desc again, for last time we may only get 8 bytes accord spec usb 2.0 section 5.5.3.
    _LOG("USB: get full descriptor\n");
    USB_Request Request3 = {USB_REQ_READ | USB_REQTYPE_STANDARD, USB_REQ_GET_DESCRIPTOR, USB_DT_DEVICE << 8, 0, sizeof(USB_DeviceDesc)};
    result = USB_SyncSendRequest(pDevice, &Request3, Buffer);
    if(result != 0)
    {
        assert(FALSE);
        return FALSE;
    }
    pDevice->Desc = *pDesc;
    //desc details
    if (pDevice->Desc.biManufacture || pDevice->Desc.biProduct || pDevice->Desc.biSerialNumber)
    {
        // get manufacturer string
        if (pDevice->Desc.biManufacture)
        {
            _LOG("USB: get manufacture\n");
            USB_GetDescriptorString(pDevice, pDevice->Desc.biManufacture, pDevice->sManufacture, sizeof(pDevice->sManufacture));
        }
        // get product string
        if (pDevice->Desc.biProduct)
        {
            _LOG("USB: get product\n");
            USB_GetDescriptorString(pDevice, pDevice->Desc.biProduct, pDevice->sProduct, sizeof(pDevice->sProduct));
        }
        // get serial number string
        if (pDevice->Desc.biSerialNumber)
        {
            _LOG("USB: get serial num\n");
            USB_GetDescriptorString(pDevice, pDevice->Desc.biSerialNumber, pDevice->sSerialNumber, sizeof(pDevice->sSerialNumber));
        }
    }
    //get config desc
    _LOG("USB: get config descriptor\n");
    USB_Request Request5 = {USB_REQ_READ, USB_REQ_GET_DESCRIPTOR, USB_DT_CONFIGURATION << 8, 0, 9}; //get TotalLength first
    result = USB_SyncSendRequest(pDevice, &Request5, Buffer);
    if(result != 0)
    {
        assert(FALSE);
        return FALSE;
    }
    uint16_t TotalLength = ((USB_ConfigDesc*)Buffer)->wTotalLength;
    _LOG("USB: config descirptor total length: %d\n", TotalLength);
    uint8_t* DescBuffer = Buffer;
    if(TotalLength > USB_DEVBUFFER_SIZE)
        DescBuffer = (uint8_t*)DPMI_DMAMalloc(TotalLength, 4);
    memset(DescBuffer, 0, TotalLength);

    Request5.wLength = TotalLength;
    result = USB_SyncSendRequest(pDevice, &Request5, DescBuffer);
    if(result != 0)
    {
        printf("Error get device config: %d\n", result);
        assert(FALSE);
        return FALSE;
    }
    USB_ParseConfiguration(DescBuffer, TotalLength, pDevice);
    if(DescBuffer != Buffer)
        DPMI_DMAFree(DescBuffer);

    // set device config. only available in addressed state
    // the device will be configured state (enabled)
    _LOG("USB: set config %d\n",pDevice->pConfigList[pDevice->bCurrentConfig].bConfigValue);
    USB_Request Request6 = {USB_REQ_WRITE|USB_REQTYPE_STANDARD, USB_REQ_SET_CONFIGURATION, 0, 0, 0};
    Request6.wValue = pDevice->pConfigList[pDevice->bCurrentConfig].bConfigValue;
    result = USB_SyncSendRequest(pDevice, &Request6, NULL);
    //assert(result == 0);
    if(result == 0)
        pDevice->bStatus = DS_Configured;
    return result == 0;
}

static void USB_ConfigEndpoints(USB_Device* pDevice)
{
    uint8_t bNumEndpoints = 0;

    for(int j = 0; j < pDevice->pConfigList[pDevice->bCurrentConfig].bNumInterfaces; ++j)
        bNumEndpoints = (uint8_t)(bNumEndpoints + pDevice->pConfigList[pDevice->bCurrentConfig].pInterfaces[j].bNumEndpoints);
    assert(bNumEndpoints < 0xF);
    if(pDevice->pEndpoints)
    {
        for(int i = 0; i < pDevice->bNumEndpoints; ++i)
            pDevice->HCDDevice.pHCI->pHCDMethod->RemoveEndPoint(&pDevice->HCDDevice, pDevice->pEndpoints[i]);
        free(pDevice->pEndpoints);
    }
    if(pDevice->pEndpointDesc)
        free(pDevice->pEndpointDesc);

    pDevice->bNumEndpoints = bNumEndpoints;
    pDevice->pEndpoints = (void**)malloc(sizeof(void*)*bNumEndpoints);
    assert(pDevice->pEndpoints);
    pDevice->pEndpointDesc = (USB_EndpointDesc**)malloc(sizeof(USB_EndpointDesc*)*bNumEndpoints);
    assert(pDevice->pEndpointDesc);

    USB_InterfaceDesc* pCurrentInterface = pDevice->pConfigList[pDevice->bCurrentConfig].pInterfaces;
    int currentEP = 0;
    for(uint8_t i = 0; i < bNumEndpoints; ++i)
    {
        USB_EndpointDesc* epd = &pCurrentInterface->pEndpoints[currentEP];
        void* ep = pDevice->HCDDevice.pHCI->pHCDMethod->CreateEndpoint(&pDevice->HCDDevice, epd->bEndpointAddressBits.Num,
            epd->bEndpointAddressBits.Dir ? HCD_TXR : HCD_TXW,
            epd->bmAttributesBits.TransferType,
            epd->wMaxPacketSizeFlags.Size,
            epd->bInterval);    //create EP
        assert(ep);
        _LOG("Class %x Endpoint %02x created: %08x [%u/%u]\n", pDevice->Desc.bDeviceClass, epd->bEndpointAddress, ep, i+1, bNumEndpoints);
        pDevice->pEndpoints[i] = ep;
        pDevice->pEndpointDesc[i] = epd;

        if(++currentEP >= pCurrentInterface->bNumEndpoints)
        {
            ++pCurrentInterface;
            currentEP = 0;
        }
    }
}

void USB_ISR(void)
{
    const uint8_t irq = PIC_GetIRQ();
    BOOL handled = FALSE;
    USB_ISR_FinalizerPtr = &USB_ISR_FinalizerHeader;

    for(int i = 0; i < USBT.HC_Count; ++i)
    {
        HCD_Interface* pHCI = &USBT.HC_List[i];
        if(!HCD_IS_CONTROLLER_VALID(pHCI))
            continue;
        if(pHCI->PCI.Header.DevHeader.Device.IRQ != irq)
            continue;
        if((handled=pHCI->pType->ISR(pHCI)) != 0)
            break; //exit. level triggered event will still exist for next device
    }
    //on some platforms (i.e.VirtualBox), default handler is empty (IRQ 9~11) and do nothing, not even EOI
    if(handled)
    {
        PIC_SendEOI();

        PIC_MaskIRQ(irq); //prevent re-entrance

        USB_ISR_FinalizerPtr = USB_ISR_FinalizerHeader.next;
        while(USB_ISR_FinalizerPtr)
        {
            USB_ISR_FinalizerPtr->FinalizeISR(USB_ISR_FinalizerPtr->data);
            USB_ISR_Finalizer* next = USB_ISR_FinalizerPtr->next;
            free(USB_ISR_FinalizerPtr);
            USB_ISR_FinalizerPtr = next;
        }
        USB_ISR_FinalizerHeader.next = NULL;

        PIC_UnmaskIRQ(irq);
        //_LOG("EOIE ");
    }
    else //if we send EOI, we stop the calling chain because the final handler in the chain may send EOI again.
    {
        //call old handler
        DPMI_ISR_HANDLE handle = *USB_FindISRHandle(irq); //need a copy for lcall
#if defined(__DJ2__)
        asm(
            "pushfl \n\t"
            "cli \n\t"
            "lcall *%0 \n\t"
            ::"m"(handle.offset)
        );
#else
        DPMI_REG r = {0};
        r.w.cs = handle.rm_cs;
        r.w.ip = handle.rm_offset;
        DPMI_CallRealModeIRET(&r);
#endif
        //original handler may mask the IRQ agian, enable
        PIC_UnmaskIRQ(irq);

    }
}

DPMI_ISR_HANDLE* USB_FindISRHandle(uint8_t irq)
{
    int i;
    for(i = 0; i < USBT.HC_Count; ++i)
    {
        if(USB_ISRHandle[i].n == PIC_IRQ2VEC(irq))
            return &USB_ISRHandle[i];
    }
    return NULL;
}

void USB_ISR_Wraper(void)
{
    USB_InISR = TRUE;
    USB_ISR();
    USB_InISR = FALSE;
}

void USB_Completion_SyncCallback(HCD_Request* pRequest)
{
    USB_SyncCallbackResult* pResult = (USB_SyncCallbackResult*)pRequest->pCBData;
    pResult->Length = pRequest->transferred;
    //if(pRequest->error)
    //    _LOG("ERROR:%x\n", pRequest->error);
    pResult->ErrorCode = pRequest->error;
    pResult->Finished = TRUE;
}

void USB_Completion_UserCallback(HCD_Request* pRequest)
{
    USB_UserCallbackResult* pUserData = (USB_UserCallbackResult*)pRequest->pCBData;
    if(pRequest->dir == HCD_TXR)
        memcpy(pUserData->pUserBuffer, pRequest->pBuffer, pRequest->transferred);
    USB_TFree(pRequest->pBuffer);
    if(pUserData->pSetup8)
        USB_TFree(pUserData->pSetup8);

    pRequest->pCBData = pUserData->pCBData;
    pRequest->pBuffer = pUserData->pUserBuffer;
    HCD_COMPLETION_CB pCB = pUserData->pCB;
    USB_TFree32(pUserData);
    if(pCB)
        pCB(pRequest);
}

#if defined(__BC__)
#include <ctype.h>
//wchar_t == char in BC. declaration copied from BC header
static size_t  _Cdecl _FARFUNC wcstombs(char _FAR *__s, const wchar_t _FAR *__pwcs, size_t __n)
{
    size_t i = 0;
    while(i < __n)
    {
        if(!isprint(*(__pwcs+i*2)))
            break;
        *(__s+i) = *(__pwcs+i*2);
        ++i;
    }
    return i;
}

#endif
