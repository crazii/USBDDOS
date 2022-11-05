#include <stdio.h>
#include <stdlib.h>
#include <dos.h>
#include <string.h>
#include <assert.h>
#include "USBDDOS/USB.H"
#include "USBDDOS/DPMI/DPMI.H"
#include "USBDDOS/PIC.H"
#include "USBDDOS/USBALLOC.H"
#include "USBDDOS/DBGUTIL.H"

#define USB_PCI_SBC_CLASS   0x0C  //PCI serial bus controller
#define USB_SUBCLASS        03    //PCI class ID 0Ch, sub class id 03h

//make idle wait can be interrupted by Ctrl+C
//HLT (halt) supported from 8086. but need privilege.
#if defined(__BC__)

#define USB_IDLE_WAIT() do {\
_LOG("");\
_ASM_BEGIN \
_ASM(pushf) \
_ASM(sti) \
_ASM(hlt)\
_ASM(popf) \
_ASM_END } while(0)

#else

#define USB_IDLE_WAIT() do {\
_ASM_BEGIN \
_ASM(pushf) \
_ASM(sti) \
_ASM(push ax)\
_ASM(mov ax, 0x1680)\
_ASM(int 0x2F)\
_ASM(pop ax)\
_ASM(popf) \
_ASM_END } while(0)

#endif


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

static DPMI_ISR_HANDLE USB_ISRHandle[USB_MAX_HC_COUNT];
static BOOL USB_IRQShared[USB_MAX_HC_COUNT];
static BOOL USB_InISR = FALSE;
static uint16_t USB_IRQMask = 0;

static void USB_Shutdown(void);
static BOOL USB_InitController(uint8_t bus, uint8_t dev, uint8_t func, PCI_DEVICE* pPCIDev);
static BOOL USB_InitDevice(HCD_Interface* pHCI, uint8_t portIndex, uint16_t portStatus);
static BOOL USB_RemoveDevice(USB_Device* pDevice);
static void USB_EnumerateDevices();
static BOOL USB_ConfigDevice(USB_Device* pDevice, uint8_t address);
static void USB_ConfigEndpoints(USB_Device* pDevice);
static DPMI_ISR_HANDLE* USB_FindISRHandle(uint8_t irq);
static void USB_ISR_Wraper(void);
static __INLINE void USB_ISR(void);
static void USB_Completion_SyncCallback(HCD_Request* pRequest); //calledin hc driver interrupt handler
static void USB_Completion_UserCallback(HCD_Request* pRequest); //calledin hc driver interrupt handler

void USB_Init(void)
{
    atexit(&USB_Shutdown);
    USB_InitAlloc();

    USBT.HC_Count = 0;
    memset(USBT.Devices, 0, sizeof(USBT.Devices));

    USB_IRQMask = PIC_GetIRQMask();
    _LOG("IRQ mask: %04x\n", USB_IRQMask);

    //enumerate host controllers
    for(uint8_t bus = 0; bus < PCI_MAX_BUS; bus++)
    {
        for(uint8_t dev = 0; dev < PCI_MAX_DEV; dev++)
        {
            for(uint8_t func = 0; func < PCI_MAX_FUNC; func++)
			{//some controller has multiple functions (i.e. 0 - UHCI,1 - UHCI,2 - UHCI,x - EHCI)
                PCI_DEVICE pcidev;
                PCI_ReadDevice(bus, dev, func, &pcidev);
				if(pcidev.Header.Class == USB_PCI_SBC_CLASS && pcidev.Header.SubClass == USB_SUBCLASS && pcidev.Header.DevHeader.Device.IRQ < 15)
                    USB_InitController(bus, dev, func, &pcidev);
                else if(func == 0) // multi function
                {
                    if (pcidev.Header.HeaderType&PCI_HEADER_MULTI_FUNCTION)
                        ;
                    else
                        break;
                }
            }
        }
    }
    USB_EnumerateDevices();
    return;
}

void USB_Shutdown(void)
{
    //clean up (if app exit normally without TSR)
    for(int address = 0; address < USB_MAX_DEVICE_COUNT; ++address)
    {
		USB_Device* pDevice = &USBT.Devices[address];
        if(!HCD_IS_DEVICE_VALID(&pDevice->HCDDevice))
            continue;
		_LOG("CPUFLAGS: %04x\n", CPU_FLAGS()); //make sure interrupt is on (IF set)
		assert(CPU_FLAGS()&CPU_IFLAG);
        while(pDevice->HCDDevice.pRequest != NULL) //waiting on interrupt to handle pending request
            USB_IDLE_WAIT();
        _LOG("Removing device at address: %d\n", address+1);
        USB_RemoveDevice(pDevice);
    }

    PIC_SetIRQMask(USB_IRQMask);
    for(int j = 0; j < USBT.HC_Count; ++j)
    {
        if(USB_ISRHandle[j].n && !USB_IRQShared[j])
        {
            _LOG("Unstalling ISR, IRQ: %d, Vector: 0x%02x\n", PIC_VEC2IRQ(USB_ISRHandle[j].n), USB_ISRHandle[j].n);
            DPMI_UninstallISR(&USB_ISRHandle[j]);
        }
    }

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
}

BOOL USB_InitController(uint8_t bus, uint8_t dev, uint8_t func, PCI_DEVICE* pPCIDev)
{
    if(USBT.HC_Count >= USB_MAX_HC_COUNT)
        return FALSE;

    BOOL OK = FALSE;
    PCI_HEADER* header = &pPCIDev->Header;
    //note: the PCI BIOS will setup the IRQ and write the IRQ register, the IRQ is setup as level triggered.
	_LOG("Host Controller: Bus:%d, Dev:%d, Func:%d, pi:0x%02x, IRQ: %d, INT Pin: INT%c#\n", bus, dev, func, header->PInterface, header->DevHeader.Device.IRQ, 'A'+header->DevHeader.Device.INTPIN-1);
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
                PIC_SetIRQMask(irqmask);
                continue;
            }
            uint16_t index = USBT.HC_Count++;
			uint8_t iv = PIC_IRQ2VEC(header->DevHeader.Device.IRQ);
			_LOG("Install ISR 0x%02x\n", iv);
			DPMI_ISR_HANDLE* handle = USB_FindISRHandle(header->DevHeader.Device.IRQ);
			if(handle != NULL)
            {
                USB_ISRHandle[index] = *handle;
                USB_IRQShared[index] = TRUE;
            }
            else if( DPMI_InstallISR(iv, USB_ISR_Wraper, &USB_ISRHandle[index]) != 0)
            {
                printf("Error: Install ISR failed.\n");
                exit(-1);
            }
			PIC_UnmaskIRQ(header->DevHeader.Device.IRQ);
			//_LOG("IRQ mask: %04x\n", PIC_GetIRQMask());
            break;
        }
    }
    return OK;
}

BOOL USB_InitDevice(HCD_Interface* pHCI, uint8_t portIndex, uint16_t portStatus)
{
    //early return
    if(pHCI->bDevCount >= USB_MAX_HC_COUNT)
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

    if( !HCD_InitDevice(pHCI, &pDevice->HCDDevice, portIndex, portStatus) )
        return FALSE;
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
        pHCI->pHCDMethod->SetPortStatus(pHCI, portIndex, USB_PORT_DISABLE); //disable it to continue enumeration on next port
    }
    // install device driver
    if(pDevice->bStatus == DS_Configured)
	{
		#if DEBUG
        USB_ShowDeviceInfo(pDevice);
        #endif
        _LOG("USB: config endpoints\n");
        USB_ConfigEndpoints(pDevice);

        uint8_t bClass = pDevice->Desc.bDeviceClass;
        _LOG("USB: Device class:%d, install drivers\n", bClass);
        BOOL DriverInstalled = FALSE;
		if(bClass < USBC_MAX_DRIVER && USBT.ClassDrivers[bClass].InitDevice != NULL)
		{
			DriverInstalled = USBT.ClassDrivers[bClass].InitDevice(pDevice);
		}

		if(!DriverInstalled)
		{
			//try vendor specific driver
			//not implemented
		}

		if(DriverInstalled)
		{
			pDevice->bStatus = DS_Ready;
		}
		return TRUE;
	}

	USB_RemoveDevice(pDevice);
	printf("Error intializing device at port: %d.\n", portIndex);
    return FALSE;
}

BOOL USB_RemoveDevice(USB_Device* pDevice)
{
    if(!HCD_IS_DEVICE_VALID(&pDevice->HCDDevice))
        return FALSE;

    for(int e = 0; e < pDevice->bNumEndpoints; ++e)
        pDevice->HCDDevice.pHCI->pHCDMethod->RemoveEndPoint(&pDevice->HCDDevice, pDevice->pEndpoints[e]);
    pDevice->HCDDevice.pHCI->pHCDMethod->RemoveEndPoint(&pDevice->HCDDevice, pDevice->pDefaultControlEP); //HCD implementation probably do nothing but better do the call.
    _LOG("Removing HCD device\n");
    HCD_RemoveDevice(&pDevice->HCDDevice);

    _LOG("Free device buffers\n");
    if(pDevice->pDeviceBuffer)
        DPMI_DMAFree(pDevice->pDeviceBuffer);
    if(pDevice->pSetup)
        DPMI_DMAFree(pDevice->pSetup);

    _LOG("Free device configs\n");
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
        return error;
        
    while(!result.Finished)
        USB_IDLE_WAIT();
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
    if(pDesc->bmAttributesBits.TransferType == USB_ENDPOINT_TRANSFER_TYPE_CTRL || pDesc->bmAttributesBits.TransferType > USB_ENDPOINT_TRANSFER_TYPE_INTR)
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
        
    //idle wait for interrupt (HW notifying finish event and hcd driver call USB_Completion_Callback)
    while(!result.Finished)
        USB_IDLE_WAIT();
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

    USB_Request request = {USB_REQ_WRITE|USB_REQTYPE_STANDARD, USB_REQ_SET_CONFIGURATION, configuration, 0, 0};
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
            memset(pInterfaceDesc, 0, sizeof(USB_InterfaceDesc)*pConfigDesc->bNumInterfaces);
            pDevice->pConfigList[ConfigIndex].pInterfaces = pInterfaceDesc;
        }
        else if(descType == USB_DT_INTERFACE)
        {
            ++InterfaceIndex;
            EndpointIndex = -1;
            assert(InterfaceIndex < pDevice->pConfigList[ConfigIndex].bNumInterfaces);

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
            memset(pEndPointDesc, 0, sizeof(USB_EndpointDesc)*EndPointNum);
            pDevice->pConfigList[ConfigIndex].pInterfaces[InterfaceIndex].pEndpoints = pEndPointDesc;
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

static void USB_EnumerateDevices()
{
    //initially all powered devices will be in default states (address 0), but ports disabled.
	//enable one port each time, and it will respond via device address (FA) 0, default pipe - endpoint 0.
    //if device inited, assign address to it (the address is used for further communication), and continue to next device.
    //otherwise disable the port (only one device can be enabled with address 0) and continue to next
    for(int j = 0; j < USBT.HC_Count; ++j)
    {
        HCD_Interface* pHCI = &USBT.HC_List[j];

        for(uint8_t i = 0; i < pHCI->bNumPorts; ++i)
        {
            uint16_t status = pHCI->pHCDMethod->GetPortStatus(pHCI, i);
            
            if(status&USB_PORT_ATTACHED)
            {
                //stablize
                pHCI->pHCDMethod->SetPortStatus(pHCI, i, USB_PORT_CONNECT_CHANGE); //clear state change
                delay(100);
                BOOL stablized = !(pHCI->pHCDMethod->GetPortStatus(pHCI, i)&USB_PORT_CONNECT_CHANGE);
                if(!stablized)
                    continue;
                
                _LOG("Enumerate device at port %d.\n",i);
                USB_InitDevice(pHCI, i, status);
            }
        }    
    }
}

static BOOL USB_ConfigDevice(USB_Device* pDevice, uint8_t address)
{
	//https://techcommunity.microsoft.com/t5/microsoft-usb-blog/how-does-usb-stack-enumerate-a-device/ba-p/270685

	HCD_Interface* pHCI = pDevice->HCDDevice.pHCI;
	// 1st reset
	pHCI->pHCDMethod->SetPortStatus(pHCI, pDevice->HCDDevice.bHubPort, USB_PORT_RESET|USB_PORT_ENABLE);
	//pHCI->pHCDMethod->SetPortStatus(pHCI, pDevice->HCDDevice.bHubPort, );

	uint8_t* Buffer = pDevice->pDeviceBuffer;
	memset(Buffer, 0, USB_DEVBUFFER_SIZE);

	// get device descriptor
	_LOG("USB: initial descriptor\n");
	pDevice->pDefaultControlEP = pHCI->pHCDMethod->CreateEndpoint(&pDevice->HCDDevice, 0, HCD_TXW, USB_ENDPOINT_TRANSFER_TYPE_CTRL, pDevice->Desc.bMaxPacketSize, 0);
	USB_Request Request1 = {USB_REQ_READ | USB_REQTYPE_STANDARD, USB_REQ_GET_DESCRIPTOR, USB_DT_DEVICE << 8, 0, 64};
	uint8_t result = USB_SyncSendRequest(pDevice, &Request1, Buffer);
	//first try may fail because incorrect max packet size
	USB_DeviceDesc* pDesc = (USB_DeviceDesc*)Buffer;
	pDevice->Desc.bMaxPacketSize = pDesc->bMaxPacketSize ? pDesc->bMaxPacketSize : pDevice->Desc.bMaxPacketSize;
	//_LOG("packet size: %d\n", pDesc->bMaxPacketSize);

	pDevice->HCDDevice.pHCI->pHCDMethod->RemoveEndPoint(&pDevice->HCDDevice, pDevice->pDefaultControlEP);
	pDevice->pDefaultControlEP = pHCI->pHCDMethod->CreateEndpoint(&pDevice->HCDDevice, 0, HCD_TXW, USB_ENDPOINT_TRANSFER_TYPE_CTRL, pDevice->Desc.bMaxPacketSize, 0);//update EP0 maxpacket

	// 2nd reset
	pHCI->pHCDMethod->SetPortStatus(pHCI, pDevice->HCDDevice.bHubPort, USB_PORT_RESET|USB_PORT_ENABLE);

    // set device address.
    // before set address, nothing can be done except get desc.
    // the device will be adressed state
    _LOG("USB: set device address\n");
    USB_Request Request2 = {USB_REQ_WRITE|USB_REQTYPE_STANDARD, USB_REQ_SET_ADDRESS, address, 0, 0};
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
        USB_Request Request4 = {USB_REQ_READ|USB_REQTYPE_STANDARD, USB_REQ_GET_DESCRIPTOR, (USB_DT_STRING << 8), USB_LANG_ID_ENG, USB_DEVBUFFER_SIZE};

        // get manufacturer string
        if (pDevice->Desc.biManufacture)
        {
            Request4.wValue = (uint16_t)((USB_DT_STRING << 8) + pDevice->Desc.biManufacture);
            if (USB_SyncSendRequest(pDevice, &Request4, Buffer) == 0)
                wcstombs(pDevice->sManufacture, (wchar_t*)(Buffer+2), min(sizeof(pDevice->sManufacture)-1, Buffer[0]));
        }
        // get product string
        if (pDevice->Desc.biProduct)
        {
            Request4.wValue = (uint16_t)((USB_DT_STRING << 8) + pDevice->Desc.biProduct);
            if (USB_SyncSendRequest(pDevice, &Request4, Buffer) == 0)
                wcstombs(pDevice->sProduct, (wchar_t*)(Buffer+2), min(sizeof(pDevice->sProduct)-1, Buffer[0]));
        }
        // get serial number string
        if (pDevice->Desc.biSerialNumber)
        {
            Request4.wValue = (uint16_t)((USB_DT_STRING << 8) + pDevice->Desc.biSerialNumber);
            if (USB_SyncSendRequest(pDevice, &Request4, Buffer) == 0)
                wcstombs(pDevice->sSerialNumber, (wchar_t*)(Buffer+2), min(sizeof(pDevice->sSerialNumber)-1, Buffer[0]));
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
    uint8_t* DescBuffer = Buffer;
    if(TotalLength > USB_DEVBUFFER_SIZE)
        DescBuffer = (uint8_t*)DPMI_DMAMalloc(TotalLength, 8);

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
    _LOG("USB: set config\n");
    USB_Request Request6 = {USB_REQ_WRITE|USB_REQTYPE_STANDARD, USB_REQ_SET_CONFIGURATION, 0, 0, 0};
    Request6.wValue = pDevice->pConfigList[pDevice->bCurrentConfig].bConfigValue;
	result = USB_SyncSendRequest(pDevice, &Request6, NULL);
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
    pDevice->pEndpointDesc = (USB_EndpointDesc**)malloc(sizeof(USB_EndpointDesc*)*bNumEndpoints);

    USB_InterfaceDesc* pCurrentInterface = pDevice->pConfigList[pDevice->bCurrentConfig].pInterfaces;
    int currentEP = 0;
    for(int8_t i = 0; i < bNumEndpoints; ++i)
    {
        USB_EndpointDesc* epd = &pCurrentInterface->pEndpoints[currentEP];
        void* ep = pDevice->HCDDevice.pHCI->pHCDMethod->CreateEndpoint(&pDevice->HCDDevice, epd->bEndpointAddressBits.Num,
            epd->bEndpointAddressBits.Dir ? HCD_TXR : HCD_TXW,
            epd->bmAttributesBits.TransferType,
            epd->wMaxPacketSizeFlags.Size,
            epd->bInterval);    //create EP
        assert(ep);
		_LOG("Endpoint %02x created: %08x\n", epd->bEndpointAddress, ep);
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
	if(irq == 0xFF)
		return;

	const uint8_t vec = PIC_IRQ2VEC(irq);

	int i;
	for(i = 0; i < USBT.HC_Count; ++i)
	{
		if(USB_ISRHandle[i].n == vec)
		{
			HCD_Interface* pHCI = &USBT.HC_List[i];
			if(!HCD_IS_CONTROLLER_VALID(pHCI))
				continue;
			if((handled=pHCI->pType->ISR(pHCI)) != 0)
				break; //exit. level triggered event will still exist for next device
		}
	}
	//default handler is empty (IRQ 9~11) and do nothing, not even EOI
	//assume 3rd party driver handler exist if previous irq mask bit is clear
	if(handled || PIC_IS_IRQ_MASKED(USB_IRQMask, irq))
	{
		PIC_SendEOI();
		return;
	}

	//call old handler
	DPMI_ISR_HANDLE* handle = USB_FindISRHandle(irq);
	//printf("%d",irq);
	assert(handle);
	DPMI_REG r = {0};
	r.w.cs = handle->rm_cs;
	r.w.ip = handle->rm_offset;
	DPMI_CallRealModeIRET(&r);

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
	//some DPMI helper save only low word part of the GPRs (pusha).
	_ASM_BEGIN _ASM(pushad) _ASM_END//just make it safe
	USB_InISR = TRUE;

    USB_ISR();

    USB_InISR = FALSE;
	_ASM_BEGIN _ASM(popad) _ASM_END
}

void USB_Completion_SyncCallback(HCD_Request* pRequest)
{
    USB_SyncCallbackResult* pResult = (USB_SyncCallbackResult*)pRequest->pCBData;
	pResult->Length = pRequest->transferred;
	//_LOG("ERROR:%x\n", pRequest->error);
    pResult->ErrorCode = pRequest->error;
    pResult->Finished = TRUE;
}

void USB_Completion_UserCallback(HCD_Request* pRequest)
{
    USB_UserCallbackResult* pUserData = (USB_UserCallbackResult*)pRequest->pCBData;
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