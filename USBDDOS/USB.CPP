#include <stdio.h>
#include <stdlib.h>
#include <dos.h>
#include <string.h>
#include <assert.h>
#include "USBDDOS/USB.H"
#include "USBDDOS/DPMI/DPMI.H"

static void USB_EnumerateDevices();
static BOOL USB_ConfigDevice(USB_Device* pDevice, uint8_t address);
static void USB_ConfigEndpoints(USB_Device* pDevice);

#define USB_CLASS 0x0C03    //PCI class ID 0Ch, sub class id 03h

static void USB_Shutdown(void);

void USB_Init(void)
{
    atexit(&USB_Shutdown);

    USBT.HC_Count = 0;
    memset(USBT.Devices, 0, sizeof(USBT.Devices));

    //enumerate host controllers
    uint8_t bus = 0, dev = 0, func = 0;
    for (bus = 0; bus < 0x10; bus++)
    {
        for (dev = 0; dev <= 0x1f; dev++)
        {
            for (func = 0; func < 0x08; func++)
            {
                if (PCI_Read_Word(bus, dev, func, 0x0a) == USB_CLASS)
                    USB_InitController(bus, dev, func);
                else if (func == 0) // multi function
                {
                    if ((PCI_Read_Byte(bus, dev, func, 0x0e) & 0x80) == 0x80)
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

static void USB_Shutdown(void)
{
    //debug clean up
    for(int address = 0; address < USB_MAX_DEVICE_COUNT; ++address)
    {
        USB_Device* pDevice = &USBT.Devices[address];
        if(HCD_IS_DEVICE_VALID(&pDevice->HCDDevice))
        {
            USBT.Devices[address].HCDDevice.pHCI->pHCDMethod->RemoveDevice(&USBT.Devices[address].HCDDevice);
            if(pDevice->pConfigList != NULL)
            {
                for(int i = 0; i < pDevice->Desc.bNumConfigurations; ++i)
                {
                    for(int j = 0; j < pDevice->pConfigList[i].bNumInterfaces; ++j)
                    {
                        delete[] pDevice->pConfigList[i].pInterfaces[j].pEndpoints;
                    }
                    delete[] pDevice->pConfigList[i].pInterfaces;
                }
                delete[] pDevice->pConfigList;
            }
        }
    }
    for(int j = 0; j < USBT.HC_Count; ++j)
    {
        HCD_Interface* pHCI = &USBT.HC_List[j];
        uint32_t id = PCI_Read_Byte(pHCI->PCIAddr.Bus, pHCI->PCIAddr.Device, pHCI->PCIAddr.Function, 0x09);
        for(int i = 0; i < USB_MAX_HC_TYPE; ++i)
        {
            if(USBT.HC_Types[i].DeinitController != NULL && USBT.HC_Types[i].dwID == id)
            {
                USBT.HC_Types[i].DeinitController(pHCI);
            }
        }
    }
    USBT.HC_Count = 0;
    memset(USBT.Devices, 0, sizeof(USBT.Devices));
}

BOOL USB_InitController(uint8_t bus, uint8_t dev, uint8_t func)
{
    if(USBT.HC_Count >= USB_MAX_HC_COUNT)
        return FALSE;

    BOOL OK = FALSE;
    uint32_t id = PCI_Read_Byte(bus, dev, func, 0x09);
    for(int i = 0; i < USB_MAX_HC_TYPE; ++i)
    {
        if(USBT.HC_Types[i].InitController != NULL && USBT.HC_Types[i].dwID == id)
        {
            _LOG("Host Controller: Bus:%d, Dev:%d, Func:%d, id:%08lx\n", bus, dev, func, id);
            HCD_Interface* pHCI = &USBT.HC_List[USBT.HC_Count];
            memset(pHCI, 0, sizeof(HCD_Interface));
            pHCI->PCIAddr.Bus = bus;
            pHCI->PCIAddr.Device = dev;
            pHCI->PCIAddr.Function = func;

            OK = USBT.HC_Types[i].InitController(pHCI) | OK;
            if(OK)
                ++USBT.HC_Count;
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
    address += 1; //1 based
    memset(pDevice, 0, sizeof(USB_Device));

    uint8_t result = 0;
    pDevice->HCDDevice.pHCI = pHCI;
    pDevice->HCDDevice.bHubPort = portIndex;
    pDevice->HCDDevice.bAddress = 0;
    pDevice->HCDDevice.bEndpointSpeed = portStatus&USB_PORT_SPEEDMASK;
    pDevice->bStatus = DS_Default;

    pHCI->pHCDMethod->InitDevice(&pDevice->HCDDevice); //allocate data

    //predetermine packet size
    if (pDevice->HCDDevice.bEndpointSpeed == USB_PORT_Low_Speed_Device)
        pDevice->Desc.bMaxPacketSize = 8;
    else if (pDevice->HCDDevice.bEndpointSpeed == USB_PORT_High_Speed_Device)
        pDevice->Desc.bMaxPacketSize = 64;
    else if(pDevice->HCDDevice.bEndpointSpeed ==  USB_PORT_Full_Speed_Device)
        pDevice->Desc.bMaxPacketSize = 8; // 8~64
    else
        assert(FALSE);

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
        uint8_t bClass = pDevice->Desc.bDeviceClass;
        //printf("Device class:%d\n", bClass);
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
            USB_ConfigEndpoints(pDevice);
            pDevice->bStatus = DS_Ready;
        }
        return TRUE;
    }
    pHCI->pHCDMethod->RemoveDevice(&pDevice->HCDDevice);
    printf("Error enabling device: %d.\n", result);
    return FALSE;
}

uint8_t USB_SendRequest(USB_Device* pDevice, USB_Request* pRequest, void* pBuffer)
{
    return pDevice->HCDDevice.pHCI->pHCDMethod->ControlTransfer(
        &pDevice->HCDDevice, 0, (pRequest->bmRequestType&USB_REQ_READ) ? HCD_TXR : HCD_TXW,
        (uint8_t*)pRequest, pBuffer, pRequest->wLength);
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
    BOOL result = USB_SendRequest(pDevice, &request, NULL);
    assert(result);
    if(!result)
        return FALSE;

    pDevice->bCurrentConfig = configuration;
    //reconfig ep
    if(pDevice->bStatus == DS_Ready)
    {
        //clean up old endpoints. TODO: better ways
        pDevice->HCDDevice.pHCI->pHCDMethod->RemoveDevice(&pDevice->HCDDevice);
        pDevice->HCDDevice.pHCI->pHCDMethod->InitDevice(&pDevice->HCDDevice);

        USB_ConfigEndpoints(pDevice);
    }
    return TRUE;
}

BOOL USB_ParseConfiguration(uint8_t* pBuffer, uint16_t length, USB_Device* pDevice)
{
    assert( ((USB_ConfigDesc*)pBuffer)->wTotalLength == length);
    pDevice->pConfigList = new USB_ConfigDesc[pDevice->Desc.bNumConfigurations];
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
            USB_InterfaceDesc* pInterfaceDesc = new USB_InterfaceDesc[pConfigDesc->bNumInterfaces];
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
            USB_EndpointDesc* pEndPointDesc = new USB_EndpointDesc[EndPointNum];
            memset(pEndPointDesc, 0, sizeof(USB_EndpointDesc)*EndPointNum);
            pDevice->pConfigList[ConfigIndex].pInterfaces[InterfaceIndex].pEndpoints = pEndPointDesc;
        }
        else if(descType == USB_DT_ENDPOINT)
        {
            ++EndpointIndex;
            assert(EndpointIndex < pDevice->pConfigList[ConfigIndex].pInterfaces[InterfaceIndex].bNumEndpoints);
            pDevice->pConfigList[ConfigIndex].pInterfaces[InterfaceIndex].pEndpoints[EndpointIndex] = *(USB_EndpointDesc*)(pBuffer+i);
        }
        i += len;
    }
    return TRUE;
}

void USB_ShowDeviceInfo(USB_Device* pDevice)
{
    printf("%s,  %s,  %s\n", pDevice->sManufacture, pDevice->sProduct, pDevice->sSerialNumber);
    printf("bDeviceClass: %x, bDeviceSubClass: %x, bDeviceProtocol: %x\n", pDevice->Desc.bDeviceClass, pDevice->Desc.bDeviceSubClass, pDevice->Desc.bDeviceProtocol);
    printf("widVendor: %04X, widProduct: %04X, wbcdUSB: %04X\n", pDevice->Desc.widVendor, pDevice->Desc.widProduct, pDevice->Desc.wbcdUSB);
    printf("bMaxPacketSize: %d, bEndpointSpeed: %x\n", pDevice->Desc.bMaxPacketSize, pDevice->HCDDevice.bEndpointSpeed);
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
                const USB_EndpointDesc& epd = pDevice->pConfigList[i].pInterfaces[j].pEndpoints[k];
                printf("        Endpoint %d: Address: %02xh, Type: %d, MaxPacketSize: %d, Interval: %d\n", k,
                    epd.bEndpointAddress, epd.bmAttributesFlags.TransferType, epd.wMaxPacketSizeFlags.Size, epd.bInterval);
            }
        }
    }

    return;
}

static void USB_EnumerateDevices()
{
    //initially all powered devices will be in default states (address 0), but ports disabled.
    //enable one port each time, and it will respond via address 0, default pipe - endpoint(FA) 0.
    //if device inited, assign address to it and continue to next
    //otherwise disable the port (only one device can be enabled with address 0) and continue to next
    for(int j = 0; j < USBT.HC_Count; ++j)
    {
        HCD_Interface* pHCI = &USBT.HC_List[j];

        for(int i = 0; i < pHCI->bNumPorts; ++i)
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
    pHCI->pHCDMethod->SetPortStatus(pHCI, pDevice->HCDDevice.bHubPort, USB_PORT_ENABLE);
    pHCI->pHCDMethod->SetPortStatus(pHCI, pDevice->HCDDevice.bHubPort, USB_PORT_RESET);

    uint8_t* Buffer = pHCI->pHCDMethod->GetHCDBuffer(&pDevice->HCDDevice);
    memset(Buffer, 0, HCDBUFFER_SIZE);

    // get device descriptor
    //_LOG("initial descriptor\n");
    pHCI->pHCDMethod->CreateEndpoint(&pDevice->HCDDevice, 0, HCD_TXW, USB_ENDPOINT_TRANSFER_TYPE_CTRL, pDevice->Desc.bMaxPacketSize, 0);
    USB_Request Request1 = {USB_REQ_READ | USB_REQTYPE_STANDARD, USB_REQ_GET_DESCRIPTOR, USB_DT_DEVICE << 8, 0, 64};
    uint8_t result = USB_SendRequest(pDevice, &Request1, Buffer);
    //first try may fail because incorrect max packet size
    USB_DeviceDesc* pDesc = (USB_DeviceDesc*)Buffer;
    pDevice->Desc.bMaxPacketSize = pDesc->bMaxPacketSize ? pDesc->bMaxPacketSize : pDevice->Desc.bMaxPacketSize;
    pHCI->pHCDMethod->CreateEndpoint(&pDevice->HCDDevice, 0, HCD_TXW, USB_ENDPOINT_TRANSFER_TYPE_CTRL, pDevice->Desc.bMaxPacketSize, 0);//notify maxpacket

    // 2nd reset
    pHCI->pHCDMethod->SetPortStatus(pHCI, pDevice->HCDDevice.bHubPort, USB_PORT_RESET);

    // set device address.
    // before set address, nothing can be done except get desc.
    // the device will be adressed state
    //_LOG("set device address\n");
    USB_Request Request2 = {USB_REQ_WRITE|USB_REQTYPE_STANDARD, USB_REQ_SET_ADDRESS, address, 0, 0};
    result = USB_SendRequest(pDevice, &Request2, NULL);
    if(result != 0)
    {
        assert(FALSE);
        return FALSE;
    }
    pDevice->bStatus = DS_Addressed;
    pDevice->HCDDevice.bAddress = address;
    delay(2); // spec required.
    pHCI->pHCDMethod->CreateEndpoint(&pDevice->HCDDevice, 0, HCD_TXW, USB_ENDPOINT_TRANSFER_TYPE_CTRL, pDevice->Desc.bMaxPacketSize, 0); //update EP0 address

    // get desc again, for last time we may only get 8 bytes accord spec usb 2.0 section 5.5.3.
    //_LOG("get full descriptor\n");
    USB_Request Request3 = {USB_REQ_READ | USB_REQTYPE_STANDARD, USB_REQ_GET_DESCRIPTOR, USB_DT_DEVICE << 8, 0, sizeof(USB_DeviceDesc)};
    result = USB_SendRequest(pDevice, &Request3, Buffer);
    if(result != 0)
    {
        assert(FALSE);
        return FALSE;
    }
    pDevice->Desc = *pDesc;
    //desc details
    if (pDevice->Desc.biManufacture || pDevice->Desc.biProduct || pDevice->Desc.biSerialNumber)
    {
        USB_Request Request4 = {USB_REQ_READ|USB_REQTYPE_STANDARD, USB_REQ_GET_DESCRIPTOR, (USB_DT_STRING << 8), USB_LANG_ID_ENG, HCDBUFFER_SIZE};

        // get manufacturer string
        if (pDevice->Desc.biManufacture)
        {
            Request4.wValue = (USB_DT_STRING << 8) + pDevice->Desc.biManufacture;
            if (USB_SendRequest(pDevice, &Request4, Buffer) == 0)
                wcstombs(pDevice->sManufacture, (wchar_t*)(Buffer+2), min(sizeof(pDevice->sManufacture)-1, Buffer[0]));
        }
        // get product string
        if (pDevice->Desc.biProduct)
        {
            Request4.wValue = (USB_DT_STRING << 8) + pDevice->Desc.biProduct;
            if (USB_SendRequest(pDevice, &Request4, Buffer) == 0)
                wcstombs(pDevice->sProduct, (wchar_t*)(Buffer+2), min(sizeof(pDevice->sProduct)-1, Buffer[0]));
        }
        // get serial number string
        if (pDevice->Desc.biSerialNumber)
        {
            Request4.wValue = (USB_DT_STRING << 8) + pDevice->Desc.biSerialNumber;
            if (USB_SendRequest(pDevice, &Request4, Buffer) == 0)
                wcstombs(pDevice->sSerialNumber, (wchar_t*)(Buffer+2), min(sizeof(pDevice->sSerialNumber)-1, Buffer[0]));
        }
    }
    //get config desc
    //_LOG("get config descriptor\n");
    USB_Request Request5 = {USB_REQ_READ, USB_REQ_GET_DESCRIPTOR, USB_DT_CONFIGURATION << 8, 0, 9}; //get TotalLength first
    result = USB_SendRequest(pDevice, &Request5, Buffer);
    if(result != 0)
    {
        assert(FALSE);
        return FALSE;
    }
    uint16_t TotalLength = ((USB_ConfigDesc*)Buffer)->wTotalLength;
    //_LOG("total length:%d\n", TotalLength);
    uint8_t* DescBuffer = Buffer;
    if(TotalLength > HCDBUFFER_SIZE)
        DescBuffer = (uint8_t*)DPMI_MappedMalloc(TotalLength, 8);

    Request5.wLength = TotalLength;
    result = USB_SendRequest(pDevice, &Request5, DescBuffer);
    if(result != 0)
    {
        printf("Error get device config: %d\n", result);
        assert(FALSE);
        return FALSE;
    }
    USB_ParseConfiguration(DescBuffer, TotalLength, pDevice);
    if(DescBuffer != Buffer)
        DPMI_MappedFree(DescBuffer);

    // set device config. only available in addressed state
    // the device will be configured state (enabled)
    //_LOG("set config\n");
    USB_Request Request6 = {USB_REQ_WRITE|USB_REQTYPE_STANDARD, USB_REQ_SET_CONFIGURATION, 0, 0, 0};
    Request6.wValue = pDevice->pConfigList[pDevice->bCurrentConfig].bConfigValue;
    result = USB_SendRequest(pDevice, &Request6, NULL);
    if(result == 0)
        pDevice->bStatus = DS_Configured;
    return result == 0;
}

static void USB_ConfigEndpoints(USB_Device* pDevice)
{
    int8_t bNumEndpoints = 0;

    for(int j = 0; j < pDevice->pConfigList[pDevice->bCurrentConfig].bNumInterfaces; ++j)
        bNumEndpoints += pDevice->pConfigList[pDevice->bCurrentConfig].pInterfaces[j].bNumEndpoints;
    assert(bNumEndpoints < 0xF);
    pDevice->HCDDevice.bNumEndpoints = bNumEndpoints;

    USB_InterfaceDesc* pCurrentInterface = pDevice->pConfigList[pDevice->bCurrentConfig].pInterfaces;
    int currentEP = 0;
    for(int8_t i = 0; i < bNumEndpoints; ++i)
    {
        const USB_EndpointDesc& epd = pCurrentInterface->pEndpoints[currentEP];
        pDevice->HCDDevice.pHCI->pHCDMethod->CreateEndpoint(&pDevice->HCDDevice, epd.bEndpointAddressFlags.Num,
            epd.bEndpointAddressFlags.Dir ? HCD_TXR : HCD_TXW,
            epd.bmAttributesFlags.TransferType,
            epd.wMaxPacketSizeFlags.Size,
            epd.bInterval);    //create EP
        if(++currentEP >= pCurrentInterface->bNumEndpoints)
        {
            ++pCurrentInterface;
            currentEP = 0;
        }
    }
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