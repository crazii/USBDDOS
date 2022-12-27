#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <malloc.h>
#include <ctype.h>
#include <conio.h>
#include "USBDDOS/CLASS/HID.H"
#include "USBDDOS/DPMI/DPMI.H"
#include "USBDDOS/PIC.H"
#include "USBDDOS/DBGUTIL.H"

//usb keycodes to scan codes and ascii
//ref: USB HID to PS/2 Scan Code Translation Table
//https://download.microsoft.com/download/1/6/1/161ba512-40e2-4cc9-843a-923143f3456c/translate.pdf
static uint8_t USB_HID_KEYBOARD_USAGE2SCANCODES[256*2] = 
{
    0, 0,           //reserved
    0, 0,           //RollOver
    0, 0,           //POSTFail
    0, 0,           //Unddefined
    0x1E, 0,
    0x30, 0,
    0x2E, 0,
    0x20, 0,
    0x12, 0,
    0x21, 0,
    0x22, 0,
    0x23, 0,
    0x17, 0,
    0x24, 0,
    0x25, 0,
    0x26, 0,
    0x32, 0,
    0x31, 0,
    0x18, 0,
    0x19, 0,
    0x10, 0,
    0x13, 0,
    0x1F, 0,
    0x14, 0,
    0x16, 0,
    0x2F, 0,
    0x11, 0,
    0x2D, 0,
    0x15, 0,
    0x2C, 0,
    0x02, 0,
    0x03, 0,
    0x04, 0,
    0x05, 0,
    0x06, 0,
    0x07, 0,
    0x08, 0,
    0x09, 0,
    0x0A, 0,
    0x0B, 0,
    0x1C, 0,   //ENTER
    0x01, 0,    //ESC
    0x0E, 0,   //BACKSPACE
    0x0F, 0,    //TAB
    0x39, 0,    //SPACE
    0x0C, 0,
    0x0D, 0,
    0x1A, 0,
    0x1B, 0,
    0x2B, 0,
    0x2B, 0,    //EUROPE 1 (NOTE2)
    0x27, 0,
    0x28, 0,
    0x29, 0,
    0x33, 0,
    0x34, 0,
    0x35, 0,
    0x3A, 0,    //CAPSLOCK
    0x3B, 0,    //F1
    0x3C, 0,    //F2
    0x3D, 0,    //F3
    0x3E, 0,    //F4
    0x3F, 0,    //F5
    0x40, 0,    //F6
    0x41, 0,    //F7
    0x42, 0,    //F8
    0x43, 0,    //F9
    0x44, 0,    //F10
    0x57, 0,    //F11
    0x58, 0,    //F12

    0x37, 0,    //PrScrn, E0 37
    0x46, 0,    //SCROLLLOCK
    0x1D, 0xE1,   //PAUSE, E1 1D 45
    0x52, 0xE0,    //INSERT
    0x47, 0xE0,    //HOME
    0x49, 0xE0,    //PGUP
    0x53, 0xE0,    //DEL
    0x4F, 0xE0,    //END
    0x51, 0xE0,    //PGDOWN
    0x4D, 0xE0,    //RIGHT ARROW
    0x4B, 0xE0,    //LEFT ARROW
    0x50, 0xE0,    //DOWN ARROW
    0x48, 0xE0,    //UP ARROW

    0x45, 0,    //NUMLOCK
    0x35, 0xE0, //KEYPAD/, E0 35
    0x37, 0,    //KEYPAD*
    0x4A, 0,    //KEYPAD-
    0x4E, 0,    //KEYPAD+
    0x1C, 0xE0, //KEYPAD ENTER, E0 1C
    0x4F, 0,    //KEYPAD1
    0x50, 0,    //KEYPAD2
    0x51, 0,    //KEYPAD3
    0x4B, 0,    //KEYPAD4
    0x4C, 0,    //KEYPAD5
    0x4D, 0,    //KEYPAD6
    0x47, 0,    //KEYPAD7
    0x48, 0,    //KEYPAD8
    0x49, 0,    //KEYPAD9
    0x52, 0,    //KEYPAD0
    0x53, 0,    //KEYPAD.
    0x56, 0,    //EUROPE 2 (NOTE 2)
    0x5D, 0xE0, //APP, E0 5D
    0x5E, 0xE0, //KB POWER, E0 5E
    0x59, 0,    //KEYPAD=
};

//TODO: INT 16h, ah=03h, al=06h: typematic rate and delay //http://mirror.cs.msu.ru/oldlinux.org/Linux.old/docs/interrupts/int-html/rb-1757.htm
#define USB_HID_KEYBOARD_REPEAT_DELAY   300     //300ms
#define USB_HID_KEYBOARD_REPEAT         50      //repeat interval

#define USB_HID_BIOS_SCRLLOCK_S 0x0010 //scroll locked staus
#define USB_HID_BIOS_NUMLOCK_S  0x0020 //num locked staus
#define USB_HID_BIOS_CAPSLOCK_S 0x0040 //caps locked staus
#define USB_HID_BIOS_MMASK      0x0070

#define WAIT_KEYBOARD_IN_EMPTY() while((inp(0x64)&2))
#define WAIT_KEYBOARD_OUT_EMPTY() while((inp(0x64)&1)) {STI();NOP();CLI();}//USB_IdleWait()

//keyboard device input processing
static BOOL USB_HID_Keyboard_IsInputEmpty(const USB_HID_Data* data);
static int USB_HID_Keyboard_GetInputCount(const USB_HID_Data* data);
static int USB_HID_Keyboard_CompareInput(const USB_HID_Data* data1, const USB_HID_Data* data2);
static int USB_HID_Keyboard_FindNewInput(const USB_HID_Data* data1, const USB_HID_Data* data2); //find new input in data1, return -1 if not found
static BOOL USB_HID_Keyboard_SetupLED(USB_Device* pDevice); //updae LED and BIOS modifier
//keyboard input record management
static BOOL USB_HID_Keyboard_PushRecord(USB_HID_Interface* keyboard, uint8_t KeyIndex);
static uint8_t USB_HID_Keyboard_TopRecord(const USB_HID_Interface* keyboard);
static BOOL USB_HID_Keyboard_RemoveRecord(USB_HID_Interface* keyboard, uint8_t KeyIndex);

static void USB_HID_Keyboard_GenerateKey(uint8_t scancode);
static void USB_HID_Keyboard_Finalizer(void* data);
static void USB_HID_Mouse_GenerateSample(uint8_t byte);
static void USB_HID_Mouse_Finalizer(void* data);
//driver routines
static void USB_HID_DummyCallback(HCD_Request* pRequest) {}
static void USB_HID_InputCallback(HCD_Request* pRequest);
static void USB_HID_InputKeyboard(USB_Device* pDevice);
static void USB_HID_InputMouse(USB_Device* pDevice);

BOOL USB_HID_InitDevice(USB_Device* pDevice)
{
    uint8_t bNumInterfaces = pDevice->pConfigList[pDevice->bCurrentConfig].bNumInterfaces;
    USB_InterfaceDesc* pIntfaceDesc0 = pDevice->pConfigList[pDevice->bCurrentConfig].pInterfaces;

    uint16_t DescLength = pDevice->pConfigList[pDevice->bCurrentConfig].wTotalLength;
    uint8_t* pDescBuffer = (uint8_t*)DPMI_DMAMalloc(DescLength, 4);
    uint8_t result = USB_GetConfigDescriptor(pDevice, pDescBuffer, DescLength);
    if(result)
    {
        DPMI_DMAFree(pDescBuffer);
        return FALSE;
    }
    
    USB_HID_DriverData* pDriverData = (USB_HID_DriverData*)malloc(sizeof(USB_HID_DriverData));
    memset(pDriverData, 0, sizeof(USB_HID_DriverData));

    int valid = 0;
    for(int j = 0; j < bNumInterfaces; ++j)
    {
        USB_InterfaceDesc* pIntfaceDesc = pIntfaceDesc0 + j;
        _LOG("HID bInterfaceProtocol: %x\n", pIntfaceDesc->bInterfaceProtocol);
        _LOG("HID bSubClass: %x\n", pIntfaceDesc->bInterfaceSubClass);
        if(pIntfaceDesc->bInterfaceClass == USBC_HID
            && pIntfaceDesc->bInterfaceSubClass == USB_HIDSC_BOOT_INTERFACE
            && (pIntfaceDesc->bInterfaceProtocol == USB_HIDP_KEYBOARD || pIntfaceDesc->bInterfaceProtocol == USB_HIDP_MOUSE) )
        {
            int index = pIntfaceDesc->bInterfaceProtocol - 1;
            assert(index >= USB_HID_KEYBOARD && index <= USB_HID_MOUSE);
            USB_HID_Interface* DrvIntface = &pDriverData->Interface[index];
            DrvIntface->bInterface = pIntfaceDesc->bInterfaceNumber;
            
            for(int i = 0; i < pIntfaceDesc->bNumEndpoints; ++i)
            {
                USB_EndpointDesc* pEndpointDesc = pIntfaceDesc->pEndpoints + i;
                assert(pEndpointDesc->bmAttributesBits.TransferType == USB_ENDPOINT_TRANSFER_TYPE_INTR); //interrupt only
                DrvIntface->pDataEP[pEndpointDesc->bEndpointAddressBits.Dir] = USB_FindEndpoint(pDevice, pEndpointDesc);
                DrvIntface->bEPAddr[pEndpointDesc->bEndpointAddressBits.Dir] = pEndpointDesc->bEndpointAddress;
                if(pEndpointDesc->bEndpointAddressBits.Dir)
                    DrvIntface->Interval = pEndpointDesc->bInterval&0x7FU;
            }
            //_LOG("%x %x\n", DrvIntface->pDataEP[0], DrvIntface->pDataEP[1]);

            uint8_t* desc = pDescBuffer + pIntfaceDesc->offset + pIntfaceDesc->bLength;
            while(*(desc+1) != USB_DT_INTERFACE && desc + *desc < pDescBuffer + DescLength) //until next interface or to the end
            {
                if(*(desc+1) == USB_DT_HID)
                {
                    uint8_t length = *desc;
                    DrvIntface->Descirptors = (USB_HID_DESC*)malloc(length); //variable length, need malloc
                    memcpy(DrvIntface->Descirptors, desc, length);
                }
                desc += *desc;
            }

            //set boot protocol
            _LOG("HID: Set boot protocol\n");
            USB_Request req = {USB_REQ_WRITE | USB_REQ_TYPE_HID, USB_REQ_HID_SET_PROTOCOL, USB_HID_PROTOCOL_BOOT, (uint16_t)DrvIntface->bInterface, 0};
            if(DrvIntface->pDataEP[HCD_TXR] != NULL && USB_SyncSendRequest(pDevice, &req, NULL) == 0)
                ++valid;

            //bye default the device will send data even if no input data change, set_idle will make it only sending data only changes, i.e. keydown/keyup
            //maybe we can use the frequent interrupt to implement 'repeating'. otherwise we need a timer to implement it.
            //now use non-idle to implement repeating, will save timer irq and DPMI RMCB, but use USB badwidth when a key is long pressed 
            //for an 10ms rate keyboard, the bandwith will be 8*100=800 bytes per second
            _LOG("HID Set idle\n");
            USB_Request req2 = {USB_REQ_WRITE | USB_REQ_TYPE_HID, USB_REQ_HID_SET_IDLE, USB_HID_MAKE_IDLE(USB_HID_IDLE_INDEFINITE, USB_HID_IDLE_REPORTALL) /*indefinite until input detected, all reports*/, (uint16_t)DrvIntface->bInterface, 0};
            USB_SyncSendRequest(pDevice, &req2, NULL);
            DrvIntface->Idle = TRUE;
        }
    }
    DPMI_DMAFree(pDescBuffer);

    #if DEBUG
    if(pDriverData->Interface[0].Descirptors)
        DBG_DumpB((uint8_t*)pDriverData->Interface[0].Descirptors, pDriverData->Interface[0].Descirptors->bLength, NULL);
    if(pDriverData->Interface[1].Descirptors)
        DBG_DumpB((uint8_t*)pDriverData->Interface[1].Descirptors, pDriverData->Interface[1].Descirptors->bLength, NULL);
    #endif
    if(valid > 0)
        pDevice->pDriverData = pDriverData;
    else
        free(pDriverData);
    return valid > 0;
}

BOOL USB_HID_DOS_Install()
{
    int count = 0;
    for(int j = 0; j < USBT.HC_Count; ++j)
    {
        HCD_Interface* pHCI = USBT.HC_List+j;

        for(int i = 0; i < HCD_MAX_DEVICE_COUNT; ++i)
        {
            if(!HCD_IS_DEVICE_VALID(pHCI->DeviceList[i]))
                continue;
            USB_Device* pDevice = HC2USB(pHCI->DeviceList[i]);
            if(pDevice->Desc.bDeviceClass == USBC_HID && pDevice->bStatus == DS_Ready)
            {
                ++count;
                USB_HID_DriverData* pDriverData = (USB_HID_DriverData*)pDevice->pDriverData;

                if(pDriverData->Interface[USB_HID_KEYBOARD].Descirptors != NULL)
                    USB_HID_Keyboard_SetupLED(pDevice); //initial setup of LED. not working for tested keyboard, still dunno why

                _LOG("HID: start driver.\n");
                //start input, aynsc (interrupt)
                for(int i = 0; i < 2; ++i)
                {
                    if(pDriverData->Interface[i].Descirptors && pDriverData->Interface[i].pDataEP[HCD_TXR])
                    {
                        printf("Found USB %s: %s\n", i == 0 ? "keyboard" : "mouse", pDevice->sProduct);
                        USB_Transfer(pDevice, pDriverData->Interface[i].pDataEP[HCD_TXR], pDriverData->Interface[i].Data[0].Buffer, sizeof(USB_HID_Data), &USB_HID_InputCallback, (void*)i);
                    }
                }
            }
        }
    }
    return count > 0;
}

BOOL USB_HID_DOS_Uninstall()
{
    return FALSE;
}

BOOL USB_HID_DeinitDevice(USB_Device* pDevice)
{
    USB_HID_DriverData* pDriverData = (USB_HID_DriverData*)pDevice->pDriverData;
    if(pDriverData)
    {
        if(pDriverData->Interface[USB_HID_KEYBOARD].Descirptors)
            free(pDriverData->Interface[USB_HID_KEYBOARD].Descirptors);
        if(pDriverData->Interface[USB_HID_MOUSE].Descirptors)
            free(pDriverData->Interface[USB_HID_MOUSE].Descirptors);
        free(pDriverData);
    }
    pDevice->pDriverData = NULL;
    return FALSE;
}

void USB_HID_PreInit()
{
    USB_HID_KEYBOARD_USAGE2SCANCODES[0x9A*2] = 0;  //sys req

    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE0*2] = 0x1D;  //left control
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE1*2] = 0x2A;  //left shift
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE2*2] = 0x38;  //left alt
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE3*2] = 0x5B;  //left gui, E0 5B
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE3*2+1] = 0xE0;
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE4*2] = 0x1D;  //right control, E0 1D
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE4*2+1] = 0xE0;
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE5*2] = 0x36;  //right shift
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE6*2] = 0x38;  //right alt
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE6*2+1] = 0xE0;  //right alt
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE7*2] = 0x5C;  //right gui
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE7*2+1] = 0xE0;  //right gui
}

void USB_HID_PostDeInit()
{

}

static BOOL USB_HID_Keyboard_IsInputEmpty(const USB_HID_Data* data)
{
    //static char empty[8] = {0};
    //return /*data->Key.modifier == 0 && */memcmp(&data->Key.Keycodes[0], &empty[0], 6) == 0; //treat pure modifier keys are empty, so no repeating for them
    const uint32_t* p = (const uint32_t*)&data->Key;
    return (p[0]&0xFFFF0000) == 0 && p[1] == 0;
}

static int USB_HID_Keyboard_GetInputCount(const USB_HID_Data* data)
{
    int count = 0;
    for(int i = 0; i < 6; ++i)
        count += (data->Key.Keycodes[i] != 0) ? 1 : 0;
    return count;
}

static int USB_HID_Keyboard_CompareInput(const USB_HID_Data* data1, const USB_HID_Data* data2)
{
    int val = (int)data1->Key.Modifier - (int)data2->Key.Modifier;
    //return val != 0 ? val : memcmp(&data1->Key.Keycodes[0], &data2->Key.Keycodes[0], sizeof(data1->Key.Keycodes));
    if(val != 0)
        return val;
    USB_HID_Data d = *data2;
    for(int i = 0; i < 6; ++i)
    {
        uint8_t input = data1->Key.Keycodes[i];
        if(input == 0)
            continue;
        int found;
        for(int j = 0; j < 6; ++j)
        {
            if((found=(input == d.Key.Keycodes[j])))
            {
                d.Key.Keycodes[j] = 0;
                break;
            }
        }
        if(!found)
            return 1;
    }
    return USB_HID_Keyboard_IsInputEmpty(&d) ? 0 : -1;
}

static int USB_HID_Keyboard_FindNewInput(const USB_HID_Data* data1, const USB_HID_Data* data2)
{
    for(int i = 0; i < 6; ++i)
    {
        uint8_t input = data1->Key.Keycodes[i];
        if(input == 0)
            continue;
        int found;
        for(int j = 0; j < 6; ++j)
        {
            if((found=(input == data2->Key.Keycodes[j])))
                break;
        }
        if(!found)
            return input;
    }
    return -1;
}

static BOOL USB_HID_Keyboard_SetupLED(USB_Device* pDevice)
{
    if(pDevice == NULL)
        return FALSE;
    USB_HID_DriverData* pDriverData = (USB_HID_DriverData*)pDevice->pDriverData;
    if(pDriverData == NULL)
        return FALSE;
    uint16_t BiosModifier = DPMI_LoadW(DPMI_SEGOFF2L(0x40,0x17)); //bios data area: 16 bit modifier https://stanislavs.org/helppc/bios_data_area.html
    if(!((BiosModifier^pDriverData->Interface[USB_HID_KEYBOARD].BIOSModifier)&USB_HID_BIOS_MMASK))
        return TRUE;

    int LED = 0;
    if(BiosModifier&USB_HID_BIOS_SCRLLOCK_S) LED |= USB_HID_LED_SCROLLLOCK;
    if(BiosModifier&USB_HID_BIOS_CAPSLOCK_S) LED |= USB_HID_LED_CAPSLOCK;
    if(BiosModifier&USB_HID_BIOS_NUMLOCK_S) LED |= USB_HID_LED_NUMLOCK;
    pDriverData->Interface[USB_HID_KEYBOARD].BIOSModifier = BiosModifier;

    USB_Request req = {USB_REQ_WRITE | USB_REQ_TYPE_HID, USB_REQ_HID_SET_REPORT, USB_HID_MAKE_REPORT(USB_HID_REPORT_OUTPUT, 0), (uint16_t)pDriverData->Interface[USB_HID_KEYBOARD].bInterface, 1}; //1 byte data
    USB_SendRequest(pDevice, &req, &LED, USB_HID_DummyCallback, NULL);
    return TRUE;
}


static BOOL USB_HID_Keyboard_PushRecord(USB_HID_Interface* keyboard, uint8_t KeyIndex)
{
    if(keyboard->RecordCount < 6)
    {
        keyboard->Records[keyboard->RecordCount++] = KeyIndex;
        return TRUE;
    }
    return FALSE;
}

static uint8_t USB_HID_Keyboard_TopRecord(const USB_HID_Interface* keyboard)
{
    if(keyboard->RecordCount == 0)
        return 0;
    return keyboard->Records[keyboard->RecordCount-1];
}

static BOOL USB_HID_Keyboard_RemoveRecord(USB_HID_Interface* keyboard, uint8_t KeyIndex)
{
    for(int i = 0; i < keyboard->RecordCount; ++i)
    {
        if(keyboard->Records[i] == KeyIndex)
        {
            for(int j = i; j < keyboard->RecordCount-1; ++j)
                keyboard->Records[i] = keyboard->Records[i+1];
            --keyboard->RecordCount;
            return TRUE;
        }
    }
    return FALSE;
}

static void USB_HID_Keyboard_GenerateKey(uint8_t scancode)
{
    //putting the key to BIOS keyboard buffer works for applications that use BIOS function (int 16h) to access the keyboard
    //but a lot program doesn't do that. instead, they read the keyborad port
    //so here is another method to fake keyboard input using port IO, from another USB driver made years ago:
    //https://bretjohnson.us/
    //other refs:
    //https://wiki.osdev.org/%228042%22_PS/2_Controller

    //note: USb_IdleWait() will temporarily enable interrupt and let IRQ1 handled
    //so that the 2nd scancode after prefix can write to the port
    //the code of https://bretjohnson.us/ didn't do that so it cannot send 2 bytes with prefix
    //we don't need to do special fix Numlock for [HOME,UP ARROW] etc.

    //method 1 from https://bretjohnson.us/
#if !defined(__BC__)
    WAIT_KEYBOARD_IN_EMPTY();
    outp(0x64, 0xD2);   //write to out buffer (fake input)
    WAIT_KEYBOARD_IN_EMPTY();
    outp(0x60, scancode);
    WAIT_KEYBOARD_IN_EMPTY();
    WAIT_KEYBOARD_OUT_EMPTY();
#else
    //method 2. worked if put in finalizer
    PIC_MaskIRQ(1);
    WAIT_KEYBOARD_IN_EMPTY();
    outp(0x64, 0x60);   //write KCCB
    WAIT_KEYBOARD_IN_EMPTY();
    outp(0x60, scancode);
    WAIT_KEYBOARD_IN_EMPTY();
    outp(0x64, 0x20);   //KCCB to port 60h
    WAIT_KEYBOARD_IN_EMPTY();
    outp(0x64, 0x60);   //write KCCB
    WAIT_KEYBOARD_IN_EMPTY();
    outp(0x60, 0x45);   //return normal mode
    DPMI_REG r = {0};
    DPMI_CallRealModeINT(0x09, &r); //call kbd irq
    PIC_UnmaskIRQ(1);
#endif
}

static void USB_HID_Keyboard_Finalizer(void* data)
{
    uint32_t bytes = (uint32_t)(uintptr_t)data;
    uint8_t scancode = (uint8_t)bytes;
    uint8_t prefix = (uint8_t)(bytes>>8);

    uint16_t mask = PIC_GetIRQMask();
    PIC_SetIRQMask(PIC_IRQ_UNMASK(0xFFFF,1));

    if(prefix)
        USB_HID_Keyboard_GenerateKey(prefix);
    USB_HID_Keyboard_GenerateKey(scancode);

    if(prefix == 0xE0 && scancode == 0x46)
    { //Ctrl + Break 0xE0 0x46 0xE0 0xC6
        USB_HID_Keyboard_GenerateKey(0xE0);
        USB_HID_Keyboard_GenerateKey(0xC6);
    }

    PIC_SetIRQMask(mask);
}

void USB_HID_Mouse_GenerateSample(uint8_t byte)
{
    WAIT_KEYBOARD_IN_EMPTY();
    outp(0x64, 0xD3);   //write to out buffer (fake input)
    WAIT_KEYBOARD_IN_EMPTY();
    outp(0x60, byte);
    WAIT_KEYBOARD_IN_EMPTY();
    WAIT_KEYBOARD_OUT_EMPTY();
}

void USB_HID_Mouse_Finalizer(void* data)
{
    USB_HID_Data* hiddata = (USB_HID_Data*)data;
    //https://wiki.osdev.org/Mouse_Input

    int status = (hiddata->Mouse.Button&0x7) | 0x08;
    status |= hiddata->Mouse.DX < 0 ? 0x10 : 0;
    status |= hiddata->Mouse.DY > 0 ? 0x20 : 0;

    //PIC_MaskIRQ(1); //disable keyboard interrupt
    //note: the 3 bytes must be all sent to mouse irq or all the successive mouse data will be corrupted
    //keyboard irq need be disabled for sure because kbd & mouse use the same port
    //but any hooked irq handler may read the port (timer?), i.e. mouse become random when play Warcraft2
    //so disable them all
    uint16_t mask = PIC_GetIRQMask();
    //PIC_SetIRQMask(PIC_IRQ_UNMASK(PIC_IRQ_UNMASK(0xFFFF,12),2));
    PIC_SetIRQMask((uint16_t)(~0x1004U)); //only enable IRQ12 & PIC slave
    USB_HID_Mouse_GenerateSample((uint8_t)status);
    USB_HID_Mouse_GenerateSample((uint8_t)hiddata->Mouse.DX);
    USB_HID_Mouse_GenerateSample((uint8_t)(-hiddata->Mouse.DY));
    //PIC_UnmaskIRQ(1);
    PIC_SetIRQMask(mask);
}

static void USB_HID_InputCallback(HCD_Request* pRequest)
{
    USB_Device* pDevice = HC2USB(pRequest->pDevice);
    USB_HID_DriverData* pDriverData = (USB_HID_DriverData*)pDevice->pDriverData;

    int mode = (int)(uintptr_t)pRequest->pCBData;
    if(mode == USB_HID_KEYBOARD)
        USB_HID_InputKeyboard(pDevice);
    else
        USB_HID_InputMouse(pDevice);

    //continue input, async don't wait
    int index = (pDriverData->Interface[mode].Index+1)&0x1;
    pDriverData->Interface[mode].Index = (uint8_t)index&0x1U;
    USB_Transfer(pDevice, pDriverData->Interface[mode].pDataEP[HCD_TXR], pDriverData->Interface[mode].Data[index].Buffer, sizeof(USB_HID_Data), &USB_HID_InputCallback, (void*)mode);
}

static void USB_HID_InputKeyboard(USB_Device* pDevice)
{
    USB_HID_DriverData* pDriverData = (USB_HID_DriverData*)pDevice->pDriverData;
    USB_HID_Interface* kbd = &pDriverData->Interface[USB_HID_KEYBOARD];
    USB_HID_Data* data = &kbd->Data[kbd->Index];
    USB_HID_Data* prev = &kbd->Data[(kbd->Index+1)&0x1];
    BOOL empty = USB_HID_Keyboard_IsInputEmpty(data);

    //change idle state
    if(!empty && kbd->Idle) //stop idle if there'is key input, we use the non-idle interrupt to do repeating.
    {
        USB_Request req = {USB_REQ_TYPE_HID, USB_REQ_HID_SET_IDLE, USB_HID_MAKE_IDLE(1L, USB_HID_IDLE_REPORTALL) /*minimal*/, (uint16_t)kbd->bInterface, 0};
        USB_SendRequest(pDevice, &req, NULL, USB_HID_DummyCallback, NULL);
        kbd->Idle = FALSE;
    }
    else if(empty && !kbd->Idle) //start idle if no input (no interrupts)
    {
        USB_Request req = {USB_REQ_TYPE_HID, USB_REQ_HID_SET_IDLE, USB_HID_MAKE_IDLE(USB_HID_IDLE_INDEFINITE, USB_HID_IDLE_REPORTALL) /*indefinite until input detected, all reports*/, (uint16_t)kbd->bInterface, 0};
        USB_SendRequest(pDevice, &req, NULL, USB_HID_DummyCallback, NULL);
        kbd->Idle = TRUE;
    }

    USB_HID_Keyboard_SetupLED(pDevice);
    
    int count = USB_HID_Keyboard_GetInputCount(data);
    int PrevCount = kbd->PrevCount;
    kbd->PrevCount = (uint8_t)count;

    //_LOG("delay timer: %d, rpt timer: %d\n", kbd->DelayTimer, kbd->RepeatingTimer);
    static const int KEYOUT_DOWN = 1;
    static const int KEYOUT_REPEAT = 2;

    int DoOut = 0;
    BOOL InputChanged = (count != PrevCount) || (USB_HID_Keyboard_CompareInput(prev, data) != 0);
    if(InputChanged) //input changed
    {
        kbd->DelayTimer = 0;
        kbd->RepeatingTimer = 0;
        DoOut = KEYOUT_DOWN;
    }
    else
    {
        if(kbd->DelayTimer >= USB_HID_KEYBOARD_REPEAT_DELAY)
        {
            kbd->RepeatingTimer = (uint16_t)(kbd->RepeatingTimer+kbd->Interval);
            if(kbd->RepeatingTimer >= USB_HID_KEYBOARD_REPEAT)
            {
                DoOut = KEYOUT_REPEAT;
                kbd->RepeatingTimer = 0;
            }
        }                
        else
            kbd->DelayTimer = (uint16_t)(kbd->DelayTimer+kbd->Interval);
    }
    if(!DoOut || (!InputChanged && empty))
        return;

    #if 0
    if(InputChanged)
    {
        DBG_DumpB(data->Key.Keycodes, 6, NULL);
        DBG_DumpB(prev->Key.Keycodes, 6, NULL);
    }
    #endif
    int index = InputChanged ?
        (count < PrevCount ? USB_HID_Keyboard_FindNewInput(prev, data) : USB_HID_Keyboard_FindNewInput(data, prev))
        : USB_HID_Keyboard_TopRecord(kbd); //compare to find the change. by the spec, the new downed key is not always the first

    if(index == -1) //only modifier changed. seems that modifier keys won't get to keycodes but only the modifier mask.
    {
        uint8_t modifier = prev->Key.Modifier^data->Key.Modifier;
        if(modifier != 0)
        {
            uint32_t bitIndex = BSF((uint32_t)modifier);
            index = 0xE0 + (int)bitIndex; //modifier key inited in USB_HID_PreInit. the bit order is the same order in the table.
        }
        //else _LOG("ERROR");
        if(prev->Key.Modifier&modifier) //prev exist, now cleared. fake break (PrevCount > count)
            PrevCount = count + 1;
    }

    uint8_t scancode = USB_HID_KEYBOARD_USAGE2SCANCODES[index*2];
    uint8_t prefix = USB_HID_KEYBOARD_USAGE2SCANCODES[index*2+1];
    if(count > PrevCount) //key make
        USB_HID_Keyboard_PushRecord(kbd, (uint8_t)index);
    else if(count < PrevCount) //key break
    {
        USB_HID_Keyboard_RemoveRecord(kbd, (uint8_t)index); //remove key record
        scancode = (uint8_t)(scancode + 0x80);
    }

    //_LOG("%d %02x %02x", index, prefix, scancode);

    if(prefix == 0xE1) //pause/break. handle ctrl+break
    {
        if(scancode == 0x1D+0x80 || !(data->Key.Modifier&(USB_HID_LCTRL|USB_HID_RCTRL)) || DoOut == KEYOUT_REPEAT) //key up, or no ctrl pressed, or repeat
            return;
        prefix = 0xE0;
        scancode = 0x46;
    }
        
    //IRQ1 handler will send EOI and conflict with current interrupt handler.
    //need send our EOI first. add support to custom finalize function to do key out after USB EOI
    USB_ISR_Finalizer* finalizer = (USB_ISR_Finalizer*)malloc(sizeof(USB_ISR_Finalizer));
    finalizer->FinalizeISR = USB_HID_Keyboard_Finalizer;
    finalizer->data = (void*)(uintptr_t)((prefix<<8) | scancode);
    USB_ISR_AddFinalizer(finalizer);
}

static void USB_HID_InputMouse(USB_Device* pDevice)
{
    USB_HID_DriverData* pDriverData = (USB_HID_DriverData*)pDevice->pDriverData;
    USB_HID_Interface* mouse = &pDriverData->Interface[USB_HID_MOUSE];
    USB_HID_Data* data = &mouse->Data[mouse->Index];

    //already set indefinite idle, but idle is optional for boot mouse
    if((data->Mouse.Button&0x7) == 0 && data->Mouse.DX == 0 && data->Mouse.DY == 0)
        return;

    USB_ISR_Finalizer* finalizer = (USB_ISR_Finalizer*)malloc(sizeof(USB_ISR_Finalizer));
    finalizer->FinalizeISR = USB_HID_Mouse_Finalizer;
    finalizer->data = data;
    USB_ISR_AddFinalizer(finalizer);
}
