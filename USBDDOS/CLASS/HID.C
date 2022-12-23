#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <malloc.h>
#include <ctype.h>
#include "USBDDOS/CLASS/HID.H"
#include "USBDDOS/DPMI/DPMI.H"
#include "USBDDOS/DBGUTIL.H"

#define USB_HID_HOOK_INT16h 0

//usb keycodes to scan codes and ascii
//ref: USB HID to PS/2 Scan Code Translation Table
//https://download.microsoft.com/download/1/6/1/161ba512-40e2-4cc9-843a-923143f3456c/translate.pdf
static uint8_t USB_HID_KEYBOARD_USAGE2SCANCODES[256*3] = 
{
    0, 0, 0,       //reserved
    0, 0, 0,      //RollOver
    0, 0, 0,      //POSTFail
    0, 0, 0,      //Unddefined
    0x1E, 'A', 'a',
    0x30, 'B', 'b',
    0x2E, 'C', 'c',
    0x20, 'D', 'd',
    0x12, 'E', 'e',
    0x21, 'F', 'f',
    0x22, 'G', 'g',
    0x23, 'H', 'h',
    0x17, 'I', 'i',
    0x24, 'J', 'j',
    0x25, 'K', 'k',
    0x26, 'L', 'l',
    0x32, 'M', 'm',
    0x31, 'N', 'n',
    0x18, 'O', 'o',
    0x19, 'P', 'p',
    0x10, 'Q', 'q',
    0x13, 'R', 'r',
    0x1F, 'S', 's',
    0x14, 'T', 't',
    0x16, 'U', 'u',
    0x2F, 'V', 'v',
    0x11, 'W', 'w',
    0x2D, 'X', 'x',
    0x15, 'Y', 'y',
    0x2C, 'Z', 'z',
    0x02, '1', '!',
    0x03, '2', '@',
    0x04, '3', '#',
    0x05, '4', '$',
    0x06, '5', '%',
    0x07, '6', '^',
    0x08, '7', '&',
    0x09, '8', '*',
    0x0A, '9', '(',
    0x0B, '0', ')',
    0x1C, '\r', '\r',   //ENTER
    0x01, 0,   0,       //ESC
    0x0E, '\b', '\b',   //BACKSPACE
    0x0F, '\t', 't',    //TAB
    0x39, ' ',  ' ',    //SPACE
    0x0C, '-',  '_',
    0x0D, '=',  '+',
    0x1A, '[',  '{',
    0x1B, ']',  '}',
    0x2B, '\\', '|',
    0x2B, '\\', '|',    //EUROPE 1 (NOTE2)
    0x27, ';', ':',
    0x28, '\'', '|',
    0x29, '`', '~',
    0x33, ',', '<',
    0x34, '.', '>',
    0x35, '/', '?',
    0x3A, 0, 0,    //CAPSLOCK
    0x3B, 0, 0,    //F1
    0x3C, 0, 0,    //F2
    0x3D, 0, 0,    //F3
    0x3E, 0, 0,    //F4
    0x3F, 0, 0,    //F5
    0x40, 0, 0,    //F6
    0x41, 0, 0,    //F7
    0x42, 0, 0,    //F8
    0x43, 0, 0,    //F9
    0x44, 0, 0,    //F10
    0x57, 0, 0,    //F11
    0x58, 0, 0,    //F12

    0x37, 0, 0,    //PrScrn, E0 37
    0x46, 0, 0,    //SCROLLLOCK
    0xC6, 0xE0, 0xE0,    //PAUSE, E0 46/E0 C6
    0x52, 0xE0, 0xE0,    //INSERT
    0x47, 0xE0, 0xE0,    //HOME
    0x49, 0xE0, 0xE0,    //PGUP
    0x53, 0xE0, 0xE0,    //DEL
    0x4F, 0xE0, 0xE0,    //END
    0x51, 0xE0, 0xE0,    //PGDOWN
    0x4D, 0xE0, 0xE0,    //RIGHT ARROW
    0x4B, 0xE0, 0xE0,    //LEFT ARROW
    0x50, 0xE0, 0xE0,    //DOWN ARROW
    0x48, 0xE0, 0xE0,    //UP ARROW

    0x45, 0, 0,    //NUMLOCK
    0x35, 0xE0, 0xE0, //KEYPAD/, E0 35
    0x37, '*', '*',    //KEYPAD*
    0x4A, '-', '-',    //KEYPAD-
    0x4E, '+', '+',    //KEYPAD+
    0x1C, 0xE0, 0xE0, //KEYPAD ENTER, E0 1C
    0x4F, '1', '1',    //KEYPAD1
    0x50, '2', '2',    //KEYPAD2
    0x51, '3', '3',    //KEYPAD3
    0x4B, '4', '4',    //KEYPAD4
    0x4C, '5', '5',    //KEYPAD5
    0x4D, '6', '6',    //KEYPAD6
    0x47, '7', '7',    //KEYPAD7
    0x48, '8', '8',    //KEYPAD8
    0x49, '9', '9',    //KEYPAD9
    0x52, '0', '0',    //KEYPAD0
    0x53, '.', '.',    //KEYPAD.
    0x56, 0, 0,    //EUROPE 2 (NOTE 2)
    0x5D, 0xE0, 0xE0, //APP, E0 5D
    0x5E, 0xE0, 0xE0, //KB POWER, E0 5E
    0x59, 0, 0,    //KEYPAD=
};

//TODO: INT 16h, ah=03h, al=06h: typematic rate and delay //http://mirror.cs.msu.ru/oldlinux.org/Linux.old/docs/interrupts/int-html/rb-1757.htm
#define USB_HID_KEYBOARD_REPEAT_DELAY   300     //300ms
#define USB_HID_KEYBOARD_REPEAT         50      //repeat interval

static BOOL USB_HID_Keyboard_IsDataEmpty(USB_HID_Data* data);
static int USB_HID_Keyboard_CompareInput(USB_HID_Data* data1, USB_HID_Data* data2);
static uint16_t USB_HID_Keyboard_CheckKeyDown(USB_HID_Data* data, uint8_t BIOSScanCode);
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
            assert(index >= 0 && index <= 1);
            USB_HID_Interface* DrvIntface = &pDriverData->Interface[index];
            DrvIntface->bInterface = pIntfaceDesc->bInterfaceNumber;
            
            for(int i = 0; i < pIntfaceDesc->bNumEndpoints; ++i)
            {
                USB_EndpointDesc* pEndpointDesc = pIntfaceDesc->pEndpoints + i;
                assert(pEndpointDesc->bmAttributesBits.TransferType == USB_ENDPOINT_TRANSFER_TYPE_INTR); //interrupt only
                DrvIntface->pDataEP[pEndpointDesc->bEndpointAddressBits.Dir] = USB_FindEndpoint(pDevice, pEndpointDesc);
                DrvIntface->bEPAddr[pEndpointDesc->bEndpointAddressBits.Dir] = pEndpointDesc->bEndpointAddress;
                if(pEndpointDesc->bEndpointAddressBits.Dir)
                    DrvIntface->Interval = pEndpointDesc->bInterval;
            }
            //_LOG("%x %x\n", DrvIntface->pDataEP[0], DrvIntface->pDataEP[1]);

            uint8_t* desc = pDescBuffer + pIntfaceDesc->offset + pIntfaceDesc->bLength;
            while(*(desc+1) != USB_DT_INTERFACE && desc + *desc <= pDescBuffer + DescLength) //until next interface or to the end
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
            USB_Request req = {USB_REQ_TYPE_HID, USB_REQ_HID_SET_PROTOCOL, USB_HID_PROTOCOL_BOOT, (uint16_t)DrvIntface->bInterface, 0};
            if(DrvIntface->pDataEP[HCD_TXR] != NULL && USB_SyncSendRequest(pDevice, &req, NULL) == 0)
                ++valid;

            //bye default the device will send data even if no input data change, set_idle will make it only sending data only changes, i.e. keydown/keyup
            //maybe we can use the frequent interrupt to implement 'repeating'. otherwise we need a timer to implement it.
            USB_Request req2 = {USB_REQ_TYPE_HID, USB_REQ_HID_SET_IDLE, USB_HID_MAKE_IDLE(USB_HID_IDLE_INDEFINITE, USB_HID_IDLE_REPORTALL) /*indefinite until input detected, all reports*/, (uint16_t)DrvIntface->bInterface, 0};
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

    //start input, aynsc (interrupt)
    for(int i = 0; i < 2; ++i)
    {
        if(pDriverData->Interface[i].Descirptors && pDriverData->Interface[i].pDataEP[HCD_TXR])
            USB_Transfer(pDevice, pDriverData->Interface[i].pDataEP[HCD_TXR], pDriverData->Interface[i].Data[0].Buffer, sizeof(USB_HID_Data), &USB_HID_InputCallback, (void*)i);
    }
    return valid > 0;
}

BOOL USB_HID_DOS_Install()
{
    return TRUE;
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
        if(pDriverData->Interface[0].Descirptors)
            free(pDriverData->Interface[0].Descirptors);
        if(pDriverData->Interface[1].Descirptors)
            free(pDriverData->Interface[1].Descirptors);
        free(pDriverData);
    }
    pDevice->pDriverData = NULL;
    return FALSE;
}

void USB_HID_PreInit()
{
    USB_HID_KEYBOARD_USAGE2SCANCODES[0x9A*3] = 0;  //sys req

    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE0*3] = 0x1D;  //left control
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE1*3] = 0x2A;  //left shift
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE2*3] = 0x38;  //left alt
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE3*3] = 0x5B;  //left gui, E0 5B
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE3*3+1] = 0xE0;
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE3*3+2] = 0xE0;
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE4*3] = 0x1D;  //left control, E0 1D
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE4*3+1] = 0xE0;
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE4*3+2] = 0xE0;
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE5*3] = 0x36;  //left shift
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE6*3] = 0x38;  //left alt
    USB_HID_KEYBOARD_USAGE2SCANCODES[0xE7*3] = 0x5C;  //left gui
}

void USB_HID_PostDeInit()
{

}


static BOOL USB_HID_Keyboard_IsDataEmpty(USB_HID_Data* data)
{
    static char empty[8] = {0};
    return data->Buffer[0] == 0 && memcmp(&data->Buffer[2], &empty[2], 6) == 0;
}

static int USB_HID_Keyboard_CompareInput(USB_HID_Data* data1, USB_HID_Data* data2)
{
    int val = (int)data1->Buffer[0] - (int)data2->Buffer[0];
    return val != 0 ? val : memcmp(&data1->Buffer[2], &data2->Buffer[2], sizeof(USB_HID_Data));
}

static uint16_t USB_HID_Keyboard_CheckKeyDown(USB_HID_Data* data, uint8_t BIOSScanCode)
{
    for(int i = 2; i < 8; ++i)
    {
        uint8_t index = data->Buffer[i];
        if(index >= 4 && USB_HID_KEYBOARD_USAGE2SCANCODES[index*2] == BIOSScanCode)
            return (uint16_t)((((uint16_t)USB_HID_KEYBOARD_USAGE2SCANCODES[index*2+1])<<8) | index);
    }
    return 0;
}

static void USB_HID_InputCallback(HCD_Request* pRequest)
{
    USB_Device* pDevice = HC2USB(pRequest->pDevice);
    USB_HID_DriverData* pDriverData = (USB_HID_DriverData*)pDevice->pDriverData;

    int mode = (int)(uintptr_t)pRequest->pCBData;
    if(mode == 0) //keyboard
        USB_HID_InputKeyboard(pDevice);
    else
        USB_HID_InputMouse(pDevice);

    //continue input, async don't wait
    int index = (pDriverData->Interface[mode].Index+1)&0x1;
    pDriverData->Interface[mode].Index = (uint8_t)index;
    USB_Transfer(pDevice, pDriverData->Interface[mode].pDataEP[HCD_TXR], pDriverData->Interface[mode].Data[index].Buffer, sizeof(USB_HID_Data), &USB_HID_InputCallback, (void*)mode);
}

static void USB_HID_InputKeyboard(USB_Device* pDevice)
{
    USB_HID_DriverData* pDriverData = (USB_HID_DriverData*)pDevice->pDriverData;
    USB_HID_Interface* kbd = &pDriverData->Interface[0];
    USB_HID_Data* data = &kbd->Data[kbd->Index];
    USB_HID_Data* prev = &kbd->Data[(kbd->Index+1)&0x1];
    BOOL empty = USB_HID_Keyboard_IsDataEmpty(data);

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
    if(empty)
        return;

    //_LOG("delay timer: %d, rpt timer: %d\n", kbd->DelayTimer, kbd->RepeatingTimer);
    BOOL DoOut = FALSE;
    if(USB_HID_Keyboard_CompareInput(prev, data) != 0) //input changed
    {
        kbd->DelayTimer = 0;
        kbd->RepeatingTimer = 0;
        DoOut = TRUE;
    }
    else
    {
        if(kbd->DelayTimer >= USB_HID_KEYBOARD_REPEAT_DELAY)
        {
            kbd->RepeatingTimer += kbd->Interval;
            if(kbd->RepeatingTimer > USB_HID_KEYBOARD_REPEAT)
            {
                DoOut = TRUE;
                kbd->RepeatingTimer = 0;
            }
        }                
        else
            kbd->DelayTimer += kbd->Interval;
    }
    if(!DoOut)
        return;

    uint16_t BIOSModifer = DPMI_LoadW(DPMI_SEGOFF2L(0x40,0x17)); //don't store modifer for each keyboard device, instead, sync to BIOS's flag

    BIOSModifer |= USB_HID_Keyboard_CheckKeyDown(data, 0x46) != 0 ? USB_HID_BIOS_SCRLLOCK : 0; //scroll lock
    BIOSModifer |= USB_HID_Keyboard_CheckKeyDown(data, 0x45) != 0 ? USB_HID_BIOS_NUMLOCK : 0; //scroll lock
    BIOSModifer |= USB_HID_Keyboard_CheckKeyDown(data, 0x3A) != 0 ? USB_HID_BIOS_CAPSLOCK : 0; //scroll lock
    //BIOSModifer |= USB_HID_Keyboard_CheckKeyDown(data, 0x00) != 0 ? USB_HID_BIOS_SYSREQ : 0; //scroll lock
    if((BIOSModifer&USB_HID_BIOS_SCRLLOCK) && USB_HID_Keyboard_CheckKeyDown(prev, 0x46) == 0) BIOSModifer ^= USB_HID_BIOS_SCRLLOCK_S;
    if((BIOSModifer&USB_HID_BIOS_NUMLOCK) && USB_HID_Keyboard_CheckKeyDown(prev, 0x45) == 0) BIOSModifer ^= USB_HID_BIOS_NUMLOCK_S;
    if((BIOSModifer&USB_HID_BIOS_CAPSLOCK) && USB_HID_Keyboard_CheckKeyDown(prev, 0x3A) == 0) BIOSModifer ^= USB_HID_BIOS_CAPSLOCK_S;
    uint16_t insert;
    if(USB_HID_Keyboard_CheckKeyDown(prev, 0x52) == 0 && (insert=USB_HID_Keyboard_CheckKeyDown(data, 0x52)) != 0) //insert
    {
        if((insert&0xE000) == 0xE000 || !(BIOSModifer&USB_HID_BIOS_NUMLOCK_S)) //"normal" insert, or insert in numpad with numlock off
            BIOSModifer ^= USB_HID_BIOS_INSLOCK_S;
    }
    if(data->Key.modifier&USB_HID_LCTRL) BIOSModifer |= USB_HID_BIOS_LCTRL | USB_HID_BIOS_CTRL;
    if(data->Key.modifier&USB_HID_LSHIFT) BIOSModifer |= USB_HID_BIOS_LSHIFT;
    if(data->Key.modifier&USB_HID_LALT) BIOSModifer |= USB_HID_BIOS_LALT | USB_HID_BIOS_ALT;
    if(data->Key.modifier&USB_HID_RCTRL) BIOSModifer |= USB_HID_BIOS_RCTRL | USB_HID_BIOS_CTRL;
    if(data->Key.modifier&USB_HID_RSHIFT) BIOSModifer |= USB_HID_BIOS_RSHIFT;
    if(data->Key.modifier&USB_HID_RALT) BIOSModifer |= USB_HID_BIOS_RALT | USB_HID_BIOS_ALT;
    //TODO: update LED
    
    int index = data->Key.keycodes[0]*2; //TODO: compare to find the change. by the spec, the new downed key is not always the first
    int offset = 0;
    uint8_t scancode = USB_HID_KEYBOARD_USAGE2SCANCODES[index++];

    //handle shift
    if(BIOSModifer&(USB_HID_BIOS_RSHIFT|USB_HID_BIOS_LSHIFT))
        ++offset;
    //handle caps lock
    if(!(BIOSModifer&USB_HID_BIOS_CAPSLOCK_S)  && isalpha(USB_HID_KEYBOARD_USAGE2SCANCODES[index + offset]))
        offset = 1 - offset;

    uint8_t ascii = USB_HID_KEYBOARD_USAGE2SCANCODES[index + offset];
    
    //handle numlock
    if(!(BIOSModifer&USB_HID_BIOS_NUMLOCK_S) && scancode >=0x47 && scancode <= 0x53 && ascii !='5' && (ascii == '.' || isdigit(ascii)))
        ascii = 0;

    //put directly to BIOS keyboard buffer
    //we are modifying the buffer int Int handler so it is safe.
    uint16_t head = DPMI_LoadW(DPMI_SEGOFF2L(0x40,0x1A));
    uint16_t tail = DPMI_LoadW(DPMI_SEGOFF2L(0x40,0x1C));
    if((tail+2-0x1E) % 32 != head - 0x1E)
    {
        DPMI_StoreW(DPMI_SEGOFF2L(0x40,tail), (uint16_t)(((uint16_t)scancode<<8) | ascii));
        tail = (uint16_t)(((tail+2 - 0x1E) % 32) + 0x1E);
        DPMI_StoreW(DPMI_SEGOFF2L(0x40,0x1C), tail);
    }
}

static void USB_HID_InputMouse(USB_Device* pDevice)
{

}
