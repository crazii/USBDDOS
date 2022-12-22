#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <malloc.h>
#include <ctype.h>
#include "USBDDOS/CLASS/HID.H"
#include "USBDDOS/DPMI/DPMI.H"
#include "USBDDOS/DBGUTIL.H"

//usb keycodes to scan codes and ascii
//ref: USB HID to PS/2 Scan Code Translation Table
//https://download.microsoft.com/download/1/6/1/161ba512-40e2-4cc9-843a-923143f3456c/translate.pdf
static uint8_t KEY_USBCODES2SCANCODES[256*2] = 
{
    0, 0,       //reserved
    0, 0,       //RollOver
    0, 0,       //POSTFail
    0, 0,       //Unddefined
    0x1E, 'A',
    0x30, 'B',
    0x2E, 'C',
    0x20, 'D',
    0x12, 'E',
    0x21, 'F',
    0x22, 'G',
    0x23, 'H',
    0x17, 'I',
    0x24, 'J',
    0x25, 'K',
    0x26, 'L',
    0x32, 'M',
    0x31, 'N',
    0x18, 'O',
    0x19, 'P',
    0x10, 'Q',
    0x13, 'R',
    0x1F, 'S',
    0x14, 'T',
    0x16, 'U',
    0x2F, 'V',
    0x11, 'W',
    0x2D, 'X',
    0x15, 'Y',
    0x2C, 'Z',
    0x02, '1',  //!
    0x03, '2',  //@
    0x04, '3',  //#
    0x05, '4',  //$
    0x06, '5',  //%
    0x07, '6',  //^
    0x08, '7',  //&
    0x09, '8',  //*
    0x0A, '9',  //(
    0x0B, '0',  //)
    0x1C, '\n', //ENTER
    0x01, 0,    //ESC
    0x0E, '\b', //BACKSPACE
    0x0F, '\t', //TAB
    0x39, ' ',  //SPACE
    0x0C, '-',  //_
    0x0D, '=',  //+
    0x1A, '[',  //{
    0x1B, ']',  //}
    0x2B, '\\', //|
    0x2B, '\\', //|, EUROPE 1 (NOTE2)
    0x27, ';',  //:
    0x28, '\'', //"
    0x29, '`',  //~
    0x33, ',',  //<
    0x34, '.',  //>
    0x35, '/',  //?
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
    0xC6, 0xE0,    //PAUSE, E0 46/E0 C6
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

//shared data among all HID devices, each device's interrupt routine put its input to here.
//the interrupts are not re-entrant so there's no conflict
#define USB_HID_KEYBUFFER_SIZE 64
static volatile uint16_t USB_HID_KeyBuffer[USB_HID_KEYBUFFER_SIZE];
static volatile uint8_t USB_HID_KBHead;
static volatile uint8_t USB_HID_KBTail;
static volatile uint16_t USB_HID_KeyModifer;


static void USB_HID_InputCallback(HCD_Request* pRequest);
static void USB_HID_INT16Hook(void);

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
            USB_Request req2 = {USB_REQ_TYPE_HID, USB_REQ_HID_SET_IDLE, 0 /*indefinite until input detected, all reports*/, (uint16_t)DrvIntface->bInterface, 0};
            USB_SyncSendRequest(pDevice, &req2, NULL);
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
            USB_Transfer(pDevice, pDriverData->Interface[i].pDataEP[HCD_TXR], pDriverData->Interface[i].Local, sizeof(pDriverData->Interface[i].Local), &USB_HID_InputCallback, (void*)i);
    }
    return valid > 0;
}

DPMI_ISR_HANDLE USB_HID_INT16Handle;
BOOL USB_HID_DOS_Install()
{
    return DPMI_InstallISR(0x16, &USB_HID_INT16Hook, &USB_HID_INT16Handle) == 0;
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
    KEY_USBCODES2SCANCODES[0xE0*2] = 0x1D;  //left control
    KEY_USBCODES2SCANCODES[0xE1*2] = 0x2A;  //left shift
    KEY_USBCODES2SCANCODES[0xE2*2] = 0x38;  //left alt
    KEY_USBCODES2SCANCODES[0xE3*2] = 0x5B;  //left gui, E0 5B
    KEY_USBCODES2SCANCODES[0xE3*2+1] = 0xE0;
    KEY_USBCODES2SCANCODES[0xE4*2] = 0x1D;  //left control, E0 1D
    KEY_USBCODES2SCANCODES[0xE4*2+1] = 0xE0;
    KEY_USBCODES2SCANCODES[0xE5*2] = 0x36;  //left shift
    KEY_USBCODES2SCANCODES[0xE6*2] = 0x38;  //left alt
    KEY_USBCODES2SCANCODES[0xE7*2] = 0x5C;  //left gui
}

void USB_HID_PostDeInit()
{

}

static int16_t USB_HID_INT16Handler(uint8_t ah)
{
    switch (ah)
    {
    case 0x00: case 0x10: //read next key (destructive)
    {
        uint16_t keycode = 0;
        do
        {
            CLIS();
            if(USB_HID_KBHead != USB_HID_KBTail)
            {
                keycode = USB_HID_KeyBuffer[USB_HID_KBHead];
                USB_HID_KBHead = (USB_HID_KBHead+1) % USB_HID_KEYBUFFER_SIZE;
            }
            else //read BIOS keyboard buffer
            { //https://stanislavs.org/helppc/bios_data_area.html
                uint16_t head = DPMI_LoadW(DPMI_SEGOFF2L(0x40,0x1A));
                uint16_t tail = DPMI_LoadW(DPMI_SEGOFF2L(0x40,0x1C));
                if(head != tail)
                {
                    keycode = DPMI_LoadW(DPMI_SEGOFF2L(0x40,head));
                    head = (head + 2) % 32;
                    DPMI_StoreW(DPMI_SEGOFF2L(0x40,0x1A), head);
                }
            }
            STIL();
            if(!keycode)
                USB_IdleWait();
        }while(keycode == 0);
        if(ah == 0x00)
            keycode = (uint16_t)(keycode&~0xFFL);
    }
    break;
    case 0x01: case 0x11: //peek key (non-destructive) ZF=1: no key, ZF=0: key exist
    {
        uint16_t keycode = 0;
        CLIS();
        if(USB_HID_KBHead != USB_HID_KBTail)
            keycode = USB_HID_KeyBuffer[USB_HID_KBHead];
        else //read BIOS keyboard buffer
        {
            uint16_t head = DPMI_LoadW(DPMI_SEGOFF2L(0x40,0x1A));
            uint16_t tail = DPMI_LoadW(DPMI_SEGOFF2L(0x40,0x1C));
            if(head != tail)
                keycode = DPMI_LoadW(DPMI_SEGOFF2L(0x40,head));
        }
        STIL();
        if(ah == 0x00 && keycode&0xFF == 0xE0 )
            keycode = (uint16_t)(keycode&0xFF00);
    }
    break;
    case 0x02: case 0x12: //get modifier
    {
        CLIS();
        uint16_t Modifier = USB_HID_KeyModifer;
        Modifier |= DPMI_LoadW(DPMI_SEGOFF2L(0x40,0x17));
        STIL();
        if(ah == 0x02)
            Modifier = (uint16_t)(Modifier&0x00FF);
    }
    break;
    default:
        break;
    }
    return 0;
}

static void USB_HID_InputCallback(HCD_Request* pRequest)
{
    USB_Device* pDevice = HC2USB(pRequest->pDevice);
    USB_HID_DriverData* pDriverData = (USB_HID_DriverData*)pDevice->pDriverData;

    int mode = (int)(uintptr_t)pRequest->pCBData;
    if(mode == 0) //keyboard
    {
        int index = pDriverData->Interface[mode].Local[2]*2;
        uint8_t scancode = KEY_USBCODES2SCANCODES[index];
        uint8_t ascii = KEY_USBCODES2SCANCODES[index+1];
        if(USB_HID_KBTail == (USB_HID_KBTail+1)%USB_HID_KEYBUFFER_SIZE) //full
        {
            //beep?
        }
        else
        {
            USB_HID_KeyBuffer[USB_HID_KBTail] = (uint16_t)(((uint16_t)scancode<<8) | ascii);
            USB_HID_KBTail = (USB_HID_KBTail+1) % USB_HID_KEYBUFFER_SIZE;
        }
    }
    else
    {

    }

    //continue input, async don't wait
    USB_Transfer(pDevice, pDriverData->Interface[mode].pDataEP[HCD_TXR], pDriverData->Interface[mode].Local, sizeof(pDriverData->Interface[mode].Local), &USB_HID_InputCallback, (void*)mode);
}

static void USB_HID_INT16Hook(void)
{
    uint32_t IntNO = 0;
    #if defined(__DJ2__)
    asm("mov %%eax, %0\n\t":"=m"(IntNO));
    #else
    _ASM_BEGIN _ASM(mov IntNO, eax) _ASM_END
    #endif
    uint8_t AH = (uint8_t)((IntNO>>8)&0xFF);

    uint16_t result = 0;
    int zf = 0;

    if(AH == 0x00 || AH == 0x01 || AH == 0x02
    || AH == 0x10 || AH == 0x11 || AH == 0x12)
    {
        result = USB_HID_INT16Handler(AH);
    }
    else
    {
        DPMI_REG r = {0};
        r.w.cs = USB_HID_INT16Handle.rm_cs;
        r.w.ip = USB_HID_INT16Handle.rm_offset;
        DPMI_CallRealModeIRET(&r);
        result = r.w.ax;
    }
    zf = result ? 0 : CPU_ZFLAG;

    //the pm int handler may not return modified registers
    //need to use RMCB and set realmode handler. the RMCB have register set to read & modify
    //result AH:AL
    #if defined(__DJ2__)
    asm("pushf\n\t and $0xFFBF, 0(%%esp)\n\t mov %0, %%ax\n\t or %%ax, 0(%%eesp)\n\t mov %1, %%ax\n\t "::"m"(zf),"m"(result));
    #else
    _ASM_BEGIN _ASM(pushf) _ASM(and [esp], 0xFFBF) _ASM(mov ax, word ptr zf) _ASM(or [esp], ax) _ASM(popf) _ASM(mov ax, result) _ASM_END
    #endif
}