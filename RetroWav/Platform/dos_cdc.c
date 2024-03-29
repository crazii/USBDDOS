#include <assert.h>
#include "RetroWav/Platform/dos_cdc.h"
#include "USBDDOS/DPMI/dpmi.h"
#include "USBDDOS/CLASS/cdc.h"
#include "USBDDOS/usb.h"
#include "USBDDOS/dbgutil.h"

#if _LOG_ENABLE || defined(__BC__)
static const char log_tag[] = "retrowave platform dos_cdc";
#endif

static void io_finish_callback(HCD_Request* pRequest)
{
    retrowave_flush((RetroWaveContext*)pRequest->pCBData); //continue if new data exists
    //_LOG("io finish: %02x, %d, %d\n", pRequest->error, pRequest->size, pRequest->transferred);
    //assert(pRequest->size == pRequest->transferred);
}

static void io_callback(void *userp, uint32_t data_rate, const void *tx_buf, void *rx_buf, uint32_t len) {
    unused(data_rate);
    unused(rx_buf);

    RetroWavePlatform_DOS_CDC *ctx = (RetroWavePlatform_DOS_CDC *)userp;
    USB_Device* device = (USB_Device*)ctx->device;
    
    uint32_t packed_len = retrowave_protocol_serial_packed_length(len);
    uint8_t *packed_data;

    if (packed_len > USB_DEVBUFFER_SIZE) {
        packed_data = (uint8_t*)DPMI_DMAMalloc(packed_len, 4);
        if(!packed_data) {
            _LOG("%s: FATAL: failed to allocate memory: %u\n", log_tag, packed_len);
            return;
        }
    }
    else
        packed_data = device->pDeviceBuffer;

    assert(packed_data);
    
    uint32_t apacked_len = retrowave_protocol_serial_pack(tx_buf, len, packed_data);
    assert(apacked_len == packed_len);
    unused(apacked_len);

    uint8_t result = USB_CDC_Transfer(device, HCD_TXW, packed_data, (uint16_t)packed_len, io_finish_callback, ctx->ctx);
    //uint16_t written = 0;
    //uint8_t result = USB_CDC_SyncTransfer(device, HCD_TXW, packed_data, packed_len, &written);
    if(result != 0) {
        _LOG("%s: FATAL: failed to transfer to device %d.\n", log_tag, result);
        return;
    }
    if (packed_len > USB_DEVBUFFER_SIZE)
        DPMI_DMAFree(packed_data);
}

//Thoese are from Sudomaker
#define RETROWAVE_VENDOR_EXPRESS 0x04D8
#define RETROWAVE_DEVID_EXPRESS 0x000A
#define RETROWAVE_VENDOR_LITE 0x0483
#define RETROWAVE_DEVID_LITE 0x5740

int retrowave_init_dos_cdc(RetroWaveContext *ctx) {

    USB_Device* device = NULL;
    for(int j = 0; j < USBT.HC_Count; ++j)
    {
        HCD_Interface* pHCI = USBT.HC_List+j;

        for(uint8_t i = 0; i < pHCI->bDevCount; ++i)
        {
            USB_Device* dev = HC2USB(pHCI->DeviceList[i]);
            if((dev->Desc.widVendor == RETROWAVE_VENDOR_EXPRESS && dev->Desc.widProduct == RETROWAVE_DEVID_EXPRESS)
            || (dev->Desc.widVendor == RETROWAVE_VENDOR_LITE && dev->Desc.widProduct == RETROWAVE_DEVID_LITE)
            || memicmp(dev->sProduct, "RetroWave OPL", 13) == 0)
            {
                device = dev;
                break;
            }
        }
    }

    if(device != NULL)
        printf("Vendor: %s, Name: %s\n", device->sManufacture, device->sProduct);
    else
    {
        printf("Retro wave device not found.\n");
        return -1;
    }

    retrowave_init(ctx);
    #if 0
    USB_CDC_LINE_CODING lc = {115200, USB_CDC_LINE_CODING_1STOPBIT, USB_CDC_PARITY_NONE, 8};
    if(USB_CDC_SetLineCoding((USB_Device*)device, &lc) != 0)
    {
        puts("Error: Failed set line coding.\n");
        return -1;
    }
    #endif
    ctx->user_data = malloc(sizeof(RetroWavePlatform_DOS_CDC));
    RetroWavePlatform_DOS_CDC *pctx = (RetroWavePlatform_DOS_CDC *)ctx->user_data;
    pctx->device = device;
    pctx->ctx = ctx;
    ctx->callback_io = io_callback;

    return 0;
}

void retrowave_deinit_dos_cdc(RetroWaveContext *ctx) {

    RetroWavePlatform_DOS_CDC *pctx = (RetroWavePlatform_DOS_CDC *)ctx->user_data;
    free(pctx);
}
