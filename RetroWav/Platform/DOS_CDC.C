#include <assert.h>
#include "RetroWav/Platform/DOS_CDC.H"
#include "USBDDOS/DPMI/DPMI.H"
#include "USBDDOS/CLASS/CDC.H"
#include "USBDDOS/USB.H"

static const char log_tag[] = "retrowave platform dos_cdc";

static void io_callback(void *userp, uint32_t data_rate, const void *tx_buf, void *rx_buf, uint32_t len) {
    unused(data_rate);
    unused(rx_buf);

    RetroWavePlatform_DOS_CDC *ctx = (RetroWavePlatform_DOS_CDC *)userp;
    USB_Device* device = (USB_Device*)ctx->device;
    HCD_Method* hcdm = device->HCDDevice.pHCI->pHCDMethod;

    uint32_t packed_len = retrowave_protocol_serial_packed_length(len);
    uint8_t *packed_data;

    //need use device data or mapped data for device to access
    if (packed_len > HCDBUFFER_SIZE) {
        packed_data = (uint8_t*)DPMI_MappedMalloc(packed_len, 4);
        if(!packed_data) {
            _LOG("%s: FATAL: failed to allocate memory: %u\n", log_tag, packed_len);
            return;
        }
    }
    else
        packed_data = (uint8_t*)hcdm->GetHCDBuffer(&device->HCDDevice);

    assert(packed_data);
    
    uint32_t apacked_len = retrowave_protocol_serial_pack(tx_buf, len, packed_data);
    assert(apacked_len == packed_len);

    size_t written = 0;

    while (written < packed_len) {
        uint16_t tred = 0;
        BOOL result = USB_CDC_Transfer(device, HCD_TXW, packed_data + written, packed_len - written, &tred);
        written += tred;
        if(!result) {
            _LOG("%s: FATAL: failed to transfer to device: %d\n", log_tag, result);
            return;
        }
    }

    if (packed_len > HCDBUFFER_SIZE)
        DPMI_MappedFree(packed_data);
}

int retrowave_init_dos_cdc(RetroWaveContext *ctx, void *device) {

    retrowave_init(ctx);
    USB_CDC_LINE_CODING lc = {115200, USB_CDC_LINE_CODING_1STOPBIT, USB_CDC_PARITY_NONE, 8};
    if(USB_CDC_SetLineCoding((USB_Device*)device, &lc) != 0)
    {
        puts("Error: Failed set line coding.\n");
        return -1;
    }
    ctx->user_data = malloc(sizeof(RetroWavePlatform_DOS_CDC));
    RetroWavePlatform_DOS_CDC *pctx = (RetroWavePlatform_DOS_CDC *)ctx->user_data;
    pctx->device = device;
    ctx->callback_io = io_callback;

    return 0;
}

void retrowave_deinit_dos_cdc(RetroWaveContext *ctx) {

    RetroWavePlatform_DOS_CDC *pctx = (RetroWavePlatform_DOS_CDC *)ctx->user_data;
    free(pctx);
}
