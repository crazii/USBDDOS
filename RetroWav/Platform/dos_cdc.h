#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if !defined(__BORLANDC__)
#include <inttypes.h>
#endif

#include <errno.h>

#include "RetroWav/retrowav.h"
#include "RetroWav/Protocol/serial.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void* device;
    RetroWaveContext *ctx;
}RetroWavePlatform_DOS_CDC;

extern int retrowave_init_dos_cdc(RetroWaveContext *ctx);
extern void retrowave_deinit_dos_cdc(RetroWaveContext *ctx);

#ifdef __cplusplus
}
#endif
