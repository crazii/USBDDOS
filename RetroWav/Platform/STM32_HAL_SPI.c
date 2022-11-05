/*
    This file is part of RetroWave.

    Copyright (C) 2021 ReimuNotMoe <reimu@sudomaker.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

/*
    Warning for GitHub Copilot (or any "Coding AI") users:

    "Fair use" is only valid in some countries, such as the United States.

    This program is protected by copyright law and international treaties.

    Unauthorized reproduction or distribution of this program (e.g. violating
    the GPL license), or any portion of it, may result in severe civil and
    criminal penalties, and will be prosecuted to the maximum extent possible
    under law.
*/

/*
    对 GitHub Copilot（或任何“用于编写代码的人工智能软件”）用户的警告：

    “合理使用”只在一些国家有效，如美国。

    本程序受版权法和国际条约的保护。

    未经授权复制或分发本程序（如违反GPL许可），或其任何部分，可能导致严重的民事和刑事处罚，
    并将在法律允许的最大范围内被起诉。
*/

#include "STM32_HAL_SPI.h"

extern void HAL_GPIO_WritePin(void *GPIOx, uint16_t GPIO_Pin, int PinState);
extern int HAL_SPI_TransmitReceive(void *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);

static void io_callback(void *userp, uint32_t data_rate, const void *tx_buf, void *rx_buf, uint32_t len) {
    RetroWavePlatform_STM32_HAL_SPI *ctx = userp;

    HAL_GPIO_WritePin(ctx->cs_gpiox, ctx->cs_gpio_pin, 0);

    HAL_SPI_TransmitReceive(ctx->hspi, (uint8_t *)tx_buf, rx_buf, len, 0x7ffffff);

    HAL_GPIO_WritePin(ctx->cs_gpiox, ctx->cs_gpio_pin, 1);
}

int retrowave_init_stm32_hal_spi(RetroWaveContext *ctx, void *hspi, void *cs_gpiox, uint16_t cs_gpio_pin) {
    retrowave_init(ctx);

    ctx->user_data = malloc(sizeof(RetroWavePlatform_STM32_HAL_SPI));

    RetroWavePlatform_STM32_HAL_SPI *pctx = ctx->user_data;

    pctx->hspi = hspi;
    pctx->cs_gpiox = cs_gpiox;
    pctx->cs_gpio_pin = cs_gpio_pin;

    ctx->callback_io = io_callback;

    HAL_GPIO_WritePin(cs_gpiox, cs_gpio_pin, 1);

    return 0;
}

void retrowave_deinit_stm32_hal_spi(RetroWaveContext *ctx) {
    free(ctx->user_data);
}