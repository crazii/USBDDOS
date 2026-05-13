/*
    This file is part of RetroWave.

    Copyright (C) 2021 ReimuNotMoe <reimu@sudomaker.com>
    Copyright (C) 2021 Yukino Song <yukino@sudomaker.com>

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

#include "RetroWav/retrowav.h"
#include <assert.h>

#define RETROWAVE_CMD_BUFFER_SIZE 110

void retrowave_init(RetroWaveContext *ctx) {
    memset(ctx, 0, sizeof(RetroWaveContext));
    ctx->cmd_buffer = (unsigned char*)malloc(RETROWAVE_CMD_BUFFER_SIZE);
}

void retrowave_deinit(RetroWaveContext *ctx) {
    free(ctx->cmd_buffer);
}

void retrowave_io_init(RetroWaveContext *ctx) {
    // Sync CS state
    uint8_t empty_byte = 0;
    ctx->callback_io(ctx->user_data, 1e6, &empty_byte, NULL, 1);

    {
        uint8_t init_sequence_1[] = {
            0x00,
            0x0a,    // IOCON register
            0x28    // Enable: HAEN, SEQOP
        };

        uint8_t init_sequence_2[] = {
            0x00,
            0x00,    // IODIRA register
            0x00,    // Set output
            0x00    // Set output
        };

        uint8_t init_sequence_3[] = {
            0x00,
            0x12,    // GPIOA register
            0xff,    // Set all HIGH
            0xff    // Set all HIGH
        };

        uint8_t i;
        for (i=0x20; i<0x28; i++) {
            uint8_t addr = (uint8_t)(i << 1);

            init_sequence_1[0] = init_sequence_2[0] = init_sequence_3[0] = addr;
            ctx->callback_io(ctx->user_data, 1e6, init_sequence_1, NULL, sizeof(init_sequence_1));
            ctx->callback_io(ctx->user_data, 1e6, init_sequence_2, NULL, sizeof(init_sequence_2));
            ctx->callback_io(ctx->user_data, 1e6, init_sequence_3, NULL, sizeof(init_sequence_3));
        }
    }
}

void retrowave_cmd_buffer_init(RetroWaveContext *ctx, RetroWaveBoardType board_type, uint8_t first_reg, uint32_t next_len) {
    if (ctx->cmd_buffer_used) {
        if (ctx->cmd_buffer[0] != board_type) {
            retrowave_flush(ctx);
        }
    }

    assert(next_len + 2 <= RETROWAVE_CMD_BUFFER_SIZE);
    
    if(ctx->cmd_buffer_used + next_len > RETROWAVE_CMD_BUFFER_SIZE)
        retrowave_flush(ctx);

    if (!ctx->cmd_buffer_used) {
        ctx->cmd_buffer[0] = board_type;
        ctx->cmd_buffer[1] = first_reg;
        ctx->cmd_buffer_used = 2;
    }
}

static void cmd_buffer_deinit(RetroWaveContext *ctx) {
    ctx->cmd_buffer_used = 0;
}

void retrowave_flush(RetroWaveContext *ctx) {
    if (ctx->cmd_buffer_used) {
        ctx->callback_io(ctx->user_data, ctx->transfer_speed_hint, ctx->cmd_buffer, NULL, ctx->cmd_buffer_used);
        cmd_buffer_deinit(ctx);
    }
}

