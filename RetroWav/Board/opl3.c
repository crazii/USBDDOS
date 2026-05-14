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

#include "RetroWav/Board/opl3.h"

static const int transfer_speed = 2e6;

void retrowave_opl3_queue_port0(RetroWaveContext *ctx, uint8_t reg, uint8_t val) {
    retrowave_cmd_buffer_init(ctx, RetroWave_Board_OPL3, 0x12, 6);
    ctx->transfer_speed_hint = transfer_speed;

    ctx->cmd_buffer[ctx->cmd_buffer_used++] = 0xe1;
    ctx->cmd_buffer[ctx->cmd_buffer_used++] = reg;
    ctx->cmd_buffer[ctx->cmd_buffer_used++] = 0xe3;
    ctx->cmd_buffer[ctx->cmd_buffer_used++] = val;
    ctx->cmd_buffer[ctx->cmd_buffer_used++] = 0xfb;
    ctx->cmd_buffer[ctx->cmd_buffer_used++] = val;
}

void retrowave_opl3_queue_port1(RetroWaveContext *ctx, uint8_t reg, uint8_t val) {
    retrowave_cmd_buffer_init(ctx, RetroWave_Board_OPL3, 0x12, 6);
    ctx->transfer_speed_hint = transfer_speed;

    ctx->cmd_buffer[ctx->cmd_buffer_used++] = 0xe5;
    ctx->cmd_buffer[ctx->cmd_buffer_used++] = reg;
    ctx->cmd_buffer[ctx->cmd_buffer_used++] = 0xe7;
    ctx->cmd_buffer[ctx->cmd_buffer_used++] = val;
    ctx->cmd_buffer[ctx->cmd_buffer_used++] = 0xfb;
    ctx->cmd_buffer[ctx->cmd_buffer_used++] = val;
}

void retrowave_opl3_queue_delay(RetroWaveContext *ctx)
{
    retrowave_cmd_buffer_init(ctx, RetroWave_Board_OPL3, 0x12, 6);
    ctx->transfer_speed_hint = transfer_speed;

    ctx->cmd_buffer[ctx->cmd_buffer_used++] = 0xe5;
    ctx->cmd_buffer[ctx->cmd_buffer_used++] = 0xFF;
    ctx->cmd_buffer[ctx->cmd_buffer_used++] = 0xe7;
    ctx->cmd_buffer[ctx->cmd_buffer_used++] = 0;
    ctx->cmd_buffer[ctx->cmd_buffer_used++] = 0xfb;
    ctx->cmd_buffer[ctx->cmd_buffer_used++] = 0;
}

void retrowave_opl3_emit_port0(RetroWaveContext *ctx, uint8_t reg, uint8_t val) {
    uint8_t buf[] = {RetroWave_Board_OPL3, 0x12, 0xe1, 0, 0xe3, 0, 0xfb, 0};
    buf[3] = reg;
    buf[5] = buf[7] = val;
    ctx->callback_io(ctx->user_data, transfer_speed, buf, NULL, sizeof(buf));
}

void retrowave_opl3_emit_port1(RetroWaveContext *ctx, uint8_t reg, uint8_t val) {
    uint8_t buf[] = {RetroWave_Board_OPL3, 0x12, 0xe5, 0, 0xe7, 0, 0xfb, 0};
    buf[3] = reg;
    buf[5] = buf[7] = val;
    ctx->callback_io(ctx->user_data, transfer_speed, buf, NULL, sizeof(buf));
}

void retrowave_opl3_emit_delay(RetroWaveContext *ctx)
{
    uint8_t buf[] = {RetroWave_Board_OPL3, 0x12, 0xe5, 0x0, 0xe7, 0, 0xfb, 0};
    ctx->callback_io(ctx->user_data, transfer_speed, buf, NULL, sizeof(buf));
}

void retrowave_opl3_reset(RetroWaveContext *ctx) {
    uint8_t buf[] = {RetroWave_Board_OPL3, 0x12, 0xfe};
    ctx->callback_io(ctx->user_data, transfer_speed / 10, buf, NULL, sizeof(buf));
    buf[2] = 0xff;
    ctx->callback_io(ctx->user_data, transfer_speed / 10, buf, NULL, sizeof(buf));
}

void retrowave_opl3_mute(RetroWaveContext *ctx) {
    uint8_t i;
    for (i = 0x20; i <= 0xF5; i++) {
        retrowave_opl3_emit_port0(ctx, i, i >= 0x40 && i <= 0x55 ? 0xFF : 0x00);
        retrowave_opl3_emit_port1(ctx, i, i >= 0x40 && i <= 0x55 ? 0xFF : 0x00);
    }
}
