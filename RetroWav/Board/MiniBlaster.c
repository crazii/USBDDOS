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

#include "MiniBlaster.h"

static const int transfer_speed = 0.8e6;

// Under construction!

void retrowave_miniblaster_queue(RetroWaveContext *ctx, uint8_t reg, uint8_t val) {
    retrowave_cmd_buffer_init(ctx, RetroWave_Board_MiniBlaster, 0x12, 24);
    ctx->transfer_speed_hint = transfer_speed;

    if (reg < 0x80) {
        reg &= 0x7f;
        ctx->cmd_buffer_used += 12;
        ctx->cmd_buffer[ctx->cmd_buffer_used - 12] = 0xfd;        // A0 = 1, CS# = 0, WR# = 1
        ctx->cmd_buffer[ctx->cmd_buffer_used - 11] = reg;
        ctx->cmd_buffer[ctx->cmd_buffer_used - 10] = 0xf9;        // A0 = 1, CS# = 0, WR# = 0
        ctx->cmd_buffer[ctx->cmd_buffer_used - 9] = reg;
        ctx->cmd_buffer[ctx->cmd_buffer_used - 8] = 0xff;        //
        ctx->cmd_buffer[ctx->cmd_buffer_used - 7] = val;
        ctx->cmd_buffer[ctx->cmd_buffer_used - 6] = 0xfc;        // A0 = 0, CS# = 0, WR# = 1
        ctx->cmd_buffer[ctx->cmd_buffer_used - 5] = val;
        ctx->cmd_buffer[ctx->cmd_buffer_used - 4] = 0xf8;        // A0 = 0, CS# = 0, WR# = 0
        ctx->cmd_buffer[ctx->cmd_buffer_used - 3] = val;
        ctx->cmd_buffer[ctx->cmd_buffer_used - 2] = 0xfe;        // A0 = 0
        ctx->cmd_buffer[ctx->cmd_buffer_used - 1] = val;
    } else {
        reg &= 0x7f;
        ctx->cmd_buffer_used += 12;
        ctx->cmd_buffer[ctx->cmd_buffer_used - 12] = 0xdf;        // A0 = 1, CS# = 0, WR# = 1
        ctx->cmd_buffer[ctx->cmd_buffer_used - 11] = reg;
        ctx->cmd_buffer[ctx->cmd_buffer_used - 10] = 0x9f;        // A0 = 1, CS# = 0, WR# = 0
        ctx->cmd_buffer[ctx->cmd_buffer_used - 9] = reg;
        ctx->cmd_buffer[ctx->cmd_buffer_used - 8] = 0xff;        //
        ctx->cmd_buffer[ctx->cmd_buffer_used - 7] = val;
        ctx->cmd_buffer[ctx->cmd_buffer_used - 6] = 0xcf;        // A0 = 0, CS# = 0, WR# = 1
        ctx->cmd_buffer[ctx->cmd_buffer_used - 5] = val;
        ctx->cmd_buffer[ctx->cmd_buffer_used - 4] = 0x8f;        // A0 = 0, CS# = 0, WR# = 0
        ctx->cmd_buffer[ctx->cmd_buffer_used - 3] = val;
        ctx->cmd_buffer[ctx->cmd_buffer_used - 2] = 0xef;        // A0 = 0
        ctx->cmd_buffer[ctx->cmd_buffer_used - 1] = val;
    }
}