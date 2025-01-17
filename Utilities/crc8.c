/*******************************************************************************
* Copyright (C) 2018 Maxim Integrated Products, Inc., All rights Reserved.
*
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/

#include <stdint.h>
#include <stdbool.h>
/*
 * CRC-8:
 *
 * Width:   8
 * Poly:    0x07
 * Init:    0x00
 * RefIn:   False
 * RefOut:  False
 * XOROur:  0x00
 * Check:   0xF4
 */

static const uint8_t CRC8_POLY = 0x07;
static bool crc8_initialized = false;

static uint8_t crc8_table[256];


static void init_crc8()
{
    uint8_t msb = 1 << 7;
    uint8_t bit;
    uint8_t rem;
    uint32_t byte = 0;

    if (crc8_initialized)
        return;

    while (byte < 256) {
        rem = byte;
        for (bit = 0; bit < 8; bit++) {
            if (rem & msb)
                rem = (rem << 1) ^ CRC8_POLY;
            else
                rem <<= 1;
        }

        crc8_table[byte++] = rem;
    }

    crc8_initialized = true;
}

uint8_t crc8(uint8_t *data, uint32_t length)
{
    uint8_t crc = 0x00;
    uint32_t msg_idx = 0;

    if (length == 0xFFFFFFFF)
        return 0;

    init_crc8();

    while (msg_idx < length) {
        crc = crc8_table[crc ^ data[msg_idx++]];
    }

    return crc;
}
