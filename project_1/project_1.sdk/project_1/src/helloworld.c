/******************************************************************************
 *
 * Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * Use of the Software is limited solely to applications:
 * (a) running on a Xilinx device, or
 * (b) that interact with a Xilinx device through a bus or interconnect.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Except as contained in this notice, the name of the Xilinx shall not be used
 * in advertising or otherwise to promote the sale, use or other dealings in
 * this Software without prior written authorization from Xilinx.
 *
 ******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include "xil_io.h"
#include <stdint.h>
#include <stdio.h>

#include "platform.h"

#define MAT_SIZE 7
#define MAT_LED_START 8
#define OUT_PORT 0x40000000
#define DELAY 10000000

void mat_mul(uint8_t A[MAT_SIZE][MAT_SIZE], uint8_t B[MAT_SIZE][MAT_SIZE], uint8_t C[MAT_SIZE][MAT_SIZE]) {
    for (size_t i = 0; i < MAT_SIZE; i++) {
        for (size_t j = 0; j < MAT_SIZE; j++) {
            C[i][j] = 0;
            for (size_t k = 0; k < MAT_SIZE; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void mat_add(uint8_t A[MAT_SIZE][MAT_SIZE], uint8_t B[MAT_SIZE][MAT_SIZE], uint8_t C[MAT_SIZE][MAT_SIZE]) {
    for (size_t i = 0; i < MAT_SIZE; i++) {
        for (size_t j = 0; j < MAT_SIZE; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

int main() {
    init_platform();

    uint8_t A[MAT_SIZE][MAT_SIZE] = {
        { 0, 4, 8, 8, 9, 10, 5 },
        { 8, 1, 2, 9, 9, 8, 7 },
        { 1, 8, 4, 10, 5, 4, 4 },
        { 10, 4, 8, 5, 3, 2, 7 },
        { 0, 4, 7, 4, 0, 1, 9 },
        { 0, 3, 9, 10, 3, 1, 9 },
        { 1, 7, 1, 9, 4, 0, 7 },
    };

    uint8_t B[MAT_SIZE][MAT_SIZE] = {
        { 69, 4, 8, 8, 9, 10, 5 },
        { 8, 69, 2, 9, 9, 8, 7 },
        { 1, 8, 4, 10, 5, 4, 4 },
        { 10, 4, 8, 5, 3, 2, 7 },
        { 0, 4, 7, 4, 0, 1, 9 },
        { 0, 3, 9, 10, 3, 1, 9 },
        { 1, 7, 1, 9, 4, 0, 69 },
    };

    uint8_t C[MAT_SIZE][MAT_SIZE]; // A + B*B

    /* Loop forever blinking the LED */
    while (1) {
        uint16_t leds = 0x0001;
        Xil_Out16(OUT_PORT, leds);

        mat_mul(B, B, C);
        mat_add(A, C, C);

        leds = 0x0002;
        Xil_Out16(OUT_PORT, leds);
        
        for (size_t delay = 0; delay < DELAY; delay++);

        for (size_t i = 0; i < MAT_SIZE; i++) {
            for (size_t j = 0; j < MAT_SIZE; j++) {
                leds = C[i][j] << MAT_LED_START;

                Xil_Out16(OUT_PORT, leds);
                for (size_t delay = 0; delay < DELAY; delay++);

                Xil_Out16(OUT_PORT, 0x0000);
                for (size_t delay = 0; delay < DELAY; delay++);
            }
        }
    }

    cleanup_platform();
    return 0;
}
