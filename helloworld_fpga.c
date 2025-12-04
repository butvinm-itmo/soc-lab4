/******************************************************************************
 * Lab 4: Heterogeneous SoC Architecture - FPGA Version
 *
 * SW Mode (Switch[0] = 0): Software computation
 * HW Mode (Switch[0] = 1): HLS accelerator
 *
 * LED[0] = mode, LED[15:8] = result values (cycling)
 ******************************************************************************/

#include "xil_io.h"
#include <stdint.h>
#include "platform.h"

#define GPIO_LED_DATA   0x40000000
#define GPIO_SW_DATA    0x40000008

#define HLS_BASE        0x44A00000
#define HLS_CTRL        (HLS_BASE + 0x00)
#define HLS_A_BASE      (HLS_BASE + 0x40)
#define HLS_B_BASE      (HLS_BASE + 0x80)
#define HLS_C_BASE      (HLS_BASE + 0xC0)

#define AP_START        0x01
#define AP_DONE         0x02
#define AP_IDLE         0x04

#define MAT_SIZE        7
#define DELAY_COUNT     5000000

void mat_mul(uint8_t A[MAT_SIZE][MAT_SIZE], uint8_t B[MAT_SIZE][MAT_SIZE], uint8_t C[MAT_SIZE][MAT_SIZE]) {
    for (int i = 0; i < MAT_SIZE; i++)
        for (int j = 0; j < MAT_SIZE; j++) {
            C[i][j] = 0;
            for (int k = 0; k < MAT_SIZE; k++)
                C[i][j] += A[i][k] * B[k][j];
        }
}

void mat_add(uint8_t A[MAT_SIZE][MAT_SIZE], uint8_t B[MAT_SIZE][MAT_SIZE], uint8_t C[MAT_SIZE][MAT_SIZE]) {
    for (int i = 0; i < MAT_SIZE; i++)
        for (int j = 0; j < MAT_SIZE; j++)
            C[i][j] = A[i][j] + B[i][j];
}

void sw_compute(uint8_t A[MAT_SIZE][MAT_SIZE], uint8_t B[MAT_SIZE][MAT_SIZE], uint8_t C[MAT_SIZE][MAT_SIZE]) {
    uint8_t temp[MAT_SIZE][MAT_SIZE];
    mat_mul(B, B, temp);
    mat_add(A, temp, C);
}

void hls_write(uint32_t addr, uint8_t mat[MAT_SIZE][MAT_SIZE]) {
    uint8_t *p = (uint8_t *)mat;
    for (int i = 0; i < 48; i += 4)
        Xil_Out32(addr + i, p[i] | (p[i+1]<<8) | (p[i+2]<<16) | (p[i+3]<<24));
    Xil_Out32(addr + 48, p[48]);
}

void hls_read(uint32_t addr, uint8_t mat[MAT_SIZE][MAT_SIZE]) {
    uint8_t *p = (uint8_t *)mat;
    for (int i = 0; i < 48; i += 4) {
        uint32_t w = Xil_In32(addr + i);
        p[i] = w; p[i+1] = w>>8; p[i+2] = w>>16; p[i+3] = w>>24;
    }
    p[48] = Xil_In32(addr + 48);
}

void hw_compute(uint8_t A[MAT_SIZE][MAT_SIZE], uint8_t B[MAT_SIZE][MAT_SIZE], uint8_t C[MAT_SIZE][MAT_SIZE]) {
    while (!(Xil_In32(HLS_CTRL) & AP_IDLE));
    hls_write(HLS_A_BASE, A);
    hls_write(HLS_B_BASE, B);
    Xil_Out32(HLS_CTRL, AP_START);
    while (!(Xil_In32(HLS_CTRL) & AP_DONE));
    hls_read(HLS_C_BASE, C);
}

void delay(void) { for (volatile int i = 0; i < DELAY_COUNT; i++); }

int main() {
    init_platform();

    uint8_t A[MAT_SIZE][MAT_SIZE] = {
        {0,4,8,8,9,10,5}, {8,1,2,9,9,8,7}, {1,8,4,10,5,4,4},
        {10,4,8,5,3,2,7}, {0,4,7,4,0,1,9}, {0,3,9,10,3,1,9}, {1,7,1,9,4,0,7}
    };
    uint8_t B[MAT_SIZE][MAT_SIZE] = {
        {5,4,8,8,9,10,5}, {8,5,2,9,9,8,7}, {1,8,4,10,5,4,4},
        {10,4,8,5,3,2,7}, {0,4,7,4,0,1,9}, {0,3,9,10,3,1,9}, {1,7,1,9,4,0,5}
    };
    uint8_t C[MAT_SIZE][MAT_SIZE];

    while (1) {
        uint32_t mode = Xil_In16(GPIO_SW_DATA) & 0x01;

        if (mode == 0) sw_compute(A, B, C);
        else hw_compute(A, B, C);

        Xil_Out16(GPIO_LED_DATA, mode | 0x02);
        delay();

        for (int i = 0; i < MAT_SIZE && (Xil_In16(GPIO_SW_DATA) & 1) == mode; i++)
            for (int j = 0; j < MAT_SIZE && (Xil_In16(GPIO_SW_DATA) & 1) == mode; j++) {
                Xil_Out16(GPIO_LED_DATA, (C[i][j] << 8) | mode);
                delay();
                Xil_Out16(GPIO_LED_DATA, mode);
                delay();
            }
    }
    cleanup_platform();
    return 0;
}
