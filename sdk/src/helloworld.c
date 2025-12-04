/******************************************************************************
 * Lab 4: Heterogeneous SoC Architecture
 *
 * MicroBlaze software with dual-mode computation:
 *   - SW Mode (Switch[0] = 0): Software-only matrix computation
 *   - HW Mode (Switch[0] = 1): Hardware accelerator (HLS mat_compute)
 *
 * Algorithm: C = A + B*B for 7x7 uint8 matrices
 ******************************************************************************/

#include "xil_io.h"
#include <stdint.h>
#include <stdio.h>
#include "platform.h"

/*===========================================================================*/
/* Hardware Addresses                                                        */
/*===========================================================================*/

/* AXI GPIO - Dual Channel */
#define GPIO_BASE       0x40000000
#define GPIO_LED_DATA   (GPIO_BASE + 0x0000)  /* Channel 1: LED outputs */
#define GPIO_LED_TRI    (GPIO_BASE + 0x0004)  /* Channel 1: Tri-state */
#define GPIO_SW_DATA    (GPIO_BASE + 0x0008)  /* Channel 2: Switch inputs */
#define GPIO_SW_TRI     (GPIO_BASE + 0x000C)  /* Channel 2: Tri-state */

/* AXI UARTLite */
#define UART_BASE       0x40600000

/* HLS mat_compute Accelerator */
#define HLS_BASE        0x44A00000
#define HLS_CTRL        (HLS_BASE + 0x00)     /* Control: ap_start, ap_done, ap_idle, ap_ready */
#define HLS_GIE         (HLS_BASE + 0x04)     /* Global Interrupt Enable */
#define HLS_IER         (HLS_BASE + 0x08)     /* IP Interrupt Enable */
#define HLS_ISR         (HLS_BASE + 0x0C)     /* IP Interrupt Status */
#define HLS_A_BASE      (HLS_BASE + 0x40)     /* Matrix A: 49 x 8-bit */
#define HLS_B_BASE      (HLS_BASE + 0x80)     /* Matrix B: 49 x 8-bit */
#define HLS_C_BASE      (HLS_BASE + 0xC0)     /* Matrix C: 49 x 8-bit */

/* Control register bits */
#define AP_START        0x01
#define AP_DONE         0x02
#define AP_IDLE         0x04
#define AP_READY        0x08

/*===========================================================================*/
/* Constants                                                                 */
/*===========================================================================*/

#define MAT_SIZE        7
#define MAT_ELEMENTS    49
#define MAT_LED_START   8
#define DELAY_COUNT     10000000

/* Mode definitions */
#define MODE_SW         0
#define MODE_HW         1

/*===========================================================================*/
/* Software Matrix Operations (from Lab 1)                                   */
/*===========================================================================*/

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

/* SW mode: C = A + B*B */
void sw_mat_compute(uint8_t A[MAT_SIZE][MAT_SIZE], uint8_t B[MAT_SIZE][MAT_SIZE], uint8_t C[MAT_SIZE][MAT_SIZE]) {
    uint8_t temp[MAT_SIZE][MAT_SIZE];
    mat_mul(B, B, temp);   /* temp = B * B */
    mat_add(A, temp, C);   /* C = A + temp */
}

/*===========================================================================*/
/* Hardware Accelerator Driver                                               */
/*===========================================================================*/

/* Write a 7x7 matrix to HLS IP (packed 4 bytes per 32-bit word) */
void hls_write_matrix(uint32_t base_addr, uint8_t mat[MAT_SIZE][MAT_SIZE]) {
    uint8_t *flat = (uint8_t *)mat;

    /* Write 12 full words (48 bytes) */
    for (int i = 0; i < 48; i += 4) {
        uint32_t word = flat[i] |
                       (flat[i+1] << 8) |
                       (flat[i+2] << 16) |
                       (flat[i+3] << 24);
        Xil_Out32(base_addr + i, word);
    }

    /* Write last element (index 48) - only lower byte valid */
    uint32_t last_word = flat[48];
    Xil_Out32(base_addr + 48, last_word);
}

/* Read a 7x7 matrix from HLS IP */
void hls_read_matrix(uint32_t base_addr, uint8_t mat[MAT_SIZE][MAT_SIZE]) {
    uint8_t *flat = (uint8_t *)mat;

    /* Read 12 full words (48 bytes) */
    for (int i = 0; i < 48; i += 4) {
        uint32_t word = Xil_In32(base_addr + i);
        flat[i]   = word & 0xFF;
        flat[i+1] = (word >> 8) & 0xFF;
        flat[i+2] = (word >> 16) & 0xFF;
        flat[i+3] = (word >> 24) & 0xFF;
    }

    /* Read last element */
    uint32_t last_word = Xil_In32(base_addr + 48);
    flat[48] = last_word & 0xFF;
}

/* Start HLS accelerator */
void hls_start(void) {
    Xil_Out32(HLS_CTRL, AP_START);
}

/* Wait for HLS accelerator to complete */
void hls_wait_done(void) {
    while (!(Xil_In32(HLS_CTRL) & AP_DONE));
}

/* Check if HLS accelerator is idle */
int hls_is_idle(void) {
    return (Xil_In32(HLS_CTRL) & AP_IDLE) != 0;
}

/* HW mode: C = A + B*B using HLS accelerator */
void hw_mat_compute(uint8_t A[MAT_SIZE][MAT_SIZE], uint8_t B[MAT_SIZE][MAT_SIZE], uint8_t C[MAT_SIZE][MAT_SIZE]) {
    /* Wait for accelerator to be idle */
    while (!hls_is_idle());

    /* Write input matrices */
    hls_write_matrix(HLS_A_BASE, A);
    hls_write_matrix(HLS_B_BASE, B);

    /* Start computation */
    hls_start();

    /* Wait for completion */
    hls_wait_done();

    /* Read result matrix */
    hls_read_matrix(HLS_C_BASE, C);
}

/*===========================================================================*/
/* Mode Selection                                                            */
/*===========================================================================*/

/* Read mode from DIP switch[0]: 0=SW, 1=HW */
uint32_t get_mode(void) {
    return Xil_In32(GPIO_SW_DATA) & 0x01;
}

/*===========================================================================*/
/* Utility Functions                                                         */
/*===========================================================================*/

void delay(void) {
    for (volatile size_t i = 0; i < DELAY_COUNT; i++);
}

/*===========================================================================*/
/* Main Application                                                          */
/*===========================================================================*/

int main() {
    init_platform();

    /* Test matrices (same as Lab 1) */
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

    uint8_t C[MAT_SIZE][MAT_SIZE]; /* Result: C = A + B*B */

    xil_printf("Lab 4: Heterogeneous SoC Architecture\r\n");
    xil_printf("Switch[0]: 0=SW Mode, 1=HW Mode\r\n");
    xil_printf("==========================================\r\n");

    uint32_t last_mode = 0xFF; /* Invalid initial value */

    /* Main loop - continuously compute and display results */
    while (1) {
        uint32_t mode = get_mode();

        /* Report mode change */
        if (mode != last_mode) {
            if (mode == MODE_SW) {
                xil_printf("Mode: SOFTWARE (MicroBlaze)\r\n");
            } else {
                xil_printf("Mode: HARDWARE (HLS Accelerator)\r\n");
            }
            last_mode = mode;
        }

        /* Show mode on LED[0] */
        uint16_t leds = mode;
        Xil_Out16(GPIO_LED_DATA, leds);

        /* Compute C = A + B*B */
        if (mode == MODE_SW) {
            sw_mat_compute(A, B, C);
        } else {
            hw_mat_compute(A, B, C);
        }

        /* Show computation complete on LED[1] */
        leds = mode | 0x0002;
        Xil_Out16(GPIO_LED_DATA, leds);

        delay();

        /* Display result matrix on LEDs (cycle through elements) */
        for (size_t i = 0; i < MAT_SIZE; i++) {
            for (size_t j = 0; j < MAT_SIZE; j++) {
                /* LED[15:8] = matrix value, LED[0] = mode */
                leds = (C[i][j] << MAT_LED_START) | mode;
                Xil_Out16(GPIO_LED_DATA, leds);
                delay();

                /* Brief off period */
                Xil_Out16(GPIO_LED_DATA, mode);
                delay();

                /* Check for mode change during display */
                if (get_mode() != mode) {
                    break; /* Exit to update mode immediately */
                }
            }
            if (get_mode() != mode) {
                break;
            }
        }
    }

    cleanup_platform();
    return 0;
}
