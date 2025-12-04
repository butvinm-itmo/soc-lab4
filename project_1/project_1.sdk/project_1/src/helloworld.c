/******************************************************************************
 * Lab 4: Heterogeneous SoC Architecture
 *
 * MicroBlaze software with dual-mode computation:
 *   - SW Mode (Switch[0] = 0): Software-only matrix computation
 *   - HW Mode (Switch[0] = 1): Hardware accelerator (HLS mat_compute)
 *
 * Algorithm: C = A + B*B for 7x7 uint8 matrices
 *
 * GPIO Protocol (compatible with Lab2 testbench):
 *   Input:  gpio_switch[15]=1 with data in [7:0], mode in [0]
 *   Ack:    gpio_led[15] pulse
 *   Output: gpio_led[14]=1 with data in [7:0] (low then high byte)
 ******************************************************************************/

#include "xil_io.h"
#include <stdint.h>
#include <stdio.h>
#include "platform.h"

/*===========================================================================*/
/* Hardware Addresses                                                        */
/*===========================================================================*/

#define GPIO_BASE       0x40000000
#define GPIO_LED_DATA   (GPIO_BASE + 0x0000)  /* Channel 1: LED outputs */
#define GPIO_SW_DATA    (GPIO_BASE + 0x0008)  /* Channel 2: Switch inputs */

/* HLS mat_compute Accelerator */
#define HLS_BASE        0x44A00000
#define HLS_CTRL        (HLS_BASE + 0x00)
#define HLS_A_BASE      (HLS_BASE + 0x40)
#define HLS_B_BASE      (HLS_BASE + 0x80)
#define HLS_C_BASE      (HLS_BASE + 0xC0)

#define AP_START        0x01
#define AP_DONE         0x02
#define AP_IDLE         0x04

/*===========================================================================*/
/* Constants                                                                 */
/*===========================================================================*/

#define MAT_SIZE        7
#define MAT_ELEMENTS    49
#define MODE_SW         0
#define MODE_HW         1

/*===========================================================================*/
/* GPIO Communication Protocol                                               */
/*===========================================================================*/

uint16_t get_value(void) {
    uint16_t value = Xil_In16(GPIO_SW_DATA);

    /* Wait for gpio_switch[15] = 1 (valid data from testbench) */
    while ((value & 0x8000) == 0) {
        value = Xil_In16(GPIO_SW_DATA);
    }

    /* Extract 8-bit value from lower bits */
    value = value & 0x00FF;

    /* Send acknowledgment: pulse gpio_led[15] */
    Xil_Out16(GPIO_LED_DATA, 0x8000);
    Xil_Out16(GPIO_LED_DATA, 0x0000);

    return value;
}

void send_value(uint16_t value) {
    /* Send low byte with gpio_led[14]=1 as strobe */
    uint16_t low_byte = (value & 0x00FF) | 0x4000;
    Xil_Out16(GPIO_LED_DATA, low_byte);
    Xil_Out16(GPIO_LED_DATA, 0x0000);

    /* Send high byte with gpio_led[14]=1 as strobe */
    uint16_t high_byte = ((value >> 8) & 0x00FF) | 0x4000;
    Xil_Out16(GPIO_LED_DATA, high_byte);
    Xil_Out16(GPIO_LED_DATA, 0x0000);
}

uint32_t get_mode(void) {
    return (Xil_In16(GPIO_SW_DATA) >> 8) & 0x01;
}

/*===========================================================================*/
/* Software Matrix Operations                                                */
/*===========================================================================*/

void mat_mul(uint16_t A[MAT_SIZE][MAT_SIZE], uint16_t B[MAT_SIZE][MAT_SIZE], uint16_t C[MAT_SIZE][MAT_SIZE]) {
    for (int i = 0; i < MAT_SIZE; i++) {
        for (int j = 0; j < MAT_SIZE; j++) {
            C[i][j] = 0;
            for (int k = 0; k < MAT_SIZE; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void mat_add(uint16_t A[MAT_SIZE][MAT_SIZE], uint16_t B[MAT_SIZE][MAT_SIZE], uint16_t C[MAT_SIZE][MAT_SIZE]) {
    for (int i = 0; i < MAT_SIZE; i++) {
        for (int j = 0; j < MAT_SIZE; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

void sw_mat_compute(uint16_t A[MAT_SIZE][MAT_SIZE], uint16_t B[MAT_SIZE][MAT_SIZE], uint16_t C[MAT_SIZE][MAT_SIZE]) {
    uint16_t temp[MAT_SIZE][MAT_SIZE];
    mat_mul(B, B, temp);
    mat_add(A, temp, C);
}

/*===========================================================================*/
/* Hardware Accelerator Driver                                               */
/*===========================================================================*/

void hls_write_matrix(uint32_t base_addr, uint16_t mat[MAT_SIZE][MAT_SIZE]) {
    uint8_t flat[MAT_ELEMENTS];
    for (int i = 0; i < MAT_SIZE; i++) {
        for (int j = 0; j < MAT_SIZE; j++) {
            flat[i * MAT_SIZE + j] = (uint8_t)(mat[i][j] & 0xFF);
        }
    }
    for (int i = 0; i < 48; i += 4) {
        uint32_t word = flat[i] | (flat[i+1] << 8) | (flat[i+2] << 16) | (flat[i+3] << 24);
        Xil_Out32(base_addr + i, word);
    }
    Xil_Out32(base_addr + 48, flat[48]);
}

void hls_read_matrix(uint32_t base_addr, uint16_t mat[MAT_SIZE][MAT_SIZE]) {
    uint8_t flat[MAT_ELEMENTS];
    for (int i = 0; i < 48; i += 4) {
        uint32_t word = Xil_In32(base_addr + i);
        flat[i]   = word & 0xFF;
        flat[i+1] = (word >> 8) & 0xFF;
        flat[i+2] = (word >> 16) & 0xFF;
        flat[i+3] = (word >> 24) & 0xFF;
    }
    flat[48] = Xil_In32(base_addr + 48) & 0xFF;
    for (int i = 0; i < MAT_SIZE; i++) {
        for (int j = 0; j < MAT_SIZE; j++) {
            mat[i][j] = flat[i * MAT_SIZE + j];
        }
    }
}

void hw_mat_compute(uint16_t A[MAT_SIZE][MAT_SIZE], uint16_t B[MAT_SIZE][MAT_SIZE], uint16_t C[MAT_SIZE][MAT_SIZE]) {
    while (!(Xil_In32(HLS_CTRL) & AP_IDLE));
    hls_write_matrix(HLS_A_BASE, A);
    hls_write_matrix(HLS_B_BASE, B);
    Xil_Out32(HLS_CTRL, AP_START);
    while (!(Xil_In32(HLS_CTRL) & AP_DONE));
    hls_read_matrix(HLS_C_BASE, C);
}

/*===========================================================================*/
/* Main Application                                                          */
/*===========================================================================*/

int main() {
    init_platform();

    while (1) {
        uint16_t A[MAT_SIZE][MAT_SIZE];
        uint16_t B[MAT_SIZE][MAT_SIZE];
        uint16_t C[MAT_SIZE][MAT_SIZE];

        /* Receive matrix A via GPIO */
        for (int i = 0; i < MAT_SIZE; i++) {
            for (int j = 0; j < MAT_SIZE; j++) {
                A[i][j] = get_value();
            }
        }

        /* Receive matrix B via GPIO */
        for (int i = 0; i < MAT_SIZE; i++) {
            for (int j = 0; j < MAT_SIZE; j++) {
                B[i][j] = get_value();
            }
        }

        /* Compute C = A + B*B based on mode */
        if (get_mode() == MODE_SW) {
            sw_mat_compute(A, B, C);
        } else {
            hw_mat_compute(A, B, C);
        }

        /* Send result matrix C via GPIO */
        for (int i = 0; i < MAT_SIZE; i++) {
            for (int j = 0; j < MAT_SIZE; j++) {
                send_value(C[i][j]);
            }
        }
    }

    cleanup_platform();
    return 0;
}
