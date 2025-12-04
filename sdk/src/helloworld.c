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

/* Mode definitions */
#define MODE_SW         0
#define MODE_HW         1

/*===========================================================================*/
/* GPIO Communication Protocol (Lab2 compatible)                             */
/*===========================================================================*/

/* Get a value from GPIO with handshake */
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

/* Send a 16-bit value via GPIO (low byte first, then high byte) */
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

/* Read current mode from switch[0] */
uint32_t get_mode(void) {
    return Xil_In16(GPIO_SW_DATA) & 0x01;
}

/*===========================================================================*/
/* Software Matrix Operations                                                */
/*===========================================================================*/

void mat_mul(uint16_t A[MAT_SIZE][MAT_SIZE], uint16_t B[MAT_SIZE][MAT_SIZE], uint16_t C[MAT_SIZE][MAT_SIZE]) {
    for (size_t i = 0; i < MAT_SIZE; i++) {
        for (size_t j = 0; j < MAT_SIZE; j++) {
            C[i][j] = 0;
            for (size_t k = 0; k < MAT_SIZE; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void mat_add(uint16_t A[MAT_SIZE][MAT_SIZE], uint16_t B[MAT_SIZE][MAT_SIZE], uint16_t C[MAT_SIZE][MAT_SIZE]) {
    for (size_t i = 0; i < MAT_SIZE; i++) {
        for (size_t j = 0; j < MAT_SIZE; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

/* SW mode: C = A + B*B */
void sw_mat_compute(uint16_t A[MAT_SIZE][MAT_SIZE], uint16_t B[MAT_SIZE][MAT_SIZE], uint16_t C[MAT_SIZE][MAT_SIZE]) {
    uint16_t temp[MAT_SIZE][MAT_SIZE];
    mat_mul(B, B, temp);   /* temp = B * B */
    mat_add(A, temp, C);   /* C = A + temp */
}

/*===========================================================================*/
/* Hardware Accelerator Driver                                               */
/*===========================================================================*/

/* Write a 7x7 matrix to HLS IP (packed 4 bytes per 32-bit word) */
void hls_write_matrix(uint32_t base_addr, uint16_t mat[MAT_SIZE][MAT_SIZE]) {
    uint8_t flat[MAT_ELEMENTS];

    /* Convert uint16 matrix to uint8 (truncate to 8 bits) */
    for (int i = 0; i < MAT_SIZE; i++) {
        for (int j = 0; j < MAT_SIZE; j++) {
            flat[i * MAT_SIZE + j] = (uint8_t)(mat[i][j] & 0xFF);
        }
    }

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
void hls_read_matrix(uint32_t base_addr, uint16_t mat[MAT_SIZE][MAT_SIZE]) {
    uint8_t flat[MAT_ELEMENTS];

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

    /* Convert to uint16 matrix */
    for (int i = 0; i < MAT_SIZE; i++) {
        for (int j = 0; j < MAT_SIZE; j++) {
            mat[i][j] = flat[i * MAT_SIZE + j];
        }
    }
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
void hw_mat_compute(uint16_t A[MAT_SIZE][MAT_SIZE], uint16_t B[MAT_SIZE][MAT_SIZE], uint16_t C[MAT_SIZE][MAT_SIZE]) {
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
/* Main Application                                                          */
/*===========================================================================*/

int main() {
    init_platform();

    xil_printf("Lab 4: Heterogeneous SoC Architecture\r\n");
    xil_printf("GPIO Protocol Mode - Waiting for testbench...\r\n");

    while (1) {
        uint16_t A[MAT_SIZE][MAT_SIZE];
        uint16_t B[MAT_SIZE][MAT_SIZE];
        uint16_t C[MAT_SIZE][MAT_SIZE];

        /* Receive matrix A via GPIO */
        for (size_t i = 0; i < MAT_SIZE; i++) {
            for (size_t j = 0; j < MAT_SIZE; j++) {
                A[i][j] = get_value();
            }
        }

        /* Receive matrix B via GPIO */
        for (size_t i = 0; i < MAT_SIZE; i++) {
            for (size_t j = 0; j < MAT_SIZE; j++) {
                B[i][j] = get_value();
            }
        }

        /* Check mode and compute C = A + B*B */
        uint32_t mode = get_mode();
        if (mode == MODE_SW) {
            sw_mat_compute(A, B, C);
        } else {
            hw_mat_compute(A, B, C);
        }

        /* Send result matrix C via GPIO */
        for (size_t i = 0; i < MAT_SIZE; i++) {
            for (size_t j = 0; j < MAT_SIZE; j++) {
                send_value(C[i][j]);
            }
        }
    }

    cleanup_platform();
    return 0;
}
