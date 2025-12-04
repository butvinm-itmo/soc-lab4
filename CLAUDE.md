# Lab 4: Heterogeneous SoC - Technical Reference

## Project Overview

Heterogeneous SoC combining MicroBlaze soft processor with HLS hardware accelerator for matrix computation (C = A + B×B). Real-time mode switching via DIP switch.

## Project Structure

```
lab4/
├── modify_project.tcl          # TCL script to modify lab1 project
├── constraints.xdc             # Timing/bitstream constraints
├── README.md                   # User documentation
├── CLAUDE.md                   # This file
├── sdk/src/
│   └── helloworld.c            # MicroBlaze dual-mode application
├── sim/
│   ├── tb_defines.svh          # Test constants
│   ├── dev_tb.v                # Top-level testbench
│   ├── agent.sv                # Test coordinator with mode switching
│   ├── driver.sv               # GPIO stimulus driver
│   ├── sequencer.sv            # Test pattern generator
│   ├── monitor.sv              # Output capture
│   └── scoreboard.sv           # Result verification + timing
└── project_1/                  # Vivado project (copied from lab1)
```

## Build Process

The project is built by copying lab1 and modifying it:

```bash
# 1. Copy lab1 as template (already done)
cp -r ../lab1/project_1 .

# 2. Run modification script
vivado -mode tcl -source modify_project.tcl

# 3. Open in GUI for synthesis/implementation
vivado project_1/project_1.xpr
```

## Key Components

### Block Design
- MicroBlaze v11.0 processor
- AXI GPIO (dual-channel): LEDs (Ch1) + Switches (Ch2)
- AXI UARTLite: Debug serial output
- mat_compute HLS IP: Pipeline-optimized accelerator (from lab3)
- AXI Interconnect: 3 master ports

### Software (sdk/src/helloworld.c)
- Mode selection via GPIO switch[0]
- SW functions: `mat_mul()`, `mat_add()`, `sw_mat_compute()`
- HW driver: `hls_write_matrix()`, `hls_read_matrix()`, `hls_start()`, `hls_wait_done()`
- Main loop: Continuous compute and LED display

### Testbench (sim/*.sv)
- UVM-like architecture from Lab 2
- Mode switching: 2 SW tests → 2 HW tests
- Timing comparison and speedup calculation

## Address Map

| Peripheral | Base Address | Size |
|------------|--------------|------|
| AXI GPIO | 0x40000000 | 64KB |
| AXI UARTLite | 0x40600000 | 64KB |
| mat_compute HLS | 0x44A00000 | 64KB |
| Local BRAM | 0x00000000 | 32KB |

## Register Addresses

```c
// GPIO
#define GPIO_BASE       0x40000000
#define GPIO_LED_DATA   0x40000000  // Channel 1 (output)
#define GPIO_SW_DATA    0x40000008  // Channel 2 (input)

// HLS Accelerator
#define HLS_BASE        0x44A00000
#define HLS_CTRL        0x44A00000  // Control register
#define HLS_A_BASE      0x44A00040  // Matrix A (49 x 8-bit)
#define HLS_B_BASE      0x44A00080  // Matrix B (49 x 8-bit)
#define HLS_C_BASE      0x44A000C0  // Matrix C (result)

// Control bits
#define AP_START        0x01
#define AP_DONE         0x02
#define AP_IDLE         0x04
```

## Matrix Packing (HLS IP)

Matrices are packed 4 bytes per 32-bit word:
```
Word n: [A[4n+3]:A[4n+2]:A[4n+1]:A[4n]]
        [bits 31:24]:[23:16]:[15:8]:[7:0]
```

For 49 elements: 12 full words + 1 partial word (element 48)

## Mode Protocol

1. Read switch[0] from GPIO_SW_DATA
2. If mode == 0: Call sw_mat_compute()
3. If mode == 1: Call hw_mat_compute()
4. Display mode on LED[0]

## HW Accelerator Protocol

```c
void hw_mat_compute(A, B, C) {
    while (!(HLS_CTRL & AP_IDLE));  // Wait idle
    hls_write_matrix(HLS_A_BASE, A);
    hls_write_matrix(HLS_B_BASE, B);
    HLS_CTRL = AP_START;            // Start
    while (!(HLS_CTRL & AP_DONE));  // Wait done
    hls_read_matrix(HLS_C_BASE, C);
}
```

## Build Commands

```bash
# Modify project (from lab4 directory)
vivado -mode tcl -source modify_project.tcl

# Or open GUI directly
vivado project_1/project_1.xpr

# Generate bitstream (in Vivado TCL console)
launch_runs synth_1 -jobs 4
wait_on_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
```

## Known Issues & Solutions

1. **Board part not found**: Lab1 uses Nexys4 DDR board files which may not be installed. The modify_project.tcl script uses `upgrade_ip` to handle this.

2. **Locked IPs**: When copying from lab1, IPs are locked due to board mismatch. Script automatically upgrades them.

3. **HLS IP not found**: Ensure lab3 pipeline variant exists at:
   `../lab3/mat_comparison_pipeline/solution/impl/ip/`

## Performance Metrics

| Mode | Latency | Time @ 100MHz |
|------|---------|---------------|
| SW (MicroBlaze) | ~1600+ cycles | ~16+ µs |
| HW (Pipeline) | 741 cycles | 7.41 µs |
| **Speedup** | **~2.2x** | |

## Dependencies

- Lab 1 project: Base MicroBlaze SoC
- Lab 3 HLS IP: `../lab3/mat_comparison_pipeline/solution/impl/ip/`
- Target: xc7a100tcsg324-1
- Tool: Vivado 2019.1

## FPGA Testing

- **SW[0] = 0**: Software mode (LED[0] off)
- **SW[0] = 1**: Hardware mode (LED[0] on)
- **LED[15:8]**: Matrix result values (cycling)
- Mode switching is real-time, no reset required
