# Lab 4: Heterogeneous SoC Architecture

## Overview

This lab integrates the HLS hardware accelerator (developed in Lab 3) into the MicroBlaze SoC (from Lab 1) to create a heterogeneous system-on-chip with dual computation modes.

**Algorithm**: C = A + B×B for 7×7 uint8 matrices

## Features

- **Dual-Mode Operation**: Switch between SW and HW computation in real-time
- **HLS Accelerator**: Pipeline-optimized mat_compute IP from Lab 3
- **Real-time Switching**: DIP switch[0] controls mode without reset
- **Performance Comparison**: ~2.2x speedup with HW accelerator

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    MicroBlaze SoC                           │
│  ┌───────────────┐                                          │
│  │  MicroBlaze   │──────┐                                   │
│  │    CPU        │      │                                   │
│  └───────────────┘      │                                   │
│         │               │                                   │
│         │ LMB           │ AXI                               │
│         ▼               ▼                                   │
│  ┌───────────────┐  ┌─────────────────────────────────┐    │
│  │  Local BRAM   │  │     AXI Interconnect            │    │
│  │   (32 KB)     │  │  ┌──────┬──────┬──────────┐     │    │
│  └───────────────┘  │  │M00   │M01   │M02       │     │    │
│                     │  └──┬───┴──┬───┴────┬─────┘     │    │
│                     └─────┼──────┼────────┼───────────┘    │
│                           │      │        │                 │
│                           ▼      ▼        ▼                 │
│                     ┌─────────┐┌──────┐┌───────────┐       │
│                     │UART     ││GPIO  ││mat_compute│       │
│                     │Lite     ││(Dual)││HLS IP     │       │
│                     │@0x40600K││@0x40M││@0x44A00K  │       │
│                     └─────────┘└──┬───┘└───────────┘       │
│                                   │                         │
└───────────────────────────────────┼─────────────────────────┘
                                    │
                          ┌─────────┴─────────┐
                          │  Ch1: LEDs (out)  │
                          │  Ch2: Switches(in)│
                          └───────────────────┘
```

## Address Map

| Peripheral | Base Address | Size | Description |
|------------|--------------|------|-------------|
| Local BRAM | 0x00000000 | 32KB | Program + Data memory |
| AXI GPIO | 0x40000000 | 64KB | LEDs (Ch1) + Switches (Ch2) |
| AXI UARTLite | 0x40600000 | 64KB | Serial debug output |
| mat_compute | 0x44A00000 | 256B | HLS accelerator |

## HLS IP Register Map

| Offset | Register | Description |
|--------|----------|-------------|
| 0x00 | Control | ap_start[0], ap_done[1], ap_idle[2], ap_ready[3] |
| 0x04 | GIE | Global Interrupt Enable |
| 0x08 | IER | IP Interrupt Enable |
| 0x0C | ISR | IP Interrupt Status |
| 0x40-0x7F | Matrix A | 49 × 8-bit elements (packed 4/word) |
| 0x80-0xBF | Matrix B | 49 × 8-bit elements (packed 4/word) |
| 0xC0-0xFF | Matrix C | 49 × 8-bit result elements |

## Building the Project

### Prerequisites
- Xilinx Vivado 2019.1
- Xilinx SDK 2019.1
- Lab 3 HLS IP exported (mat_comparison_pipeline)

### Steps

1. **Create Vivado Project**:
   ```bash
   cd /path/to/lab4
   vivado -mode tcl -source create_project.tcl
   ```

2. **Generate Bitstream**:
   - Open project in Vivado GUI
   - Run Synthesis → Implementation → Generate Bitstream

3. **Export to SDK**:
   - File → Export → Export Hardware (include bitstream)
   - File → Launch SDK

4. **Build Software**:
   - Create new Application Project
   - Copy `sdk/src/helloworld.c`
   - Build Project

5. **Program FPGA**:
   - Xilinx → Program FPGA
   - Run As → Launch on Hardware

## Usage

### Mode Selection
- **SW[0] = 0**: Software mode (MicroBlaze computes)
- **SW[0] = 1**: Hardware mode (HLS accelerator computes)

### LED Indicators
- **LED[0]**: Current mode (0=SW, 1=HW)
- **LED[1]**: Computation complete
- **LED[15:8]**: Matrix result values (cycling)

### UART Output (115200 baud)
```
Lab 4: Heterogeneous SoC Architecture
Switch[0]: 0=SW Mode, 1=HW Mode
==========================================
Mode: SOFTWARE (MicroBlaze)
Mode: HARDWARE (HLS Accelerator)
```

## Simulation

Run behavioral simulation in Vivado:
1. Add simulation sources from `project_1.srcs/sim_1/new/`
2. Set `dev_tb` as top module
3. Run Behavioral Simulation

The testbench will:
- Run 2 tests in SW mode
- Switch to HW mode
- Run 2 tests in HW mode
- Compare and report timing speedup

## Expected Performance

| Mode | Latency | Time @ 100MHz | Speedup |
|------|---------|---------------|---------|
| SW (MicroBlaze) | ~1600+ cycles | ~16+ µs | 1.0x |
| HW (Pipeline) | 741 cycles | 7.41 µs | ~2.2x |

## Files

```
lab4/
├── create_project.tcl              # Vivado project creation script
├── sdk/src/
│   └── helloworld.c                # MicroBlaze application
├── project_1/project_1.srcs/
│   ├── sim_1/new/
│   │   ├── tb_defines.svh          # Test constants
│   │   ├── dev_tb.v                # Top testbench
│   │   ├── agent.sv                # Test coordinator
│   │   ├── driver.sv               # GPIO stimulus
│   │   ├── sequencer.sv            # Test generator
│   │   ├── monitor.sv              # Output capture
│   │   └── scoreboard.sv           # Result verification
│   └── constrs_1/new/
│       └── constraints.xdc         # Timing constraints
├── README.md                       # This file
└── CLAUDE.md                       # Technical reference
```

## References

- Lab 1: Basic MicroBlaze SoC
- Lab 2: GPIO switch interface
- Lab 3: HLS mat_compute accelerator (Pipeline variant)
