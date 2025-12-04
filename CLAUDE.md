# Lab 4: Heterogeneous SoC - Technical Reference

## Project Overview

Heterogeneous SoC combining MicroBlaze soft processor with HLS hardware accelerator for matrix computation (C = A + B×B). Supports both simulation verification and FPGA demo.

## Project Structure

```
lab4/
├── modify_project.tcl          # TCL script to modify lab1 project
├── constraints.xdc             # Timing/bitstream constraints
├── helloworld_fpga.c           # FPGA version (hardcoded matrices, LED output)
├── sim/
│   ├── tb_defines.svh          # Test constants
│   ├── dev_tb.v                # Top-level testbench
│   ├── agent.sv                # Test coordinator with mode switching
│   ├── driver.sv               # GPIO stimulus driver
│   ├── sequencer.sv            # Test pattern generator
│   ├── monitor.sv              # Output capture
│   └── scoreboard.sv           # Result verification + timing
└── project_1/                  # Vivado project
    └── project_1.sdk/
        └── project_1/src/
            └── helloworld.c    # Testbench version (GPIO protocol)
```

## Two Software Versions

### 1. Testbench Version (`project_1.sdk/.../helloworld.c`)
- **Purpose**: Simulation verification with UVM-like testbench
- **Input**: Receives matrices via GPIO handshake protocol
- **Mode**: Controlled by testbench via gpio_switch[8]
- **Protocol**:
  - TB sends data: `gpio_switch[15]=1`, data in `[7:0]`
  - SW acknowledges: pulse `gpio_led[15]`
  - SW sends result: `gpio_led[14]=1`, data in `[7:0]` (low byte, then high byte)

### 2. FPGA Version (`helloworld_fpga.c`)
- **Purpose**: Demo on real FPGA hardware
- **Input**: Hardcoded test matrices
- **Mode**: Physical switch SW[0]
- **Output**: LED[0]=mode, LED[15:8]=result values (cycling)

## Simulation Setup

### Associate ELF for Simulation
```tcl
add_files /path/to/project_1.elf
set_property SCOPED_TO_REF design_1 [get_files project_1.elf]
set_property SCOPED_TO_CELLS microblaze_0 [get_files project_1.elf]
reset_target simulation [get_files design_1.bd]
generate_target simulation [get_files design_1.bd]
```

### Verify BRAM Uses Correct ELF
Check `project_1.sim/sim_1/behav/xsim/design_1_lmb_bram_0.mem`:
- Should reference `project_1.elf`, NOT `mb_bootloop_le.elf`

### Run Simulation
1. Open `project_1/project_1.xpr`
2. Flow Navigator → Run Simulation → Run Behavioral Simulation

## Testbench Architecture

```
dev_tb
├── agent
│   ├── sequencer    # Generates random test matrices
│   ├── driver       # Sends matrices via GPIO, waits for ACK
│   ├── monitor      # Captures result from gpio_led
│   └── scoreboard   # Verifies results, measures timing
└── design_1_wrapper (DUT)
```

### Test Flow
1. Sequencer generates random matrices A and B
2. Driver sends A (49 elements) via GPIO handshake
3. Driver sends B (49 elements) via GPIO handshake
4. MicroBlaze computes C = A + B×B (SW or HW mode)
5. MicroBlaze sends C (49 elements) via GPIO
6. Monitor captures result
7. Scoreboard verifies and reports timing

### Mode Switching
From `tb_defines.svh`: `TEST_RUNS = 4`
- Tests 1-2: Software mode (gpio_switch[8] = 0)
- Tests 3-4: Hardware mode (gpio_switch[8] = 1)

## Address Map

| Peripheral | Base Address |
|------------|--------------|
| AXI GPIO | 0x40000000 |
| AXI UARTLite | 0x40600000 |
| mat_compute HLS | 0x44A00000 |
| Local BRAM | 0x00000000 |

## GPIO Protocol (Testbench)

### Input (TB → MicroBlaze)
- `gpio_switch[15]` = valid data flag
- `gpio_switch[8]` = mode (0=SW, 1=HW)
- `gpio_switch[7:0]` = data byte

### Output (MicroBlaze → TB)
- `gpio_led[15]` = ACK pulse (for input)
- `gpio_led[14]` = valid data flag (for output)
- `gpio_led[7:0]` = data byte

## FPGA Usage

1. Copy `helloworld_fpga.c` to SDK project src folder
2. Rebuild in SDK
3. Generate bitstream in Vivado
4. Program FPGA

**Controls:**
- **SW[0] = 0**: Software mode
- **SW[0] = 1**: Hardware mode
- **LED[0]**: Current mode
- **LED[15:8]**: Result values (cycling through matrix)

## Build Commands

```bash
# Open project
vivado project_1/project_1.xpr

# Generate bitstream (TCL console)
launch_runs synth_1 -jobs 4
wait_on_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
```

## Known Issues

1. **ELF not loaded in simulation**: Check BRAM .mem file references correct ELF. Re-run `generate_target simulation`.

2. **Simulation hangs**: MicroBlaze not responding means wrong ELF (bootloop). Re-associate ELF.

3. **Result mismatches**: Check gpio_switch bit assignments. Mode bit must not overwrite data bits.

## Dependencies

- Lab 3 HLS IP: `../lab3/mat_comparison_pipeline/solution/impl/ip/`
- Target: xc7a100tcsg324-1
- Tool: Vivado 2019.1
