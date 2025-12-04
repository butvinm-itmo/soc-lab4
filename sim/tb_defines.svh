`ifndef TB_DEFINES_SVH
`define TB_DEFINES_SVH

// Matrix dimensions
`define MATRIX_SIZE 49  // 7x7 matrix flattened
`define MATRIX_DIM 7    // Matrix dimension

// Test configuration
`define TEST_RUNS 4     // Number of test iterations (2 SW + 2 HW)

// Mode definitions
`define MODE_SW 1'b0    // Software mode (MicroBlaze only)
`define MODE_HW 1'b1    // Hardware mode (HLS accelerator)

`endif  // TB_DEFINES_SVH
