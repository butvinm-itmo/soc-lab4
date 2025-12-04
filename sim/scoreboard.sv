`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Lab 4: Heterogeneous SoC Architecture
// Scoreboard Module - Verifies results and compares SW vs HW performance
//////////////////////////////////////////////////////////////////////////////////

`include "tb_defines.svh"

module scoreboard (
    input clk_i,
    input rst_i,
    input [15:0] matrix_input[`MATRIX_SIZE],
    input matrix_vld,
    input [15:0] result_matrix[`MATRIX_SIZE],
    input result_vld,
    input current_mode,
    output logic test_done
);
    localparam WAIT_A = 0, WAIT_B = 1, WAIT_RESULT = 2, CHECK = 3;

    logic [15:0] matrix_a[`MATRIX_SIZE];
    logic [15:0] matrix_b[`MATRIX_SIZE];
    logic [1:0] state;

    integer log_file, start_time, end_time;
    integer test_count, tests_passed, tests_failed;
    integer sw_total_time, hw_total_time, sw_test_count, hw_test_count;

    initial begin
        log_file = $fopen("scoreboard_log.txt", "w");
        $fwrite(log_file, "Lab 4: Heterogeneous SoC - Test Results\n");
        $fwrite(log_file, "========================================\n\n");
    end

    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            state <= WAIT_A;
            test_done <= 0;
            test_count <= 0;
            tests_passed <= 0;
            tests_failed <= 0;
            sw_total_time <= 0;
            hw_total_time <= 0;
            sw_test_count <= 0;
            hw_test_count <= 0;
            for (int i = 0; i < `MATRIX_SIZE; i++) begin
                matrix_a[i] <= 0;
                matrix_b[i] <= 0;
            end
        end else if (state == WAIT_A) begin
            test_done <= 0;
            if (matrix_vld) begin
                matrix_a <= matrix_input;
                state <= WAIT_B;
            end
        end else if (state == WAIT_B) begin
            if (matrix_vld) begin
                matrix_b <= matrix_input;
                start_time = $time;
                state <= WAIT_RESULT;
            end
        end else if (state == WAIT_RESULT) begin
            if (result_vld) begin
                end_time = $time;
                state <= CHECK;
            end
        end else begin
            // CHECK state
            logic [15:0] expected_matrix[`MATRIX_SIZE];
            logic [15:0] bb_matrix[`MATRIX_SIZE];
            logic test_passed;
            integer exec_time;

            test_passed = 1;
            test_count = test_count + 1;
            exec_time = end_time - start_time;

            if (current_mode == `MODE_SW) begin
                sw_total_time = sw_total_time + exec_time;
                sw_test_count = sw_test_count + 1;
            end else begin
                hw_total_time = hw_total_time + exec_time;
                hw_test_count = hw_test_count + 1;
            end

            // Compute expected: C = A + B*B
            for (int i = 0; i < `MATRIX_DIM; i++) begin
                for (int j = 0; j < `MATRIX_DIM; j++) begin
                    bb_matrix[i*`MATRIX_DIM+j] = 0;
                    for (int k = 0; k < `MATRIX_DIM; k++) begin
                        bb_matrix[i*`MATRIX_DIM+j] = bb_matrix[i*`MATRIX_DIM+j] +
                            (matrix_b[i*`MATRIX_DIM+k] * matrix_b[k*`MATRIX_DIM+j]);
                    end
                end
            end
            for (int i = 0; i < `MATRIX_SIZE; i++)
                expected_matrix[i] = matrix_a[i] + bb_matrix[i];

            // Verify
            for (int i = 0; i < `MATRIX_SIZE; i++) begin
                if (expected_matrix[i] != result_matrix[i]) begin
                    $display("MISMATCH [%0d]: exp=%0d got=%0d", i, expected_matrix[i], result_matrix[i]);
                    test_passed = 0;
                end
            end

            $display("Test #%0d [%s]: %s (time: %0t ns)",
                test_count, current_mode == `MODE_SW ? "SW" : "HW",
                test_passed ? "PASSED" : "FAILED", exec_time);

            $fwrite(log_file, "Test #%0d [%s]: %s (time: %0t ns)\n",
                test_count, current_mode == `MODE_SW ? "SW" : "HW",
                test_passed ? "PASSED" : "FAILED", exec_time);

            if (test_passed) tests_passed = tests_passed + 1;
            else tests_failed = tests_failed + 1;

            test_done <= 1;
            state <= WAIT_A;
        end
    end

    final begin
        real avg_sw, avg_hw, speedup;
        $display("\n========== FINAL SUMMARY ==========");
        $display("Total: %0d | Passed: %0d | Failed: %0d", test_count, tests_passed, tests_failed);

        if (sw_test_count > 0) begin
            avg_sw = real'(sw_total_time) / real'(sw_test_count);
            $display("SW avg: %0.0f ns", avg_sw);
        end
        if (hw_test_count > 0) begin
            avg_hw = real'(hw_total_time) / real'(hw_test_count);
            $display("HW avg: %0.0f ns", avg_hw);
        end
        if (sw_test_count > 0 && hw_test_count > 0 && avg_hw > 0) begin
            speedup = avg_sw / avg_hw;
            $display("Speedup: %.2fx", speedup);
        end
        $display("====================================\n");
        $fclose(log_file);
    end
endmodule
