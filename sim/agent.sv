`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Lab 4: Heterogeneous SoC Architecture
// Agent Module - Coordinates verification components and mode switching
//////////////////////////////////////////////////////////////////////////////////

`include "tb_defines.svh"

module agent (
    input clk_i,
    input rst_i,
    output [15:0] gpio_switch,
    input [15:0] gpio_led
);

    logic [15:0] tmp_sequence[`MATRIX_SIZE];
    logic [15:0] result_sequence[`MATRIX_SIZE];
    logic sequence_valid, sequence_send, result_valid;
    logic test_done, test_start;

    integer test_counter;

    // Mode control
    logic current_mode;
    wire [15:0] driver_gpio_switch;

    sequencer sequencer_impl (
        .clk_i(clk_i),
        .rst_i(rst_i),
        .sequence_o(tmp_sequence),
        .sequence_valid_o(sequence_valid),
        .sequence_send_i(sequence_send),
        .test_start_i(test_start)
    );

    driver driver_impl (
        .clk(clk_i),
        .rst(rst_i),
        .sequence_i(tmp_sequence),
        .sequence_valid(sequence_valid),
        .sequence_send(sequence_send),
        .gpio_switch(driver_gpio_switch),
        .gpio_led(gpio_led)
    );

    monitor monitor_impl (
        .clk(clk_i),
        .rst(rst_i),
        .gpio_led(gpio_led),
        .result_matrix_o(result_sequence),
        .result_valid(result_valid),
        .test_done_i(test_done)
    );

    scoreboard scoreboard_impl (
        .clk_i(clk_i),
        .rst_i(rst_i),
        .matrix_input(tmp_sequence),
        .matrix_vld(sequence_valid),
        .result_matrix(result_sequence),
        .result_vld(result_valid),
        .current_mode(current_mode),
        .test_done(test_done)
    );

    // Combine driver GPIO with mode bit
    // gpio_switch[15:1] = driver data, gpio_switch[0] = mode
    assign gpio_switch = {driver_gpio_switch[15:1], current_mode};

    // Test orchestration with mode switching
    always_ff @(posedge clk_i) begin
        if (rst_i) begin
            test_counter <= 0;
            test_start   <= 1;
            current_mode <= `MODE_SW;
            $display("[%0t] AGENT: Reset - initializing test_start=1, mode=SW", $time);
        end else begin
            if (test_done) begin
                $display("[%0t] AGENT: Test %0d done", $time, test_counter);
                test_counter <= test_counter + 1;
                if (test_counter + 1 >= `TEST_RUNS) begin
                    $display("\n=== All %0d tests completed ===", `TEST_RUNS);
                    $finish();
                end else begin
                    if (test_counter + 1 == `TEST_RUNS / 2) begin
                        current_mode <= `MODE_HW;
                        $display("\n>>> Switching to HARDWARE mode <<<\n");
                    end
                    test_start <= 1;
                end
            end else begin
                test_start <= 0;
            end
        end
    end

    initial begin
        @(negedge rst_i);
        $display("[%0t] AGENT: Reset released", $time);
        $display("\n>>> Starting in SOFTWARE mode <<<\n");
    end

endmodule
