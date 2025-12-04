`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Lab 4: Heterogeneous SoC Architecture
// Monitor Module - Captures DUT output results
//////////////////////////////////////////////////////////////////////////////////

`include "tb_defines.svh"

module monitor (
    input clk,
    input rst,
    input [15:0] gpio_led,
    output logic [15:0] result_matrix_o[`MATRIX_SIZE],
    output result_valid,
    input test_done_i
);
    localparam IDLE = 0, RECEIVE_LOW = 1, DELAY_LOW = 2, RECEIVE_HIGH = 3, DELAY_HIGH = 4;

    logic [5:0] temp_idx;
    logic [2:0] state;
    logic [7:0] low_byte;
    logic reception_done;

    always_ff @(posedge clk) begin
        if (rst) begin
            temp_idx <= 0;
            state <= IDLE;
            low_byte <= 0;
            reception_done <= 0;
            for (int i = 0; i < `MATRIX_SIZE; i++) result_matrix_o[i] <= '0;
            $display("[%0t] MONITOR: Reset", $time);
        end else begin
            if (test_done_i) begin
                temp_idx <= 0;
                state <= IDLE;
                low_byte <= 0;
                reception_done <= 0;
                $display("[%0t] MONITOR: Test done, resetting", $time);
            end else if (!reception_done) begin
                case (state)
                    IDLE: begin
                        // Wait for gpio_led[14] to go low (ready state)
                        if (!gpio_led[14]) begin
                            state <= RECEIVE_LOW;
                            $display("[%0t] MONITOR: Starting reception", $time);
                        end
                    end
                    RECEIVE_LOW: begin
                        // Wait for gpio_led[14]=1 with low byte
                        if (gpio_led[14]) begin
                            low_byte <= gpio_led[7:0];
                            state <= DELAY_LOW;
                        end
                    end
                    DELAY_LOW: begin
                        // Wait for gpio_led[14] to go low
                        if (!gpio_led[14]) state <= RECEIVE_HIGH;
                    end
                    RECEIVE_HIGH: begin
                        // Wait for gpio_led[14]=1 with high byte
                        if (gpio_led[14]) begin
                            if (temp_idx < `MATRIX_SIZE) begin
                                result_matrix_o[temp_idx] <= {gpio_led[7:0], low_byte};
                                $display("[%0t] MONITOR: Received element %0d = %h",
                                         $time, temp_idx, {gpio_led[7:0], low_byte});
                                temp_idx <= temp_idx + 1'b1;
                                if (temp_idx + 1 == `MATRIX_SIZE) begin
                                    reception_done <= 1;
                                    $display("[%0t] MONITOR: All elements received!", $time);
                                end
                            end
                            state <= DELAY_HIGH;
                        end
                    end
                    DELAY_HIGH: begin
                        // Wait for gpio_led[14] to go low
                        if (!gpio_led[14]) state <= RECEIVE_LOW;
                    end
                endcase
            end
        end
    end

    assign result_valid = (temp_idx == `MATRIX_SIZE);
endmodule
