`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Lab 4: Heterogeneous SoC Architecture
// Driver Module - Drives test data to DUT via GPIO
//////////////////////////////////////////////////////////////////////////////////

`include "tb_defines.svh"

module driver (
    input clk,
    input rst,
    input [15:0] sequence_i[`MATRIX_SIZE],
    input sequence_valid,
    output logic sequence_send,
    output logic [15:0] gpio_switch,
    input [15:0] gpio_led
);
    localparam WAIT_SEQ = 0;
    localparam SEND_SEQ = 1;

    logic state;
    logic receive;
    logic [15:0] temp_matrix[`MATRIX_SIZE];
    logic [5:0] temp_idx;

    wire send_vld = gpio_switch[15];
    wire receive_vld = gpio_led[15];

    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < `MATRIX_SIZE; i++) temp_matrix[i] <= '0;
            temp_idx <= 0;
            state <= WAIT_SEQ;
            receive <= 0;
            sequence_send <= 0;
            gpio_switch <= 0;
            $display("[%0t] DRIVER: Reset", $time);
        end else begin
            case (state)
                WAIT_SEQ: begin
                    receive <= 0;
                    temp_idx <= 0;
                    sequence_send <= 0;
                    gpio_switch <= 0;
                    if (sequence_valid) begin
                        temp_matrix <= sequence_i;
                        state <= SEND_SEQ;
                        $display("[%0t] DRIVER: Got sequence, moving to SEND_SEQ", $time);
                    end
                end
                SEND_SEQ: begin
                    if (temp_idx == `MATRIX_SIZE) begin
                        state <= WAIT_SEQ;
                        sequence_send <= 1;
                        gpio_switch <= 0;
                        $display("[%0t] DRIVER: Finished sending all %0d elements", $time, `MATRIX_SIZE);
                    end else begin
                        if (send_vld) begin
                            // We've sent data, wait for ack
                            if (receive && !receive_vld) begin
                                // Ack went low, move to next
                                temp_idx <= temp_idx + 1'b1;
                                gpio_switch <= 0;
                                receive <= 0;
                                $display("[%0t] DRIVER: Sent element %0d/%0d, value=%h",
                                         $time, temp_idx + 1, `MATRIX_SIZE, temp_matrix[temp_idx]);
                            end else if (receive_vld && !receive) begin
                                // Got ack high
                                receive <= 1;
                                $display("[%0t] DRIVER: Got ACK for element %0d", $time, temp_idx);
                            end
                        end else begin
                            // Send data with bit 15 set
                            gpio_switch <= temp_matrix[temp_idx] | 16'h8000;
                        end
                    end
                end
            endcase
        end
    end
endmodule
