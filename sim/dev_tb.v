`timescale 1ns / 1ns
//////////////////////////////////////////////////////////////////////////////////
// Lab 4: Heterogeneous SoC Architecture
// Top-level Testbench
//////////////////////////////////////////////////////////////////////////////////

module dev_tb ();
    localparam MATRIX_SIZE = 49;

    wire [15:0] gpio_switch;
    wire [15:0] gpio_led;
    reg rst_n;
    reg clk;
    reg usb_uart_rxd;
    wire usb_uart_txd;

    agent agent_impl (
        .clk_i(clk),
        .rst_i(~rst_n),
        .gpio_switch(gpio_switch),
        .gpio_led(gpio_led)
    );

    design_1_wrapper uut (
        .dip_switches_16bits_tri_i(gpio_switch),
        .led_16bits_tri_o(gpio_led),
        .reset(rst_n),
        .sys_clock(clk),
        .usb_uart_rxd(usb_uart_rxd),
        .usb_uart_txd(usb_uart_txd)
    );

    initial begin
        rst_n = 0;
        clk = 0;
        usb_uart_rxd = 0;
        #200 rst_n = 1;

        $display("============================================");
        $display("Lab 4: Heterogeneous SoC Testbench");
        $display("Testing SW and HW computation modes");
        $display("============================================");
    end

    always #5 clk = ~clk;
endmodule
