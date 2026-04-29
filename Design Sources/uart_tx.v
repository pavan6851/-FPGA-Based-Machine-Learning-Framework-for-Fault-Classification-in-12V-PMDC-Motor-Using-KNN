// =============================================================
//  uart_tx.v  -  Simple UART Transmitter
//  8N1, baud rate set by CLK_DIV parameter
//  115200 baud @ 100 MHz  →  CLK_DIV = 868
//
//  Interface:
//    tx_data_i  [7:0]  - byte to transmit
//    tx_valid_i        - pulse high 1 cycle to start transmission
//    tx_ready_o        - high when idle (safe to send next byte)
//    tx_o              - UART TX line
// =============================================================

`timescale 1ns / 1ps

module uart_tx #(
    parameter CLK_DIV = 868   // 100 MHz / 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] tx_data_i,
    input  wire       tx_valid_i,
    output reg        tx_ready_o,
    output reg        tx_o
);

    reg [9:0]  shift_reg;   // start + 8 data + stop
    reg [9:0]  clk_cnt;
    reg [3:0]  bit_cnt;
    reg        busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_o       <= 1'b1;
            tx_ready_o <= 1'b1;
            busy       <= 1'b0;
            clk_cnt    <= 0;
            bit_cnt    <= 0;
            shift_reg  <= 10'h3FF;
        end else begin
            if (!busy) begin
                tx_ready_o <= 1'b1;
                if (tx_valid_i) begin
                    // Load: start=0, 8 data bits LSB first, stop=1
                    shift_reg  <= {1'b1, tx_data_i, 1'b0};
                    busy       <= 1'b1;
                    tx_ready_o <= 1'b0;
                    clk_cnt    <= 0;
                    bit_cnt    <= 0;
                    tx_o       <= 1'b0;  // start bit
                end
            end else begin
                if (clk_cnt == CLK_DIV - 1) begin
                    clk_cnt   <= 0;
                    shift_reg <= {1'b1, shift_reg[9:1]};
                    tx_o      <= shift_reg[1];
                    bit_cnt   <= bit_cnt + 1;
                    if (bit_cnt == 9) begin   // sent start + 8 + stop
                        busy <= 1'b0;
                    end
                end else begin
                    clk_cnt <= clk_cnt + 1;
                end
            end
        end
    end

endmodule