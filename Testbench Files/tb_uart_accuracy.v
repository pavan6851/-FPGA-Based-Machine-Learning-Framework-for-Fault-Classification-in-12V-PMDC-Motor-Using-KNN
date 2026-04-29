// =============================================================
//  tb_uart_accuracy.v
//  Simulates ONLY the accuracy calculation + UART print logic.
//  Does NOT need BRAMs or knn_core - just checks the output.
//
//  Expected UART output:
//    KNN FPGA Results
//    Correct : 210 / 240
//    Accuracy: 87.50%
//    CM:
//       H F1 F2 F3
//    H  47 01 12 00
//    1  01 59 00 00
//    2  16 00 44 00
//    3  00 00 00 60
//    ==
//
//  Run in Vivado: Add as sim source, set as top, run 50ms
// =============================================================

`timescale 1ns / 1ps
`include "knn_params.vh"

module tb_uart_accuracy;

    reg clk, rst_n;
    always #5 clk = ~clk;

    // ── Accuracy wires (copy from fpga_top_knn) ──────────────
    reg [7:0] correct_cnt;

    wire [21:0] acc_x10000   = {14'd0, correct_cnt} * 22'd10000;
    wire [21:0] acc_div      = acc_x10000 / 22'd240;
    wire [7:0]  accuracy_int = acc_div / 22'd100;
    wire [7:0]  acc_frac     = acc_div % 22'd100;

    // ── Confusion matrix (hardcoded from simulation results) ──
    reg [7:0] cm [0:3][0:3];

    // ── UART signals ──────────────────────────────────────────
    wire uart_txd;
    reg  [7:0] uart_byte;
    reg        uart_send;
    wire       uart_ready;

    uart_tx #(.CLK_DIV(`UART_CLK_DIV)) u_uart (
        .clk       (clk),
        .rst_n     (rst_n),
        .tx_data_i (uart_byte),
        .tx_valid_i(uart_send),
        .tx_ready_o(uart_ready),
        .tx_o      (uart_txd)
    );

    // ── Monitor: decode UART TX back to ASCII ─────────────────
    // Sample mid-bit, print received character
    reg [9:0]  uart_shift;
    reg [3:0]  uart_bit_cnt;
    reg [10:0] uart_clk_cnt;
    reg        uart_capturing;

    always @(posedge clk) begin
        if (!uart_capturing && uart_txd == 0) begin
            // Start bit detected
            uart_capturing <= 1;
            uart_clk_cnt   <= `UART_CLK_DIV / 2; // sample mid-bit
            uart_bit_cnt   <= 0;
        end else if (uart_capturing) begin
            if (uart_clk_cnt == `UART_CLK_DIV - 1) begin
                uart_clk_cnt <= 0;
                uart_shift   <= {uart_txd, uart_shift[9:1]};
                uart_bit_cnt <= uart_bit_cnt + 1;
                if (uart_bit_cnt == 9) begin
                    uart_capturing <= 0;
                    // Print decoded byte
                    $write("%c", uart_shift[8:1]);
                end
            end else
                uart_clk_cnt <= uart_clk_cnt + 1;
        end
    end

    // ── Print state machine ───────────────────────────────────
    // Same FSM logic as fpga_top_knn - simplified for standalone test

    function [7:0] dec_char;
        input [7:0] d;
        dec_char = 8'd48 + d;
    endfunction

    integer phase, ci, cj;

    task send_byte;
        input [7:0] b;
        begin
            @(posedge clk);
            wait(uart_ready);
            @(posedge clk); #1;
            uart_byte = b;
            uart_send = 1;
            @(posedge clk); #1;
            uart_send = 0;
            // wait for transmission complete (~868 cycles × 10 = ~87µs)
            repeat(`UART_CLK_DIV * 11) @(posedge clk);
        end
    endtask

    task send_str;
        input [8*32-1:0] s;
        input integer len;
        integer k;
        begin
            for (k = len-1; k >= 0; k = k - 1)
                send_byte(s[k*8 +: 8]);
        end
    endtask

    initial begin
        clk           = 0;
        rst_n         = 0;
        uart_send     = 0;
        uart_byte     = 0;
        uart_capturing= 0;
        uart_shift    = 0;
        uart_bit_cnt  = 0;
        uart_clk_cnt  = 0;

        // Load known correct count and confusion matrix
        correct_cnt = 8'd210;
        cm[0][0]=47; cm[0][1]=1;  cm[0][2]=12; cm[0][3]=0;
        cm[1][0]=1;  cm[1][1]=59; cm[1][2]=0;  cm[1][3]=0;
        cm[2][0]=16; cm[2][1]=0;  cm[2][2]=44; cm[2][3]=0;
        cm[3][0]=0;  cm[3][1]=0;  cm[3][2]=0;  cm[3][3]=60;

        repeat(5) @(posedge clk);
        rst_n = 1;
        repeat(10) @(posedge clk);

        $display("\n=== UART Decoded Output ===");

        // Header
        send_byte(8'h0D); send_byte(8'h0A);
        send_byte("K"); send_byte("N"); send_byte("N"); send_byte(" ");
        send_byte("F"); send_byte("P"); send_byte("G"); send_byte("A"); send_byte(" ");
        send_byte("R"); send_byte("e"); send_byte("s"); send_byte("u");
        send_byte("l"); send_byte("t"); send_byte("s");
        send_byte(8'h0D); send_byte(8'h0A);

        // Correct line
        send_byte("C"); send_byte("o"); send_byte("r"); send_byte("r");
        send_byte("e"); send_byte("c"); send_byte("t"); send_byte(" ");
        send_byte(":"); send_byte(" ");
        send_byte(dec_char(correct_cnt / 100));
        send_byte(dec_char((correct_cnt % 100) / 10));
        send_byte(dec_char(correct_cnt % 10));
        send_byte(" "); send_byte("/"); send_byte(" ");
        send_byte("2"); send_byte("4"); send_byte("0");
        send_byte(8'h0D); send_byte(8'h0A);

        // Accuracy line
        send_byte("A"); send_byte("c"); send_byte("c"); send_byte("u");
        send_byte("r"); send_byte("a"); send_byte("c"); send_byte("y");
        send_byte(":"); send_byte(" ");
        send_byte(dec_char(accuracy_int / 10));
        send_byte(dec_char(accuracy_int % 10));
        send_byte(".");
        send_byte(dec_char(acc_frac / 10));
        send_byte(dec_char(acc_frac % 10));
        send_byte("%");
        send_byte(8'h0D); send_byte(8'h0A);

        // Confusion matrix
        send_byte("C"); send_byte("M"); send_byte(":");
        send_byte(8'h0D); send_byte(8'h0A);
        send_byte(" "); send_byte(" "); send_byte(" ");
        send_byte("H"); send_byte(" ");
        send_byte("F"); send_byte("1"); send_byte(" ");
        send_byte("F"); send_byte("2"); send_byte(" ");
        send_byte("F"); send_byte("3");
        send_byte(8'h0D); send_byte(8'h0A);

        for (ci = 0; ci < 4; ci = ci + 1) begin
            case (ci)
                0: send_byte("H");
                1: send_byte("1");
                2: send_byte("2");
                3: send_byte("3");
            endcase
            send_byte(" "); send_byte(" ");
            for (cj = 0; cj < 4; cj = cj + 1) begin
                send_byte(dec_char(cm[ci][cj] / 10));
                send_byte(dec_char(cm[ci][cj] % 10));
                send_byte(" ");
            end
            send_byte(8'h0D); send_byte(8'h0A);
        end

        send_byte("="); send_byte("=");
        send_byte(8'h0D); send_byte(8'h0A);

        $display("\n=== Done ===");

        // Verify accuracy values
        $display("accuracy_int = %0d  (expected 87)", accuracy_int);
        $display("acc_frac     = %0d  (expected 50)", acc_frac);

        if (accuracy_int == 87 && acc_frac == 50)
            $display("PASS: Accuracy prints as 87.50%%");
        else
            $display("FAIL: Got %0d.%0d%%", accuracy_int, acc_frac);

        $finish;
    end

    // Timeout
    initial begin
        #50_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule