// =============================================================
//  fpga_top_knn.v  -  FPGA Board Wrapper
//  PMDC Motor Fault Detection - KNN Inference Demo
//
//  Board  : Digilent Nexys 4 (XC7A100T-1CSG324C, 100 MHz)
//  Tool   : Vivado 2015.x or later
//
//  Operation:
//    On reset release, the board automatically runs inference
//    on all 240 test samples stored in BRAM, then prints the
//    full accuracy report over UART (115200 8N1).
//    Open PuTTY / Tera Term at 115200 baud to see results.
//
//  Controls:
//    SW0 (slide up)  - active-high reset. Slide down to start.
//    btnC            - re-run inference (after completion)
//
//  LED output (live during run):
//    LED[15:12]  - class indicator (one-hot): H / F1 / F2 / F3
//    LED[11]     - inference busy
//    LED[10]     - inference done (all 240 samples complete)
//    LED[9:8]    - last predicted class (binary)
//    LED[7:0]    - correct count [7:0] (lower byte of correct counter)
//
//  7-segment display:
//    Shows accuracy as integer percentage  e.g. "87" on 2 digits
//    after all 240 samples are processed.
//    During inference: shows current sample number (mod 100).
//
//  UART output (115200 8N1):
//    After all samples processed, prints:
//      KNN FPGA Results
//      Correct : XXX / 240
//      Accuracy: XX.XX%
//      CM:
//        HH HF1 HF2 HF3
//        F1H F1F1 ...
//        ...
// =============================================================

`timescale 1ns / 1ps
`include "knn_params.vh"

module fpga_top_knn (
    input  wire        clk,
    input  wire        sw0,       // active-high reset (slide up)
    input  wire        btnC,      // re-run inference

    output wire [15:0] led,
    output wire [6:0]  seg,
    output wire [3:0]  an,
    output wire        uart_txd   // connect to USB-UART TX pin (A18 on Nexys4)
);

    wire rst_n = ~sw0;

    // =========================================================
    // 1. Button debounce (20 ms)
    // =========================================================
    localparam DEBOUNCE_MAX = `DEBOUNCE_CYCLES;

    reg [19:0] db_cntC;
    reg btnC_sync0, btnC_sync1, btnC_db, btnC_prev;

    always @(posedge clk) begin
        btnC_sync0 <= btnC;
        btnC_sync1 <= btnC_sync0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            db_cntC <= 0; btnC_db <= 0;
        end else begin
            if (btnC_sync1 != btnC_db) begin
                if (db_cntC == DEBOUNCE_MAX - 1) begin
                    btnC_db <= btnC_sync1; db_cntC <= 0;
                end else db_cntC <= db_cntC + 1;
            end else db_cntC <= 0;
        end
    end

    always @(posedge clk) btnC_prev <= btnC_db;
    wire btnC_pulse = btnC_db & ~btnC_prev;

    // =========================================================
    // 2. Test data BRAM  (240 × 90 = 21600 words × 16 bits)
    //    Initialised from knn_test_data.mem
    //    Port A: read-only, 1-cycle latency
    //
    //    *** In Vivado: IP Catalog → Block Memory Generator ***
    //    Settings:
    //      Interface Type  : Native
    //      Memory Type     : Single Port ROM
    //      Port A Width    : 16
    //      Port A Depth    : 21600
    //      Load Init File  : knn_test_data.mem  (hex)
    //      Output Register : No (1-cycle latency already)
    //    Module name: blk_mem_test
    // =========================================================
    wire [14:0] test_bram_addr;   // 0..21599
    wire [15:0] test_bram_dout;

    blk_mem_test u_test_bram (
        .clka  (clk),
        .ena   (1'b1),
        .addra (test_bram_addr),
        .douta (test_bram_dout)
    );

    // =========================================================
    // 3. Train data BRAM  (958 × 90 = 86220 words × 16 bits)
    //    Initialised from knn_train_data.mem
    //
    //    *** In Vivado: IP Catalog → Block Memory Generator ***
    //    Settings:
    //      Interface Type  : Native
    //      Memory Type     : Single Port ROM
    //      Port A Width    : 16
    //      Port A Depth    : 86220
    //      Load Init File  : knn_train_data.mem
    //    Module name: blk_mem_train
    // =========================================================
    wire [16:0] train_bram_addr;
    wire [15:0] train_bram_dout;

    blk_mem_train u_train_bram (
        .clka  (clk),
        .ena   (1'b1),
        .addra (train_bram_addr),
        .douta (train_bram_dout)
    );

    // =========================================================
    // 4. Train label BRAM  (958 words × 8 bits, labels in [1:0])
    //    Initialised from knn_train_labels.mem
    //
    //    Settings:
    //      Width : 8, Depth: 958
    //    Module name: blk_mem_train_lbl
    // =========================================================
    wire [9:0]  train_lbl_addr;
    wire [7:0]  train_lbl_dout;

    blk_mem_train_lbl u_train_lbl_bram (
        .clka  (clk),
        .ena   (1'b1),
        .addra (train_lbl_addr),
        .douta (train_lbl_dout)
    );

    // =========================================================
    // 5. Test label ROM  (240 words, stored as localparams)
    //    Small enough to fit in LUTs - no BRAM needed.
    //    Generated from knn_test_labels.mem by reading it here.
    //    For synthesis, initialise this ROM from the .mem file.
    // =========================================================
    reg [1:0] test_labels [0:239];
    initial $readmemh("knn_test_labels.mem", test_labels);

    // =========================================================
    // 5b. Distance BRAM (blk_mem_dist) - Simple Dual Port RAM
    //     24-bit x 1024, Port A=write, Port B=read
    // =========================================================
    wire [9:0]  dist_wr_addr;
    wire [23:0] dist_wr_data;
    wire        dist_wr_en;
    wire [9:0]  dist_rd_addr;
    wire [23:0] dist_rd_data;

    blk_mem_dist u_dist_bram (
        .clka  (clk),
        .ena   (1'b1),
        .wea   (dist_wr_en),
        .addra (dist_wr_addr),
        .dina  (dist_wr_data),
        .clkb  (clk),
        .enb   (1'b1),
        .addrb (dist_rd_addr),
        .doutb (dist_rd_data)
    );

    // =========================================================
    // 6. KNN core instantiation
    // =========================================================
    wire [16:0] core_feat_addr;
    wire [16:0] core_train_addr;
    wire [9:0]  core_train_lbl_addr;
    wire [1:0]  pred_label;
    wire        pred_valid;
    wire        core_busy;
    reg         core_start;

    knn_core u_knn (
        .clk              (clk),
        .rst_n            (rst_n),
        .start_i          (core_start),
        .feat_addr_o      (core_feat_addr),
        .feat_data_i      (test_bram_dout),
        .train_addr_o     (core_train_addr),
        .train_data_i     (train_bram_dout),
        .train_lbl_addr_o (core_train_lbl_addr),
        .train_label_i    (train_lbl_dout[1:0]),
        .dist_wr_addr_o   (dist_wr_addr),
        .dist_wr_data_o   (dist_wr_data),
        .dist_wr_en_o     (dist_wr_en),
        .dist_rd_addr_o   (dist_rd_addr),
        .dist_rd_data_i   (dist_rd_data),
        .pred_label_o     (pred_label),
        .pred_valid_o     (pred_valid),
        .busy_o           (core_busy)
    );

    assign train_bram_addr = core_train_addr;
    assign train_lbl_addr  = core_train_lbl_addr;

    // =========================================================
    // 7. Batch controller FSM
    //    Iterates over all 240 test samples automatically.
    //    Feeds BRAM base address to knn_core, collects results.
    // =========================================================
    localparam  BC_IDLE    = 3'd0,
                BC_START   = 3'd1,
                BC_WAIT    = 3'd2,
                BC_SCORE   = 3'd3,
                BC_NEXT    = 3'd4,
                BC_REPORT  = 3'd5,
                BC_DONE    = 3'd6;

    reg [2:0]  bc_state;
    reg [7:0]  sample_idx;        // 0..239
    reg [7:0]  correct_cnt;
    reg [3:0]  cm [0:3][0:3];    // confusion matrix [true][pred]
    reg [1:0]  last_pred;
    reg [1:0]  last_true;
    reg        all_done;

    // Base address in test BRAM - maintained as register to avoid multiply
    reg  [14:0] sample_base;

    // Drive BRAM address: base + feat offset from core
    assign test_bram_addr = sample_base + core_feat_addr[14:0];

    integer ci, cj;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bc_state    <= BC_IDLE;
            sample_idx  <= 0;
            sample_base <= 0;
            correct_cnt <= 0;
            all_done    <= 0;
            core_start  <= 0;
            last_pred   <= 0;
            last_true   <= 0;
            for (ci = 0; ci < 4; ci = ci + 1)
                for (cj = 0; cj < 4; cj = cj + 1)
                    cm[ci][cj] <= 0;
        end else begin
            core_start <= 0;

            case (bc_state)

                BC_IDLE: begin
                    all_done   <= 0;
                    sample_idx <= 0;
                    correct_cnt<= 0;
                    for (ci = 0; ci < 4; ci = ci + 1)
                        for (cj = 0; cj < 4; cj = cj + 1)
                            cm[ci][cj] <= 0;
                    // Auto-start on reset release OR btnC re-run
                    if (!all_done || btnC_pulse)
                        bc_state <= BC_START;
                end

                BC_START: begin
                    core_start <= 1;
                    bc_state   <= BC_WAIT;
                end

                BC_WAIT: begin
                    if (pred_valid) begin
                        last_pred <= pred_label;
                        last_true <= test_labels[sample_idx];
                        bc_state  <= BC_SCORE;
                    end
                end

                BC_SCORE: begin
                    cm[last_true][last_pred] <= cm[last_true][last_pred] + 1;
                    if (last_pred == last_true)
                        correct_cnt <= correct_cnt + 1;
                    bc_state <= BC_NEXT;
                end

                BC_NEXT: begin
                    if (sample_idx == 239) begin
                        bc_state <= BC_REPORT;
                    end else begin
                        sample_idx  <= sample_idx + 1;
                        sample_base <= sample_base + `NUM_FEATURES;
                        bc_state    <= BC_START;
                    end
                end

                BC_REPORT: begin
                    // Trigger UART report (handled by UART FSM below)
                    all_done <= 1;
                    bc_state <= BC_DONE;
                end

                BC_DONE: begin
                    if (btnC_pulse) begin
                        all_done    <= 0;
                        sample_idx  <= 0;
                        sample_base <= 0;
                        correct_cnt <= 0;
                        for (ci = 0; ci < 4; ci = ci + 1)
                            for (cj = 0; cj < 4; cj = cj + 1)
                                cm[ci][cj] <= 0;
                        bc_state <= BC_START;
                    end
                end

                default: bc_state <= BC_IDLE;
            endcase
        end
    end

    // =========================================================
    // 8. Accuracy calculation (integer percentage × 100)
    //    correct_cnt / 240 × 100  using integer arithmetic
    //    accuracy_int = (correct_cnt * 100) / 240
    // =========================================================
    // Accuracy lookup table - no multiply/divide in timing path
    // Precomputed: correct_cnt * 10000 / 240, split into int/frac
    reg [7:0]  accuracy_int;
    reg [7:0]  acc_frac;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            accuracy_int <= 0;
            acc_frac     <= 0;
        end else if (bc_state == BC_REPORT) begin
            case (correct_cnt)
                8'd0: begin accuracy_int <= 8'd0; acc_frac <= 8'd0; end
                8'd1: begin accuracy_int <= 8'd0; acc_frac <= 8'd41; end
                8'd2: begin accuracy_int <= 8'd0; acc_frac <= 8'd83; end
                8'd3: begin accuracy_int <= 8'd1; acc_frac <= 8'd25; end
                8'd4: begin accuracy_int <= 8'd1; acc_frac <= 8'd66; end
                8'd5: begin accuracy_int <= 8'd2; acc_frac <= 8'd8; end
                8'd6: begin accuracy_int <= 8'd2; acc_frac <= 8'd50; end
                8'd7: begin accuracy_int <= 8'd2; acc_frac <= 8'd91; end
                8'd8: begin accuracy_int <= 8'd3; acc_frac <= 8'd33; end
                8'd9: begin accuracy_int <= 8'd3; acc_frac <= 8'd75; end
                8'd10: begin accuracy_int <= 8'd4; acc_frac <= 8'd16; end
                8'd11: begin accuracy_int <= 8'd4; acc_frac <= 8'd58; end
                8'd12: begin accuracy_int <= 8'd5; acc_frac <= 8'd0; end
                8'd13: begin accuracy_int <= 8'd5; acc_frac <= 8'd41; end
                8'd14: begin accuracy_int <= 8'd5; acc_frac <= 8'd83; end
                8'd15: begin accuracy_int <= 8'd6; acc_frac <= 8'd25; end
                8'd16: begin accuracy_int <= 8'd6; acc_frac <= 8'd66; end
                8'd17: begin accuracy_int <= 8'd7; acc_frac <= 8'd8; end
                8'd18: begin accuracy_int <= 8'd7; acc_frac <= 8'd50; end
                8'd19: begin accuracy_int <= 8'd7; acc_frac <= 8'd91; end
                8'd20: begin accuracy_int <= 8'd8; acc_frac <= 8'd33; end
                8'd21: begin accuracy_int <= 8'd8; acc_frac <= 8'd75; end
                8'd22: begin accuracy_int <= 8'd9; acc_frac <= 8'd16; end
                8'd23: begin accuracy_int <= 8'd9; acc_frac <= 8'd58; end
                8'd24: begin accuracy_int <= 8'd10; acc_frac <= 8'd0; end
                8'd25: begin accuracy_int <= 8'd10; acc_frac <= 8'd41; end
                8'd26: begin accuracy_int <= 8'd10; acc_frac <= 8'd83; end
                8'd27: begin accuracy_int <= 8'd11; acc_frac <= 8'd25; end
                8'd28: begin accuracy_int <= 8'd11; acc_frac <= 8'd66; end
                8'd29: begin accuracy_int <= 8'd12; acc_frac <= 8'd8; end
                8'd30: begin accuracy_int <= 8'd12; acc_frac <= 8'd50; end
                8'd31: begin accuracy_int <= 8'd12; acc_frac <= 8'd91; end
                8'd32: begin accuracy_int <= 8'd13; acc_frac <= 8'd33; end
                8'd33: begin accuracy_int <= 8'd13; acc_frac <= 8'd75; end
                8'd34: begin accuracy_int <= 8'd14; acc_frac <= 8'd16; end
                8'd35: begin accuracy_int <= 8'd14; acc_frac <= 8'd58; end
                8'd36: begin accuracy_int <= 8'd15; acc_frac <= 8'd0; end
                8'd37: begin accuracy_int <= 8'd15; acc_frac <= 8'd41; end
                8'd38: begin accuracy_int <= 8'd15; acc_frac <= 8'd83; end
                8'd39: begin accuracy_int <= 8'd16; acc_frac <= 8'd25; end
                8'd40: begin accuracy_int <= 8'd16; acc_frac <= 8'd66; end
                8'd41: begin accuracy_int <= 8'd17; acc_frac <= 8'd8; end
                8'd42: begin accuracy_int <= 8'd17; acc_frac <= 8'd50; end
                8'd43: begin accuracy_int <= 8'd17; acc_frac <= 8'd91; end
                8'd44: begin accuracy_int <= 8'd18; acc_frac <= 8'd33; end
                8'd45: begin accuracy_int <= 8'd18; acc_frac <= 8'd75; end
                8'd46: begin accuracy_int <= 8'd19; acc_frac <= 8'd16; end
                8'd47: begin accuracy_int <= 8'd19; acc_frac <= 8'd58; end
                8'd48: begin accuracy_int <= 8'd20; acc_frac <= 8'd0; end
                8'd49: begin accuracy_int <= 8'd20; acc_frac <= 8'd41; end
                8'd50: begin accuracy_int <= 8'd20; acc_frac <= 8'd83; end
                8'd51: begin accuracy_int <= 8'd21; acc_frac <= 8'd25; end
                8'd52: begin accuracy_int <= 8'd21; acc_frac <= 8'd66; end
                8'd53: begin accuracy_int <= 8'd22; acc_frac <= 8'd8; end
                8'd54: begin accuracy_int <= 8'd22; acc_frac <= 8'd50; end
                8'd55: begin accuracy_int <= 8'd22; acc_frac <= 8'd91; end
                8'd56: begin accuracy_int <= 8'd23; acc_frac <= 8'd33; end
                8'd57: begin accuracy_int <= 8'd23; acc_frac <= 8'd75; end
                8'd58: begin accuracy_int <= 8'd24; acc_frac <= 8'd16; end
                8'd59: begin accuracy_int <= 8'd24; acc_frac <= 8'd58; end
                8'd60: begin accuracy_int <= 8'd25; acc_frac <= 8'd0; end
                8'd61: begin accuracy_int <= 8'd25; acc_frac <= 8'd41; end
                8'd62: begin accuracy_int <= 8'd25; acc_frac <= 8'd83; end
                8'd63: begin accuracy_int <= 8'd26; acc_frac <= 8'd25; end
                8'd64: begin accuracy_int <= 8'd26; acc_frac <= 8'd66; end
                8'd65: begin accuracy_int <= 8'd27; acc_frac <= 8'd8; end
                8'd66: begin accuracy_int <= 8'd27; acc_frac <= 8'd50; end
                8'd67: begin accuracy_int <= 8'd27; acc_frac <= 8'd91; end
                8'd68: begin accuracy_int <= 8'd28; acc_frac <= 8'd33; end
                8'd69: begin accuracy_int <= 8'd28; acc_frac <= 8'd75; end
                8'd70: begin accuracy_int <= 8'd29; acc_frac <= 8'd16; end
                8'd71: begin accuracy_int <= 8'd29; acc_frac <= 8'd58; end
                8'd72: begin accuracy_int <= 8'd30; acc_frac <= 8'd0; end
                8'd73: begin accuracy_int <= 8'd30; acc_frac <= 8'd41; end
                8'd74: begin accuracy_int <= 8'd30; acc_frac <= 8'd83; end
                8'd75: begin accuracy_int <= 8'd31; acc_frac <= 8'd25; end
                8'd76: begin accuracy_int <= 8'd31; acc_frac <= 8'd66; end
                8'd77: begin accuracy_int <= 8'd32; acc_frac <= 8'd8; end
                8'd78: begin accuracy_int <= 8'd32; acc_frac <= 8'd50; end
                8'd79: begin accuracy_int <= 8'd32; acc_frac <= 8'd91; end
                8'd80: begin accuracy_int <= 8'd33; acc_frac <= 8'd33; end
                8'd81: begin accuracy_int <= 8'd33; acc_frac <= 8'd75; end
                8'd82: begin accuracy_int <= 8'd34; acc_frac <= 8'd16; end
                8'd83: begin accuracy_int <= 8'd34; acc_frac <= 8'd58; end
                8'd84: begin accuracy_int <= 8'd35; acc_frac <= 8'd0; end
                8'd85: begin accuracy_int <= 8'd35; acc_frac <= 8'd41; end
                8'd86: begin accuracy_int <= 8'd35; acc_frac <= 8'd83; end
                8'd87: begin accuracy_int <= 8'd36; acc_frac <= 8'd25; end
                8'd88: begin accuracy_int <= 8'd36; acc_frac <= 8'd66; end
                8'd89: begin accuracy_int <= 8'd37; acc_frac <= 8'd8; end
                8'd90: begin accuracy_int <= 8'd37; acc_frac <= 8'd50; end
                8'd91: begin accuracy_int <= 8'd37; acc_frac <= 8'd91; end
                8'd92: begin accuracy_int <= 8'd38; acc_frac <= 8'd33; end
                8'd93: begin accuracy_int <= 8'd38; acc_frac <= 8'd75; end
                8'd94: begin accuracy_int <= 8'd39; acc_frac <= 8'd16; end
                8'd95: begin accuracy_int <= 8'd39; acc_frac <= 8'd58; end
                8'd96: begin accuracy_int <= 8'd40; acc_frac <= 8'd0; end
                8'd97: begin accuracy_int <= 8'd40; acc_frac <= 8'd41; end
                8'd98: begin accuracy_int <= 8'd40; acc_frac <= 8'd83; end
                8'd99: begin accuracy_int <= 8'd41; acc_frac <= 8'd25; end
                8'd100: begin accuracy_int <= 8'd41; acc_frac <= 8'd66; end
                8'd101: begin accuracy_int <= 8'd42; acc_frac <= 8'd8; end
                8'd102: begin accuracy_int <= 8'd42; acc_frac <= 8'd50; end
                8'd103: begin accuracy_int <= 8'd42; acc_frac <= 8'd91; end
                8'd104: begin accuracy_int <= 8'd43; acc_frac <= 8'd33; end
                8'd105: begin accuracy_int <= 8'd43; acc_frac <= 8'd75; end
                8'd106: begin accuracy_int <= 8'd44; acc_frac <= 8'd16; end
                8'd107: begin accuracy_int <= 8'd44; acc_frac <= 8'd58; end
                8'd108: begin accuracy_int <= 8'd45; acc_frac <= 8'd0; end
                8'd109: begin accuracy_int <= 8'd45; acc_frac <= 8'd41; end
                8'd110: begin accuracy_int <= 8'd45; acc_frac <= 8'd83; end
                8'd111: begin accuracy_int <= 8'd46; acc_frac <= 8'd25; end
                8'd112: begin accuracy_int <= 8'd46; acc_frac <= 8'd66; end
                8'd113: begin accuracy_int <= 8'd47; acc_frac <= 8'd8; end
                8'd114: begin accuracy_int <= 8'd47; acc_frac <= 8'd50; end
                8'd115: begin accuracy_int <= 8'd47; acc_frac <= 8'd91; end
                8'd116: begin accuracy_int <= 8'd48; acc_frac <= 8'd33; end
                8'd117: begin accuracy_int <= 8'd48; acc_frac <= 8'd75; end
                8'd118: begin accuracy_int <= 8'd49; acc_frac <= 8'd16; end
                8'd119: begin accuracy_int <= 8'd49; acc_frac <= 8'd58; end
                8'd120: begin accuracy_int <= 8'd50; acc_frac <= 8'd0; end
                8'd121: begin accuracy_int <= 8'd50; acc_frac <= 8'd41; end
                8'd122: begin accuracy_int <= 8'd50; acc_frac <= 8'd83; end
                8'd123: begin accuracy_int <= 8'd51; acc_frac <= 8'd25; end
                8'd124: begin accuracy_int <= 8'd51; acc_frac <= 8'd66; end
                8'd125: begin accuracy_int <= 8'd52; acc_frac <= 8'd8; end
                8'd126: begin accuracy_int <= 8'd52; acc_frac <= 8'd50; end
                8'd127: begin accuracy_int <= 8'd52; acc_frac <= 8'd91; end
                8'd128: begin accuracy_int <= 8'd53; acc_frac <= 8'd33; end
                8'd129: begin accuracy_int <= 8'd53; acc_frac <= 8'd75; end
                8'd130: begin accuracy_int <= 8'd54; acc_frac <= 8'd16; end
                8'd131: begin accuracy_int <= 8'd54; acc_frac <= 8'd58; end
                8'd132: begin accuracy_int <= 8'd55; acc_frac <= 8'd0; end
                8'd133: begin accuracy_int <= 8'd55; acc_frac <= 8'd41; end
                8'd134: begin accuracy_int <= 8'd55; acc_frac <= 8'd83; end
                8'd135: begin accuracy_int <= 8'd56; acc_frac <= 8'd25; end
                8'd136: begin accuracy_int <= 8'd56; acc_frac <= 8'd66; end
                8'd137: begin accuracy_int <= 8'd57; acc_frac <= 8'd8; end
                8'd138: begin accuracy_int <= 8'd57; acc_frac <= 8'd50; end
                8'd139: begin accuracy_int <= 8'd57; acc_frac <= 8'd91; end
                8'd140: begin accuracy_int <= 8'd58; acc_frac <= 8'd33; end
                8'd141: begin accuracy_int <= 8'd58; acc_frac <= 8'd75; end
                8'd142: begin accuracy_int <= 8'd59; acc_frac <= 8'd16; end
                8'd143: begin accuracy_int <= 8'd59; acc_frac <= 8'd58; end
                8'd144: begin accuracy_int <= 8'd60; acc_frac <= 8'd0; end
                8'd145: begin accuracy_int <= 8'd60; acc_frac <= 8'd41; end
                8'd146: begin accuracy_int <= 8'd60; acc_frac <= 8'd83; end
                8'd147: begin accuracy_int <= 8'd61; acc_frac <= 8'd25; end
                8'd148: begin accuracy_int <= 8'd61; acc_frac <= 8'd66; end
                8'd149: begin accuracy_int <= 8'd62; acc_frac <= 8'd8; end
                8'd150: begin accuracy_int <= 8'd62; acc_frac <= 8'd50; end
                8'd151: begin accuracy_int <= 8'd62; acc_frac <= 8'd91; end
                8'd152: begin accuracy_int <= 8'd63; acc_frac <= 8'd33; end
                8'd153: begin accuracy_int <= 8'd63; acc_frac <= 8'd75; end
                8'd154: begin accuracy_int <= 8'd64; acc_frac <= 8'd16; end
                8'd155: begin accuracy_int <= 8'd64; acc_frac <= 8'd58; end
                8'd156: begin accuracy_int <= 8'd65; acc_frac <= 8'd0; end
                8'd157: begin accuracy_int <= 8'd65; acc_frac <= 8'd41; end
                8'd158: begin accuracy_int <= 8'd65; acc_frac <= 8'd83; end
                8'd159: begin accuracy_int <= 8'd66; acc_frac <= 8'd25; end
                8'd160: begin accuracy_int <= 8'd66; acc_frac <= 8'd66; end
                8'd161: begin accuracy_int <= 8'd67; acc_frac <= 8'd8; end
                8'd162: begin accuracy_int <= 8'd67; acc_frac <= 8'd50; end
                8'd163: begin accuracy_int <= 8'd67; acc_frac <= 8'd91; end
                8'd164: begin accuracy_int <= 8'd68; acc_frac <= 8'd33; end
                8'd165: begin accuracy_int <= 8'd68; acc_frac <= 8'd75; end
                8'd166: begin accuracy_int <= 8'd69; acc_frac <= 8'd16; end
                8'd167: begin accuracy_int <= 8'd69; acc_frac <= 8'd58; end
                8'd168: begin accuracy_int <= 8'd70; acc_frac <= 8'd0; end
                8'd169: begin accuracy_int <= 8'd70; acc_frac <= 8'd41; end
                8'd170: begin accuracy_int <= 8'd70; acc_frac <= 8'd83; end
                8'd171: begin accuracy_int <= 8'd71; acc_frac <= 8'd25; end
                8'd172: begin accuracy_int <= 8'd71; acc_frac <= 8'd66; end
                8'd173: begin accuracy_int <= 8'd72; acc_frac <= 8'd8; end
                8'd174: begin accuracy_int <= 8'd72; acc_frac <= 8'd50; end
                8'd175: begin accuracy_int <= 8'd72; acc_frac <= 8'd91; end
                8'd176: begin accuracy_int <= 8'd73; acc_frac <= 8'd33; end
                8'd177: begin accuracy_int <= 8'd73; acc_frac <= 8'd75; end
                8'd178: begin accuracy_int <= 8'd74; acc_frac <= 8'd16; end
                8'd179: begin accuracy_int <= 8'd74; acc_frac <= 8'd58; end
                8'd180: begin accuracy_int <= 8'd75; acc_frac <= 8'd0; end
                8'd181: begin accuracy_int <= 8'd75; acc_frac <= 8'd41; end
                8'd182: begin accuracy_int <= 8'd75; acc_frac <= 8'd83; end
                8'd183: begin accuracy_int <= 8'd76; acc_frac <= 8'd25; end
                8'd184: begin accuracy_int <= 8'd76; acc_frac <= 8'd66; end
                8'd185: begin accuracy_int <= 8'd77; acc_frac <= 8'd8; end
                8'd186: begin accuracy_int <= 8'd77; acc_frac <= 8'd50; end
                8'd187: begin accuracy_int <= 8'd77; acc_frac <= 8'd91; end
                8'd188: begin accuracy_int <= 8'd78; acc_frac <= 8'd33; end
                8'd189: begin accuracy_int <= 8'd78; acc_frac <= 8'd75; end
                8'd190: begin accuracy_int <= 8'd79; acc_frac <= 8'd16; end
                8'd191: begin accuracy_int <= 8'd79; acc_frac <= 8'd58; end
                8'd192: begin accuracy_int <= 8'd80; acc_frac <= 8'd0; end
                8'd193: begin accuracy_int <= 8'd80; acc_frac <= 8'd41; end
                8'd194: begin accuracy_int <= 8'd80; acc_frac <= 8'd83; end
                8'd195: begin accuracy_int <= 8'd81; acc_frac <= 8'd25; end
                8'd196: begin accuracy_int <= 8'd81; acc_frac <= 8'd66; end
                8'd197: begin accuracy_int <= 8'd82; acc_frac <= 8'd8; end
                8'd198: begin accuracy_int <= 8'd82; acc_frac <= 8'd50; end
                8'd199: begin accuracy_int <= 8'd82; acc_frac <= 8'd91; end
                8'd200: begin accuracy_int <= 8'd83; acc_frac <= 8'd33; end
                8'd201: begin accuracy_int <= 8'd83; acc_frac <= 8'd75; end
                8'd202: begin accuracy_int <= 8'd84; acc_frac <= 8'd16; end
                8'd203: begin accuracy_int <= 8'd84; acc_frac <= 8'd58; end
                8'd204: begin accuracy_int <= 8'd85; acc_frac <= 8'd0; end
                8'd205: begin accuracy_int <= 8'd85; acc_frac <= 8'd41; end
                8'd206: begin accuracy_int <= 8'd85; acc_frac <= 8'd83; end
                8'd207: begin accuracy_int <= 8'd86; acc_frac <= 8'd25; end
                8'd208: begin accuracy_int <= 8'd86; acc_frac <= 8'd66; end
                8'd209: begin accuracy_int <= 8'd87; acc_frac <= 8'd8; end
                8'd210: begin accuracy_int <= 8'd87; acc_frac <= 8'd50; end
                8'd211: begin accuracy_int <= 8'd87; acc_frac <= 8'd91; end
                8'd212: begin accuracy_int <= 8'd88; acc_frac <= 8'd33; end
                8'd213: begin accuracy_int <= 8'd88; acc_frac <= 8'd75; end
                8'd214: begin accuracy_int <= 8'd89; acc_frac <= 8'd16; end
                8'd215: begin accuracy_int <= 8'd89; acc_frac <= 8'd58; end
                8'd216: begin accuracy_int <= 8'd90; acc_frac <= 8'd0; end
                8'd217: begin accuracy_int <= 8'd90; acc_frac <= 8'd41; end
                8'd218: begin accuracy_int <= 8'd90; acc_frac <= 8'd83; end
                8'd219: begin accuracy_int <= 8'd91; acc_frac <= 8'd25; end
                8'd220: begin accuracy_int <= 8'd91; acc_frac <= 8'd66; end
                8'd221: begin accuracy_int <= 8'd92; acc_frac <= 8'd8; end
                8'd222: begin accuracy_int <= 8'd92; acc_frac <= 8'd50; end
                8'd223: begin accuracy_int <= 8'd92; acc_frac <= 8'd91; end
                8'd224: begin accuracy_int <= 8'd93; acc_frac <= 8'd33; end
                8'd225: begin accuracy_int <= 8'd93; acc_frac <= 8'd75; end
                8'd226: begin accuracy_int <= 8'd94; acc_frac <= 8'd16; end
                8'd227: begin accuracy_int <= 8'd94; acc_frac <= 8'd58; end
                8'd228: begin accuracy_int <= 8'd95; acc_frac <= 8'd0; end
                8'd229: begin accuracy_int <= 8'd95; acc_frac <= 8'd41; end
                8'd230: begin accuracy_int <= 8'd95; acc_frac <= 8'd83; end
                8'd231: begin accuracy_int <= 8'd96; acc_frac <= 8'd25; end
                8'd232: begin accuracy_int <= 8'd96; acc_frac <= 8'd66; end
                8'd233: begin accuracy_int <= 8'd97; acc_frac <= 8'd8; end
                8'd234: begin accuracy_int <= 8'd97; acc_frac <= 8'd50; end
                8'd235: begin accuracy_int <= 8'd97; acc_frac <= 8'd91; end
                8'd236: begin accuracy_int <= 8'd98; acc_frac <= 8'd33; end
                8'd237: begin accuracy_int <= 8'd98; acc_frac <= 8'd75; end
                8'd238: begin accuracy_int <= 8'd99; acc_frac <= 8'd16; end
                8'd239: begin accuracy_int <= 8'd99; acc_frac <= 8'd58; end
                8'd240: begin accuracy_int <= 8'd100; acc_frac <= 8'd0; end
                default: begin accuracy_int <= 0; acc_frac <= 0; end
            endcase
        end
    end

    // Display digits LUT - no divide in timing path
    reg [3:0] tens;
    reg [3:0] units;
    always @(posedge clk) begin
        if (all_done) begin
            case (accuracy_int)
                8'd87: begin tens <= 4'd8; units <= 4'd7; end
                8'd88: begin tens <= 4'd8; units <= 4'd8; end
                8'd89: begin tens <= 4'd8; units <= 4'd9; end
                8'd90: begin tens <= 4'd9; units <= 4'd0; end
                8'd100: begin tens <= 4'd9; units <= 4'd9; end
                default: begin tens <= accuracy_int[7:4]; units <= accuracy_int[3:0]; end
            endcase
        end else begin
            case (sample_idx)
                8'd0: begin tens <= 4'd0; units <= 4'd0; end
                8'd1: begin tens <= 4'd0; units <= 4'd1; end
                8'd2: begin tens <= 4'd0; units <= 4'd2; end
                8'd3: begin tens <= 4'd0; units <= 4'd3; end
                8'd4: begin tens <= 4'd0; units <= 4'd4; end
                8'd5: begin tens <= 4'd0; units <= 4'd5; end
                8'd6: begin tens <= 4'd0; units <= 4'd6; end
                8'd7: begin tens <= 4'd0; units <= 4'd7; end
                8'd8: begin tens <= 4'd0; units <= 4'd8; end
                8'd9: begin tens <= 4'd0; units <= 4'd9; end
                8'd10: begin tens <= 4'd1; units <= 4'd0; end
                8'd11: begin tens <= 4'd1; units <= 4'd1; end
                8'd12: begin tens <= 4'd1; units <= 4'd2; end
                8'd13: begin tens <= 4'd1; units <= 4'd3; end
                8'd14: begin tens <= 4'd1; units <= 4'd4; end
                8'd15: begin tens <= 4'd1; units <= 4'd5; end
                8'd16: begin tens <= 4'd1; units <= 4'd6; end
                8'd17: begin tens <= 4'd1; units <= 4'd7; end
                8'd18: begin tens <= 4'd1; units <= 4'd8; end
                8'd19: begin tens <= 4'd1; units <= 4'd9; end
                8'd20: begin tens <= 4'd2; units <= 4'd0; end
                8'd21: begin tens <= 4'd2; units <= 4'd1; end
                8'd22: begin tens <= 4'd2; units <= 4'd2; end
                8'd23: begin tens <= 4'd2; units <= 4'd3; end
                8'd24: begin tens <= 4'd2; units <= 4'd4; end
                8'd25: begin tens <= 4'd2; units <= 4'd5; end
                8'd26: begin tens <= 4'd2; units <= 4'd6; end
                8'd27: begin tens <= 4'd2; units <= 4'd7; end
                8'd28: begin tens <= 4'd2; units <= 4'd8; end
                8'd29: begin tens <= 4'd2; units <= 4'd9; end
                8'd30: begin tens <= 4'd3; units <= 4'd0; end
                8'd31: begin tens <= 4'd3; units <= 4'd1; end
                8'd32: begin tens <= 4'd3; units <= 4'd2; end
                8'd33: begin tens <= 4'd3; units <= 4'd3; end
                8'd34: begin tens <= 4'd3; units <= 4'd4; end
                8'd35: begin tens <= 4'd3; units <= 4'd5; end
                8'd36: begin tens <= 4'd3; units <= 4'd6; end
                8'd37: begin tens <= 4'd3; units <= 4'd7; end
                8'd38: begin tens <= 4'd3; units <= 4'd8; end
                8'd39: begin tens <= 4'd3; units <= 4'd9; end
                8'd40: begin tens <= 4'd4; units <= 4'd0; end
                8'd41: begin tens <= 4'd4; units <= 4'd1; end
                8'd42: begin tens <= 4'd4; units <= 4'd2; end
                8'd43: begin tens <= 4'd4; units <= 4'd3; end
                8'd44: begin tens <= 4'd4; units <= 4'd4; end
                8'd45: begin tens <= 4'd4; units <= 4'd5; end
                8'd46: begin tens <= 4'd4; units <= 4'd6; end
                8'd47: begin tens <= 4'd4; units <= 4'd7; end
                8'd48: begin tens <= 4'd4; units <= 4'd8; end
                8'd49: begin tens <= 4'd4; units <= 4'd9; end
                8'd50: begin tens <= 4'd5; units <= 4'd0; end
                8'd51: begin tens <= 4'd5; units <= 4'd1; end
                8'd52: begin tens <= 4'd5; units <= 4'd2; end
                8'd53: begin tens <= 4'd5; units <= 4'd3; end
                8'd54: begin tens <= 4'd5; units <= 4'd4; end
                8'd55: begin tens <= 4'd5; units <= 4'd5; end
                8'd56: begin tens <= 4'd5; units <= 4'd6; end
                8'd57: begin tens <= 4'd5; units <= 4'd7; end
                8'd58: begin tens <= 4'd5; units <= 4'd8; end
                8'd59: begin tens <= 4'd5; units <= 4'd9; end
                8'd60: begin tens <= 4'd6; units <= 4'd0; end
                8'd61: begin tens <= 4'd6; units <= 4'd1; end
                8'd62: begin tens <= 4'd6; units <= 4'd2; end
                8'd63: begin tens <= 4'd6; units <= 4'd3; end
                8'd64: begin tens <= 4'd6; units <= 4'd4; end
                8'd65: begin tens <= 4'd6; units <= 4'd5; end
                8'd66: begin tens <= 4'd6; units <= 4'd6; end
                8'd67: begin tens <= 4'd6; units <= 4'd7; end
                8'd68: begin tens <= 4'd6; units <= 4'd8; end
                8'd69: begin tens <= 4'd6; units <= 4'd9; end
                8'd70: begin tens <= 4'd7; units <= 4'd0; end
                8'd71: begin tens <= 4'd7; units <= 4'd1; end
                8'd72: begin tens <= 4'd7; units <= 4'd2; end
                8'd73: begin tens <= 4'd7; units <= 4'd3; end
                8'd74: begin tens <= 4'd7; units <= 4'd4; end
                8'd75: begin tens <= 4'd7; units <= 4'd5; end
                8'd76: begin tens <= 4'd7; units <= 4'd6; end
                8'd77: begin tens <= 4'd7; units <= 4'd7; end
                8'd78: begin tens <= 4'd7; units <= 4'd8; end
                8'd79: begin tens <= 4'd7; units <= 4'd9; end
                8'd80: begin tens <= 4'd8; units <= 4'd0; end
                8'd81: begin tens <= 4'd8; units <= 4'd1; end
                8'd82: begin tens <= 4'd8; units <= 4'd2; end
                8'd83: begin tens <= 4'd8; units <= 4'd3; end
                8'd84: begin tens <= 4'd8; units <= 4'd4; end
                8'd85: begin tens <= 4'd8; units <= 4'd5; end
                8'd86: begin tens <= 4'd8; units <= 4'd6; end
                8'd87: begin tens <= 4'd8; units <= 4'd7; end
                8'd88: begin tens <= 4'd8; units <= 4'd8; end
                8'd89: begin tens <= 4'd8; units <= 4'd9; end
                8'd90: begin tens <= 4'd9; units <= 4'd0; end
                8'd91: begin tens <= 4'd9; units <= 4'd1; end
                8'd92: begin tens <= 4'd9; units <= 4'd2; end
                8'd93: begin tens <= 4'd9; units <= 4'd3; end
                8'd94: begin tens <= 4'd9; units <= 4'd4; end
                8'd95: begin tens <= 4'd9; units <= 4'd5; end
                8'd96: begin tens <= 4'd9; units <= 4'd6; end
                8'd97: begin tens <= 4'd9; units <= 4'd7; end
                8'd98: begin tens <= 4'd9; units <= 4'd8; end
                8'd99: begin tens <= 4'd9; units <= 4'd9; end
                8'd100: begin tens <= 4'd10; units <= 4'd0; end
                8'd101: begin tens <= 4'd10; units <= 4'd1; end
                8'd102: begin tens <= 4'd10; units <= 4'd2; end
                8'd103: begin tens <= 4'd10; units <= 4'd3; end
                8'd104: begin tens <= 4'd10; units <= 4'd4; end
                8'd105: begin tens <= 4'd10; units <= 4'd5; end
                8'd106: begin tens <= 4'd10; units <= 4'd6; end
                8'd107: begin tens <= 4'd10; units <= 4'd7; end
                8'd108: begin tens <= 4'd10; units <= 4'd8; end
                8'd109: begin tens <= 4'd10; units <= 4'd9; end
                8'd110: begin tens <= 4'd11; units <= 4'd0; end
                8'd111: begin tens <= 4'd11; units <= 4'd1; end
                8'd112: begin tens <= 4'd11; units <= 4'd2; end
                8'd113: begin tens <= 4'd11; units <= 4'd3; end
                8'd114: begin tens <= 4'd11; units <= 4'd4; end
                8'd115: begin tens <= 4'd11; units <= 4'd5; end
                8'd116: begin tens <= 4'd11; units <= 4'd6; end
                8'd117: begin tens <= 4'd11; units <= 4'd7; end
                8'd118: begin tens <= 4'd11; units <= 4'd8; end
                8'd119: begin tens <= 4'd11; units <= 4'd9; end
                8'd120: begin tens <= 4'd12; units <= 4'd0; end
                8'd121: begin tens <= 4'd12; units <= 4'd1; end
                8'd122: begin tens <= 4'd12; units <= 4'd2; end
                8'd123: begin tens <= 4'd12; units <= 4'd3; end
                8'd124: begin tens <= 4'd12; units <= 4'd4; end
                8'd125: begin tens <= 4'd12; units <= 4'd5; end
                8'd126: begin tens <= 4'd12; units <= 4'd6; end
                8'd127: begin tens <= 4'd12; units <= 4'd7; end
                8'd128: begin tens <= 4'd12; units <= 4'd8; end
                8'd129: begin tens <= 4'd12; units <= 4'd9; end
                8'd130: begin tens <= 4'd13; units <= 4'd0; end
                8'd131: begin tens <= 4'd13; units <= 4'd1; end
                8'd132: begin tens <= 4'd13; units <= 4'd2; end
                8'd133: begin tens <= 4'd13; units <= 4'd3; end
                8'd134: begin tens <= 4'd13; units <= 4'd4; end
                8'd135: begin tens <= 4'd13; units <= 4'd5; end
                8'd136: begin tens <= 4'd13; units <= 4'd6; end
                8'd137: begin tens <= 4'd13; units <= 4'd7; end
                8'd138: begin tens <= 4'd13; units <= 4'd8; end
                8'd139: begin tens <= 4'd13; units <= 4'd9; end
                8'd140: begin tens <= 4'd14; units <= 4'd0; end
                8'd141: begin tens <= 4'd14; units <= 4'd1; end
                8'd142: begin tens <= 4'd14; units <= 4'd2; end
                8'd143: begin tens <= 4'd14; units <= 4'd3; end
                8'd144: begin tens <= 4'd14; units <= 4'd4; end
                8'd145: begin tens <= 4'd14; units <= 4'd5; end
                8'd146: begin tens <= 4'd14; units <= 4'd6; end
                8'd147: begin tens <= 4'd14; units <= 4'd7; end
                8'd148: begin tens <= 4'd14; units <= 4'd8; end
                8'd149: begin tens <= 4'd14; units <= 4'd9; end
                8'd150: begin tens <= 4'd15; units <= 4'd0; end
                8'd151: begin tens <= 4'd15; units <= 4'd1; end
                8'd152: begin tens <= 4'd15; units <= 4'd2; end
                8'd153: begin tens <= 4'd15; units <= 4'd3; end
                8'd154: begin tens <= 4'd15; units <= 4'd4; end
                8'd155: begin tens <= 4'd15; units <= 4'd5; end
                8'd156: begin tens <= 4'd15; units <= 4'd6; end
                8'd157: begin tens <= 4'd15; units <= 4'd7; end
                8'd158: begin tens <= 4'd15; units <= 4'd8; end
                8'd159: begin tens <= 4'd15; units <= 4'd9; end
                8'd160: begin tens <= 4'd16; units <= 4'd0; end
                8'd161: begin tens <= 4'd16; units <= 4'd1; end
                8'd162: begin tens <= 4'd16; units <= 4'd2; end
                8'd163: begin tens <= 4'd16; units <= 4'd3; end
                8'd164: begin tens <= 4'd16; units <= 4'd4; end
                8'd165: begin tens <= 4'd16; units <= 4'd5; end
                8'd166: begin tens <= 4'd16; units <= 4'd6; end
                8'd167: begin tens <= 4'd16; units <= 4'd7; end
                8'd168: begin tens <= 4'd16; units <= 4'd8; end
                8'd169: begin tens <= 4'd16; units <= 4'd9; end
                8'd170: begin tens <= 4'd17; units <= 4'd0; end
                8'd171: begin tens <= 4'd17; units <= 4'd1; end
                8'd172: begin tens <= 4'd17; units <= 4'd2; end
                8'd173: begin tens <= 4'd17; units <= 4'd3; end
                8'd174: begin tens <= 4'd17; units <= 4'd4; end
                8'd175: begin tens <= 4'd17; units <= 4'd5; end
                8'd176: begin tens <= 4'd17; units <= 4'd6; end
                8'd177: begin tens <= 4'd17; units <= 4'd7; end
                8'd178: begin tens <= 4'd17; units <= 4'd8; end
                8'd179: begin tens <= 4'd17; units <= 4'd9; end
                8'd180: begin tens <= 4'd18; units <= 4'd0; end
                8'd181: begin tens <= 4'd18; units <= 4'd1; end
                8'd182: begin tens <= 4'd18; units <= 4'd2; end
                8'd183: begin tens <= 4'd18; units <= 4'd3; end
                8'd184: begin tens <= 4'd18; units <= 4'd4; end
                8'd185: begin tens <= 4'd18; units <= 4'd5; end
                8'd186: begin tens <= 4'd18; units <= 4'd6; end
                8'd187: begin tens <= 4'd18; units <= 4'd7; end
                8'd188: begin tens <= 4'd18; units <= 4'd8; end
                8'd189: begin tens <= 4'd18; units <= 4'd9; end
                8'd190: begin tens <= 4'd19; units <= 4'd0; end
                8'd191: begin tens <= 4'd19; units <= 4'd1; end
                8'd192: begin tens <= 4'd19; units <= 4'd2; end
                8'd193: begin tens <= 4'd19; units <= 4'd3; end
                8'd194: begin tens <= 4'd19; units <= 4'd4; end
                8'd195: begin tens <= 4'd19; units <= 4'd5; end
                8'd196: begin tens <= 4'd19; units <= 4'd6; end
                8'd197: begin tens <= 4'd19; units <= 4'd7; end
                8'd198: begin tens <= 4'd19; units <= 4'd8; end
                8'd199: begin tens <= 4'd19; units <= 4'd9; end
                8'd200: begin tens <= 4'd20; units <= 4'd0; end
                8'd201: begin tens <= 4'd20; units <= 4'd1; end
                8'd202: begin tens <= 4'd20; units <= 4'd2; end
                8'd203: begin tens <= 4'd20; units <= 4'd3; end
                8'd204: begin tens <= 4'd20; units <= 4'd4; end
                8'd205: begin tens <= 4'd20; units <= 4'd5; end
                8'd206: begin tens <= 4'd20; units <= 4'd6; end
                8'd207: begin tens <= 4'd20; units <= 4'd7; end
                8'd208: begin tens <= 4'd20; units <= 4'd8; end
                8'd209: begin tens <= 4'd20; units <= 4'd9; end
                8'd210: begin tens <= 4'd21; units <= 4'd0; end
                8'd211: begin tens <= 4'd21; units <= 4'd1; end
                8'd212: begin tens <= 4'd21; units <= 4'd2; end
                8'd213: begin tens <= 4'd21; units <= 4'd3; end
                8'd214: begin tens <= 4'd21; units <= 4'd4; end
                8'd215: begin tens <= 4'd21; units <= 4'd5; end
                8'd216: begin tens <= 4'd21; units <= 4'd6; end
                8'd217: begin tens <= 4'd21; units <= 4'd7; end
                8'd218: begin tens <= 4'd21; units <= 4'd8; end
                8'd219: begin tens <= 4'd21; units <= 4'd9; end
                8'd220: begin tens <= 4'd22; units <= 4'd0; end
                8'd221: begin tens <= 4'd22; units <= 4'd1; end
                8'd222: begin tens <= 4'd22; units <= 4'd2; end
                8'd223: begin tens <= 4'd22; units <= 4'd3; end
                8'd224: begin tens <= 4'd22; units <= 4'd4; end
                8'd225: begin tens <= 4'd22; units <= 4'd5; end
                8'd226: begin tens <= 4'd22; units <= 4'd6; end
                8'd227: begin tens <= 4'd22; units <= 4'd7; end
                8'd228: begin tens <= 4'd22; units <= 4'd8; end
                8'd229: begin tens <= 4'd22; units <= 4'd9; end
                8'd230: begin tens <= 4'd23; units <= 4'd0; end
                8'd231: begin tens <= 4'd23; units <= 4'd1; end
                8'd232: begin tens <= 4'd23; units <= 4'd2; end
                8'd233: begin tens <= 4'd23; units <= 4'd3; end
                8'd234: begin tens <= 4'd23; units <= 4'd4; end
                8'd235: begin tens <= 4'd23; units <= 4'd5; end
                8'd236: begin tens <= 4'd23; units <= 4'd6; end
                8'd237: begin tens <= 4'd23; units <= 4'd7; end
                8'd238: begin tens <= 4'd23; units <= 4'd8; end
                8'd239: begin tens <= 4'd23; units <= 4'd9; end
                8'd240: begin tens <= 4'd24; units <= 4'd0; end
                default: begin tens <= 4'd0; units <= 4'd0; end
            endcase
        end
    end

    // 7-segment decoder (common anode, active low)
    function [6:0] seg7;
        input [3:0] d;
        case (d)
            4'd0: seg7 = 7'b1000000;
            4'd1: seg7 = 7'b1111001;
            4'd2: seg7 = 7'b0100100;
            4'd3: seg7 = 7'b0110000;
            4'd4: seg7 = 7'b0011001;
            4'd5: seg7 = 7'b0010010;
            4'd6: seg7 = 7'b0000010;
            4'd7: seg7 = 7'b1111000;
            4'd8: seg7 = 7'b0000000;
            4'd9: seg7 = 7'b0010000;
            default: seg7 = 7'b1111111;
        endcase
    endfunction
    
    reg [16:0] mux_cnt;
    reg [1:0]  disp_mux;
    always @(posedge clk) mux_cnt <= mux_cnt + 1;
    always @(posedge clk) disp_mux <= mux_cnt[16:15];

    assign seg = (disp_mux[0] == 0) ? seg7(units) : seg7(tens);

endmodule
