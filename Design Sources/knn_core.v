// =============================================================
//  knn_core.v  -  KNN Inference Core
//
//  TIMING FIX LOG
//  Round 1: Vote pipeline          WNS -8.827 → -2.254ns
//  Round 2: Distance pipeline      WNS -2.254 → -0.698ns
//  Round 3: ins_max_pos pipeline   WNS -0.698 → -0.334ns (router)
//           Split ins_max_pos_r into separate cycle after
//           ins_max_d_r is already registered.
//           Path was: tk_d→max_tree→ins_max_d_comb→
//                     equality_compare→priority_encode→ins_max_pos_r
//           Fix: ST_SORT_RD    latch ins_max_d_r  (max tree only)
//                ST_SORT_POS   latch ins_max_pos_r (equality+encode,
//                              now uses REGISTERED ins_max_d_r)
//                ST_SORT_INS   uses registered ins_max_pos_r as before
//           Each stage now ≤ 6 logic levels.
// =============================================================

`timescale 1ns / 1ps
`include "knn_params.vh"

module knn_core (
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          start_i,

    output reg  [16:0]                   feat_addr_o,
    input  wire [`FEATURE_WIDTH-1:0]     feat_data_i,

    output reg  [16:0]                   train_addr_o,
    input  wire [`FEATURE_WIDTH-1:0]     train_data_i,

    output reg  [9:0]                    train_lbl_addr_o,
    input  wire [`LABEL_WIDTH-1:0]       train_label_i,

    output reg  [9:0]                    dist_wr_addr_o,
    output reg  [`DIST_WIDTH-1:0]        dist_wr_data_o,
    output reg                           dist_wr_en_o,

    output reg  [9:0]                    dist_rd_addr_o,
    input  wire [`DIST_WIDTH-1:0]        dist_rd_data_i,

    output reg  [`LABEL_WIDTH-1:0]       pred_label_o,
    output reg                           pred_valid_o,
    output reg                           busy_o
);

    // ── Query register file ───────────────────────────────────
    reg [`FEATURE_WIDTH-1:0] query_reg [0:`NUM_FEATURES-1];

    // ── Top-K: 7 slots ───────────────────────────────────────
    reg [`DIST_WIDTH-1:0]  tk_d0, tk_d1, tk_d2, tk_d3, tk_d4, tk_d5, tk_d6;
    reg [`LABEL_WIDTH-1:0] tk_l0, tk_l1, tk_l2, tk_l3, tk_l4, tk_l5, tk_l6;

    // ── Vote accumulators ─────────────────────────────────────
    reg [`WEIGHT_WIDTH-1:0] v0, v1, v2, v3;

    // ── FSM ───────────────────────────────────────────────────
    localparam ST_IDLE        = 4'd0,
               ST_LOAD_FEAT   = 4'd1,
               ST_DIST_ADDR   = 4'd2,
               ST_DIST_LAT    = 4'd3,
               ST_DIST_DIFF   = 4'd4,
               ST_DIST_FLUSH  = 4'd5,
               ST_DIST_STORE  = 4'd6,
               ST_SORT_ADDR   = 4'd7,
               ST_SORT_RD     = 4'd8,   // latch ins_max_d_r only
               ST_SORT_POS    = 4'd9,   // NEW: latch ins_max_pos_r
               ST_SORT_INS    = 4'd10,  // use registered pos
               ST_VOTE        = 4'd11,
               ST_VOTE_S2     = 4'd12,
               ST_VOTE_S3     = 4'd13;

    reg [3:0]  state;
    reg [6:0]  feat_cnt;
    reg [9:0]  train_cnt;
    reg [9:0]  scan_cnt;
    reg [`DIST_WIDTH-1:0] acc_dist;
    reg [16:0]            train_base_addr;
    reg                   acc_en;

    // ── Distance pipeline registers ───────────────────────────
    reg [`FEATURE_WIDTH-1:0] train_data_r;
    reg [`FEATURE_WIDTH-1:0] query_r;

    wire [`FEATURE_WIDTH-1:0] abs_diff_comb =
        (query_r >= train_data_r) ?
        (query_r  - train_data_r) :
        (train_data_r - query_r);

    reg [`FEATURE_WIDTH-1:0] abs_diff_r;

    // ── Max-tree (combinational) ──────────────────────────────
    wire [`DIST_WIDTH-1:0] max01   = (tk_d0>=tk_d1) ? tk_d0 : tk_d1;
    wire [`DIST_WIDTH-1:0] max23   = (tk_d2>=tk_d3) ? tk_d2 : tk_d3;
    wire [`DIST_WIDTH-1:0] max45   = (tk_d4>=tk_d5) ? tk_d4 : tk_d5;
    wire [`DIST_WIDTH-1:0] max0123 = (max01>=max23)  ? max01 : max23;
    wire [`DIST_WIDTH-1:0] max4567 = (max45>=tk_d6)  ? max45 : tk_d6;
    wire [`DIST_WIDTH-1:0] ins_max_d_comb =
                               (max0123>=max4567) ? max0123 : max4567;

    // ins_max_pos_comb now uses REGISTERED ins_max_d_r (not comb)
    // This breaks the long chain: max_tree result is registered first,
    // then the equality comparison runs in the next cycle.
// Registered max and pos
    reg [`DIST_WIDTH-1:0] ins_max_d_r;
    reg [2:0]             ins_max_pos_r;
    
    // ins_max_pos_comb now uses REGISTERED ins_max_d_r (not comb)
    wire [2:0] ins_max_pos_comb =
        (tk_d0==ins_max_d_r) ? 3'd0 :
        (tk_d1==ins_max_d_r) ? 3'd1 :
        (tk_d2==ins_max_d_r) ? 3'd2 :
        (tk_d3==ins_max_d_r) ? 3'd3 :
        (tk_d4==ins_max_d_r) ? 3'd4 :
        (tk_d5==ins_max_d_r) ? 3'd5 : 3'd6;

    // ── Vote pipeline stage-1 ─────────────────────────────────
    wire [`DIST_WIDTH-1:0] safe_max_comb =
                               (ins_max_d_r==0) ? 1 : ins_max_d_r;
    reg [`DIST_WIDTH-1:0]  safe_max_r;
    reg [`DIST_WIDTH-1:0]  w0_r,w1_r,w2_r,w3_r,w4_r,w5_r,w6_r;
    reg [`LABEL_WIDTH-1:0] tl0_r,tl1_r,tl2_r,tl3_r,tl4_r,tl5_r,tl6_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=ST_IDLE; feat_cnt<=0; train_cnt<=0; scan_cnt<=0;
            acc_dist<=0; train_base_addr<=0; acc_en<=0;
            pred_label_o<=0; pred_valid_o<=0; busy_o<=0;
            feat_addr_o<=0; train_addr_o<=0; train_lbl_addr_o<=0;
            dist_wr_addr_o<=0; dist_wr_data_o<=0; dist_wr_en_o<=0;
            dist_rd_addr_o<=0;
            train_data_r<=0; query_r<=0; abs_diff_r<=0;
            tk_d0<={`DIST_WIDTH{1'b1}}; tk_l0<=0;
            tk_d1<={`DIST_WIDTH{1'b1}}; tk_l1<=0;
            tk_d2<={`DIST_WIDTH{1'b1}}; tk_l2<=0;
            tk_d3<={`DIST_WIDTH{1'b1}}; tk_l3<=0;
            tk_d4<={`DIST_WIDTH{1'b1}}; tk_l4<=0;
            tk_d5<={`DIST_WIDTH{1'b1}}; tk_l5<=0;
            tk_d6<={`DIST_WIDTH{1'b1}}; tk_l6<=0;
            ins_max_d_r<=0; ins_max_pos_r<=0; safe_max_r<=1;
            w0_r<=0;w1_r<=0;w2_r<=0;w3_r<=0;
            w4_r<=0;w5_r<=0;w6_r<=0;
            tl0_r<=0;tl1_r<=0;tl2_r<=0;tl3_r<=0;
            tl4_r<=0;tl5_r<=0;tl6_r<=0;
            v0<=0;v1<=0;v2<=0;v3<=0;
        end else begin
            pred_valid_o<=0;
            dist_wr_en_o<=0;

            case (state)

                ST_IDLE: begin
                    busy_o<=0;
                    if (start_i) begin
                        busy_o<=1; feat_cnt<=0; feat_addr_o<=0;
                        train_base_addr<=0;
                        tk_d0<={`DIST_WIDTH{1'b1}}; tk_l0<=0;
                        tk_d1<={`DIST_WIDTH{1'b1}}; tk_l1<=0;
                        tk_d2<={`DIST_WIDTH{1'b1}}; tk_l2<=0;
                        tk_d3<={`DIST_WIDTH{1'b1}}; tk_l3<=0;
                        tk_d4<={`DIST_WIDTH{1'b1}}; tk_l4<=0;
                        tk_d5<={`DIST_WIDTH{1'b1}}; tk_l5<=0;
                        tk_d6<={`DIST_WIDTH{1'b1}}; tk_l6<=0;
                        state<=ST_LOAD_FEAT;
                    end
                end

                ST_LOAD_FEAT: begin
                    feat_addr_o         <= feat_addr_o+1;
                    query_reg[feat_cnt] <= feat_data_i;
                    if (feat_cnt==`NUM_FEATURES-1) begin
                        feat_cnt<=0; train_cnt<=0;
                        train_base_addr<=0; acc_dist<=0; acc_en<=0;
                        state<=ST_DIST_ADDR;
                    end else
                        feat_cnt<=feat_cnt+1;
                end

                // ── Distance pipeline (Round-2 fix) ──────────
                ST_DIST_ADDR: begin
                    train_addr_o <= train_base_addr + feat_cnt;
                    state        <= ST_DIST_LAT;
                end

                ST_DIST_LAT: begin
                    train_data_r <= train_data_i;
                    query_r      <= query_reg[feat_cnt];
                    if (acc_en)
                        acc_dist <= acc_dist +
                            {{(`DIST_WIDTH-`FEATURE_WIDTH){1'b0}}, abs_diff_r};
                    state <= ST_DIST_DIFF;
                end

                ST_DIST_DIFF: begin
                    abs_diff_r <= abs_diff_comb;
                    acc_en     <= 1;
                    if (feat_cnt==`NUM_FEATURES-1) begin
                        feat_cnt <= 0;
                        state    <= ST_DIST_FLUSH;
                    end else begin
                        feat_cnt     <= feat_cnt+1;
                        train_addr_o <= train_base_addr + feat_cnt + 1;
                        state        <= ST_DIST_LAT;
                    end
                end

                ST_DIST_FLUSH: begin
                    acc_dist <= acc_dist +
                        {{(`DIST_WIDTH-`FEATURE_WIDTH){1'b0}}, abs_diff_r};
                    state <= ST_DIST_STORE;
                end

                ST_DIST_STORE: begin
                    dist_wr_addr_o <= train_cnt[9:0];
                    dist_wr_data_o <= acc_dist;
                    dist_wr_en_o   <= 1;
                    acc_dist       <= 0;
                    if (train_cnt==`NUM_TRAIN-1) begin
                        scan_cnt<=0; state<=ST_SORT_ADDR;
                    end else begin
                        train_cnt       <= train_cnt+1;
                        train_base_addr <= train_base_addr+`NUM_FEATURES;
                        feat_cnt        <= 0; acc_en <= 0;
                        state           <= ST_DIST_ADDR;
                    end
                end

                // ── Sort (Round-3 fix: split into 2 stages) ──
                ST_SORT_ADDR: begin
                    dist_rd_addr_o   <= scan_cnt[9:0];
                    train_lbl_addr_o <= scan_cnt[9:0];
                    state            <= ST_SORT_RD;
                end

                // Stage 1: latch max distance only
                // Path: tk_d regs → max-tree → ins_max_d_r FF  ≈ 5ns ✓
                ST_SORT_RD: begin
                    ins_max_d_r <= ins_max_d_comb;
                    state       <= ST_SORT_POS;
                end

                // Stage 2: latch max position using REGISTERED ins_max_d_r
                // Path: tk_d regs + ins_max_d_r → equality compare
                //       → priority encode → ins_max_pos_r FF  ≈ 4ns ✓
                // (No max-tree in this path - it was already registered)
                ST_SORT_POS: begin
                    ins_max_pos_r <= ins_max_pos_comb;
                    state         <= ST_SORT_INS;
                end

                ST_SORT_INS: begin
                    if (dist_rd_data_i < ins_max_d_r) begin
                        case (ins_max_pos_r)
                            3'd0:begin tk_d0<=dist_rd_data_i;tk_l0<=train_label_i;end
                            3'd1:begin tk_d1<=dist_rd_data_i;tk_l1<=train_label_i;end
                            3'd2:begin tk_d2<=dist_rd_data_i;tk_l2<=train_label_i;end
                            3'd3:begin tk_d3<=dist_rd_data_i;tk_l3<=train_label_i;end
                            3'd4:begin tk_d4<=dist_rd_data_i;tk_l4<=train_label_i;end
                            3'd5:begin tk_d5<=dist_rd_data_i;tk_l5<=train_label_i;end
                            default:begin tk_d6<=dist_rd_data_i;tk_l6<=train_label_i;end
                        endcase
                    end
                    if (scan_cnt==`NUM_TRAIN-1)
                        state<=ST_VOTE;
                    else begin
                        scan_cnt<=scan_cnt+1;
                        state<=ST_SORT_ADDR;
                    end
                end

                // ── Vote pipeline (Round-1 fix) ───────────────
                ST_VOTE: begin
                    ins_max_d_r <= ins_max_d_comb;
                    safe_max_r  <= safe_max_comb;
                    w0_r<=(tk_d0<safe_max_comb)?(safe_max_comb-tk_d0):1;
                    w1_r<=(tk_d1<safe_max_comb)?(safe_max_comb-tk_d1):1;
                    w2_r<=(tk_d2<safe_max_comb)?(safe_max_comb-tk_d2):1;
                    w3_r<=(tk_d3<safe_max_comb)?(safe_max_comb-tk_d3):1;
                    w4_r<=(tk_d4<safe_max_comb)?(safe_max_comb-tk_d4):1;
                    w5_r<=(tk_d5<safe_max_comb)?(safe_max_comb-tk_d5):1;
                    w6_r<=(tk_d6<safe_max_comb)?(safe_max_comb-tk_d6):1;
                    tl0_r<=tk_l0;tl1_r<=tk_l1;tl2_r<=tk_l2;tl3_r<=tk_l3;
                    tl4_r<=tk_l4;tl5_r<=tk_l5;tl6_r<=tk_l6;
                    state<=ST_VOTE_S2;
                end

                ST_VOTE_S2: begin
                    v0<=(tl0_r==0?w0_r:0)+(tl1_r==0?w1_r:0)+(tl2_r==0?w2_r:0)+
                        (tl3_r==0?w3_r:0)+(tl4_r==0?w4_r:0)+(tl5_r==0?w5_r:0)+
                        (tl6_r==0?w6_r:0);
                    v1<=(tl0_r==1?w0_r:0)+(tl1_r==1?w1_r:0)+(tl2_r==1?w2_r:0)+
                        (tl3_r==1?w3_r:0)+(tl4_r==1?w4_r:0)+(tl5_r==1?w5_r:0)+
                        (tl6_r==1?w6_r:0);
                    v2<=(tl0_r==2?w0_r:0)+(tl1_r==2?w1_r:0)+(tl2_r==2?w2_r:0)+
                        (tl3_r==2?w3_r:0)+(tl4_r==2?w4_r:0)+(tl5_r==2?w5_r:0)+
                        (tl6_r==2?w6_r:0);
                    v3<=(tl0_r==3?w0_r:0)+(tl1_r==3?w1_r:0)+(tl2_r==3?w2_r:0)+
                        (tl3_r==3?w3_r:0)+(tl4_r==3?w4_r:0)+(tl5_r==3?w5_r:0)+
                        (tl6_r==3?w6_r:0);
                    state<=ST_VOTE_S3;
                end

                ST_VOTE_S3: begin
                    begin : winner_block
                        reg [`WEIGHT_WIDTH-1:0] mx;
                        reg [`LABEL_WIDTH-1:0]  wn;
                        mx=v0; wn=0;
                        if(v1>mx)begin mx=v1;wn=1;end
                        if(v2>mx)begin mx=v2;wn=2;end
                        if(v3>mx)begin mx=v3;wn=3;end
                        pred_label_o<=wn;
                    end
                    pred_valid_o<=1;
                    busy_o<=0;
                    state<=ST_IDLE;
                end

                default: state<=ST_IDLE;
            endcase
        end
    end

endmodule