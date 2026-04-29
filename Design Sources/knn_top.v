// =============================================================
//  knn_top.v  -  K-Nearest Neighbour Classifier (4-Class)
//  Target      : FPGA (Xilinx / Intel)
//  Distance    : Manhattan  (no sqrt, FPGA-friendly)
//  Fixed-point : 16-bit unsigned per feature
//  UPDATED     : Parameters synced to Python export (90 features,
//                K=7, 958 train samples, distance-weighted voting)
// =============================================================
//
//  USAGE:
//    1. Synthesise together with:
//         knn_distance.v  (if separate)
//         knn_sorter.v    (if separate)
//         knn_voter.v     (if separate)
//    2. Load knn_train_data.mem into BRAM or ROM initialisation.
//    3. .mem files exported by Python must match NUM_FEATURES=90,
//       NUM_TRAIN=958 exactly.
//
// =============================================================

`timescale 1ns / 1ps

module knn_top #(
    parameter NUM_FEATURES  = 90,   // 85 original + 5 engineered  ← UPDATED
    parameter NUM_TRAIN     = 958,  // training samples             ← UPDATED
    parameter K             = 7,    // best K from Python sweep     ← UPDATED
    parameter FEATURE_WIDTH = 16,   // bits per feature (fixed-point)
    parameter DIST_WIDTH    = 24,   // 16 + ceil(log2(90))=7 → 23, use 24
    parameter NUM_CLASSES   = 4,
    parameter LABEL_WIDTH   = 2     // ceil(log2(NUM_CLASSES))
)(
    input  wire                              clk,
    input  wire                              rst_n,
    // Query sample input (one feature at a time, index driven)
    input  wire                              query_valid,
    input  wire [FEATURE_WIDTH-1:0]          query_data,
    input  wire [$clog2(NUM_FEATURES)-1:0]   query_idx,
    // Result output
    output reg  [LABEL_WIDTH-1:0]            pred_label,
    output reg                               pred_valid
);

    // ──────────────────────────────────────────────────────────
    // 1. Query register file
    // ──────────────────────────────────────────────────────────
    reg [FEATURE_WIDTH-1:0] query_reg [0:NUM_FEATURES-1];
    reg query_loaded;
    integer qi;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            query_loaded <= 0;
            for (qi = 0; qi < NUM_FEATURES; qi = qi + 1)
                query_reg[qi] <= 0;
        end else if (query_valid) begin
            query_reg[query_idx] <= query_data;
            if (query_idx == NUM_FEATURES - 1)
                query_loaded <= 1;
            else
                query_loaded <= 0;
        end else begin
            query_loaded <= 0;
        end
    end

    // ──────────────────────────────────────────────────────────
    // 2. Training data ROM  (initialised from .mem file)
    // ──────────────────────────────────────────────────────────
    reg [FEATURE_WIDTH-1:0] train_data  [0:NUM_TRAIN*NUM_FEATURES-1];
    reg [LABEL_WIDTH-1:0]   train_label [0:NUM_TRAIN-1];

    initial begin
        $readmemh("knn_train_data.mem",   train_data);
        $readmemh("knn_train_labels.mem", train_label);
    end

    // ──────────────────────────────────────────────────────────
    // 3. Distance computation  (sequential, one train sample / cycle)
    // ──────────────────────────────────────────────────────────
    reg  [DIST_WIDTH-1:0]        dist_array  [0:NUM_TRAIN-1];
    reg  [$clog2(NUM_TRAIN)-1:0] comp_idx;
    reg  compute_busy;
    reg  compute_done;

    reg  [DIST_WIDTH-1:0]            acc_dist;
    reg  [$clog2(NUM_FEATURES)-1:0]  feat_idx;
    reg  feat_busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            comp_idx     <= 0;
            compute_busy <= 0;
            compute_done <= 0;
            feat_idx     <= 0;
            feat_busy    <= 0;
            acc_dist     <= 0;
        end else begin

            compute_done <= 0;

            if (query_loaded && !compute_busy) begin
                compute_busy <= 1;
                comp_idx     <= 0;
                feat_idx     <= 0;
                feat_busy    <= 1;
                acc_dist     <= 0;
            end

            if (compute_busy) begin
                if (feat_busy) begin
                    begin : abs_diff
                        reg [FEATURE_WIDTH-1:0] a, b;
                        a = query_reg[feat_idx];
                        b = train_data[comp_idx * NUM_FEATURES + feat_idx];
                        acc_dist <= acc_dist + ((a >= b) ? (a - b) : (b - a));
                    end

                    if (feat_idx == NUM_FEATURES - 1) begin
                        feat_idx  <= 0;
                        feat_busy <= 0;
                    end else begin
                        feat_idx <= feat_idx + 1;
                    end
                end else begin
                    dist_array[comp_idx] <= acc_dist;
                    acc_dist             <= 0;
                    feat_busy            <= 1;

                    if (comp_idx == NUM_TRAIN - 1) begin
                        compute_busy <= 0;
                        compute_done <= 1;
                    end else begin
                        comp_idx <= comp_idx + 1;
                    end
                end
            end
        end
    end

    // ──────────────────────────────────────────────────────────
    // 4. K-Nearest Selection  (linear scan top-K insertion)
    // ──────────────────────────────────────────────────────────
    reg  [DIST_WIDTH-1:0]        topk_dist  [0:K-1];
    reg  [LABEL_WIDTH-1:0]       topk_label [0:K-1];
    reg  sort_busy;
    reg  sort_done;
    reg  [$clog2(NUM_TRAIN)-1:0] scan_idx;

    integer ki;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sort_busy <= 0;
            sort_done <= 0;
            scan_idx  <= 0;
            for (ki = 0; ki < K; ki = ki + 1) begin
                topk_dist[ki]  <= {DIST_WIDTH{1'b1}};
                topk_label[ki] <= 0;
            end
        end else begin
            sort_done <= 0;

            if (compute_done) begin
                sort_busy <= 1;
                scan_idx  <= 0;
                for (ki = 0; ki < K; ki = ki + 1)
                    topk_dist[ki] <= {DIST_WIDTH{1'b1}};
            end

            if (sort_busy) begin
                begin : insert_knn
                    reg [DIST_WIDTH-1:0]  cur_d;
                    reg [LABEL_WIDTH-1:0] cur_l;
                    reg [DIST_WIDTH-1:0]  max_d;
                    reg [$clog2(K)-1:0]   max_pos;
                    integer ki2;

                    cur_d   = dist_array[scan_idx];
                    cur_l   = train_label[scan_idx];
                    max_d   = topk_dist[0];
                    max_pos = 0;

                    for (ki2 = 1; ki2 < K; ki2 = ki2 + 1) begin
                        if (topk_dist[ki2] > max_d) begin
                            max_d   = topk_dist[ki2];
                            max_pos = ki2;
                        end
                    end

                    if (cur_d < max_d) begin
                        topk_dist[max_pos]  <= cur_d;
                        topk_label[max_pos] <= cur_l;
                    end
                end

                if (scan_idx == NUM_TRAIN - 1) begin
                    sort_busy <= 0;
                    sort_done <= 1;
                end else begin
                    scan_idx <= scan_idx + 1;
                end
            end
        end
    end

    // ──────────────────────────────────────────────────────────
    // 5. Distance-Weighted Majority Vote  ← UPDATED
    //    Approximation: weight = (MAX_DIST - d)
    //    Avoids division; monotonically decreases with distance.
    //    Ties in distance use weight=1 to avoid divide-by-zero.
    // ──────────────────────────────────────────────────────────

    // Accumulate weights per class using (MAX_DIST - d) weighting
    // Weight width: DIST_WIDTH bits, sum over K=7 → need DIST_WIDTH+3 bits
    localparam WEIGHT_WIDTH = DIST_WIDTH + 3;  // 24+3=27 bits, safe for K=7

    integer vi;
    reg [WEIGHT_WIDTH-1:0] votes      [0:NUM_CLASSES-1];
    reg [WEIGHT_WIDTH-1:0] max_votes;
    reg [LABEL_WIDTH-1:0]  winner;

    // Find max distance among top-K for weight normalisation
    reg [DIST_WIDTH-1:0] max_topk_dist;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pred_valid    <= 0;
            pred_label    <= 0;
            max_topk_dist <= 0;
        end else begin
            pred_valid <= 0;

            if (sort_done) begin
                // ── Find max distance in top-K ──
                max_topk_dist = topk_dist[0];
                for (vi = 1; vi < K; vi = vi + 1)
                    if (topk_dist[vi] > max_topk_dist)
                        max_topk_dist = topk_dist[vi];

                // Guard: if max_topk_dist == 0 all neighbours are identical;
                // fall back to uniform vote (weight=1 each)
                if (max_topk_dist == 0) max_topk_dist = 1;

                // ── Clear vote accumulators ──
                for (vi = 0; vi < NUM_CLASSES; vi = vi + 1)
                    votes[vi] = 0;

                // ── Accumulate (MAX_DIST - d) weights ──
                for (vi = 0; vi < K; vi = vi + 1) begin
                    if (topk_dist[vi] < max_topk_dist)
                        // Normal case: closer = higher weight
                        votes[topk_label[vi]] = votes[topk_label[vi]]
                                                + (max_topk_dist - topk_dist[vi]);
                    else
                        // Farthest neighbour(s): give weight 1 (not 0)
                        votes[topk_label[vi]] = votes[topk_label[vi]] + 1;
                end

                // ── Find class with highest accumulated weight ──
                max_votes = 0;
                winner    = 0;
                for (vi = 0; vi < NUM_CLASSES; vi = vi + 1) begin
                    if (votes[vi] > max_votes) begin
                        max_votes = votes[vi];
                        winner    = vi;
                    end
                end

                pred_label <= winner;
                pred_valid <= 1;
            end
        end
    end

endmodule