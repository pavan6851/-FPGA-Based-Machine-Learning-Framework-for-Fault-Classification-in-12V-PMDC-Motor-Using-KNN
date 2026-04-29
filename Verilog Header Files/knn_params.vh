// =============================================================
//  knn_params.vh  -  KNN Classifier Parameters
//  PMDC Motor Fault Detection - KNN Inference Core
//  Board  : Digilent Nexys 4 (XC7A100T-1CSG324C)
//
//  Python export summary:
//    Features      : 90  (85 original + 5 engineered)
//    Training set  : 958 samples
//    Test set      : 240 samples
//    K             : 7
//    Distance      : Manhattan (L1)
//    Voting        : Distance-weighted (MAX_D - d)
//    CV Accuracy   : 89.57%
//    Hold-out Acc  : 87.50%
//    FPGA Sim Acc  : 87.50%
// =============================================================

`ifndef KNN_PARAMS_VH
`define KNN_PARAMS_VH

// ── Core KNN parameters ──────────────────────────────────────
`define NUM_FEATURES   90        // 85 original + 5 engineered
`define NUM_TRAIN      958       // training samples
`define NUM_TEST       240       // test samples (for batch mode)
`define K_NEIGHBOURS   7         // K value from Python sweep

// ── Fixed-point format ───────────────────────────────────────
`define FEATURE_WIDTH  16        // bits per feature (unsigned Q16)
`define DIST_WIDTH     24        // Manhattan dist: 16 + ceil(log2(90))=7 → 24
`define LABEL_WIDTH    2         // ceil(log2(4 classes))
`define WEIGHT_WIDTH   27        // DIST_WIDTH + ceil(log2(K)) = 24+3

// ── Class encodings ──────────────────────────────────────────
`define CLASS_HEALTHY  2'd0
`define CLASS_FAULT1   2'd1
`define CLASS_FAULT2   2'd2
`define CLASS_FAULT3   2'd3

// ── BRAM derived parameters ──────────────────────────────────
// Train data BRAM  : 958 * 90 = 86220 words × 16 bits
// Test  data BRAM  : 240 * 90 = 21600 words × 16 bits
// Label BRAMs      : 958 / 240 words × 2 bits (stored as 8-bit)
`define TRAIN_BRAM_DEPTH  86220
`define TEST_BRAM_DEPTH   21600

// ── UART parameters (115200 baud @ 100 MHz) ──────────────────
`define UART_CLK_DIV   868       // 100_000_000 / 115200 ≈ 868

// ── Board timing ─────────────────────────────────────────────
`define DEBOUNCE_CYCLES 21'd2_000_000

`endif  // KNN_PARAMS_VH
