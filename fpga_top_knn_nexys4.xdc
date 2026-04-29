## =============================================================================
##  fpga_top_knn_nexys4.xdc
##  Constraints for: PMDC Motor Fault Detection - KNN Inference Demo
##  Board  : Digilent Nexys 4 (original, NOT Nexys 4 DDR)
##  FPGA   : Xilinx Artix-7  XC7A100T-1CSG324C
##  Tool   : Vivado 2015.x or later
##
##  Ports mapped:
##    clk      → E3   (100 MHz crystal oscillator)
##    sw0      → U9   (slide switch SW0, active-high reset: slide UP = reset)
##    btnC     → E16  (centre button - re-run inference)
##    led[15:0]→ LD0-LD15
##    seg[6:0] → CA-CG (7-segment cathodes, active low)
##    an[3:0]  → AN0-AN3 (anodes, active low - we use AN0+AN1 for 2-digit display)
##    uart_txd → D4   (USB-UART TX, goes to PC via USB cable)
## =============================================================================

## ── Clock ────────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN E3         [get_ports clk]
set_property IOSTANDARD  LVCMOS33   [get_ports clk]
create_clock -add -name sys_clk_pin -period 10.000 -waveform {0 5} [get_ports clk]

## ── Reset switch (SW0) ────────────────────────────────────────────────────────
set_property PACKAGE_PIN U9         [get_ports sw0]
set_property IOSTANDARD  LVCMOS33   [get_ports sw0]

## ── Buttons ──────────────────────────────────────────────────────────────────
## btnC - centre button (re-run inference)
set_property PACKAGE_PIN E16        [get_ports btnC]
set_property IOSTANDARD  LVCMOS33   [get_ports btnC]

## ── LEDs ─────────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN T8         [get_ports {led[0]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {led[0]}]
set_property PACKAGE_PIN V9         [get_ports {led[1]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {led[1]}]
set_property PACKAGE_PIN R8         [get_ports {led[2]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {led[2]}]
set_property PACKAGE_PIN T6         [get_ports {led[3]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {led[3]}]
set_property PACKAGE_PIN T5         [get_ports {led[4]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {led[4]}]
set_property PACKAGE_PIN T4         [get_ports {led[5]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {led[5]}]
set_property PACKAGE_PIN U7         [get_ports {led[6]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {led[6]}]
set_property PACKAGE_PIN U6         [get_ports {led[7]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {led[7]}]
set_property PACKAGE_PIN V4         [get_ports {led[8]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {led[8]}]
set_property PACKAGE_PIN U3         [get_ports {led[9]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {led[9]}]
set_property PACKAGE_PIN V1         [get_ports {led[10]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {led[10]}]
set_property PACKAGE_PIN R1         [get_ports {led[11]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {led[11]}]
set_property PACKAGE_PIN P5         [get_ports {led[12]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {led[12]}]
set_property PACKAGE_PIN U1         [get_ports {led[13]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {led[13]}]
set_property PACKAGE_PIN R2         [get_ports {led[14]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {led[14]}]
set_property PACKAGE_PIN P2         [get_ports {led[15]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {led[15]}]

## ── 7-Segment Cathodes (active low) ──────────────────────────────────────────
set_property PACKAGE_PIN L3         [get_ports {seg[0]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {seg[0]}]
set_property PACKAGE_PIN N1         [get_ports {seg[1]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {seg[1]}]
set_property PACKAGE_PIN L5         [get_ports {seg[2]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {seg[2]}]
set_property PACKAGE_PIN L4         [get_ports {seg[3]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {seg[3]}]
set_property PACKAGE_PIN K3         [get_ports {seg[4]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {seg[4]}]
set_property PACKAGE_PIN M2         [get_ports {seg[5]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {seg[5]}]
set_property PACKAGE_PIN L6         [get_ports {seg[6]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {seg[6]}]

## ── 7-Segment Anodes (active low) ────────────────────────────────────────────
## AN0 = rightmost digit (units), AN1 = second from right (tens)
## AN2, AN3 driven high (off) by assign an = 4'b1100
set_property PACKAGE_PIN N6         [get_ports {an[0]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {an[0]}]
set_property PACKAGE_PIN M6         [get_ports {an[1]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {an[1]}]
set_property PACKAGE_PIN M3         [get_ports {an[2]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {an[2]}]
set_property PACKAGE_PIN N5         [get_ports {an[3]}]
set_property IOSTANDARD  LVCMOS33   [get_ports {an[3]}]

## ── UART TX ──────────────────────────────────────────────────────────────────
## Nexys4 USB-UART bridge: FPGA TX → D4  (UART_TXD_IN on schematic)
## Connect PuTTY / Tera Term to the board's COM port at 115200 8N1
set_property PACKAGE_PIN D4         [get_ports uart_txd]
set_property IOSTANDARD  LVCMOS33   [get_ports uart_txd]

## ── Timing false paths (async inputs) ───────────────────────────────────────
set_false_path -from [get_ports sw0]
set_false_path -from [get_ports btnC]

## ── Bitstream settings ───────────────────────────────────────────────────────
set_property CFGBVS         VCCO    [current_design]
set_property CONFIG_VOLTAGE 3.3     [current_design]

