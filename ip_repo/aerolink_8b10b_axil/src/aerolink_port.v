`timescale 1ns / 1ps
// =============================================================================
// AeroLink-8b10b Single Port Wrapper
// =============================================================================
// Instantiates TX engine, RX engine, and all 8 FIFOs for one AeroLink port.
// =============================================================================

module aerolink_port #(
    parameter CLK_FREQ_HZ = 125_000_000,
    parameter SYMBOL_RATE = 2_500_000,
    parameter IS_MASTER   = 1,
    parameter FIFO_DEPTH  = 128
) (
    input  wire        clk,
    input  wire        rst,

    // Control register bits
    input  wire        ctrl_tx_enable,
    input  wire        ctrl_rx_enable,
    input  wire        ctrl_tx_reset,
    input  wire        ctrl_rx_reset,
    input  wire        ctrl_drop_errored,

    // TX high-priority data FIFO write interface
    input  wire [31:0] tx_hipri_data_in,
    input  wire        tx_hipri_data_wr,
    output wire        tx_hipri_data_full,
    output wire        tx_hipri_data_empty,

    // TX high-priority control word FIFO
    input  wire [31:0] tx_hipri_ctrl_in,
    input  wire        tx_hipri_ctrl_wr,
    output wire        tx_hipri_ctrl_full,
    output wire        tx_hipri_ctrl_empty,

    // TX regular-priority data FIFO
    input  wire [31:0] tx_regpri_data_in,
    input  wire        tx_regpri_data_wr,
    output wire        tx_regpri_data_full,
    output wire        tx_regpri_data_empty,

    // TX regular-priority control word FIFO
    input  wire [31:0] tx_regpri_ctrl_in,
    input  wire        tx_regpri_ctrl_wr,
    output wire        tx_regpri_ctrl_full,
    output wire        tx_regpri_ctrl_empty,

    // RX high-priority data FIFO read interface
    output wire [31:0] rx_hipri_data_out,
    input  wire        rx_hipri_data_rd,
    output wire        rx_hipri_data_empty,
    output wire        rx_hipri_data_full,

    // RX high-priority control word FIFO
    output wire [31:0] rx_hipri_ctrl_out,
    input  wire        rx_hipri_ctrl_rd,
    output wire        rx_hipri_ctrl_empty,
    output wire        rx_hipri_ctrl_full,

    // RX regular-priority data FIFO
    output wire [31:0] rx_regpri_data_out,
    input  wire        rx_regpri_data_rd,
    output wire        rx_regpri_data_empty,
    output wire        rx_regpri_data_full,

    // RX regular-priority control word FIFO
    output wire [31:0] rx_regpri_ctrl_out,
    input  wire        rx_regpri_ctrl_rd,
    output wire        rx_regpri_ctrl_empty,
    output wire        rx_regpri_ctrl_full,

    // Statistics counters
    output reg  [15:0] stat_tx_frames,
    output reg  [15:0] stat_tx_idle_frames,
    output reg  [15:0] stat_rx_frames,
    output reg  [15:0] stat_rx_crc_errors,
    output reg  [15:0] stat_rx_disp_errors,
    output reg  [15:0] stat_rx_sym_errors,

    // Interrupt sources
    output wire        irq_tx_hipri_empty,
    output wire        irq_tx_regpri_empty,
    output wire        irq_tx_hipri_complete,
    output wire        irq_tx_regpri_complete,
    output wire        irq_rx_hipri_not_empty,
    output wire        irq_rx_regpri_not_empty,

    // Serial I/O (active-high tristate: aerolink_t=1 means high-Z)
    output wire        aerolink_o,
    input  wire        aerolink_i,
    output wire        aerolink_t
);

    // =========================================================================
    // Reset generation
    // =========================================================================
    wire tx_rst = rst | ctrl_tx_reset;
    wire rx_rst = rst | ctrl_rx_reset;

    // =========================================================================
    // Internal wires: TX engine <-> TX FIFOs
    // =========================================================================
    wire [31:0] tx_eng_hipri_data;
    wire        tx_eng_hipri_data_empty;
    wire        tx_eng_hipri_data_rd;

    wire [31:0] tx_eng_hipri_ctrl;
    wire        tx_eng_hipri_ctrl_empty;
    wire        tx_eng_hipri_ctrl_rd;

    wire [31:0] tx_eng_regpri_data;
    wire        tx_eng_regpri_data_empty;
    wire        tx_eng_regpri_data_rd;

    wire [31:0] tx_eng_regpri_ctrl;
    wire        tx_eng_regpri_ctrl_empty;
    wire        tx_eng_regpri_ctrl_rd;

    // =========================================================================
    // Internal wires: RX engine <-> RX FIFOs
    // =========================================================================
    wire [31:0] rx_eng_hipri_data;
    wire        rx_eng_hipri_data_wr;

    wire [31:0] rx_eng_hipri_ctrl;
    wire        rx_eng_hipri_ctrl_wr;

    wire [31:0] rx_eng_regpri_data;
    wire        rx_eng_regpri_data_wr;

    wire [31:0] rx_eng_regpri_ctrl;
    wire        rx_eng_regpri_ctrl_wr;

    // =========================================================================
    // Serial I/O tristate logic
    // =========================================================================
    wire        tx_eng_serial;
    wire        tx_eng_tx_active;

    assign aerolink_o = tx_eng_serial;
    assign aerolink_t = ~tx_eng_tx_active;

    // Suppress local echo: RX sees idle (1) while local TX is driving
    wire rx_serial_gated = tx_eng_tx_active ? 1'b1 : aerolink_i;

    // =========================================================================
    // Internal wires: TX engine status/IRQ
    // =========================================================================
    wire        tx_eng_stat_tx_frame;
    wire        tx_eng_stat_tx_idle;
    wire        tx_eng_irq_hipri_empty;
    wire        tx_eng_irq_regpri_empty;
    wire        tx_eng_irq_hipri_complete;
    wire        tx_eng_irq_regpri_complete;

    // =========================================================================
    // Internal wires: RX engine status
    // =========================================================================
    wire        rx_eng_stat_rx_frame;
    wire        rx_eng_stat_rx_crc_err;
    wire        rx_eng_stat_rx_disp_err;
    wire        rx_eng_stat_rx_sym_err;
    wire        rx_eng_rx_frame_done;

    // =========================================================================
    // TX FIFOs
    // =========================================================================

    // TX high-priority data FIFO
    aerolink_fifo #(
        .WIDTH (32),
        .DEPTH (FIFO_DEPTH)
    ) u_tx_hipri_data_fifo (
        .clk     (clk),
        .rst     (tx_rst),
        .wr_en   (tx_hipri_data_wr),
        .wr_data (tx_hipri_data_in),
        .rd_en   (tx_eng_hipri_data_rd),
        .rd_data (tx_eng_hipri_data),
        .full    (tx_hipri_data_full),
        .empty   (tx_hipri_data_empty),
        .count   ()
    );

    // TX high-priority control FIFO
    aerolink_fifo #(
        .WIDTH (32),
        .DEPTH (32)
    ) u_tx_hipri_ctrl_fifo (
        .clk     (clk),
        .rst     (tx_rst),
        .wr_en   (tx_hipri_ctrl_wr),
        .wr_data (tx_hipri_ctrl_in),
        .rd_en   (tx_eng_hipri_ctrl_rd),
        .rd_data (tx_eng_hipri_ctrl),
        .full    (tx_hipri_ctrl_full),
        .empty   (tx_hipri_ctrl_empty),
        .count   ()
    );

    // TX regular-priority data FIFO
    aerolink_fifo #(
        .WIDTH (32),
        .DEPTH (FIFO_DEPTH)
    ) u_tx_regpri_data_fifo (
        .clk     (clk),
        .rst     (tx_rst),
        .wr_en   (tx_regpri_data_wr),
        .wr_data (tx_regpri_data_in),
        .rd_en   (tx_eng_regpri_data_rd),
        .rd_data (tx_eng_regpri_data),
        .full    (tx_regpri_data_full),
        .empty   (tx_regpri_data_empty),
        .count   ()
    );

    // TX regular-priority control FIFO
    aerolink_fifo #(
        .WIDTH (32),
        .DEPTH (32)
    ) u_tx_regpri_ctrl_fifo (
        .clk     (clk),
        .rst     (tx_rst),
        .wr_en   (tx_regpri_ctrl_wr),
        .wr_data (tx_regpri_ctrl_in),
        .rd_en   (tx_eng_regpri_ctrl_rd),
        .rd_data (tx_eng_regpri_ctrl),
        .full    (tx_regpri_ctrl_full),
        .empty   (tx_regpri_ctrl_empty),
        .count   ()
    );

    // =========================================================================
    // RX FIFOs
    // =========================================================================

    // RX high-priority data FIFO
    aerolink_fifo #(
        .WIDTH (32),
        .DEPTH (FIFO_DEPTH)
    ) u_rx_hipri_data_fifo (
        .clk     (clk),
        .rst     (rx_rst),
        .wr_en   (rx_eng_hipri_data_wr),
        .wr_data (rx_eng_hipri_data),
        .rd_en   (rx_hipri_data_rd),
        .rd_data (rx_hipri_data_out),
        .full    (rx_hipri_data_full),
        .empty   (rx_hipri_data_empty),
        .count   ()
    );

    // RX high-priority control FIFO
    aerolink_fifo #(
        .WIDTH (32),
        .DEPTH (32)
    ) u_rx_hipri_ctrl_fifo (
        .clk     (clk),
        .rst     (rx_rst),
        .wr_en   (rx_eng_hipri_ctrl_wr),
        .wr_data (rx_eng_hipri_ctrl),
        .rd_en   (rx_hipri_ctrl_rd),
        .rd_data (rx_hipri_ctrl_out),
        .full    (rx_hipri_ctrl_full),
        .empty   (rx_hipri_ctrl_empty),
        .count   ()
    );

    // RX regular-priority data FIFO
    aerolink_fifo #(
        .WIDTH (32),
        .DEPTH (FIFO_DEPTH)
    ) u_rx_regpri_data_fifo (
        .clk     (clk),
        .rst     (rx_rst),
        .wr_en   (rx_eng_regpri_data_wr),
        .wr_data (rx_eng_regpri_data),
        .rd_en   (rx_regpri_data_rd),
        .rd_data (rx_regpri_data_out),
        .full    (rx_regpri_data_full),
        .empty   (rx_regpri_data_empty),
        .count   ()
    );

    // RX regular-priority control FIFO
    aerolink_fifo #(
        .WIDTH (32),
        .DEPTH (32)
    ) u_rx_regpri_ctrl_fifo (
        .clk     (clk),
        .rst     (rx_rst),
        .wr_en   (rx_eng_regpri_ctrl_wr),
        .wr_data (rx_eng_regpri_ctrl),
        .rd_en   (rx_regpri_ctrl_rd),
        .rd_data (rx_regpri_ctrl_out),
        .full    (rx_regpri_ctrl_full),
        .empty   (rx_regpri_ctrl_empty),
        .count   ()
    );

    // =========================================================================
    // TX Engine
    // =========================================================================
    aerolink_tx_engine #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .SYMBOL_RATE (SYMBOL_RATE),
        .IS_MASTER   (IS_MASTER)
    ) u_tx_engine (
        .clk                (clk),
        .rst                (tx_rst),
        .tx_enable          (ctrl_tx_enable),
        .rx_frame_done      (rx_eng_rx_frame_done),
        .hipri_data         (tx_eng_hipri_data),
        .hipri_data_empty   (tx_hipri_data_empty),
        .hipri_data_rd      (tx_eng_hipri_data_rd),
        .hipri_ctrl         (tx_eng_hipri_ctrl),
        .hipri_ctrl_empty   (tx_hipri_ctrl_empty),
        .hipri_ctrl_rd      (tx_eng_hipri_ctrl_rd),
        .regpri_data        (tx_eng_regpri_data),
        .regpri_data_empty  (tx_regpri_data_empty),
        .regpri_data_rd     (tx_eng_regpri_data_rd),
        .regpri_ctrl        (tx_eng_regpri_ctrl),
        .regpri_ctrl_empty  (tx_regpri_ctrl_empty),
        .regpri_ctrl_rd     (tx_eng_regpri_ctrl_rd),
        .tx_serial          (tx_eng_serial),
        .tx_active          (tx_eng_tx_active),
        .stat_tx_frame      (tx_eng_stat_tx_frame),
        .stat_tx_idle       (tx_eng_stat_tx_idle),
        .irq_hipri_empty    (tx_eng_irq_hipri_empty),
        .irq_regpri_empty   (tx_eng_irq_regpri_empty),
        .irq_hipri_complete (tx_eng_irq_hipri_complete),
        .irq_regpri_complete(tx_eng_irq_regpri_complete)
    );

    // =========================================================================
    // RX Engine
    // =========================================================================
    aerolink_rx_engine #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .SYMBOL_RATE (SYMBOL_RATE)
    ) u_rx_engine (
        .clk              (clk),
        .rst              (rx_rst),
        .rx_enable        (ctrl_rx_enable),
        .drop_errored     (ctrl_drop_errored),
        .rx_serial        (rx_serial_gated),
        .hipri_data       (rx_eng_hipri_data),
        .hipri_data_wr    (rx_eng_hipri_data_wr),
        .hipri_ctrl       (rx_eng_hipri_ctrl),
        .hipri_ctrl_wr    (rx_eng_hipri_ctrl_wr),
        .regpri_data      (rx_eng_regpri_data),
        .regpri_data_wr   (rx_eng_regpri_data_wr),
        .regpri_ctrl      (rx_eng_regpri_ctrl),
        .regpri_ctrl_wr   (rx_eng_regpri_ctrl_wr),
        .stat_rx_frame    (rx_eng_stat_rx_frame),
        .stat_rx_crc_err  (rx_eng_stat_rx_crc_err),
        .stat_rx_disp_err (rx_eng_stat_rx_disp_err),
        .stat_rx_sym_err  (rx_eng_stat_rx_sym_err),
        .rx_frame_done    (rx_eng_rx_frame_done)
    );

    // =========================================================================
    // Statistics counters (saturating at 16'hFFFF)
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            stat_tx_frames     <= 16'd0;
            stat_tx_idle_frames <= 16'd0;
            stat_rx_frames     <= 16'd0;
            stat_rx_crc_errors <= 16'd0;
            stat_rx_disp_errors <= 16'd0;
            stat_rx_sym_errors <= 16'd0;
        end else begin
            if (tx_eng_stat_tx_frame && stat_tx_frames != 16'hFFFF)
                stat_tx_frames <= stat_tx_frames + 16'd1;

            if (tx_eng_stat_tx_idle && stat_tx_idle_frames != 16'hFFFF)
                stat_tx_idle_frames <= stat_tx_idle_frames + 16'd1;

            if (rx_eng_stat_rx_frame && stat_rx_frames != 16'hFFFF)
                stat_rx_frames <= stat_rx_frames + 16'd1;

            if (rx_eng_stat_rx_crc_err && stat_rx_crc_errors != 16'hFFFF)
                stat_rx_crc_errors <= stat_rx_crc_errors + 16'd1;

            if (rx_eng_stat_rx_disp_err && stat_rx_disp_errors != 16'hFFFF)
                stat_rx_disp_errors <= stat_rx_disp_errors + 16'd1;

            if (rx_eng_stat_rx_sym_err && stat_rx_sym_errors != 16'hFFFF)
                stat_rx_sym_errors <= stat_rx_sym_errors + 16'd1;
        end
    end

    // =========================================================================
    // IRQ source assignments
    // =========================================================================
    assign irq_tx_hipri_empty      = tx_eng_irq_hipri_empty;
    assign irq_tx_regpri_empty     = tx_eng_irq_regpri_empty;
    assign irq_tx_hipri_complete   = tx_eng_irq_hipri_complete;
    assign irq_tx_regpri_complete  = tx_eng_irq_regpri_complete;
    assign irq_rx_hipri_not_empty  = ~rx_hipri_data_empty;
    assign irq_rx_regpri_not_empty = ~rx_regpri_data_empty;

endmodule
