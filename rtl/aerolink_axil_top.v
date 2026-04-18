`timescale 1ns / 1ps
// =============================================================================
// AeroLink-8b10b AXI-Lite Top-Level
// =============================================================================
// Instantiates the register interface and N AeroLink ports with generate.
// =============================================================================

module aerolink_axil_top #(
    parameter CLK_FREQ_HZ  = 125_000_000,
    parameter SYMBOL_RATE  = 2_500_000,
    parameter NUM_PORTS    = 1,
    parameter [NUM_PORTS-1:0] PORT_IS_MASTER = {NUM_PORTS{1'b1}},
    parameter FIFO_DEPTH   = 128,
    parameter ADDR_WIDTH   = 16,
    parameter DATA_WIDTH   = 32
) (
    // AXI-Lite slave interface
    input  wire                    s_axi_aclk,
    input  wire                    s_axi_aresetn,
    input  wire [ADDR_WIDTH-1:0]   s_axi_awaddr,
    input  wire [2:0]              s_axi_awprot,
    input  wire                    s_axi_awvalid,
    output wire                    s_axi_awready,
    input  wire [DATA_WIDTH-1:0]   s_axi_wdata,
    input  wire [DATA_WIDTH/8-1:0] s_axi_wstrb,
    input  wire                    s_axi_wvalid,
    output wire                    s_axi_wready,
    output wire [1:0]              s_axi_bresp,
    output wire                    s_axi_bvalid,
    input  wire                    s_axi_bready,
    input  wire [ADDR_WIDTH-1:0]   s_axi_araddr,
    input  wire [2:0]              s_axi_arprot,
    input  wire                    s_axi_arvalid,
    output wire                    s_axi_arready,
    output wire [DATA_WIDTH-1:0]   s_axi_rdata,
    output wire [1:0]              s_axi_rresp,
    output wire                    s_axi_rvalid,
    input  wire                    s_axi_rready,

    // AeroLink serial I/O (active-high tristate: _t=1 means high-Z)
    output wire [NUM_PORTS-1:0]    aerolink_o,
    input  wire [NUM_PORTS-1:0]    aerolink_i,
    output wire [NUM_PORTS-1:0]    aerolink_t,

    // Combined interrupt
    output wire                    irq
);

    // =========================================================================
    // Clock and reset conversion
    // =========================================================================
    wire clk = s_axi_aclk;
    wire rst = ~s_axi_aresetn;

    // =========================================================================
    // Per-port signal buses (flat vectors for register interface)
    // =========================================================================

    // Control
    wire [NUM_PORTS-1:0] port_ctrl_tx_enable;
    wire [NUM_PORTS-1:0] port_ctrl_rx_enable;
    wire [NUM_PORTS-1:0] port_ctrl_tx_reset;
    wire [NUM_PORTS-1:0] port_ctrl_rx_reset;
    wire [NUM_PORTS-1:0] port_ctrl_drop_errored;

    // TX FIFO write interfaces
    wire [NUM_PORTS*32-1:0] port_tx_hipri_data;
    wire [NUM_PORTS-1:0]    port_tx_hipri_data_wr;
    wire [NUM_PORTS*32-1:0] port_tx_hipri_ctrl;
    wire [NUM_PORTS-1:0]    port_tx_hipri_ctrl_wr;
    wire [NUM_PORTS*32-1:0] port_tx_regpri_data;
    wire [NUM_PORTS-1:0]    port_tx_regpri_data_wr;
    wire [NUM_PORTS*32-1:0] port_tx_regpri_ctrl;
    wire [NUM_PORTS-1:0]    port_tx_regpri_ctrl_wr;

    // RX FIFO read interfaces
    wire [NUM_PORTS*32-1:0] port_rx_hipri_data;
    wire [NUM_PORTS-1:0]    port_rx_hipri_data_rd;
    wire [NUM_PORTS*32-1:0] port_rx_hipri_ctrl;
    wire [NUM_PORTS-1:0]    port_rx_hipri_ctrl_rd;
    wire [NUM_PORTS*32-1:0] port_rx_regpri_data;
    wire [NUM_PORTS-1:0]    port_rx_regpri_data_rd;
    wire [NUM_PORTS*32-1:0] port_rx_regpri_ctrl;
    wire [NUM_PORTS-1:0]    port_rx_regpri_ctrl_rd;

    // FIFO status bits
    wire [NUM_PORTS-1:0] port_tx_hipri_data_full;
    wire [NUM_PORTS-1:0] port_tx_hipri_data_empty;
    wire [NUM_PORTS-1:0] port_tx_hipri_ctrl_full;
    wire [NUM_PORTS-1:0] port_tx_hipri_ctrl_empty;
    wire [NUM_PORTS-1:0] port_tx_regpri_data_full;
    wire [NUM_PORTS-1:0] port_tx_regpri_data_empty;
    wire [NUM_PORTS-1:0] port_tx_regpri_ctrl_full;
    wire [NUM_PORTS-1:0] port_tx_regpri_ctrl_empty;
    wire [NUM_PORTS-1:0] port_rx_hipri_data_full;
    wire [NUM_PORTS-1:0] port_rx_hipri_data_empty;
    wire [NUM_PORTS-1:0] port_rx_hipri_ctrl_full;
    wire [NUM_PORTS-1:0] port_rx_hipri_ctrl_empty;
    wire [NUM_PORTS-1:0] port_rx_regpri_data_full;
    wire [NUM_PORTS-1:0] port_rx_regpri_data_empty;
    wire [NUM_PORTS-1:0] port_rx_regpri_ctrl_full;
    wire [NUM_PORTS-1:0] port_rx_regpri_ctrl_empty;

    // Statistics
    wire [NUM_PORTS*16-1:0] port_stat_tx_frames;
    wire [NUM_PORTS*16-1:0] port_stat_tx_idle_frames;
    wire [NUM_PORTS*16-1:0] port_stat_rx_frames;
    wire [NUM_PORTS*16-1:0] port_stat_rx_crc_errors;
    wire [NUM_PORTS*16-1:0] port_stat_rx_disp_errors;
    wire [NUM_PORTS*16-1:0] port_stat_rx_sym_errors;
    wire [NUM_PORTS*6-1:0]  port_stat_clr;

    // IRQ sources
    wire [NUM_PORTS-1:0] port_irq_tx_hipri_empty;
    wire [NUM_PORTS-1:0] port_irq_tx_regpri_empty;
    wire [NUM_PORTS-1:0] port_irq_tx_hipri_complete;
    wire [NUM_PORTS-1:0] port_irq_tx_regpri_complete;
    wire [NUM_PORTS-1:0] port_irq_rx_hipri_not_empty;
    wire [NUM_PORTS-1:0] port_irq_rx_regpri_not_empty;

    // =========================================================================
    // Register Interface
    // =========================================================================
    aerolink_axil_regs #(
        .NUM_PORTS  (NUM_PORTS),
        .ADDR_WIDTH (ADDR_WIDTH),
        .DATA_WIDTH (DATA_WIDTH)
    ) u_regs (
        .clk                        (clk),
        .rst                        (rst),

        // AXI-Lite
        .s_axi_awaddr               (s_axi_awaddr),
        .s_axi_awvalid              (s_axi_awvalid),
        .s_axi_awready              (s_axi_awready),
        .s_axi_wdata                (s_axi_wdata),
        .s_axi_wstrb                (s_axi_wstrb),
        .s_axi_wvalid               (s_axi_wvalid),
        .s_axi_wready               (s_axi_wready),
        .s_axi_bresp                (s_axi_bresp),
        .s_axi_bvalid               (s_axi_bvalid),
        .s_axi_bready               (s_axi_bready),
        .s_axi_araddr               (s_axi_araddr),
        .s_axi_arvalid              (s_axi_arvalid),
        .s_axi_arready              (s_axi_arready),
        .s_axi_rdata                (s_axi_rdata),
        .s_axi_rresp                (s_axi_rresp),
        .s_axi_rvalid               (s_axi_rvalid),
        .s_axi_rready               (s_axi_rready),

        // Control
        .port_ctrl_tx_enable        (port_ctrl_tx_enable),
        .port_ctrl_rx_enable        (port_ctrl_rx_enable),
        .port_ctrl_tx_reset         (port_ctrl_tx_reset),
        .port_ctrl_rx_reset         (port_ctrl_rx_reset),
        .port_ctrl_drop_errored     (port_ctrl_drop_errored),

        // TX FIFO write
        .port_tx_hipri_data         (port_tx_hipri_data),
        .port_tx_hipri_data_wr      (port_tx_hipri_data_wr),
        .port_tx_hipri_ctrl         (port_tx_hipri_ctrl),
        .port_tx_hipri_ctrl_wr      (port_tx_hipri_ctrl_wr),
        .port_tx_regpri_data        (port_tx_regpri_data),
        .port_tx_regpri_data_wr     (port_tx_regpri_data_wr),
        .port_tx_regpri_ctrl        (port_tx_regpri_ctrl),
        .port_tx_regpri_ctrl_wr     (port_tx_regpri_ctrl_wr),

        // RX FIFO read
        .port_rx_hipri_data         (port_rx_hipri_data),
        .port_rx_hipri_data_rd      (port_rx_hipri_data_rd),
        .port_rx_hipri_ctrl         (port_rx_hipri_ctrl),
        .port_rx_hipri_ctrl_rd      (port_rx_hipri_ctrl_rd),
        .port_rx_regpri_data        (port_rx_regpri_data),
        .port_rx_regpri_data_rd     (port_rx_regpri_data_rd),
        .port_rx_regpri_ctrl        (port_rx_regpri_ctrl),
        .port_rx_regpri_ctrl_rd     (port_rx_regpri_ctrl_rd),

        // FIFO status
        .port_tx_hipri_data_full    (port_tx_hipri_data_full),
        .port_tx_hipri_data_empty   (port_tx_hipri_data_empty),
        .port_tx_hipri_ctrl_full    (port_tx_hipri_ctrl_full),
        .port_tx_hipri_ctrl_empty   (port_tx_hipri_ctrl_empty),
        .port_tx_regpri_data_full   (port_tx_regpri_data_full),
        .port_tx_regpri_data_empty  (port_tx_regpri_data_empty),
        .port_tx_regpri_ctrl_full   (port_tx_regpri_ctrl_full),
        .port_tx_regpri_ctrl_empty  (port_tx_regpri_ctrl_empty),
        .port_rx_hipri_data_full    (port_rx_hipri_data_full),
        .port_rx_hipri_data_empty   (port_rx_hipri_data_empty),
        .port_rx_hipri_ctrl_full    (port_rx_hipri_ctrl_full),
        .port_rx_hipri_ctrl_empty   (port_rx_hipri_ctrl_empty),
        .port_rx_regpri_data_full   (port_rx_regpri_data_full),
        .port_rx_regpri_data_empty  (port_rx_regpri_data_empty),
        .port_rx_regpri_ctrl_full   (port_rx_regpri_ctrl_full),
        .port_rx_regpri_ctrl_empty  (port_rx_regpri_ctrl_empty),

        // Statistics
        .port_stat_tx_frames        (port_stat_tx_frames),
        .port_stat_tx_idle_frames   (port_stat_tx_idle_frames),
        .port_stat_rx_frames        (port_stat_rx_frames),
        .port_stat_rx_crc_errors    (port_stat_rx_crc_errors),
        .port_stat_rx_disp_errors   (port_stat_rx_disp_errors),
        .port_stat_rx_sym_errors    (port_stat_rx_sym_errors),
        .port_stat_clr              (port_stat_clr),

        // IRQ sources
        .port_irq_tx_hipri_empty    (port_irq_tx_hipri_empty),
        .port_irq_tx_regpri_empty   (port_irq_tx_regpri_empty),
        .port_irq_tx_hipri_complete (port_irq_tx_hipri_complete),
        .port_irq_tx_regpri_complete(port_irq_tx_regpri_complete),
        .port_irq_rx_hipri_not_empty(port_irq_rx_hipri_not_empty),
        .port_irq_rx_regpri_not_empty(port_irq_rx_regpri_not_empty),

        // Combined IRQ
        .irq                        (irq)
    );

    // =========================================================================
    // Port instances
    // =========================================================================
    genvar gi;
    generate
        for (gi = 0; gi < NUM_PORTS; gi = gi + 1) begin : gen_port
            aerolink_port #(
                .CLK_FREQ_HZ (CLK_FREQ_HZ),
                .SYMBOL_RATE (SYMBOL_RATE),
                .IS_MASTER   (PORT_IS_MASTER[gi]),
                .FIFO_DEPTH  (FIFO_DEPTH)
            ) u_port (
                .clk                    (clk),
                .rst                    (rst),

                // Control
                .ctrl_tx_enable         (port_ctrl_tx_enable[gi]),
                .ctrl_rx_enable         (port_ctrl_rx_enable[gi]),
                .ctrl_tx_reset          (port_ctrl_tx_reset[gi]),
                .ctrl_rx_reset          (port_ctrl_rx_reset[gi]),
                .ctrl_drop_errored      (port_ctrl_drop_errored[gi]),

                // TX high-priority data FIFO
                .tx_hipri_data_in       (port_tx_hipri_data[gi*32 +: 32]),
                .tx_hipri_data_wr       (port_tx_hipri_data_wr[gi]),
                .tx_hipri_data_full     (port_tx_hipri_data_full[gi]),
                .tx_hipri_data_empty    (port_tx_hipri_data_empty[gi]),

                // TX high-priority control FIFO
                .tx_hipri_ctrl_in       (port_tx_hipri_ctrl[gi*32 +: 32]),
                .tx_hipri_ctrl_wr       (port_tx_hipri_ctrl_wr[gi]),
                .tx_hipri_ctrl_full     (port_tx_hipri_ctrl_full[gi]),
                .tx_hipri_ctrl_empty    (port_tx_hipri_ctrl_empty[gi]),

                // TX regular-priority data FIFO
                .tx_regpri_data_in      (port_tx_regpri_data[gi*32 +: 32]),
                .tx_regpri_data_wr      (port_tx_regpri_data_wr[gi]),
                .tx_regpri_data_full    (port_tx_regpri_data_full[gi]),
                .tx_regpri_data_empty   (port_tx_regpri_data_empty[gi]),

                // TX regular-priority control FIFO
                .tx_regpri_ctrl_in      (port_tx_regpri_ctrl[gi*32 +: 32]),
                .tx_regpri_ctrl_wr      (port_tx_regpri_ctrl_wr[gi]),
                .tx_regpri_ctrl_full    (port_tx_regpri_ctrl_full[gi]),
                .tx_regpri_ctrl_empty   (port_tx_regpri_ctrl_empty[gi]),

                // RX high-priority data FIFO
                .rx_hipri_data_out      (port_rx_hipri_data[gi*32 +: 32]),
                .rx_hipri_data_rd       (port_rx_hipri_data_rd[gi]),
                .rx_hipri_data_empty    (port_rx_hipri_data_empty[gi]),
                .rx_hipri_data_full     (port_rx_hipri_data_full[gi]),

                // RX high-priority control FIFO
                .rx_hipri_ctrl_out      (port_rx_hipri_ctrl[gi*32 +: 32]),
                .rx_hipri_ctrl_rd       (port_rx_hipri_ctrl_rd[gi]),
                .rx_hipri_ctrl_empty    (port_rx_hipri_ctrl_empty[gi]),
                .rx_hipri_ctrl_full     (port_rx_hipri_ctrl_full[gi]),

                // RX regular-priority data FIFO
                .rx_regpri_data_out     (port_rx_regpri_data[gi*32 +: 32]),
                .rx_regpri_data_rd      (port_rx_regpri_data_rd[gi]),
                .rx_regpri_data_empty   (port_rx_regpri_data_empty[gi]),
                .rx_regpri_data_full    (port_rx_regpri_data_full[gi]),

                // RX regular-priority control FIFO
                .rx_regpri_ctrl_out     (port_rx_regpri_ctrl[gi*32 +: 32]),
                .rx_regpri_ctrl_rd      (port_rx_regpri_ctrl_rd[gi]),
                .rx_regpri_ctrl_empty   (port_rx_regpri_ctrl_empty[gi]),
                .rx_regpri_ctrl_full    (port_rx_regpri_ctrl_full[gi]),

                // Statistics
                .stat_tx_frames         (port_stat_tx_frames[gi*16 +: 16]),
                .stat_tx_idle_frames    (port_stat_tx_idle_frames[gi*16 +: 16]),
                .stat_rx_frames         (port_stat_rx_frames[gi*16 +: 16]),
                .stat_rx_crc_errors     (port_stat_rx_crc_errors[gi*16 +: 16]),
                .stat_rx_disp_errors    (port_stat_rx_disp_errors[gi*16 +: 16]),
                .stat_rx_sym_errors     (port_stat_rx_sym_errors[gi*16 +: 16]),
                .stat_clr               (port_stat_clr[gi*6 +: 6]),

                // IRQ sources
                .irq_tx_hipri_empty     (port_irq_tx_hipri_empty[gi]),
                .irq_tx_regpri_empty    (port_irq_tx_regpri_empty[gi]),
                .irq_tx_hipri_complete  (port_irq_tx_hipri_complete[gi]),
                .irq_tx_regpri_complete (port_irq_tx_regpri_complete[gi]),
                .irq_rx_hipri_not_empty (port_irq_rx_hipri_not_empty[gi]),
                .irq_rx_regpri_not_empty(port_irq_rx_regpri_not_empty[gi]),

                // Serial I/O
                .aerolink_o             (aerolink_o[gi]),
                .aerolink_i             (aerolink_i[gi]),
                .aerolink_t             (aerolink_t[gi])
            );
        end
    endgenerate

endmodule
