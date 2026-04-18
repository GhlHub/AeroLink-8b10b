`timescale 1ns / 1ps
// =============================================================================
// AeroLink-8b10b AXI-Lite Register Interface
// =============================================================================
// Decodes AXI-Lite transactions and routes to per-port control, FIFO, status,
// and interrupt registers.
// =============================================================================

module aerolink_axil_regs #(
    parameter NUM_PORTS  = 1,
    parameter ADDR_WIDTH = 16,
    parameter DATA_WIDTH = 32
) (
    input  wire                    clk,
    input  wire                    rst,

    // AXI-Lite slave interface
    input  wire [ADDR_WIDTH-1:0]   s_axi_awaddr,
    input  wire                    s_axi_awvalid,
    output reg                     s_axi_awready,
    input  wire [DATA_WIDTH-1:0]   s_axi_wdata,
    input  wire [DATA_WIDTH/8-1:0] s_axi_wstrb,
    input  wire                    s_axi_wvalid,
    output reg                     s_axi_wready,
    output reg  [1:0]              s_axi_bresp,
    output reg                     s_axi_bvalid,
    input  wire                    s_axi_bready,
    input  wire [ADDR_WIDTH-1:0]   s_axi_araddr,
    input  wire                    s_axi_arvalid,
    output reg                     s_axi_arready,
    output reg  [DATA_WIDTH-1:0]   s_axi_rdata,
    output reg  [1:0]              s_axi_rresp,
    output reg                     s_axi_rvalid,
    input  wire                    s_axi_rready,

    // Per-port control outputs
    output reg  [NUM_PORTS-1:0]    port_ctrl_tx_enable,
    output reg  [NUM_PORTS-1:0]    port_ctrl_rx_enable,
    output reg  [NUM_PORTS-1:0]    port_ctrl_tx_reset,
    output reg  [NUM_PORTS-1:0]    port_ctrl_rx_reset,
    output reg  [NUM_PORTS-1:0]    port_ctrl_drop_errored,

    // Per-port FIFO write interfaces
    output reg  [NUM_PORTS*32-1:0] port_tx_hipri_data,
    output reg  [NUM_PORTS-1:0]    port_tx_hipri_data_wr,
    output reg  [NUM_PORTS*32-1:0] port_tx_hipri_ctrl,
    output reg  [NUM_PORTS-1:0]    port_tx_hipri_ctrl_wr,
    output reg  [NUM_PORTS*32-1:0] port_tx_regpri_data,
    output reg  [NUM_PORTS-1:0]    port_tx_regpri_data_wr,
    output reg  [NUM_PORTS*32-1:0] port_tx_regpri_ctrl,
    output reg  [NUM_PORTS-1:0]    port_tx_regpri_ctrl_wr,

    // Per-port FIFO read interfaces
    input  wire [NUM_PORTS*32-1:0] port_rx_hipri_data,
    output reg  [NUM_PORTS-1:0]    port_rx_hipri_data_rd,
    input  wire [NUM_PORTS*32-1:0] port_rx_hipri_ctrl,
    output reg  [NUM_PORTS-1:0]    port_rx_hipri_ctrl_rd,
    input  wire [NUM_PORTS*32-1:0] port_rx_regpri_data,
    output reg  [NUM_PORTS-1:0]    port_rx_regpri_data_rd,
    input  wire [NUM_PORTS*32-1:0] port_rx_regpri_ctrl,
    output reg  [NUM_PORTS-1:0]    port_rx_regpri_ctrl_rd,

    // Per-port FIFO status bits
    input  wire [NUM_PORTS-1:0]    port_tx_hipri_data_full,
    input  wire [NUM_PORTS-1:0]    port_tx_hipri_data_empty,
    input  wire [NUM_PORTS-1:0]    port_tx_hipri_ctrl_full,
    input  wire [NUM_PORTS-1:0]    port_tx_hipri_ctrl_empty,
    input  wire [NUM_PORTS-1:0]    port_tx_regpri_data_full,
    input  wire [NUM_PORTS-1:0]    port_tx_regpri_data_empty,
    input  wire [NUM_PORTS-1:0]    port_tx_regpri_ctrl_full,
    input  wire [NUM_PORTS-1:0]    port_tx_regpri_ctrl_empty,
    input  wire [NUM_PORTS-1:0]    port_rx_hipri_data_full,
    input  wire [NUM_PORTS-1:0]    port_rx_hipri_data_empty,
    input  wire [NUM_PORTS-1:0]    port_rx_hipri_ctrl_full,
    input  wire [NUM_PORTS-1:0]    port_rx_hipri_ctrl_empty,
    input  wire [NUM_PORTS-1:0]    port_rx_regpri_data_full,
    input  wire [NUM_PORTS-1:0]    port_rx_regpri_data_empty,
    input  wire [NUM_PORTS-1:0]    port_rx_regpri_ctrl_full,
    input  wire [NUM_PORTS-1:0]    port_rx_regpri_ctrl_empty,

    // Per-port statistics
    input  wire [NUM_PORTS*16-1:0] port_stat_tx_frames,
    input  wire [NUM_PORTS*16-1:0] port_stat_tx_idle_frames,
    input  wire [NUM_PORTS*16-1:0] port_stat_rx_frames,
    input  wire [NUM_PORTS*16-1:0] port_stat_rx_crc_errors,
    input  wire [NUM_PORTS*16-1:0] port_stat_rx_disp_errors,
    input  wire [NUM_PORTS*16-1:0] port_stat_rx_sym_errors,

    // Per-port statistics clear-on-read (one-cycle pulse per counter)
    output reg  [NUM_PORTS*6-1:0]  port_stat_clr,

    // Per-port IRQ sources
    input  wire [NUM_PORTS-1:0]    port_irq_tx_hipri_empty,
    input  wire [NUM_PORTS-1:0]    port_irq_tx_regpri_empty,
    input  wire [NUM_PORTS-1:0]    port_irq_tx_hipri_complete,
    input  wire [NUM_PORTS-1:0]    port_irq_tx_regpri_complete,
    input  wire [NUM_PORTS-1:0]    port_irq_rx_hipri_not_empty,
    input  wire [NUM_PORTS-1:0]    port_irq_rx_regpri_not_empty,

    // Combined IRQ output
    output wire                    irq
);

    // =========================================================================
    // Local parameters for register offsets
    // =========================================================================
    localparam [7:0] REG_CTRL            = 8'h00;
    localparam [7:0] REG_FIFO_STATUS     = 8'h04;
    localparam [7:0] REG_IRQ_STATUS      = 8'h08;
    localparam [7:0] REG_IRQ_MASK        = 8'h0C;
    localparam [7:0] REG_TX_HIPRI_DATA   = 8'h10;
    localparam [7:0] REG_TX_HIPRI_CTRL   = 8'h14;
    localparam [7:0] REG_TX_REGPRI_DATA  = 8'h18;
    localparam [7:0] REG_TX_REGPRI_CTRL  = 8'h1C;
    localparam [7:0] REG_RX_HIPRI_DATA   = 8'h20;
    localparam [7:0] REG_RX_HIPRI_CTRL   = 8'h24;
    localparam [7:0] REG_RX_REGPRI_DATA  = 8'h28;
    localparam [7:0] REG_RX_REGPRI_CTRL  = 8'h2C;
    localparam [7:0] REG_STAT_TX_FRAMES  = 8'h30;
    localparam [7:0] REG_STAT_TX_IDLE    = 8'h34;
    localparam [7:0] REG_STAT_RX_FRAMES  = 8'h38;
    localparam [7:0] REG_STAT_RX_CRC_ERR = 8'h3C;
    localparam [7:0] REG_STAT_RX_DISP_ERR = 8'h40;
    localparam [7:0] REG_STAT_RX_SYM_ERR = 8'h44;

    // =========================================================================
    // Per-port IRQ mask registers
    // =========================================================================
    reg [5:0] irq_mask [0:NUM_PORTS-1];

    // =========================================================================
    // IRQ computation
    // =========================================================================
    integer irq_i;
    reg irq_combined;

    always @(*) begin
        irq_combined = 1'b0;
        for (irq_i = 0; irq_i < NUM_PORTS; irq_i = irq_i + 1) begin
            irq_combined = irq_combined |
                (port_irq_tx_hipri_empty[irq_i]      & irq_mask[irq_i][0]) |
                (port_irq_tx_regpri_empty[irq_i]     & irq_mask[irq_i][1]) |
                (port_irq_tx_hipri_complete[irq_i]    & irq_mask[irq_i][2]) |
                (port_irq_tx_regpri_complete[irq_i]   & irq_mask[irq_i][3]) |
                (port_irq_rx_hipri_not_empty[irq_i]   & irq_mask[irq_i][4]) |
                (port_irq_rx_regpri_not_empty[irq_i]  & irq_mask[irq_i][5]);
        end
    end

    assign irq = irq_combined;

    // =========================================================================
    // AXI-Lite write channel state machine
    // =========================================================================
    reg                   aw_captured;
    reg [ADDR_WIDTH-1:0]  aw_addr_reg;
    reg                   w_captured;
    reg [DATA_WIDTH-1:0]  w_data_reg;

    // =========================================================================
    // Address decode helpers
    // =========================================================================
    // Write address (use captured value once latched)
    wire [ADDR_WIDTH-1:0] wr_addr       = aw_addr_reg;
    wire [7:0]            wr_reg_offset  = wr_addr[7:0];

    // Read address
    reg  [ADDR_WIDTH-1:0] rd_addr_reg;
    wire [7:0]            rd_reg_offset  = rd_addr_reg[7:0];

    // Port number computation uses integer to avoid bit-width issues
    // Port N is at address 0x100 + N*0x100, so port_num = (addr >> 8) - 1

    // =========================================================================
    // Write channel handling
    // =========================================================================
    wire [DATA_WIDTH-1:0] wr_data = w_data_reg;

    // Write port number as integer
    integer wr_port_num;
    always @(*) begin
        wr_port_num = (wr_addr >> 8) - 1;
    end

    // Read port number as integer
    integer rd_port_num;
    always @(*) begin
        rd_port_num = (rd_addr_reg >> 8) - 1;
    end

    integer wr_i;

    always @(posedge clk) begin
        if (rst) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            aw_captured   <= 1'b0;
            w_captured    <= 1'b0;
            aw_addr_reg   <= {ADDR_WIDTH{1'b0}};
            w_data_reg    <= {DATA_WIDTH{1'b0}};

            port_ctrl_tx_enable    <= {NUM_PORTS{1'b0}};
            port_ctrl_rx_enable    <= {NUM_PORTS{1'b0}};
            port_ctrl_tx_reset     <= {NUM_PORTS{1'b0}};
            port_ctrl_rx_reset     <= {NUM_PORTS{1'b0}};
            port_ctrl_drop_errored <= {NUM_PORTS{1'b0}};

            port_tx_hipri_data_wr  <= {NUM_PORTS{1'b0}};
            port_tx_hipri_ctrl_wr  <= {NUM_PORTS{1'b0}};
            port_tx_regpri_data_wr <= {NUM_PORTS{1'b0}};
            port_tx_regpri_ctrl_wr <= {NUM_PORTS{1'b0}};

            for (wr_i = 0; wr_i < NUM_PORTS; wr_i = wr_i + 1) begin
                irq_mask[wr_i] <= 6'b000000;
            end
        end else begin
            // Default: deassert single-cycle pulses
            port_tx_hipri_data_wr  <= {NUM_PORTS{1'b0}};
            port_tx_hipri_ctrl_wr  <= {NUM_PORTS{1'b0}};
            port_tx_regpri_data_wr <= {NUM_PORTS{1'b0}};
            port_tx_regpri_ctrl_wr <= {NUM_PORTS{1'b0}};

            // Self-clearing reset bits
            port_ctrl_tx_reset <= {NUM_PORTS{1'b0}};
            port_ctrl_rx_reset <= {NUM_PORTS{1'b0}};

            // Clear ready signals by default
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;

            // Handle write response handshake
            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end

            // Capture write address if available and not already captured
            if (s_axi_awvalid && !aw_captured && !s_axi_bvalid) begin
                aw_addr_reg   <= s_axi_awaddr;
                aw_captured   <= 1'b1;
                s_axi_awready <= 1'b1;
            end

            // Capture write data if available and not already captured
            if (s_axi_wvalid && !w_captured && !s_axi_bvalid) begin
                w_data_reg   <= s_axi_wdata;
                w_captured   <= 1'b1;
                s_axi_wready <= 1'b1;
            end

            // Execute write when both address and data are available
            if (aw_captured && w_captured) begin
                aw_captured  <= 1'b0;
                w_captured   <= 1'b0;
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00; // OKAY

                if (wr_addr >= 16'h0100 && wr_port_num >= 0 && wr_port_num < NUM_PORTS) begin
                    case (wr_reg_offset)
                        REG_CTRL: begin
                            port_ctrl_tx_enable[wr_port_num]    <= wr_data[0];
                            port_ctrl_rx_enable[wr_port_num]    <= wr_data[1];
                            port_ctrl_tx_reset[wr_port_num]     <= wr_data[2];
                            port_ctrl_rx_reset[wr_port_num]     <= wr_data[3];
                            port_ctrl_drop_errored[wr_port_num] <= wr_data[4];
                        end
                        REG_IRQ_MASK: begin
                            irq_mask[wr_port_num] <= wr_data[5:0];
                        end
                        REG_TX_HIPRI_DATA: begin
                            port_tx_hipri_data[wr_port_num*32 +: 32] <= wr_data;
                            port_tx_hipri_data_wr[wr_port_num]       <= 1'b1;
                        end
                        REG_TX_HIPRI_CTRL: begin
                            port_tx_hipri_ctrl[wr_port_num*32 +: 32] <= wr_data;
                            port_tx_hipri_ctrl_wr[wr_port_num]       <= 1'b1;
                        end
                        REG_TX_REGPRI_DATA: begin
                            port_tx_regpri_data[wr_port_num*32 +: 32] <= wr_data;
                            port_tx_regpri_data_wr[wr_port_num]       <= 1'b1;
                        end
                        REG_TX_REGPRI_CTRL: begin
                            port_tx_regpri_ctrl[wr_port_num*32 +: 32] <= wr_data;
                            port_tx_regpri_ctrl_wr[wr_port_num]       <= 1'b1;
                        end
                        default: ; // Ignore writes to read-only or undefined registers
                    endcase
                end
                // Global registers are read-only; writes to global space silently ignored
            end
        end
    end

    // =========================================================================
    // Read channel handling
    // =========================================================================
    reg rd_pending;

    always @(posedge clk) begin
        if (rst) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= {DATA_WIDTH{1'b0}};
            s_axi_rresp   <= 2'b00;
            rd_addr_reg   <= {ADDR_WIDTH{1'b0}};
            rd_pending    <= 1'b0;

            port_rx_hipri_data_rd  <= {NUM_PORTS{1'b0}};
            port_rx_hipri_ctrl_rd  <= {NUM_PORTS{1'b0}};
            port_rx_regpri_data_rd <= {NUM_PORTS{1'b0}};
            port_rx_regpri_ctrl_rd <= {NUM_PORTS{1'b0}};
            port_stat_clr          <= {(NUM_PORTS*6){1'b0}};
        end else begin
            // Default: deassert single-cycle pulses
            port_rx_hipri_data_rd  <= {NUM_PORTS{1'b0}};
            port_rx_hipri_ctrl_rd  <= {NUM_PORTS{1'b0}};
            port_rx_regpri_data_rd <= {NUM_PORTS{1'b0}};
            port_rx_regpri_ctrl_rd <= {NUM_PORTS{1'b0}};
            port_stat_clr          <= {(NUM_PORTS*6){1'b0}};

            s_axi_arready <= 1'b0;

            // Handle read response handshake
            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end

            // Accept read address
            if (s_axi_arvalid && !rd_pending && !s_axi_rvalid) begin
                rd_addr_reg   <= s_axi_araddr;
                rd_pending    <= 1'b1;
                s_axi_arready <= 1'b1;
            end

            // Process read one cycle after accepting address
            if (rd_pending) begin
                rd_pending   <= 1'b0;
                s_axi_rvalid <= 1'b1;
                s_axi_rresp  <= 2'b00; // OKAY
                s_axi_rdata  <= {DATA_WIDTH{1'b0}}; // Default

                if (rd_addr_reg < 16'h0100) begin
                    // Global register reads
                    case (rd_addr_reg[7:0])
                        8'h00: s_axi_rdata <= 32'h0001_0000;        // VERSION
                        8'h04: s_axi_rdata <= {31'd0, irq_combined}; // GLOBAL_IRQ_STATUS
                        8'h08: s_axi_rdata <= NUM_PORTS;             // NUM_PORTS
                        default: s_axi_rdata <= {DATA_WIDTH{1'b0}};
                    endcase
                end else if (rd_port_num >= 0 && rd_port_num < NUM_PORTS) begin
                    // Per-port register reads
                    case (rd_reg_offset)
                        REG_CTRL: begin
                            s_axi_rdata <= {27'd0,
                                port_ctrl_drop_errored[rd_port_num],
                                1'b0, // tx_rst reads as 0 (self-clearing)
                                1'b0, // rx_rst reads as 0 (self-clearing)
                                port_ctrl_rx_enable[rd_port_num],
                                port_ctrl_tx_enable[rd_port_num]};
                        end
                        REG_FIFO_STATUS: begin
                            s_axi_rdata <= {16'd0,
                                port_rx_regpri_ctrl_full[rd_port_num],
                                port_rx_regpri_ctrl_empty[rd_port_num],
                                port_rx_regpri_data_full[rd_port_num],
                                port_rx_regpri_data_empty[rd_port_num],
                                port_rx_hipri_ctrl_full[rd_port_num],
                                port_rx_hipri_ctrl_empty[rd_port_num],
                                port_rx_hipri_data_full[rd_port_num],
                                port_rx_hipri_data_empty[rd_port_num],
                                port_tx_regpri_ctrl_full[rd_port_num],
                                port_tx_regpri_ctrl_empty[rd_port_num],
                                port_tx_regpri_data_full[rd_port_num],
                                port_tx_regpri_data_empty[rd_port_num],
                                port_tx_hipri_ctrl_full[rd_port_num],
                                port_tx_hipri_ctrl_empty[rd_port_num],
                                port_tx_hipri_data_full[rd_port_num],
                                port_tx_hipri_data_empty[rd_port_num]};
                        end
                        REG_IRQ_STATUS: begin
                            s_axi_rdata <= {26'd0,
                                port_irq_rx_regpri_not_empty[rd_port_num],
                                port_irq_rx_hipri_not_empty[rd_port_num],
                                port_irq_tx_regpri_complete[rd_port_num],
                                port_irq_tx_hipri_complete[rd_port_num],
                                port_irq_tx_regpri_empty[rd_port_num],
                                port_irq_tx_hipri_empty[rd_port_num]};
                        end
                        REG_IRQ_MASK: begin
                            s_axi_rdata <= {26'd0, irq_mask[rd_port_num]};
                        end
                        REG_RX_HIPRI_DATA: begin
                            s_axi_rdata <= port_rx_hipri_data[rd_port_num*32 +: 32];
                            port_rx_hipri_data_rd[rd_port_num] <= 1'b1;
                        end
                        REG_RX_HIPRI_CTRL: begin
                            s_axi_rdata <= port_rx_hipri_ctrl[rd_port_num*32 +: 32];
                            port_rx_hipri_ctrl_rd[rd_port_num] <= 1'b1;
                        end
                        REG_RX_REGPRI_DATA: begin
                            s_axi_rdata <= port_rx_regpri_data[rd_port_num*32 +: 32];
                            port_rx_regpri_data_rd[rd_port_num] <= 1'b1;
                        end
                        REG_RX_REGPRI_CTRL: begin
                            s_axi_rdata <= port_rx_regpri_ctrl[rd_port_num*32 +: 32];
                            port_rx_regpri_ctrl_rd[rd_port_num] <= 1'b1;
                        end
                        REG_STAT_TX_FRAMES: begin
                            s_axi_rdata <= {16'd0, port_stat_tx_frames[rd_port_num*16 +: 16]};
                            port_stat_clr[rd_port_num*6 + 0] <= 1'b1;
                        end
                        REG_STAT_TX_IDLE: begin
                            s_axi_rdata <= {16'd0, port_stat_tx_idle_frames[rd_port_num*16 +: 16]};
                            port_stat_clr[rd_port_num*6 + 1] <= 1'b1;
                        end
                        REG_STAT_RX_FRAMES: begin
                            s_axi_rdata <= {16'd0, port_stat_rx_frames[rd_port_num*16 +: 16]};
                            port_stat_clr[rd_port_num*6 + 2] <= 1'b1;
                        end
                        REG_STAT_RX_CRC_ERR: begin
                            s_axi_rdata <= {16'd0, port_stat_rx_crc_errors[rd_port_num*16 +: 16]};
                            port_stat_clr[rd_port_num*6 + 3] <= 1'b1;
                        end
                        REG_STAT_RX_DISP_ERR: begin
                            s_axi_rdata <= {16'd0, port_stat_rx_disp_errors[rd_port_num*16 +: 16]};
                            port_stat_clr[rd_port_num*6 + 4] <= 1'b1;
                        end
                        REG_STAT_RX_SYM_ERR: begin
                            s_axi_rdata <= {16'd0, port_stat_rx_sym_errors[rd_port_num*16 +: 16]};
                            port_stat_clr[rd_port_num*6 + 5] <= 1'b1;
                        end
                        default: s_axi_rdata <= {DATA_WIDTH{1'b0}};
                    endcase
                end
            end
        end
    end

endmodule
