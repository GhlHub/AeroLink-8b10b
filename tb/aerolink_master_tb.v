//-----------------------------------------------------------------------------
// aerolink_master_tb.v
// Testbench for AeroLink-8b10b IP core — Master functionality tests
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module aerolink_master_tb;

    // -----------------------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------------------
    parameter CLK_FREQ_HZ  = 125_000_000;
    parameter SYMBOL_RATE  = 2_500_000;
    parameter FIFO_DEPTH   = 128;
    parameter ADDR_WIDTH   = 16;
    parameter DATA_WIDTH   = 32;
    parameter CLK_HALF     = 4; // 125 MHz => 8ns period

    // -----------------------------------------------------------------------
    // Global signals
    // -----------------------------------------------------------------------
    reg clk;
    reg resetn;

    // Shared LVDS differential bus with bias for defined idle state
    wire aerolink_p, aerolink_n;
    pullup(aerolink_p);
    pulldown(aerolink_n);

    // Master tristate triplet → IOBUFDS (with error injection)
    wire master_o_dut, master_i, master_t;
    reg  error_inject;
    wire master_o = master_o_dut ^ error_inject;
    IOBUFDS u_iobuf_master (
        .O   (master_i),
        .IO  (aerolink_p),
        .IOB (aerolink_n),
        .I   (master_o),
        .T   (master_t)
    );

    // Slave tristate triplet → IOBUFDS
    wire slave_o, slave_i, slave_t;
    IOBUFDS u_iobuf_slave (
        .O   (slave_i),
        .IO  (aerolink_p),
        .IOB (aerolink_n),
        .I   (slave_o),
        .T   (slave_t)
    );

    wire master_irq, slave_irq;

    // -----------------------------------------------------------------------
    // Master AXI-Lite signals
    // -----------------------------------------------------------------------
    reg  [ADDR_WIDTH-1:0]   m_awaddr;
    reg  [2:0]              m_awprot;
    reg                     m_awvalid;
    wire                    m_awready;
    reg  [DATA_WIDTH-1:0]   m_wdata;
    reg  [DATA_WIDTH/8-1:0] m_wstrb;
    reg                     m_wvalid;
    wire                    m_wready;
    wire [1:0]              m_bresp;
    wire                    m_bvalid;
    reg                     m_bready;
    reg  [ADDR_WIDTH-1:0]   m_araddr;
    reg  [2:0]              m_arprot;
    reg                     m_arvalid;
    wire                    m_arready;
    wire [DATA_WIDTH-1:0]   m_rdata;
    wire [1:0]              m_rresp;
    wire                    m_rvalid;
    reg                     m_rready;

    // -----------------------------------------------------------------------
    // Slave AXI-Lite signals
    // -----------------------------------------------------------------------
    reg  [ADDR_WIDTH-1:0]   s_awaddr;
    reg  [2:0]              s_awprot;
    reg                     s_awvalid;
    wire                    s_awready;
    reg  [DATA_WIDTH-1:0]   s_wdata;
    reg  [DATA_WIDTH/8-1:0] s_wstrb;
    reg                     s_wvalid;
    wire                    s_wready;
    wire [1:0]              s_bresp;
    wire                    s_bvalid;
    reg                     s_bready;
    reg  [ADDR_WIDTH-1:0]   s_araddr;
    reg  [2:0]              s_arprot;
    reg                     s_arvalid;
    wire                    s_arready;
    wire [DATA_WIDTH-1:0]   s_rdata;
    wire [1:0]              s_rresp;
    wire                    s_rvalid;
    reg                     s_rready;

    // -----------------------------------------------------------------------
    // Register address definitions (port 0 base = 0x100)
    // -----------------------------------------------------------------------
    localparam ADDR_VERSION        = 16'h0000;
    localparam ADDR_GLOBAL_IRQ     = 16'h0004;
    localparam ADDR_NUM_PORTS      = 16'h0008;

    localparam PORT0_BASE          = 16'h0100;
    localparam ADDR_CTRL           = PORT0_BASE + 16'h00;
    localparam ADDR_FIFO_STATUS    = PORT0_BASE + 16'h04;
    localparam ADDR_IRQ_STATUS     = PORT0_BASE + 16'h08;
    localparam ADDR_IRQ_MASK       = PORT0_BASE + 16'h0C;
    localparam ADDR_TX_HIPRI_DATA  = PORT0_BASE + 16'h10;
    localparam ADDR_TX_HIPRI_CTRL  = PORT0_BASE + 16'h14;
    localparam ADDR_TX_REGPRI_DATA = PORT0_BASE + 16'h18;
    localparam ADDR_TX_REGPRI_CTRL = PORT0_BASE + 16'h1C;
    localparam ADDR_RX_HIPRI_DATA  = PORT0_BASE + 16'h20;
    localparam ADDR_RX_HIPRI_CTRL  = PORT0_BASE + 16'h24;
    localparam ADDR_RX_REGPRI_DATA = PORT0_BASE + 16'h28;
    localparam ADDR_RX_REGPRI_CTRL = PORT0_BASE + 16'h2C;
    localparam ADDR_STAT_TX_FRAMES = PORT0_BASE + 16'h30;
    localparam ADDR_STAT_TX_IDLE   = PORT0_BASE + 16'h34;
    localparam ADDR_STAT_RX_FRAMES = PORT0_BASE + 16'h38;
    localparam ADDR_STAT_RX_CRC    = PORT0_BASE + 16'h3C;
    localparam ADDR_STAT_RX_DISP   = PORT0_BASE + 16'h40;
    localparam ADDR_STAT_RX_SYM    = PORT0_BASE + 16'h44;

    // -----------------------------------------------------------------------
    // Test result tracking
    // -----------------------------------------------------------------------
    integer pass_count;
    integer fail_count;
    reg [31:0] rd_data;

    // -----------------------------------------------------------------------
    // Clock generation
    // -----------------------------------------------------------------------
    initial clk = 0;
    always #CLK_HALF clk = ~clk;

    // -----------------------------------------------------------------------
    // DUT instantiation — Master
    // -----------------------------------------------------------------------
    aerolink_axil_top #(
        .CLK_FREQ_HZ    (CLK_FREQ_HZ),
        .SYMBOL_RATE     (SYMBOL_RATE),
        .NUM_PORTS       (1),
        .PORT_IS_MASTER  (1'b1),
        .FIFO_DEPTH      (FIFO_DEPTH),
        .ADDR_WIDTH      (ADDR_WIDTH),
        .DATA_WIDTH      (DATA_WIDTH)
    ) u_master (
        .s_axi_aclk     (clk),
        .s_axi_aresetn   (resetn),
        .s_axi_awaddr    (m_awaddr),
        .s_axi_awprot    (m_awprot),
        .s_axi_awvalid   (m_awvalid),
        .s_axi_awready   (m_awready),
        .s_axi_wdata     (m_wdata),
        .s_axi_wstrb     (m_wstrb),
        .s_axi_wvalid    (m_wvalid),
        .s_axi_wready    (m_wready),
        .s_axi_bresp     (m_bresp),
        .s_axi_bvalid    (m_bvalid),
        .s_axi_bready    (m_bready),
        .s_axi_araddr    (m_araddr),
        .s_axi_arprot    (m_arprot),
        .s_axi_arvalid   (m_arvalid),
        .s_axi_arready   (m_arready),
        .s_axi_rdata     (m_rdata),
        .s_axi_rresp     (m_rresp),
        .s_axi_rvalid    (m_rvalid),
        .s_axi_rready    (m_rready),
        .aerolink_o      (master_o_dut),
        .aerolink_i      (master_i),
        .aerolink_t      (master_t),
        .irq             (master_irq)
    );

    // -----------------------------------------------------------------------
    // DUT instantiation — Slave
    // -----------------------------------------------------------------------
    aerolink_axil_top #(
        .CLK_FREQ_HZ    (CLK_FREQ_HZ),
        .SYMBOL_RATE     (SYMBOL_RATE),
        .NUM_PORTS       (1),
        .PORT_IS_MASTER  (1'b0),
        .FIFO_DEPTH      (FIFO_DEPTH),
        .ADDR_WIDTH      (ADDR_WIDTH),
        .DATA_WIDTH      (DATA_WIDTH)
    ) u_slave (
        .s_axi_aclk     (clk),
        .s_axi_aresetn   (resetn),
        .s_axi_awaddr    (s_awaddr),
        .s_axi_awprot    (s_awprot),
        .s_axi_awvalid   (s_awvalid),
        .s_axi_awready   (s_awready),
        .s_axi_wdata     (s_wdata),
        .s_axi_wstrb     (s_wstrb),
        .s_axi_wvalid    (s_wvalid),
        .s_axi_wready    (s_wready),
        .s_axi_bresp     (s_bresp),
        .s_axi_bvalid    (s_bvalid),
        .s_axi_bready    (s_bready),
        .s_axi_araddr    (s_araddr),
        .s_axi_arprot    (s_arprot),
        .s_axi_arvalid   (s_arvalid),
        .s_axi_arready   (s_arready),
        .s_axi_rdata     (s_rdata),
        .s_axi_rresp     (s_rresp),
        .s_axi_rvalid    (s_rvalid),
        .s_axi_rready    (s_rready),
        .aerolink_o      (slave_o),
        .aerolink_i      (slave_i),
        .aerolink_t      (slave_t),
        .irq             (slave_irq)
    );

    // -----------------------------------------------------------------------
    // AXI-Lite Write Task
    // inst: 0 = master DUT, 1 = slave DUT
    // -----------------------------------------------------------------------
    task axi_write;
        input [15:0] inst;
        input [15:0] addr;
        input [31:0] data;
        begin
            @(posedge clk);
            if (inst == 16'd0) begin
                m_awaddr  <= addr;
                m_awprot  <= 3'b000;
                m_awvalid <= 1'b1;
                m_wdata   <= data;
                m_wstrb   <= 4'hF;
                m_wvalid  <= 1'b1;
                m_bready  <= 1'b1;
                @(posedge clk);
                while (!(m_awready && m_wready))
                    @(posedge clk);
                m_awvalid <= 1'b0;
                m_wvalid  <= 1'b0;
                while (!m_bvalid)
                    @(posedge clk);
                m_bready <= 1'b0;
                @(posedge clk);
            end else begin
                s_awaddr  <= addr;
                s_awprot  <= 3'b000;
                s_awvalid <= 1'b1;
                s_wdata   <= data;
                s_wstrb   <= 4'hF;
                s_wvalid  <= 1'b1;
                s_bready  <= 1'b1;
                @(posedge clk);
                while (!(s_awready && s_wready))
                    @(posedge clk);
                s_awvalid <= 1'b0;
                s_wvalid  <= 1'b0;
                while (!s_bvalid)
                    @(posedge clk);
                s_bready <= 1'b0;
                @(posedge clk);
            end
        end
    endtask

    // -----------------------------------------------------------------------
    // AXI-Lite Read Task
    // inst: 0 = master DUT, 1 = slave DUT
    // -----------------------------------------------------------------------
    task axi_read;
        input  [15:0] inst;
        input  [15:0] addr;
        output [31:0] data;
        begin
            @(posedge clk);
            if (inst == 16'd0) begin
                m_araddr  <= addr;
                m_arprot  <= 3'b000;
                m_arvalid <= 1'b1;
                m_rready  <= 1'b1;
                @(posedge clk);
                while (!m_arready)
                    @(posedge clk);
                m_arvalid <= 1'b0;
                while (!m_rvalid)
                    @(posedge clk);
                data = m_rdata;
                m_rready <= 1'b0;
                @(posedge clk);
            end else begin
                s_araddr  <= addr;
                s_arprot  <= 3'b000;
                s_arvalid <= 1'b1;
                s_rready  <= 1'b1;
                @(posedge clk);
                while (!s_arready)
                    @(posedge clk);
                s_arvalid <= 1'b0;
                while (!s_rvalid)
                    @(posedge clk);
                data = s_rdata;
                s_rready <= 1'b0;
                @(posedge clk);
            end
        end
    endtask

    // -----------------------------------------------------------------------
    // Check helper
    // -----------------------------------------------------------------------
    task check;
        input [255:0] test_name;
        input [31:0]  actual;
        input [31:0]  expected;
        begin
            if (actual === expected) begin
                $display("[PASS] %0s : got 0x%08X", test_name, actual);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] %0s : expected 0x%08X, got 0x%08X",
                         test_name, expected, actual);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task check_nonzero;
        input [255:0] test_name;
        input [31:0]  actual;
        begin
            if (actual !== 32'd0) begin
                $display("[PASS] %0s : got 0x%08X (non-zero)", test_name, actual);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] %0s : expected non-zero, got 0x%08X",
                         test_name, actual);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task check_bit;
        input [255:0] test_name;
        input [31:0]  actual;
        input [4:0]   bit_pos;
        input          expected_val;
        begin
            if (actual[bit_pos] === expected_val) begin
                $display("[PASS] %0s : bit[%0d] = %0b", test_name, bit_pos, expected_val);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] %0s : bit[%0d] expected %0b, got %0b",
                         test_name, bit_pos, expected_val, actual[bit_pos]);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // -----------------------------------------------------------------------
    // Initial block — reset and signal initialization
    // -----------------------------------------------------------------------
    initial begin
        error_inject = 0;
        // Master AXI signals
        m_awaddr  = 0; m_awprot = 0; m_awvalid = 0;
        m_wdata   = 0; m_wstrb  = 0; m_wvalid  = 0;
        m_bready  = 0;
        m_araddr  = 0; m_arprot = 0; m_arvalid = 0;
        m_rready  = 0;

        // Slave AXI signals
        s_awaddr  = 0; s_awprot = 0; s_awvalid = 0;
        s_wdata   = 0; s_wstrb  = 0; s_wvalid  = 0;
        s_bready  = 0;
        s_araddr  = 0; s_arprot = 0; s_arvalid = 0;
        s_rready  = 0;

        pass_count = 0;
        fail_count = 0;
    end

    // -----------------------------------------------------------------------
    // Timeout watchdog
    // -----------------------------------------------------------------------
    initial begin
        #100_000_000;
        $display("TIMEOUT: Simulation exceeded 100ms limit");
        $finish;
    end

    // -----------------------------------------------------------------------
    // Waveform dump
    // -----------------------------------------------------------------------
    initial begin
        $dumpfile("aerolink_master_tb.vcd");
        $dumpvars(0, aerolink_master_tb);
    end

    // -----------------------------------------------------------------------
    // Error injection background processes
    // -----------------------------------------------------------------------
    event evt_inject_crc;
    event evt_inject_sym;

    initial begin
        forever begin
            @(evt_inject_crc);
            wait(master_t == 1'b1);
            @(negedge master_t);
            #2000;
            error_inject = 1;
            #40;
            error_inject = 0;
        end
    end

    initial begin
        forever begin
            @(evt_inject_sym);
            wait(master_t == 1'b1);
            @(negedge master_t);
            #2000;
            error_inject = 1;
            #400;
            error_inject = 0;
        end
    end

    // -----------------------------------------------------------------------
    // Main test sequence
    // -----------------------------------------------------------------------
    initial begin
        resetn = 1'b0;
        #200;
        @(posedge clk);
        resetn = 1'b1;
        #200;

        $display("==========================================================");
        $display(" AeroLink-8b10b Master Testbench");
        $display("==========================================================");

        // ------------------------------------------------------------------
        // Test 1: Register Access
        // ------------------------------------------------------------------
        $display("\n--- Test 1: Register Access ---");

        axi_read(16'd0, ADDR_VERSION, rd_data);
        check("Master VERSION", rd_data, 32'h0001_0000);

        axi_read(16'd0, ADDR_NUM_PORTS, rd_data);
        check("Master NUM_PORTS", rd_data, 32'h0000_0001);

        // Write CTRL with tx_en=1, rx_en=1
        axi_write(16'd0, ADDR_CTRL, 32'h0000_0003);
        axi_read(16'd0, ADDR_CTRL, rd_data);
        check("Master CTRL read-back", rd_data[1:0], 2'b11);

        $display("--- Test 1 Complete ---");

        // ------------------------------------------------------------------
        // Test 2: High-Priority TX
        // ------------------------------------------------------------------
        $display("\n--- Test 2: High-Priority TX ---");

        // Enable master TX+RX
        axi_write(16'd0, ADDR_CTRL, 32'h0000_0003);
        // Enable slave RX
        axi_write(16'd1, ADDR_CTRL, 32'h0000_0002);

        // Write 1 word of test data to master TX high-priority FIFO
        axi_write(16'd0, ADDR_TX_HIPRI_DATA, 32'hDEAD_BEEF);

        // Write TX control word:
        //   [7:0]   = 0x80 (priority, bit[7]=1 for high)
        //   [8]     = 1 (auto-CRC)
        //   [11:9]  = 0 (repeat count = send once)
        //   [18:12] = 1 (length = 1 word)
        //   [31:19] = 0 (reserved)
        axi_write(16'd0, ADDR_TX_HIPRI_CTRL, {13'd0, 7'd1, 3'd0, 1'b1, 8'h80});

        // Wait for frame transmission
        #300_000;

        // Read slave RX high-priority control word
        axi_read(16'd1, ADDR_RX_HIPRI_CTRL, rd_data);
        check_bit("Slave RX HIPRI priority bit[7]", rd_data, 5'd7, 1'b1);

        // Read slave RX high-priority data
        axi_read(16'd1, ADDR_RX_HIPRI_DATA, rd_data);
        check("Slave RX HIPRI data", rd_data, 32'hDEAD_BEEF);

        $display("--- Test 2 Complete ---");

        // ------------------------------------------------------------------
        // Test 3: Regular-Priority TX
        // ------------------------------------------------------------------
        $display("\n--- Test 3: Regular-Priority TX ---");

        // Write test data to master TX regular-priority FIFO
        axi_write(16'd0, ADDR_TX_REGPRI_DATA, 32'hCAFE_BABE);

        // Write TX control word:
        //   [7:0]   = 0x01 (priority, bit[7]=0 for regular)
        //   [8]     = 1 (auto-CRC)
        //   [11:9]  = 0 (repeat count)
        //   [18:12] = 1 (length = 1 word)
        axi_write(16'd0, ADDR_TX_REGPRI_CTRL, {13'd0, 7'd1, 3'd0, 1'b1, 8'h01});

        // Wait for frame transmission
        #300_000;

        // Read slave RX regular-priority control word
        axi_read(16'd1, ADDR_RX_REGPRI_CTRL, rd_data);
        check_bit("Slave RX REGPRI priority bit[7]", rd_data, 5'd7, 1'b0);

        // Read slave RX regular-priority data
        axi_read(16'd1, ADDR_RX_REGPRI_DATA, rd_data);
        check("Slave RX REGPRI data", rd_data, 32'hCAFE_BABE);

        $display("--- Test 3 Complete ---");

        // ------------------------------------------------------------------
        // Test 4: Priority Ordering
        // ------------------------------------------------------------------
        $display("\n--- Test 4: Priority Ordering ---");

        // Write a regular-priority message first
        axi_write(16'd0, ADDR_TX_REGPRI_DATA, 32'h1111_1111);
        axi_write(16'd0, ADDR_TX_REGPRI_CTRL, {13'd0, 7'd1, 3'd0, 1'b1, 8'h01});

        // Write a high-priority message second
        axi_write(16'd0, ADDR_TX_HIPRI_DATA, 32'h2222_2222);
        axi_write(16'd0, ADDR_TX_HIPRI_CTRL, {13'd0, 7'd1, 3'd0, 1'b1, 8'h80});

        // Wait for TWO frame periods
        #500_000;

        // Verify high-priority data arrived at slave
        axi_read(16'd1, ADDR_RX_HIPRI_DATA, rd_data);
        check("Priority ordering HIPRI data", rd_data, 32'h2222_2222);

        // Verify regular-priority data also arrived
        axi_read(16'd1, ADDR_RX_REGPRI_DATA, rd_data);
        check("Priority ordering REGPRI data", rd_data, 32'h1111_1111);

        $display("--- Test 4 Complete ---");

        // ------------------------------------------------------------------
        // Test 5: Multi-Word Message
        // ------------------------------------------------------------------
        $display("\n--- Test 5: Multi-Word Message ---");

        // Write 4 words to TX high-priority data FIFO
        axi_write(16'd0, ADDR_TX_HIPRI_DATA, 32'hAAAA_AAAA);
        axi_write(16'd0, ADDR_TX_HIPRI_DATA, 32'hBBBB_BBBB);
        axi_write(16'd0, ADDR_TX_HIPRI_DATA, 32'hCCCC_CCCC);
        axi_write(16'd0, ADDR_TX_HIPRI_DATA, 32'hDDDD_DDDD);

        // Write control word: priority=0x80, auto_crc=1, length=4
        axi_write(16'd0, ADDR_TX_HIPRI_CTRL, {13'd0, 7'd4, 3'd0, 1'b1, 8'h80});

        // Wait for transmission
        #400_000;

        // Read slave RX high-priority control to pop the frame
        axi_read(16'd1, ADDR_RX_HIPRI_CTRL, rd_data);

        // Read 4 words from slave RX high-priority data
        axi_read(16'd1, ADDR_RX_HIPRI_DATA, rd_data);
        check("Multi-word data[0]", rd_data, 32'hAAAA_AAAA);

        axi_read(16'd1, ADDR_RX_HIPRI_DATA, rd_data);
        check("Multi-word data[1]", rd_data, 32'hBBBB_BBBB);

        axi_read(16'd1, ADDR_RX_HIPRI_DATA, rd_data);
        check("Multi-word data[2]", rd_data, 32'hCCCC_CCCC);

        axi_read(16'd1, ADDR_RX_HIPRI_DATA, rd_data);
        check("Multi-word data[3]", rd_data, 32'hDDDD_DDDD);

        $display("--- Test 5 Complete ---");

        // ------------------------------------------------------------------
        // Test 6: Idle Frame Counter
        // ------------------------------------------------------------------
        $display("\n--- Test 6: Idle Frame Counter ---");

        // Don't queue any data, just wait
        #400_000;

        // Read master TX idle counter
        axi_read(16'd0, ADDR_STAT_TX_IDLE, rd_data);
        check_nonzero("Master STAT_TX_IDLE", rd_data);

        $display("--- Test 6 Complete ---");

        // ------------------------------------------------------------------
        // Test 7: CRC Error Injection
        // ------------------------------------------------------------------
        $display("\n--- Test 7: CRC Error Injection ---");

        // Ensure slave drop_errored is OFF so errored frames are delivered
        axi_write(16'd1, ADDR_CTRL, 32'h0000_0002);

        // Clear slave stats by resetting RX path
        axi_write(16'd1, ADDR_CTRL, 32'h0000_000A); // rx_en=1, rx_rst=1
        #100;
        axi_write(16'd1, ADDR_CTRL, 32'h0000_0002); // rx_en=1

        // Queue a frame
        axi_write(16'd0, ADDR_TX_HIPRI_DATA, 32'hBAAD_F00D);
        axi_write(16'd0, ADDR_TX_HIPRI_CTRL, {13'd0, 7'd1, 3'd0, 1'b1, 8'h80});

        // Inject bit flip during data portion of the frame
        -> evt_inject_crc;
        #400_000;

        // Check slave stats
        axi_read(16'd1, ADDR_STAT_RX_CRC, rd_data);
        check_nonzero("Slave STAT_RX_CRC_ERR after inject", rd_data);

        // Frame should still arrive (drop_errored=0) — check ctrl has CRC error flag
        axi_read(16'd1, ADDR_RX_HIPRI_CTRL, rd_data);
        check_bit("CRC error flag in RX ctrl", rd_data, 5'd13, 1'b1);

        $display("--- Test 7 Complete ---");

        // ------------------------------------------------------------------
        // Test 8: Symbol Error Injection
        // ------------------------------------------------------------------
        $display("\n--- Test 8: Symbol Error Injection ---");

        // Reset slave RX path to clear stats
        axi_write(16'd1, ADDR_CTRL, 32'h0000_000A);
        #100;
        axi_write(16'd1, ADDR_CTRL, 32'h0000_0002);

        // Queue a frame
        axi_write(16'd0, ADDR_TX_HIPRI_DATA, 32'h5678_1234);
        axi_write(16'd0, ADDR_TX_HIPRI_CTRL, {13'd0, 7'd1, 3'd0, 1'b1, 8'h80});

        // Hold error_inject for a full symbol to corrupt 10-bit code completely
        -> evt_inject_sym;
        #400_000;

        // Check combined: sym or disp error count > 0
        begin : sym_err_check
            reg [31:0] sym_cnt, disp_cnt;
            axi_read(16'd1, ADDR_STAT_RX_SYM, sym_cnt);
            axi_read(16'd1, ADDR_STAT_RX_DISP, disp_cnt);
            if (sym_cnt !== 32'd0 || disp_cnt !== 32'd0) begin
                $display("[PASS] Symbol/disparity error detected : sym=%0d disp=%0d", sym_cnt, disp_cnt);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] Symbol/disparity error expected but none detected");
                fail_count = fail_count + 1;
            end
        end

        $display("--- Test 8 Complete ---");

        // ------------------------------------------------------------------
        // Test 9: Drop Errored Frames
        // ------------------------------------------------------------------
        $display("\n--- Test 9: Drop Errored Frames ---");

        // Enable drop_errored on slave
        axi_write(16'd1, ADDR_CTRL, 32'h0000_0012); // rx_en=1, drop_errored=1

        // Reset slave RX FIFOs
        axi_write(16'd1, ADDR_CTRL, 32'h0000_001A); // rx_en=1, rx_rst=1, drop_errored=1
        #100;
        axi_write(16'd1, ADDR_CTRL, 32'h0000_0012); // clear rst

        // Queue a frame that will be corrupted
        axi_write(16'd0, ADDR_TX_HIPRI_DATA, 32'hDEAD_CAFE);
        axi_write(16'd0, ADDR_TX_HIPRI_CTRL, {13'd0, 7'd1, 3'd0, 1'b1, 8'h80});

        // Inject error
        -> evt_inject_crc;
        #400_000;

        // Slave RX FIFO should be EMPTY (frame was dropped)
        axi_read(16'd1, ADDR_FIFO_STATUS, rd_data);
        check_bit("RX HIPRI data FIFO empty (frame dropped)", rd_data, 5'd8, 1'b1);

        // Now send a CLEAN frame to verify good frames still pass
        axi_write(16'd0, ADDR_TX_HIPRI_DATA, 32'hC0FFEE00);
        axi_write(16'd0, ADDR_TX_HIPRI_CTRL, {13'd0, 7'd1, 3'd0, 1'b1, 8'h80});
        #400_000;

        axi_read(16'd1, ADDR_RX_HIPRI_DATA, rd_data);
        check("Clean frame after drop", rd_data, 32'hC0FFEE00);

        $display("--- Test 9 Complete ---");

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
        $display("\n==========================================================");
        $display(" Master TB Summary: %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("==========================================================");

        #1000;
        $finish;
    end

endmodule
