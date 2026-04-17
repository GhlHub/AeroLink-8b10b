//-----------------------------------------------------------------------------
// aerolink_slave_tb.v
// Testbench for AeroLink-8b10b IP core — Slave functionality tests
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module aerolink_slave_tb;

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

    // Serial link between master and slave
    wire master_tx, slave_tx;
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
        .aerolink_tx     (master_tx),
        .aerolink_rx     (slave_tx),
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
        .aerolink_tx     (slave_tx),
        .aerolink_rx     (master_tx),
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
    // Check helpers
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

    task check_no_errors;
        input [255:0] test_name;
        input [31:0]  ctrl_word;
        begin
            // Error flags are bits [15:13]: [13]=CRC err, [14]=disp err, [15]=sym err
            if (ctrl_word[15:13] === 3'b000) begin
                $display("[PASS] %0s : no errors (err_flags=0)", test_name);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] %0s : error flags = 3'b%03b (CRC=%b, disp=%b, sym=%b)",
                         test_name, ctrl_word[15:13],
                         ctrl_word[13], ctrl_word[14], ctrl_word[15]);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // -----------------------------------------------------------------------
    // Initial block — reset and signal initialization
    // -----------------------------------------------------------------------
    initial begin
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
        #50_000_000;
        $display("TIMEOUT: Simulation exceeded 50ms limit");
        $finish;
    end

    // -----------------------------------------------------------------------
    // Waveform dump
    // -----------------------------------------------------------------------
    initial begin
        $dumpfile("aerolink_slave_tb.vcd");
        $dumpvars(0, aerolink_slave_tb);
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
        $display(" AeroLink-8b10b Slave Testbench");
        $display("==========================================================");

        // Enable both master and slave TX+RX for all tests
        axi_write(16'd0, ADDR_CTRL, 32'h0000_0003); // master: tx_en + rx_en
        axi_write(16'd1, ADDR_CTRL, 32'h0000_0003); // slave:  tx_en + rx_en

        // Allow link to establish
        #100_000;

        // ------------------------------------------------------------------
        // Test 1: Slave RX from Master
        // ------------------------------------------------------------------
        $display("\n--- Test 1: Slave RX from Master ---");

        // Master sends high-priority frame
        axi_write(16'd0, ADDR_TX_HIPRI_DATA, 32'h1234_5678);
        axi_write(16'd0, ADDR_TX_HIPRI_CTRL, {13'd0, 7'd1, 3'd0, 1'b1, 8'h80});

        // Wait for reception
        #300_000;

        // Read slave RX high-priority control word
        axi_read(16'd1, ADDR_RX_HIPRI_CTRL, rd_data);
        check("Slave RX HIPRI priority", rd_data[7:0], 8'h80);
        check_no_errors("Slave RX HIPRI errors", rd_data);

        // Read slave RX high-priority data
        axi_read(16'd1, ADDR_RX_HIPRI_DATA, rd_data);
        check("Slave RX HIPRI data", rd_data, 32'h1234_5678);

        $display("--- Test 1 Complete ---");

        // ------------------------------------------------------------------
        // Test 2: Slave TX Response
        // ------------------------------------------------------------------
        $display("\n--- Test 2: Slave TX Response ---");

        // Queue data in slave's TX high-priority FIFO
        axi_write(16'd1, ADDR_TX_HIPRI_DATA, 32'hFEED_FACE);
        axi_write(16'd1, ADDR_TX_HIPRI_CTRL, {13'd0, 7'd1, 3'd0, 1'b1, 8'h80});

        // The slave transmits during its turnaround slot
        // Wait for response to arrive at master
        #400_000;

        // Read master RX high-priority data
        axi_read(16'd0, ADDR_RX_HIPRI_CTRL, rd_data);
        axi_read(16'd0, ADDR_RX_HIPRI_DATA, rd_data);
        check("Master RX from slave", rd_data, 32'hFEED_FACE);

        $display("--- Test 2 Complete ---");

        // ------------------------------------------------------------------
        // Test 3: Slave RX Priority Routing
        // ------------------------------------------------------------------
        $display("\n--- Test 3: Slave RX Priority Routing ---");

        // Master sends high-priority frame
        axi_write(16'd0, ADDR_TX_HIPRI_DATA, 32'hAAAA_AAAA);
        axi_write(16'd0, ADDR_TX_HIPRI_CTRL, {13'd0, 7'd1, 3'd0, 1'b1, 8'h80});

        // Wait for first frame to transmit
        #300_000;

        // Master sends regular-priority frame
        axi_write(16'd0, ADDR_TX_REGPRI_DATA, 32'hBBBB_BBBB);
        axi_write(16'd0, ADDR_TX_REGPRI_CTRL, {13'd0, 7'd1, 3'd0, 1'b1, 8'h01});

        // Wait for both to be received
        #300_000;

        // Verify slave RX high-priority has 0xAAAAAAAA
        axi_read(16'd1, ADDR_RX_HIPRI_CTRL, rd_data);
        axi_read(16'd1, ADDR_RX_HIPRI_DATA, rd_data);
        check("Slave RX HIPRI routing", rd_data, 32'hAAAA_AAAA);

        // Verify slave RX regular-priority has 0xBBBBBBBB
        axi_read(16'd1, ADDR_RX_REGPRI_CTRL, rd_data);
        axi_read(16'd1, ADDR_RX_REGPRI_DATA, rd_data);
        check("Slave RX REGPRI routing", rd_data, 32'hBBBB_BBBB);

        $display("--- Test 3 Complete ---");

        // ------------------------------------------------------------------
        // Test 4: Slave Multi-Word RX
        // ------------------------------------------------------------------
        $display("\n--- Test 4: Slave Multi-Word RX ---");

        // Master sends 4-word message
        axi_write(16'd0, ADDR_TX_HIPRI_DATA, 32'h1111_1111);
        axi_write(16'd0, ADDR_TX_HIPRI_DATA, 32'h2222_2222);
        axi_write(16'd0, ADDR_TX_HIPRI_DATA, 32'h3333_3333);
        axi_write(16'd0, ADDR_TX_HIPRI_DATA, 32'h4444_4444);
        axi_write(16'd0, ADDR_TX_HIPRI_CTRL, {13'd0, 7'd4, 3'd0, 1'b1, 8'h80});

        // Wait for transmission
        #400_000;

        // Read slave RX control to pop the frame
        axi_read(16'd1, ADDR_RX_HIPRI_CTRL, rd_data);

        // Read all 4 words
        axi_read(16'd1, ADDR_RX_HIPRI_DATA, rd_data);
        check("Slave multi-word data[0]", rd_data, 32'h1111_1111);

        axi_read(16'd1, ADDR_RX_HIPRI_DATA, rd_data);
        check("Slave multi-word data[1]", rd_data, 32'h2222_2222);

        axi_read(16'd1, ADDR_RX_HIPRI_DATA, rd_data);
        check("Slave multi-word data[2]", rd_data, 32'h3333_3333);

        axi_read(16'd1, ADDR_RX_HIPRI_DATA, rd_data);
        check("Slave multi-word data[3]", rd_data, 32'h4444_4444);

        $display("--- Test 4 Complete ---");

        // ------------------------------------------------------------------
        // Test 5: Drop Errored Frames
        // ------------------------------------------------------------------
        $display("\n--- Test 5: Drop Errored Frames (Configuration) ---");

        // Enable drop_errored on slave: bit[4]=1, keep tx_en and rx_en
        axi_write(16'd1, ADDR_CTRL, 32'h0000_0013); // bits [4,1,0] = drop_errored + rx_en + tx_en

        // Read back and verify
        axi_read(16'd1, ADDR_CTRL, rd_data);
        check_bit("Slave CTRL drop_errored", rd_data, 5'd4, 1'b1);
        check_bit("Slave CTRL tx_en", rd_data, 5'd0, 1'b1);
        check_bit("Slave CTRL rx_en", rd_data, 5'd1, 1'b1);

        $display("--- Test 5 Complete ---");

        // ------------------------------------------------------------------
        // Test 6: Statistics
        // ------------------------------------------------------------------
        $display("\n--- Test 6: Statistics ---");

        // After the above tests, slave should have received frames
        axi_read(16'd1, ADDR_STAT_RX_FRAMES, rd_data);
        check_nonzero("Slave STAT_RX_FRAMES", rd_data);

        $display("--- Test 6 Complete ---");

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
        $display("\n==========================================================");
        $display(" Slave TB Summary: %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("==========================================================");

        #1000;
        $finish;
    end

endmodule
