`timescale 1ns / 1ps
// ============================================================================
// AeroLink-8b10b :: RX Engine
// Deserializes rx_serial, performs comma-detect / symbol alignment, decodes
// 8B10B symbols, parses frames, checks CRC, and writes results to FIFOs.
// ============================================================================

module aerolink_rx_engine #(
    parameter CLK_FREQ_HZ = 125_000_000,
    parameter SYMBOL_RATE = 2_500_000
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        rx_enable,
    input  wire        drop_errored,
    input  wire        rx_serial,

    // High-priority data FIFO write port
    output reg  [31:0] hipri_data,
    output reg         hipri_data_wr,

    // High-priority control word FIFO write port
    output reg  [31:0] hipri_ctrl,
    output reg         hipri_ctrl_wr,

    // Regular-priority data FIFO write port
    output reg  [31:0] regpri_data,
    output reg         regpri_data_wr,

    // Regular-priority control word FIFO write port
    output reg  [31:0] regpri_ctrl,
    output reg         regpri_ctrl_wr,

    // Statistics (one-clock pulses)
    output reg         stat_rx_frame,
    output reg         stat_rx_crc_err,
    output reg         stat_rx_disp_err,
    output reg         stat_rx_sym_err,

    // Frame received indicator
    output reg         rx_frame_done
);

    // ----------------------------------------------------------------
    // Derived parameters
    // ----------------------------------------------------------------
    localparam CLKS_PER_BIT    = CLK_FREQ_HZ / (SYMBOL_RATE * 10);
    localparam MAJORITY_THRESH = (CLKS_PER_BIT + 1) / 2;
    localparam BIT_CNT_W    = (CLKS_PER_BIT <= 2)  ? 2 :
                              (CLKS_PER_BIT <= 4)  ? 3 :
                              (CLKS_PER_BIT <= 8)  ? 4 :
                              (CLKS_PER_BIT <= 16) ? 5 : 6;

    // ----------------------------------------------------------------
    // K-character data values
    // ----------------------------------------------------------------
    localparam [7:0] K28_5_DATA = 8'hBC;
    localparam [7:0] K27_7_DATA = 8'hFB;   // /S/
    localparam [7:0] K29_7_DATA = 8'hFD;   // /T/

    // ----------------------------------------------------------------
    // Comma patterns (10-bit encoded K28.5 in RD- and RD+)
    // The unique 7-bit comma signature is in bits [9:3] of the symbol.
    // RD- K28.5 = 10'b00_1111_1010  (0x0FA)
    // RD+ K28.5 = 10'b11_0000_0101  (0x305)
    // Comma-7 patterns: 0011111 and 1100000
    // ----------------------------------------------------------------
    localparam [6:0] COMMA_POS = 7'b0011111;
    localparam [6:0] COMMA_NEG = 7'b1100000;

    // ----------------------------------------------------------------
    // 8B10B decoder (combinational)
    // ----------------------------------------------------------------
    reg  [9:0] dec_din;
    reg        dec_disp_in;
    wire [7:0] dec_dout;
    wire       dec_k;
    wire       dec_disp_out;
    wire       dec_disp_err;
    wire       dec_code_err;

    aerolink_dec_8b10b u_dec (
        .data_in  (dec_din),
        .disp_in  (dec_disp_in),
        .data_out (dec_dout),
        .k_out    (dec_k),
        .disp_out (dec_disp_out),
        .disp_err (dec_disp_err),
        .code_err (dec_code_err)
    );

    // ----------------------------------------------------------------
    // CRC-16-CCITT
    // ----------------------------------------------------------------
    reg        crc_init;
    reg        crc_valid;
    reg  [7:0] crc_din;
    wire [15:0] crc_out;

    aerolink_crc16 u_crc (
        .clk     (clk),
        .rst     (rst),
        .init    (crc_init),
        .valid   (crc_valid),
        .data_in (crc_din),
        .crc_out (crc_out)
    );

    // ----------------------------------------------------------------
    // Bit-level oversampling and majority vote
    // ----------------------------------------------------------------
    // Sample rx_serial CLKS_PER_BIT times per bit period.
    // Use majority voting over all samples to determine bit value.
    // ----------------------------------------------------------------
    reg [BIT_CNT_W-1:0] sample_cnt;
    reg [BIT_CNT_W-1:0] ones_cnt;
    reg                  voted_bit;
    reg                  bit_ready;

    always @(posedge clk) begin
        if (rst || !rx_enable) begin
            sample_cnt <= {BIT_CNT_W{1'b0}};
            ones_cnt   <= {BIT_CNT_W{1'b0}};
            voted_bit  <= 1'b0;
            bit_ready  <= 1'b0;
        end else begin
            bit_ready <= 1'b0;
            if (sample_cnt == CLKS_PER_BIT[BIT_CNT_W-1:0] - 1) begin
                // End of bit period - determine value
                // Include this last sample in the count
                if (ones_cnt + rx_serial >= MAJORITY_THRESH[BIT_CNT_W-1:0])
                    voted_bit <= 1'b1;
                else
                    voted_bit <= 1'b0;
                bit_ready  <= 1'b1;
                sample_cnt <= {BIT_CNT_W{1'b0}};
                ones_cnt   <= {BIT_CNT_W{1'b0}};
            end else begin
                sample_cnt <= sample_cnt + 1;
                ones_cnt   <= ones_cnt + {{(BIT_CNT_W-1){1'b0}}, rx_serial};
            end
        end
    end

    // ----------------------------------------------------------------
    // Bit shift register for symbol assembly / comma detect
    // ----------------------------------------------------------------
    reg [9:0] bit_sr;         // 10-bit sliding window
    reg [3:0] sym_bit_cnt;    // counts bits within aligned symbol (0..9)
    reg       aligned;        // true when symbol boundary is locked
    reg       sym_valid;      // pulse: new 10-bit symbol ready

    // Next shift register value (combinational, available before clock edge)
    wire [9:0] bit_sr_nxt = {bit_sr[8:0], voted_bit};

    wire comma_detect_nxt = (bit_sr_nxt[9:3] == COMMA_POS) ||
                            (bit_sr_nxt[9:3] == COMMA_NEG);

    always @(posedge clk) begin
        if (rst || !rx_enable) begin
            bit_sr      <= 10'd0;
            sym_bit_cnt <= 4'd0;
            aligned     <= 1'b0;
            sym_valid   <= 1'b0;
        end else begin
            sym_valid <= 1'b0;
            if (bit_ready) begin
                bit_sr <= bit_sr_nxt;

                if (!aligned) begin
                    // Hunt for comma using the about-to-be-shifted value
                    if (comma_detect_nxt) begin
                        aligned     <= 1'b1;
                        sym_bit_cnt <= 4'd0;
                        sym_valid   <= 1'b1;
                    end
                end else begin
                    // Aligned mode: count bits within 10-bit symbol
                    if (sym_bit_cnt == 4'd9) begin
                        sym_bit_cnt <= 4'd0;
                        sym_valid   <= 1'b1;
                    end else begin
                        sym_bit_cnt <= sym_bit_cnt + 4'd1;
                    end

                    // Opportunistic re-alignment on comma
                    if (comma_detect_nxt && sym_bit_cnt != 4'd9) begin
                        sym_bit_cnt <= 4'd0;
                        sym_valid   <= 1'b1;
                    end
                end
            end
        end
    end

    // Capture symbol: we want to latch bit_sr_nxt at the same time
    // sym_valid is being set. Since sym_valid uses non-blocking
    // assignment, we replicate the trigger condition here.
    reg [9:0] rx_symbol;
    wire sym_capture = bit_ready && (
        (!aligned && comma_detect_nxt) ||
        (aligned && (sym_bit_cnt == 4'd9 || comma_detect_nxt))
    );

    always @(posedge clk) begin
        if (rst)
            rx_symbol <= 10'd0;
        else if (sym_capture)
            rx_symbol <= bit_sr_nxt;
    end

    // ----------------------------------------------------------------
    // Frame receive buffer (131 bytes max: 1 prio + 128 data + 2 CRC)
    // ----------------------------------------------------------------
    reg [7:0] fbuf [0:131];
    reg [7:0] fbuf_cnt;       // number of bytes stored
    reg       frame_disp_err; // accumulated disparity error
    reg       frame_sym_err;  // accumulated symbol error

    // ----------------------------------------------------------------
    // Running disparity tracking for decoder
    // ----------------------------------------------------------------
    reg rx_running_disp;

    // ----------------------------------------------------------------
    // Frame parser state machine
    // ----------------------------------------------------------------
    localparam [3:0] F_HUNT      = 4'd0,
                     F_SYNC      = 4'd1,
                     F_WAIT_SOP  = 4'd2,
                     F_RX_DATA   = 4'd3,
                     F_CHECK_CRC = 4'd4,
                     F_OUTPUT    = 4'd5,
                     F_OUT_DATA  = 4'd6,
                     F_OUT_CTRL  = 4'd7;

    reg [3:0] fstate;

    // Decoded symbol pipeline: decode happens combinationally when
    // we present rx_symbol to the decoder. We process on sym_valid.
    reg       sym_valid_d1;
    always @(posedge clk) begin
        if (rst)
            sym_valid_d1 <= 1'b0;
        else
            sym_valid_d1 <= sym_valid;
    end

    // ----------------------------------------------------------------
    // Output helpers
    // ----------------------------------------------------------------
    reg [7:0] out_idx;        // byte index for FIFO output
    reg [7:0] out_data_end;   // exclusive end of data bytes in fbuf
    reg       out_is_hipri;
    reg       out_crc_err;
    reg [15:0] rx_crc_val;    // received CRC from frame
    reg [7:0] out_prio;       // received priority byte
    reg [7:0] out_data_len;   // data byte count (excl prio, excl CRC)

    // ----------------------------------------------------------------
    // CRC verification state
    // ----------------------------------------------------------------
    reg [7:0] crc_check_idx;
    reg [7:0] crc_check_end;  // exclusive end for CRC computation
    reg       crc_checking;

    // Assembled RX control word (combinational)
    wire [31:0] ctrl_word_w;
    assign ctrl_word_w = {rx_crc_val,
                          frame_sym_err,
                          frame_disp_err,
                          out_crc_err,
                          out_data_len[6:2],
                          out_prio};

    // ----------------------------------------------------------------
    // Main FSM
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            fstate          <= F_HUNT;
            hipri_data_wr   <= 1'b0;
            hipri_ctrl_wr   <= 1'b0;
            regpri_data_wr  <= 1'b0;
            regpri_ctrl_wr  <= 1'b0;
            hipri_data      <= 32'd0;
            hipri_ctrl      <= 32'd0;
            regpri_data     <= 32'd0;
            regpri_ctrl     <= 32'd0;
            stat_rx_frame   <= 1'b0;
            stat_rx_crc_err <= 1'b0;
            stat_rx_disp_err <= 1'b0;
            stat_rx_sym_err <= 1'b0;
            rx_frame_done   <= 1'b0;
            crc_init        <= 1'b0;
            crc_valid       <= 1'b0;
            crc_din         <= 8'd0;
            fbuf_cnt        <= 8'd0;
            frame_disp_err  <= 1'b0;
            frame_sym_err   <= 1'b0;
            rx_running_disp <= 1'b0;
            dec_din         <= 10'd0;
            dec_disp_in     <= 1'b0;
            out_idx         <= 8'd0;
            out_data_end    <= 8'd0;
            out_is_hipri    <= 1'b0;
            out_crc_err     <= 1'b0;
            rx_crc_val      <= 16'd0;
            out_prio        <= 8'd0;
            out_data_len    <= 8'd0;
            crc_check_idx   <= 8'd0;
            crc_check_end   <= 8'd0;
            crc_checking    <= 1'b0;
        end else begin
            // Clear one-cycle pulses
            stat_rx_frame    <= 1'b0;
            stat_rx_crc_err  <= 1'b0;
            stat_rx_disp_err <= 1'b0;
            stat_rx_sym_err  <= 1'b0;
            rx_frame_done    <= 1'b0;
            hipri_data_wr    <= 1'b0;
            hipri_ctrl_wr    <= 1'b0;
            regpri_data_wr   <= 1'b0;
            regpri_ctrl_wr   <= 1'b0;
            crc_init         <= 1'b0;
            crc_valid        <= 1'b0;

            case (fstate)

            // ========================================================
            F_HUNT: begin
                if (rx_enable && sym_valid) begin
                    // Decode the received symbol
                    dec_din     <= rx_symbol;
                    dec_disp_in <= rx_running_disp;
                    // Check next cycle (combinational decoder result
                    // is available immediately with current inputs)
                end
                if (rx_enable && sym_valid_d1) begin
                    rx_running_disp <= dec_disp_out;
                    if (dec_k && dec_dout == K28_5_DATA && !dec_code_err) begin
                        fstate <= F_SYNC;
                    end
                end
            end

            // ========================================================
            F_SYNC: begin
                // Expect second K28.5
                if (sym_valid) begin
                    dec_din     <= rx_symbol;
                    dec_disp_in <= rx_running_disp;
                end
                if (sym_valid_d1) begin
                    rx_running_disp <= dec_disp_out;
                    if (dec_k && dec_dout == K28_5_DATA && !dec_code_err) begin
                        fstate <= F_WAIT_SOP;
                    end else begin
                        fstate <= F_HUNT;
                    end
                end
            end

            // ========================================================
            F_WAIT_SOP: begin
                if (sym_valid) begin
                    dec_din     <= rx_symbol;
                    dec_disp_in <= rx_running_disp;
                end
                if (sym_valid_d1) begin
                    rx_running_disp <= dec_disp_out;
                    if (dec_k && dec_dout == K27_7_DATA && !dec_code_err) begin
                        // /S/ received - start frame
                        fbuf_cnt       <= 8'd0;
                        frame_disp_err <= 1'b0;
                        frame_sym_err  <= 1'b0;
                        fstate         <= F_RX_DATA;
                    end else if (dec_k && dec_dout == K28_5_DATA) begin
                        // Extra comma - stay
                        fstate <= F_WAIT_SOP;
                    end else begin
                        // Unexpected symbol after commas — master idle ended
                        rx_frame_done <= 1'b1;
                        fstate <= F_HUNT;
                    end
                end
            end

            // ========================================================
            F_RX_DATA: begin
                // Receive data bytes until /T/ is detected
                if (sym_valid) begin
                    dec_din     <= rx_symbol;
                    dec_disp_in <= rx_running_disp;
                end
                if (sym_valid_d1) begin
                    rx_running_disp <= dec_disp_out;

                    // Track errors across frame
                    if (dec_disp_err)
                        frame_disp_err <= 1'b1;
                    if (dec_code_err)
                        frame_sym_err <= 1'b1;

                    if (dec_k && dec_dout == K29_7_DATA) begin
                        // /T/ - end of frame
                        fstate <= F_CHECK_CRC;
                    end else if (dec_k) begin
                        // Unexpected K character mid-frame - abort
                        fstate <= F_HUNT;
                    end else begin
                        // Data byte - store in buffer
                        if (fbuf_cnt < 8'd132) begin
                            fbuf[fbuf_cnt] <= dec_dout;
                            fbuf_cnt       <= fbuf_cnt + 8'd1;
                        end else begin
                            // Overflow - frame too long
                            frame_sym_err <= 1'b1;
                        end
                    end
                end
            end

            // ========================================================
            F_CHECK_CRC: begin
                // fbuf layout: [0]=priority, [1..N-2]=data, [N-1]=CRC_hi, [N]=CRC_lo
                // where N = fbuf_cnt - 1
                // Minimum valid frame: prio(1) + data(4) + CRC(2) = 7 bytes
                stat_rx_frame <= 1'b1;

                if (fbuf_cnt < 8'd7) begin
                    // Too short - discard
                    if (frame_sym_err)  stat_rx_sym_err  <= 1'b1;
                    if (frame_disp_err) stat_rx_disp_err <= 1'b1;
                    fstate <= F_HUNT;
                end else begin
                    // Start CRC computation over fbuf[0..fbuf_cnt-3]
                    // (priority byte + data, excluding 2 CRC bytes)
                    crc_init      <= 1'b1;
                    crc_check_idx <= 8'd0;
                    crc_check_end <= fbuf_cnt - 8'd2;
                    crc_checking  <= 1'b1;

                    // Latch received CRC
                    rx_crc_val <= {fbuf[fbuf_cnt - 8'd2], fbuf[fbuf_cnt - 8'd1]};

                    // Latch frame metadata
                    out_prio     <= fbuf[0];
                    out_is_hipri <= fbuf[0][7];
                    out_data_len <= fbuf_cnt - 8'd3; // payload bytes (excl prio, excl CRC)
                    out_data_end <= fbuf_cnt - 8'd2; // exclusive end of prio+data

                    fstate <= F_OUTPUT;
                end
            end

            // ========================================================
            F_OUTPUT: begin
                // Feed CRC bytes sequentially
                if (crc_checking) begin
                    if (crc_check_idx < crc_check_end) begin
                        crc_din       <= fbuf[crc_check_idx];
                        crc_valid     <= 1'b1;
                        crc_check_idx <= crc_check_idx + 8'd1;
                    end else begin
                        // CRC computation complete
                        crc_checking <= 1'b0;
                        crc_valid    <= 1'b0;
                    end
                end else begin
                    // CRC result available (one cycle after last valid)
                    out_crc_err <= (crc_out != rx_crc_val);

                    if (crc_out != rx_crc_val)
                        stat_rx_crc_err <= 1'b1;
                    if (frame_sym_err)
                        stat_rx_sym_err <= 1'b1;
                    if (frame_disp_err)
                        stat_rx_disp_err <= 1'b1;

                    // Decide whether to output
                    if ((crc_out != rx_crc_val || frame_sym_err || frame_disp_err)
                        && drop_errored) begin
                        // Drop this frame
                        rx_frame_done <= 1'b1;
                        fstate        <= F_HUNT;
                    end else begin
                        // Write data to FIFO
                        out_idx <= 8'd1;  // start from fbuf[1] (first data byte)
                        fstate  <= F_OUT_DATA;
                    end
                end
            end

            // ========================================================
            F_OUT_DATA: begin
                if (out_idx + 8'd3 < out_data_end) begin
                    if (out_is_hipri) begin
                        hipri_data    <= {fbuf[out_idx], fbuf[out_idx+8'd1],
                                          fbuf[out_idx+8'd2], fbuf[out_idx+8'd3]};
                        hipri_data_wr <= 1'b1;
                    end else begin
                        regpri_data    <= {fbuf[out_idx], fbuf[out_idx+8'd1],
                                           fbuf[out_idx+8'd2], fbuf[out_idx+8'd3]};
                        regpri_data_wr <= 1'b1;
                    end
                    out_idx <= out_idx + 8'd4;
                end else begin
                    // All data written; now write control word
                    fstate <= F_OUT_CTRL;
                end
            end

            // ========================================================
            F_OUT_CTRL: begin
                // RX Control Word layout:
                //   [7:0]   received priority byte
                //   [12:8]  data length / 4 (1-32, 5 bits)
                //   [13]    CRC error
                //   [14]    disparity error
                //   [15]    symbol error
                //   [31:16] received 16-bit CRC
                if (out_is_hipri) begin
                    hipri_ctrl    <= ctrl_word_w;
                    hipri_ctrl_wr <= 1'b1;
                end else begin
                    regpri_ctrl    <= ctrl_word_w;
                    regpri_ctrl_wr <= 1'b1;
                end
                rx_frame_done <= 1'b1;
                fstate        <= F_HUNT;
            end

            default: fstate <= F_HUNT;

            endcase
        end
    end

endmodule
