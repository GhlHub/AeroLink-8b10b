`timescale 1ns / 1ps
module aerolink_tx_engine #(
    parameter CLK_FREQ_HZ = 125_000_000,
    parameter SYMBOL_RATE = 2_500_000,
    parameter IS_MASTER   = 1
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        tx_enable,
    input  wire        rx_frame_done,
    input  wire [31:0] hipri_data,
    input  wire        hipri_data_empty,
    output reg         hipri_data_rd,
    input  wire [31:0] hipri_ctrl,
    input  wire        hipri_ctrl_empty,
    output reg         hipri_ctrl_rd,
    input  wire [31:0] regpri_data,
    input  wire        regpri_data_empty,
    output reg         regpri_data_rd,
    input  wire [31:0] regpri_ctrl,
    input  wire        regpri_ctrl_empty,
    output reg         regpri_ctrl_rd,
    output reg         tx_serial,
    output reg         tx_active,
    output reg         stat_tx_frame,
    output reg         stat_tx_idle,
    output wire        irq_hipri_empty,
    output wire        irq_regpri_empty,
    output reg         irq_hipri_complete,
    output reg         irq_regpri_complete
);

    localparam CLKS_PER_BIT = CLK_FREQ_HZ / (SYMBOL_RATE * 10);
    localparam FRAME_PERIOD = CLK_FREQ_HZ / 5000;
    localparam BIT_CNT_W    = (CLKS_PER_BIT <= 2)  ? 2 :
                              (CLKS_PER_BIT <= 4)  ? 3 :
                              (CLKS_PER_BIT <= 8)  ? 4 :
                              (CLKS_PER_BIT <= 16) ? 5 : 6;

    localparam [7:0] K28_5 = 8'hBC;
    localparam [7:0] K27_7 = 8'hFB;
    localparam [7:0] K29_7 = 8'hFD;

    localparam [3:0] S_IDLE       = 4'd0,
                     S_CHECK_FIFO = 4'd1,
                     S_LATCH_CTRL = 4'd2,
                     S_LOAD_DATA  = 4'd3,
                     S_LOAD_WAIT  = 4'd4,
                     S_CRC_CALC   = 4'd5,
                     S_SEND_FIRST = 4'd6,
                     S_SHIFT_WAIT = 4'd7,
                     S_FRAME_DONE = 4'd8,
                     S_REPEAT     = 4'd9,
                     S_DONE       = 4'd10,
                     S_IDLE_FIRST = 4'd11,
                     S_IDLE_SHIFT = 4'd12;

    // Registers
    reg [3:0]  state;
    reg [31:0] period_cnt;
    reg [9:0]  sr;
    reg [3:0]  sr_bit;
    reg [BIT_CNT_W-1:0] sr_timer;
    reg        sr_busy;
    reg        running_disp;
    reg [7:0]  payload_len;
    reg [15:0] frame_crc;
    reg [7:0]  lat_prio;
    reg [7:0]  dbuf [0:127];
    reg [7:0]  sym_idx;
    reg [7:0]  sym_total;
    reg        lat_auto_crc, lat_is_hipri;
    reg [2:0]  lat_repeat, repeat_rem;
    reg [6:0]  lat_word_cnt;
    reg [7:0]  dbuf_len;
    reg [6:0]  ld_idx;
    reg [7:0]  crc_idx, crc_end;
    reg        crc_settling;
    reg [1:0]  idle_cnt;
    reg        crc_init, crc_valid;
    reg [7:0]  crc_din;

    wire [31:0] sel_data = lat_is_hipri ? hipri_data : regpri_data;
    wire period_expired = (period_cnt == 32'd0);
    wire sr_last_tick = (sr_timer == CLKS_PER_BIT[BIT_CNT_W-1:0] - 1);
    wire sr_sym_done  = sr_busy && (sr_bit == 4'd9) && sr_last_tick;

    assign irq_hipri_empty  = hipri_ctrl_empty;
    assign irq_regpri_empty = regpri_ctrl_empty;

    // Combinational 8B10B encoder inputs — always valid for current sym_idx
    reg [7:0] enc_din_c;
    reg       enc_k_c;
    wire [9:0] enc_dout;
    wire       enc_disp_out;
    wire       enc_code_err;

    always @(*) begin
        enc_din_c = K28_5;
        enc_k_c   = 1'b1;
        if (state == S_SEND_FIRST || state == S_SHIFT_WAIT || state == S_FRAME_DONE) begin
            if (sym_idx <= 8'd1) begin
                enc_din_c = K28_5; enc_k_c = 1'b1;
            end else if (sym_idx == 8'd2) begin
                enc_din_c = K27_7; enc_k_c = 1'b1;
            end else if (sym_idx == 8'd3) begin
                enc_din_c = lat_prio; enc_k_c = 1'b0;
            end else if (sym_idx < 8'd4 + payload_len) begin
                enc_din_c = dbuf[sym_idx - 8'd4]; enc_k_c = 1'b0;
            end else if (sym_idx == 8'd4 + payload_len) begin
                enc_din_c = frame_crc[15:8]; enc_k_c = 1'b0;
            end else if (sym_idx == 8'd4 + payload_len + 8'd1) begin
                enc_din_c = frame_crc[7:0]; enc_k_c = 1'b0;
            end else begin
                enc_din_c = K29_7; enc_k_c = 1'b1;
            end
        end
    end

    aerolink_enc_8b10b u_enc (
        .data_in(enc_din_c), .k_in(enc_k_c), .disp_in(running_disp),
        .data_out(enc_dout), .disp_out(enc_disp_out), .code_err(enc_code_err)
    );

    // CRC-16
    wire [15:0] crc_out;
    aerolink_crc16 u_crc (
        .clk(clk), .rst(rst), .init(crc_init),
        .valid(crc_valid), .data_in(crc_din), .crc_out(crc_out)
    );

    // Single always block: shift register + FSM
    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE; sr <= 0; sr_bit <= 0; sr_timer <= 0;
            sr_busy <= 0; tx_serial <= 0; running_disp <= 0;
            hipri_data_rd <= 0; hipri_ctrl_rd <= 0;
            regpri_data_rd <= 0; regpri_ctrl_rd <= 0;
            tx_active <= 0; stat_tx_frame <= 0; stat_tx_idle <= 0;
            irq_hipri_complete <= 0; irq_regpri_complete <= 0;
            crc_init <= 0; crc_valid <= 0; crc_din <= 0;
            lat_prio <= 0; lat_auto_crc <= 0; lat_repeat <= 0;
            lat_word_cnt <= 0; lat_is_hipri <= 0; repeat_rem <= 0;
            dbuf_len <= 0; ld_idx <= 0;
            crc_idx <= 0; crc_end <= 0; frame_crc <= 0; crc_settling <= 0;
            sym_idx <= 0; sym_total <= 0; payload_len <= 0; idle_cnt <= 0;
            period_cnt <= FRAME_PERIOD[31:0] - 1;
        end else begin
            // Default pulse clearing
            stat_tx_frame <= 0; stat_tx_idle <= 0;
            irq_hipri_complete <= 0; irq_regpri_complete <= 0;
            hipri_data_rd <= 0; hipri_ctrl_rd <= 0;
            regpri_data_rd <= 0; regpri_ctrl_rd <= 0;
            crc_init <= 0; crc_valid <= 0;

            // ============================================================
            // Shift register: gapless serialization with inline reload
            // ============================================================
            if (sr_busy) begin
                if (sr_last_tick) begin
                    sr_timer <= 0;
                    if (sr_bit == 4'd9) begin
                        // Current symbol done. Try inline reload.
                        if (state == S_SHIFT_WAIT && sym_idx < sym_total) begin
                            // Load next data-frame symbol directly from encoder
                            sr <= enc_dout;
                            sr_bit <= 0;
                            tx_serial <= enc_dout[9];
                            running_disp <= enc_disp_out;
                            sym_idx <= sym_idx + 8'd1;
                        end else if (state == S_IDLE_SHIFT && idle_cnt < 2'd3) begin
                            // Load next idle comma
                            sr <= enc_dout;
                            sr_bit <= 0;
                            tx_serial <= enc_dout[9];
                            running_disp <= enc_disp_out;
                            idle_cnt <= idle_cnt + 2'd1;
                        end else begin
                            sr_busy <= 0;
                        end
                    end else begin
                        sr_bit <= sr_bit + 4'd1;
                        tx_serial <= sr[8];
                        sr <= {sr[8:0], 1'b0};
                    end
                end else begin
                    sr_timer <= sr_timer + 1;
                end
            end

            // ============================================================
            // Frame period timer
            // ============================================================
            if (state == S_IDLE && period_cnt != 0)
                period_cnt <= period_cnt - 1;
            if (state == S_DONE)
                period_cnt <= FRAME_PERIOD[31:0] - 1;

            // ============================================================
            // FSM
            // ============================================================
            case (state)

            S_IDLE: begin
                tx_active <= 0;
                if (tx_enable) begin
                    if (IS_MASTER != 0) begin
                        if (period_expired) state <= S_CHECK_FIFO;
                    end else begin
                        if (rx_frame_done) state <= S_CHECK_FIFO;
                    end
                end
            end

            S_CHECK_FIFO: begin
                if (!hipri_ctrl_empty) begin
                    lat_is_hipri <= 1; state <= S_LATCH_CTRL;
                end else if (!regpri_ctrl_empty) begin
                    lat_is_hipri <= 0; state <= S_LATCH_CTRL;
                end else if (IS_MASTER != 0) begin
                    tx_active <= 1; idle_cnt <= 0; state <= S_IDLE_FIRST;
                end else
                    state <= S_IDLE;
            end

            S_LATCH_CTRL: begin
                if (lat_is_hipri) begin
                    lat_prio <= hipri_ctrl[7:0]; lat_auto_crc <= hipri_ctrl[8];
                    lat_repeat <= hipri_ctrl[11:9]; lat_word_cnt <= hipri_ctrl[18:12];
                    hipri_ctrl_rd <= 1;
                end else begin
                    lat_prio <= regpri_ctrl[7:0]; lat_auto_crc <= regpri_ctrl[8];
                    lat_repeat <= regpri_ctrl[11:9]; lat_word_cnt <= regpri_ctrl[18:12];
                    regpri_ctrl_rd <= 1;
                end
                ld_idx <= 0; dbuf_len <= 0; state <= S_LOAD_DATA;
            end

            S_LOAD_DATA: begin
                if (ld_idx < lat_word_cnt) begin
                    dbuf[{ld_idx, 2'b00}] <= sel_data[31:24];
                    dbuf[{ld_idx, 2'b01}] <= sel_data[23:16];
                    dbuf[{ld_idx, 2'b10}] <= sel_data[15:8];
                    dbuf[{ld_idx, 2'b11}] <= sel_data[7:0];
                    if (lat_is_hipri) hipri_data_rd <= 1;
                    else regpri_data_rd <= 1;
                    ld_idx <= ld_idx + 1; dbuf_len <= dbuf_len + 8'd4;
                    state <= S_LOAD_WAIT;
                end else begin // ld_idx >= lat_word_cnt
                    crc_init <= 1; crc_idx <= 0;
                    if (lat_auto_crc) begin
                        crc_end <= dbuf_len; payload_len <= dbuf_len;
                    end else begin
                        crc_end     <= (dbuf_len >= 8'd2) ? dbuf_len - 8'd2 : 8'd0;
                        payload_len <= (dbuf_len >= 8'd2) ? dbuf_len - 8'd2 : 8'd0;
                    end
                    state <= S_CRC_CALC;
                end
            end

            S_LOAD_WAIT: begin
                state <= S_LOAD_DATA;
            end

            S_CRC_CALC: begin
                if (crc_settling) begin
                    crc_settling <= 0;
                    frame_crc <= crc_out;
                    if (!lat_auto_crc && dbuf_len >= 8'd2)
                        frame_crc <= {dbuf[dbuf_len-8'd2], dbuf[dbuf_len-8'd1]};
                    repeat_rem <= lat_repeat;
                    sym_idx <= 0; sym_total <= 8'd4 + payload_len + 8'd3;
                    tx_active <= 1; state <= S_SEND_FIRST;
                end else if (crc_idx == 0 && !crc_valid) begin
                    crc_din <= lat_prio; crc_valid <= 1;
                end else if (crc_idx == 0 && crc_valid) begin
                    if (crc_end > 0) begin
                        crc_din <= dbuf[0]; crc_valid <= 1; crc_idx <= 1;
                    end else begin
                        crc_valid <= 0; crc_settling <= 1;
                    end
                end else if (crc_idx < crc_end) begin
                    crc_din <= dbuf[crc_idx]; crc_valid <= 1;
                    crc_idx <= crc_idx + 1;
                end else begin
                    crc_valid <= 0; crc_settling <= 1;
                end
            end

            // Load first symbol (gap OK since line was idle)
            S_SEND_FIRST: begin
                if (!sr_busy) begin
                    sr <= enc_dout; sr_bit <= 0; sr_timer <= 0;
                    sr_busy <= 1; tx_serial <= enc_dout[9];
                    running_disp <= enc_disp_out;
                    sym_idx <= sym_idx + 8'd1;
                    state <= S_SHIFT_WAIT;
                end
            end

            // Wait for shift register; inline reload handles gapless operation
            S_SHIFT_WAIT: begin
                if (sr_sym_done && sym_idx >= sym_total) begin
                    state <= S_FRAME_DONE;
                end
            end

            S_FRAME_DONE: begin
                stat_tx_frame <= 1;
                state <= S_REPEAT;
            end

            S_REPEAT: begin
                if (repeat_rem > 0) begin
                    repeat_rem <= repeat_rem - 1;
                    sym_idx <= 0; state <= S_SEND_FIRST;
                end else
                    state <= S_DONE;
            end

            S_DONE: begin
                tx_active <= 0;
                if (lat_is_hipri) irq_hipri_complete <= 1;
                else irq_regpri_complete <= 1;
                state <= S_IDLE;
            end

            // Idle: first comma (gap OK since line was idle)
            S_IDLE_FIRST: begin
                if (!sr_busy) begin
                    sr <= enc_dout; sr_bit <= 0; sr_timer <= 0;
                    sr_busy <= 1; tx_serial <= enc_dout[9];
                    running_disp <= enc_disp_out;
                    state <= S_IDLE_SHIFT;
                end
            end

            // Idle: wait; inline reload handles subsequent commas
            S_IDLE_SHIFT: begin
                if (sr_sym_done && idle_cnt >= 2'd3) begin
                    stat_tx_idle <= 1; tx_active <= 0;
                    period_cnt <= FRAME_PERIOD[31:0] - 1;
                    state <= S_IDLE;
                end
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
