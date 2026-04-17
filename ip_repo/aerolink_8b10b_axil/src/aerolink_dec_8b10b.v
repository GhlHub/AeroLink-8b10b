`timescale 1ns / 1ps
// ============================================================================
// AeroLink-8b10b :: Standard IBM 8B/10B Decoder (Combinational)
//
// Decodes 10-bit line code back to 8-bit data + K flag.
// Detects disparity errors and invalid code errors.
//
// Input:  data_in = {abcdei, fghj} where bit[9]=a was received first
// Output: data_out = {HGF, EDCBA} = {data_out[7:5], data_out[4:0]}
// ============================================================================

module aerolink_dec_8b10b (
    input  wire [9:0] data_in,   // 10-bit received symbol {abcdei, fghj}
    input  wire       disp_in,   // expected running disparity
    output reg  [7:0] data_out,  // decoded byte
    output reg        k_out,     // 1=control character
    output reg        disp_out,  // new running disparity
    output reg        disp_err,  // disparity violation
    output reg        code_err   // invalid 10-bit code
);

    // ----------------------------------------------------------------
    // Split into 6-bit and 4-bit sub-codes
    // ----------------------------------------------------------------
    wire [5:0] code_6b = data_in[9:4];  // abcdei
    wire [3:0] code_4b = data_in[3:0];  // fghj

    // ----------------------------------------------------------------
    // Count ones for disparity tracking
    // ----------------------------------------------------------------
    function [2:0] ones_6;
        input [5:0] v;
        begin
            ones_6 = v[5] + v[4] + v[3] + v[2] + v[1] + v[0];
        end
    endfunction

    function [2:0] ones_4;
        input [3:0] v;
        begin
            ones_4 = v[3] + v[2] + v[1] + v[0];
        end
    endfunction

    // ----------------------------------------------------------------
    // 6b/5b reverse lookup: abcdei -> EDCBA + validity
    // ----------------------------------------------------------------
    reg [4:0] dec_5b;
    reg       err_6b;
    reg       is_k28_6b;   // received K28 5b/6b pattern

    always @(*) begin
        dec_5b   = 5'd0;
        err_6b   = 1'b0;
        is_k28_6b = 1'b0;

        case (code_6b)
            // Each data code can appear in RD- form, RD+ form, or both (balanced)
            6'b100111, 6'b011000: dec_5b = 5'd0;
            6'b011101, 6'b100010: dec_5b = 5'd1;
            6'b101101, 6'b010010: dec_5b = 5'd2;
            6'b110001:            dec_5b = 5'd3;
            6'b110101, 6'b001010: dec_5b = 5'd4;
            6'b101001:            dec_5b = 5'd5;
            6'b011001:            dec_5b = 5'd6;
            6'b111000, 6'b000111: dec_5b = 5'd7;
            6'b111001, 6'b000110: dec_5b = 5'd8;
            6'b100101:            dec_5b = 5'd9;
            6'b010101:            dec_5b = 5'd10;
            6'b110100:            dec_5b = 5'd11;
            6'b001101:            dec_5b = 5'd12;
            6'b101100:            dec_5b = 5'd13;
            6'b011100:            dec_5b = 5'd14;
            6'b010111, 6'b101000: dec_5b = 5'd15;
            6'b011011, 6'b100100: dec_5b = 5'd16;
            6'b100011:            dec_5b = 5'd17;
            6'b010011:            dec_5b = 5'd18;
            6'b110010:            dec_5b = 5'd19;
            6'b001011:            dec_5b = 5'd20;
            6'b101010:            dec_5b = 5'd21;
            6'b011010:            dec_5b = 5'd22;
            6'b111010, 6'b000101: dec_5b = 5'd23;
            6'b110011, 6'b001100: dec_5b = 5'd24;
            6'b100110:            dec_5b = 5'd25;
            6'b010110:            dec_5b = 5'd26;
            6'b110110, 6'b001001: dec_5b = 5'd27;
            6'b001110:            dec_5b = 5'd28;
            6'b101110, 6'b010001: dec_5b = 5'd29;
            6'b011110, 6'b100001: dec_5b = 5'd30;
            6'b101011, 6'b010100: dec_5b = 5'd31;

            // K28 5b/6b patterns
            6'b001111: begin dec_5b = 5'd28; is_k28_6b = 1'b1; end
            6'b110000: begin dec_5b = 5'd28; is_k28_6b = 1'b1; end

            default: begin
                dec_5b = 5'd0;
                err_6b = 1'b1;
            end
        endcase
    end

    // ----------------------------------------------------------------
    // 4b/3b reverse lookup: fghj -> HGF + validity + K detection
    // ----------------------------------------------------------------
    reg [2:0] dec_3b;
    reg       err_4b;
    reg       is_k_4b;       // 4b code could be part of a K character
    reg       is_a7;         // received D.x.A7 alternate

    always @(*) begin
        dec_3b  = 3'd0;
        err_4b  = 1'b0;
        is_k_4b = 1'b0;
        is_a7   = 1'b0;

        case (code_4b)
            // Normal D.x.y 4b codes
            4'b1011, 4'b0100: dec_3b = 3'd0;
            4'b1001:          dec_3b = 3'd1;
            4'b0101:          dec_3b = 3'd2;
            4'b1100, 4'b0011: dec_3b = 3'd3;
            4'b1101, 4'b0010: dec_3b = 3'd4;
            4'b1010:          dec_3b = 3'd5;
            4'b0110:          dec_3b = 3'd6;

            // P7: 1110 (RD-) / 0001 (RD+)
            4'b1110: dec_3b = 3'd7;
            4'b0001: dec_3b = 3'd7;

            // A7: 0111 (RD-) / 1000 (RD+)
            4'b0111: begin dec_3b = 3'd7; is_a7 = 1'b1; end
            4'b1000: begin dec_3b = 3'd7; is_a7 = 1'b1; end

            default: begin
                dec_3b = 3'd0;
                err_4b = 1'b1;
            end
        endcase
    end

    // ----------------------------------------------------------------
    // K character identification
    // ----------------------------------------------------------------
    // K28.y: 6b code is K28 pattern (001111/110000) + any valid 4b code for K28.y
    //   K28 3b/4b codes differ from D 3b/4b codes for y=1,2,5,6
    //   K28.0: 1011/0100 (same as D.x.0)
    //   K28.1: 0110/1001
    //   K28.2: 1010/0101
    //   K28.3: 1100/0011 (same as D.x.3)
    //   K28.4: 1101/0010 (same as D.x.4)
    //   K28.5: 0101/1010
    //   K28.6: 1001/0110
    //   K28.7: 0111/1000 (same as A7)
    //
    // Kx.7: 6b code is D.x (x=23,27,29,30) + 4b = 0111/1000
    //   K23.7, K27.7, K29.7, K30.7

    reg       is_k28_char;
    reg [2:0] k28_y;

    always @(*) begin
        is_k28_char = 1'b0;
        k28_y       = 3'd0;

        if (is_k28_6b) begin
            // Decode K28.y based on 4b code
            // Need to identify which K28 3b/4b this is
            case (code_4b)
                4'b1011, 4'b0100: begin is_k28_char = 1'b1; k28_y = 3'd0; end
                4'b0110: begin
                    // Could be K28.6 (RD+ form, mid_rd=RD+) or K28.1 (RD- form, mid_rd=RD-)
                    // 6b=001111 -> mid_rd=RD+, 0110 is K28.6 RD+
                    // 6b=110000 -> mid_rd=RD-, 0110 is K28.1 RD-
                    if (code_6b == 6'b001111)
                        begin is_k28_char = 1'b1; k28_y = 3'd6; end  // K28.6
                    else
                        begin is_k28_char = 1'b1; k28_y = 3'd1; end  // K28.1
                end
                4'b1001: begin
                    // Could be K28.1 (RD+ form, mid_rd=RD+) or K28.6 (RD- form, mid_rd=RD-)
                    // 6b=001111 -> mid_rd=RD+, 1001 is K28.1 RD+
                    // 6b=110000 -> mid_rd=RD-, 1001 is K28.6 RD-
                    if (code_6b == 6'b001111)
                        begin is_k28_char = 1'b1; k28_y = 3'd1; end  // K28.1
                    else
                        begin is_k28_char = 1'b1; k28_y = 3'd6; end  // K28.6
                end
                4'b1010: begin
                    // Could be K28.5 (RD+ form, mid_rd=RD+) or K28.2 (RD- form, mid_rd=RD-)
                    // 6b=001111 -> mid_rd=RD+, 1010 is K28.5 RD+
                    // 6b=110000 -> mid_rd=RD-, 1010 is K28.2 RD-
                    if (code_6b == 6'b001111)
                        begin is_k28_char = 1'b1; k28_y = 3'd5; end  // K28.5
                    else
                        begin is_k28_char = 1'b1; k28_y = 3'd2; end  // K28.2
                end
                4'b0101: begin
                    // Could be K28.2 (RD+ form, mid_rd=RD+) or K28.5 (RD- form, mid_rd=RD-)
                    // 6b=001111 -> mid_rd=RD+, 0101 is K28.2 RD+
                    // 6b=110000 -> mid_rd=RD-, 0101 is K28.5 RD-
                    if (code_6b == 6'b001111)
                        begin is_k28_char = 1'b1; k28_y = 3'd2; end  // K28.2
                    else
                        begin is_k28_char = 1'b1; k28_y = 3'd5; end  // K28.5
                end
                4'b1100, 4'b0011: begin is_k28_char = 1'b1; k28_y = 3'd3; end
                4'b1101, 4'b0010: begin is_k28_char = 1'b1; k28_y = 3'd4; end
                4'b0111, 4'b1000: begin is_k28_char = 1'b1; k28_y = 3'd7; end
                default: ; // will be caught as code_err
            endcase
        end
    end

    reg is_kx7_char;
    always @(*) begin
        // Kx.7 detection: 5b value is 23, 27, 29, or 30, AND 4b is A7 pattern (0111/1000)
        // Note: is_a7 is already set for 0111/1000 codes
        is_kx7_char = 1'b0;
        if (is_a7 && !is_k28_6b) begin
            case (dec_5b)
                5'd23, 5'd27, 5'd29, 5'd30: is_kx7_char = 1'b1;
                default: ;
            endcase
        end
    end

    // ----------------------------------------------------------------
    // Disparity checking for 6-bit sub-block
    // ----------------------------------------------------------------
    wire [2:0] ones6 = ones_6(code_6b);
    wire       disp6_balanced = (ones6 == 3'd3);
    wire       disp6_pos      = (ones6 > 3'd3);  // more 1s
    wire       disp6_neg      = (ones6 < 3'd3);  // more 0s

    // Expected: if disp_in=RD-, code should have >= 3 ones (balanced or positive)
    //           if disp_in=RD+, code should have <= 3 ones (balanced or negative)
    wire disp6_err = (disp_in == 1'b0 && disp6_neg) ||   // RD- but got negative code
                     (disp_in == 1'b1 && disp6_pos);      // RD+ but got positive code

    // Intermediate disparity after 6-bit code
    wire disp_mid = disp6_balanced ? disp_in :
                    disp6_pos      ? 1'b1    : 1'b0;

    // ----------------------------------------------------------------
    // Disparity checking for 4-bit sub-block
    // ----------------------------------------------------------------
    wire [2:0] ones4 = ones_4(code_4b);
    wire       disp4_balanced = (ones4 == 3'd2);
    wire       disp4_pos      = (ones4 > 3'd2);
    wire       disp4_neg      = (ones4 < 3'd2);

    wire disp4_err = (disp_mid == 1'b0 && disp4_neg) ||
                     (disp_mid == 1'b1 && disp4_pos);

    // Final disparity
    wire disp_final = disp4_balanced ? disp_mid :
                      disp4_pos      ? 1'b1     : 1'b0;

    // ----------------------------------------------------------------
    // Code error detection
    // ----------------------------------------------------------------
    // A7 (0111/1000) as a data code is only valid for specific x values:
    //   RD-: A7=0111 valid for x=17,18,20 (also for K28.7 and Kx.7)
    //   RD+: A7=1000 valid for x=11,13,14 (also for K28.7 and Kx.7)
    reg a7_valid;
    always @(*) begin
        a7_valid = 1'b0;
        if (is_a7 && !is_k28_6b) begin
            // For Kx.7 characters, A7 is always valid
            case (dec_5b)
                5'd23, 5'd27, 5'd29, 5'd30: a7_valid = 1'b1;
                default: ;
            endcase
            // For data characters, check if A7 is appropriate
            if (code_4b == 4'b0111) begin
                // A7 RD- form - valid when intermediate RD is RD-
                case (dec_5b)
                    5'd17, 5'd18, 5'd20: a7_valid = 1'b1;
                    default: ;
                endcase
            end else if (code_4b == 4'b1000) begin
                // A7 RD+ form - valid when intermediate RD is RD+
                case (dec_5b)
                    5'd11, 5'd13, 5'd14: a7_valid = 1'b1;
                    default: ;
                endcase
            end
        end
    end

    // ----------------------------------------------------------------
    // Output composition
    // ----------------------------------------------------------------
    always @(*) begin
        // Decoded data byte
        if (is_k28_char)
            data_out = {k28_y, 5'd28};
        else if (is_kx7_char)
            data_out = {3'd7, dec_5b};
        else
            data_out = {dec_3b, dec_5b};

        // K flag
        k_out = is_k28_char || is_kx7_char;

        // Disparity output
        disp_out = disp_final;

        // Disparity error
        disp_err = disp6_err || disp4_err;

        // Code error: invalid 6b, invalid 4b, or invalid A7 usage
        code_err = err_6b || err_4b || (is_a7 && !is_k28_6b && !a7_valid);
    end

endmodule
