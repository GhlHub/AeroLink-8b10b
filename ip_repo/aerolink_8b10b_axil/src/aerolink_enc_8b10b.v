`timescale 1ns / 1ps
// ============================================================================
// AeroLink-8b10b :: Standard IBM 8B/10B Encoder (Combinational)
//
// Encodes 8-bit data + K flag into 10-bit line code with running disparity.
// Uses 5b/6b and 3b/4b sub-encoder lookup tables per the IBM standard.
//
// Byte mapping: data_in = {HGF, EDCBA} = {data_in[7:5], data_in[4:0]}
// Output:       data_out = {abcdei, fghj} where bit[9]=a transmitted first
// ============================================================================

module aerolink_enc_8b10b (
    input  wire [7:0] data_in,   // byte to encode (HGFEDCBA)
    input  wire       k_in,      // 1=control character
    input  wire       disp_in,   // running disparity: 0=RD-, 1=RD+
    output reg  [9:0] data_out,  // 10-bit encoded {abcdei, fghj}
    output reg        disp_out,  // new running disparity
    output reg        code_err   // invalid K character requested
);

    // ----------------------------------------------------------------
    // Input decomposition
    // ----------------------------------------------------------------
    wire [4:0] x = data_in[4:0];  // EDCBA -> 5b/6b sub-encoder input
    wire [2:0] y = data_in[7:5];  // HGF   -> 3b/4b sub-encoder input

    // ----------------------------------------------------------------
    // 5b/6b sub-encoder lookup
    // ----------------------------------------------------------------
    // Outputs: code_6b for RD- and RD+
    reg [5:0] code_6b_rdm;  // 6-bit code for RD-
    reg [5:0] code_6b_rdp;  // 6-bit code for RD+

    always @(*) begin
        case (x)
            //                     RD-      RD+
            5'd0:  begin code_6b_rdm = 6'b100111; code_6b_rdp = 6'b011000; end
            5'd1:  begin code_6b_rdm = 6'b011101; code_6b_rdp = 6'b100010; end
            5'd2:  begin code_6b_rdm = 6'b101101; code_6b_rdp = 6'b010010; end
            5'd3:  begin code_6b_rdm = 6'b110001; code_6b_rdp = 6'b110001; end
            5'd4:  begin code_6b_rdm = 6'b110101; code_6b_rdp = 6'b001010; end
            5'd5:  begin code_6b_rdm = 6'b101001; code_6b_rdp = 6'b101001; end
            5'd6:  begin code_6b_rdm = 6'b011001; code_6b_rdp = 6'b011001; end
            5'd7:  begin code_6b_rdm = 6'b111000; code_6b_rdp = 6'b000111; end
            5'd8:  begin code_6b_rdm = 6'b111001; code_6b_rdp = 6'b000110; end
            5'd9:  begin code_6b_rdm = 6'b100101; code_6b_rdp = 6'b100101; end
            5'd10: begin code_6b_rdm = 6'b010101; code_6b_rdp = 6'b010101; end
            5'd11: begin code_6b_rdm = 6'b110100; code_6b_rdp = 6'b110100; end
            5'd12: begin code_6b_rdm = 6'b001101; code_6b_rdp = 6'b001101; end
            5'd13: begin code_6b_rdm = 6'b101100; code_6b_rdp = 6'b101100; end
            5'd14: begin code_6b_rdm = 6'b011100; code_6b_rdp = 6'b011100; end
            5'd15: begin code_6b_rdm = 6'b010111; code_6b_rdp = 6'b101000; end
            5'd16: begin code_6b_rdm = 6'b011011; code_6b_rdp = 6'b100100; end
            5'd17: begin code_6b_rdm = 6'b100011; code_6b_rdp = 6'b100011; end
            5'd18: begin code_6b_rdm = 6'b010011; code_6b_rdp = 6'b010011; end
            5'd19: begin code_6b_rdm = 6'b110010; code_6b_rdp = 6'b110010; end
            5'd20: begin code_6b_rdm = 6'b001011; code_6b_rdp = 6'b001011; end
            5'd21: begin code_6b_rdm = 6'b101010; code_6b_rdp = 6'b101010; end
            5'd22: begin code_6b_rdm = 6'b011010; code_6b_rdp = 6'b011010; end
            5'd23: begin code_6b_rdm = 6'b111010; code_6b_rdp = 6'b000101; end
            5'd24: begin code_6b_rdm = 6'b110011; code_6b_rdp = 6'b001100; end
            5'd25: begin code_6b_rdm = 6'b100110; code_6b_rdp = 6'b100110; end
            5'd26: begin code_6b_rdm = 6'b010110; code_6b_rdp = 6'b010110; end
            5'd27: begin code_6b_rdm = 6'b110110; code_6b_rdp = 6'b001001; end
            5'd28: begin code_6b_rdm = 6'b001110; code_6b_rdp = 6'b001110; end
            5'd29: begin code_6b_rdm = 6'b101110; code_6b_rdp = 6'b010001; end
            5'd30: begin code_6b_rdm = 6'b011110; code_6b_rdp = 6'b100001; end
            5'd31: begin code_6b_rdm = 6'b101011; code_6b_rdp = 6'b010100; end
            default: begin code_6b_rdm = 6'b000000; code_6b_rdp = 6'b000000; end
        endcase
    end

    // ----------------------------------------------------------------
    // 3b/4b sub-encoder lookup (data characters)
    // ----------------------------------------------------------------
    reg [3:0] code_4b_rdm;  // 4-bit code for RD-
    reg [3:0] code_4b_rdp;  // 4-bit code for RD+

    always @(*) begin
        case (y)
            //                    RD-     RD+
            3'd0: begin code_4b_rdm = 4'b1011; code_4b_rdp = 4'b0100; end
            3'd1: begin code_4b_rdm = 4'b1001; code_4b_rdp = 4'b1001; end
            3'd2: begin code_4b_rdm = 4'b0101; code_4b_rdp = 4'b0101; end
            3'd3: begin code_4b_rdm = 4'b1100; code_4b_rdp = 4'b0011; end
            3'd4: begin code_4b_rdm = 4'b1101; code_4b_rdp = 4'b0010; end
            3'd5: begin code_4b_rdm = 4'b1010; code_4b_rdp = 4'b1010; end
            3'd6: begin code_4b_rdm = 4'b0110; code_4b_rdp = 4'b0110; end
            3'd7: begin code_4b_rdm = 4'b1110; code_4b_rdp = 4'b0001; end  // P7
            default: begin code_4b_rdm = 4'b0000; code_4b_rdp = 4'b0000; end
        endcase
    end

    // ----------------------------------------------------------------
    // K28.y 3b/4b lookup
    // ----------------------------------------------------------------
    reg [3:0] k28_4b_rdm;
    reg [3:0] k28_4b_rdp;

    always @(*) begin
        case (y)
            3'd0: begin k28_4b_rdm = 4'b1011; k28_4b_rdp = 4'b0100; end
            3'd1: begin k28_4b_rdm = 4'b0110; k28_4b_rdp = 4'b1001; end
            3'd2: begin k28_4b_rdm = 4'b1010; k28_4b_rdp = 4'b0101; end
            3'd3: begin k28_4b_rdm = 4'b1100; k28_4b_rdp = 4'b0011; end
            3'd4: begin k28_4b_rdm = 4'b1101; k28_4b_rdp = 4'b0010; end
            3'd5: begin k28_4b_rdm = 4'b0101; k28_4b_rdp = 4'b1010; end
            3'd6: begin k28_4b_rdm = 4'b1001; k28_4b_rdp = 4'b0110; end
            3'd7: begin k28_4b_rdm = 4'b0111; k28_4b_rdp = 4'b1000; end
            default: begin k28_4b_rdm = 4'b0000; k28_4b_rdp = 4'b0000; end
        endcase
    end

    // ----------------------------------------------------------------
    // Valid K character detection
    // ----------------------------------------------------------------
    // Valid K chars: K28.0-K28.7 (x=28), K23.7, K27.7, K29.7, K30.7
    reg k_valid;
    always @(*) begin
        if (x == 5'd28)
            k_valid = 1'b1;                           // K28.0 through K28.7
        else if (y == 3'd7 && (x == 5'd23 || x == 5'd27 || x == 5'd29 || x == 5'd30))
            k_valid = 1'b1;                           // K23.7, K27.7, K29.7, K30.7
        else
            k_valid = 1'b0;
    end

    // ----------------------------------------------------------------
    // Disparity calculation helpers
    // ----------------------------------------------------------------
    // Count ones in a 6-bit value
    function [2:0] ones_6;
        input [5:0] v;
        begin
            ones_6 = v[5] + v[4] + v[3] + v[2] + v[1] + v[0];
        end
    endfunction

    // Count ones in a 4-bit value
    function [2:0] ones_4;
        input [3:0] v;
        begin
            ones_4 = v[3] + v[2] + v[1] + v[0];
        end
    endfunction

    // ----------------------------------------------------------------
    // Encoding logic
    // ----------------------------------------------------------------
    reg [5:0] sel_6b;     // selected 6-bit code
    reg       disp_mid;   // intermediate disparity after 6-bit code
    reg [3:0] sel_4b;     // selected 4-bit code
    reg       use_a7;     // use D.x.A7 alternate
    reg       is_k28;     // current char is K28.y
    reg       is_kx7;     // current char is Kx.7 (x != 28)

    always @(*) begin
        // --------------------------------------------------------
        // Defaults
        // --------------------------------------------------------
        code_err = 1'b0;
        is_k28   = 1'b0;
        is_kx7   = 1'b0;

        // --------------------------------------------------------
        // Step 1: Select 5b/6b code
        // --------------------------------------------------------
        if (k_in && x == 5'd28) begin
            // K28.y: special 5b/6b code
            is_k28 = 1'b1;
            sel_6b = disp_in ? 6'b110000 : 6'b001111;
        end else if (k_in && y == 3'd7 && (x == 5'd23 || x == 5'd27 || x == 5'd29 || x == 5'd30)) begin
            // Kx.7: uses same 5b/6b as D.x
            is_kx7 = 1'b1;
            sel_6b = disp_in ? code_6b_rdp : code_6b_rdm;
        end else if (k_in) begin
            // Invalid K character
            code_err = 1'b1;
            sel_6b   = disp_in ? code_6b_rdp : code_6b_rdm;
        end else begin
            // Data character
            sel_6b = disp_in ? code_6b_rdp : code_6b_rdm;
        end

        // --------------------------------------------------------
        // Step 2: Compute intermediate disparity after 6-bit code
        // --------------------------------------------------------
        // If 6-bit code is balanced (3 ones, 3 zeros), disparity unchanged.
        // If unbalanced, disparity flips.
        if (ones_6(sel_6b) > 3'd3)
            disp_mid = 1'b1;   // more 1s -> RD+
        else if (ones_6(sel_6b) < 3'd3)
            disp_mid = 1'b0;   // more 0s -> RD-
        else
            disp_mid = disp_in; // balanced -> unchanged

        // --------------------------------------------------------
        // Step 3: Determine if A7 alternate encoding is needed
        // --------------------------------------------------------
        // D.x.A7 is used instead of D.x.P7 to prevent runs of 5 identical bits.
        // At intermediate RD-: use A7 for x = 17, 18, 20
        //   (their balanced 5b/6b codes end in "11", and P7 RD- = "1110" would create 5 ones)
        // At intermediate RD+: use A7 for x = 11, 13, 14
        //   (their balanced 5b/6b codes end in "00", and P7 RD+ = "0001" would create 5 zeros)
        use_a7 = 1'b0;
        if (y == 3'd7 && !k_in) begin
            if (!disp_mid && (x == 5'd17 || x == 5'd18 || x == 5'd20))
                use_a7 = 1'b1;
            if (disp_mid && (x == 5'd11 || x == 5'd13 || x == 5'd14))
                use_a7 = 1'b1;
        end

        // --------------------------------------------------------
        // Step 4: Select 3b/4b code
        // --------------------------------------------------------
        if (is_k28) begin
            // K28.y: use K28 3b/4b table
            sel_4b = disp_mid ? k28_4b_rdp : k28_4b_rdm;
        end else if (is_kx7) begin
            // Kx.7: special 3b/4b = 0111 (RD-) / 1000 (RD+)
            sel_4b = disp_mid ? 4'b1000 : 4'b0111;
        end else if (use_a7) begin
            // D.x.A7 alternate
            sel_4b = disp_mid ? 4'b1000 : 4'b0111;
        end else begin
            // Normal D.x.y
            sel_4b = disp_mid ? code_4b_rdp : code_4b_rdm;
        end

        // --------------------------------------------------------
        // Step 5: Compose 10-bit output
        // --------------------------------------------------------
        data_out = {sel_6b, sel_4b};

        // --------------------------------------------------------
        // Step 6: Compute final running disparity
        // --------------------------------------------------------
        if (ones_4(sel_4b) > 3'd2)
            disp_out = 1'b1;   // more 1s -> RD+
        else if (ones_4(sel_4b) < 3'd2)
            disp_out = 1'b0;   // more 0s -> RD-
        else
            disp_out = disp_mid; // balanced -> unchanged
    end

endmodule
