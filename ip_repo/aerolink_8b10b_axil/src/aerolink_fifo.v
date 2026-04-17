`timescale 1ns / 1ps
// ============================================================================
// AeroLink-8b10b :: Synchronous FIFO with First-Word-Fall-Through (FWFT)
// ============================================================================

module aerolink_fifo #(
    parameter WIDTH = 32,
    parameter DEPTH = 128
) (
    input  wire                      clk,
    input  wire                      rst,
    input  wire                      wr_en,
    input  wire [WIDTH-1:0]          wr_data,
    input  wire                      rd_en,
    output wire [WIDTH-1:0]          rd_data,
    output wire                      full,
    output wire                      empty,
    output wire [$clog2(DEPTH):0]    count
);

    // ----------------------------------------------------------------
    // Internal storage
    // ----------------------------------------------------------------
    localparam ADDR_W = $clog2(DEPTH);

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_W:0]  wr_ptr;
    reg [ADDR_W:0]  rd_ptr;
    reg [ADDR_W:0]  cnt;

    // ----------------------------------------------------------------
    // Status flags
    // ----------------------------------------------------------------
    assign full  = (cnt == DEPTH[ADDR_W:0]);
    assign empty = (cnt == {(ADDR_W+1){1'b0}});
    assign count = cnt;

    // ----------------------------------------------------------------
    // FWFT read data: always present head of FIFO on rd_data
    // ----------------------------------------------------------------
    assign rd_data = mem[rd_ptr[ADDR_W-1:0]];

    // ----------------------------------------------------------------
    // Qualified control signals
    // ----------------------------------------------------------------
    wire do_write = wr_en & ~full;
    wire do_read  = rd_en & ~empty;

    // ----------------------------------------------------------------
    // Pointer and count management
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            wr_ptr <= {(ADDR_W+1){1'b0}};
            rd_ptr <= {(ADDR_W+1){1'b0}};
            cnt    <= {(ADDR_W+1){1'b0}};
        end else begin
            case ({do_write, do_read})
                2'b10: begin
                    wr_ptr <= (wr_ptr[ADDR_W-1:0] == DEPTH[ADDR_W-1:0] - 1)
                              ? {~wr_ptr[ADDR_W], {ADDR_W{1'b0}}}
                              : wr_ptr + 1'b1;
                    cnt    <= cnt + 1'b1;
                end
                2'b01: begin
                    rd_ptr <= (rd_ptr[ADDR_W-1:0] == DEPTH[ADDR_W-1:0] - 1)
                              ? {~rd_ptr[ADDR_W], {ADDR_W{1'b0}}}
                              : rd_ptr + 1'b1;
                    cnt    <= cnt - 1'b1;
                end
                2'b11: begin
                    wr_ptr <= (wr_ptr[ADDR_W-1:0] == DEPTH[ADDR_W-1:0] - 1)
                              ? {~wr_ptr[ADDR_W], {ADDR_W{1'b0}}}
                              : wr_ptr + 1'b1;
                    rd_ptr <= (rd_ptr[ADDR_W-1:0] == DEPTH[ADDR_W-1:0] - 1)
                              ? {~rd_ptr[ADDR_W], {ADDR_W{1'b0}}}
                              : rd_ptr + 1'b1;
                    // cnt unchanged
                end
                default: ;
            endcase
        end
    end

    // ----------------------------------------------------------------
    // Memory write
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (do_write)
            mem[wr_ptr[ADDR_W-1:0]] <= wr_data;
    end

endmodule
