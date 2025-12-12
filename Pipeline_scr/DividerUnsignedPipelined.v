`timescale 1ns / 1ps

module DividerUnsignedPipelined (
    input             clk, rst, stall,
    input      [31:0] i_dividend,
    input      [31:0] i_divisor,
    output     [31:0] o_remainder,
    output     [31:0] o_quotient
);
    // 9 stages (0..8)
    reg [31:0] stage_dividend [0:8];
    reg [31:0] stage_remainder[0:8];
    reg [31:0] stage_quotient [0:8];
    reg [31:0] stage_divisor  [0:8];

    // Stage 0 (combinational)
    always @(*) begin
        stage_dividend[0]  = i_dividend;
        stage_remainder[0] = 32'b0;
        stage_quotient[0]  = 32'b0;
        stage_divisor[0]   = i_divisor;
    end

    genvar i, j;
    generate
        for (i = 0; i < 8; i = i + 1) begin : STAGE

            wire [31:0] t_dividend [0:4];
            wire [31:0] t_remainder[0:4];
            wire [31:0] t_quotient [0:4];

            assign t_dividend[0]  = stage_dividend[i];
            assign t_remainder[0] = stage_remainder[i];
            assign t_quotient[0]  = stage_quotient[i];

            for (j = 0; j < 4; j = j + 1) begin : ITER
                divu_1iter u_iter (
                    .i_dividend (t_dividend[j]),
                    .i_divisor  (stage_divisor[i]),
                    .i_remainder(t_remainder[j]),
                    .i_quotient (t_quotient[j]),

                    .o_dividend (t_dividend[j+1]),
                    .o_remainder(t_remainder[j+1]),
                    /* verilator lint_off PINCONNECTEMPTY */
                    .o_quotient_bit(),
                    /* verilator lint_on PINCONNECTEMPTY */
                    .o_quotient (t_quotient[j+1])
                );
            end

            always @(posedge clk) begin
                if (rst) begin
                    stage_dividend[i+1]  <= 32'b0;
                    stage_remainder[i+1] <= 32'b0;
                    stage_quotient[i+1]  <= 32'b0;
                    stage_divisor[i+1]   <= 32'b0;
                end
                else if (!stall) begin
                    stage_dividend[i+1]  <= t_dividend[4];
                    stage_remainder[i+1] <= t_remainder[4];
                    stage_quotient[i+1]  <= t_quotient[4];

                    // ✔ FIXED: pipeline divisor correctly
                    stage_divisor[i+1]   <= stage_divisor[i];
                end
            end
        end
    endgenerate

    assign o_quotient  = stage_quotient[8];
    assign o_remainder = stage_remainder[8];

endmodule

module divu_1iter (
    input      [31:0] i_dividend,
    input      [31:0] i_divisor,
    input      [31:0] i_remainder,
    input      [31:0] i_quotient,
    output     [31:0] o_dividend,
    output     [31:0] o_remainder,
    output            o_quotient_bit,
    output     [31:0] o_quotient
);
    wire [31:0] remainder_sh;
    wire [31:0] diff;
    wire        gte;

    assign remainder_sh = {i_remainder[30:0], i_dividend[31]};
    assign gte = (remainder_sh >= i_divisor);
    assign diff = remainder_sh - i_divisor;

    assign o_remainder = gte ? diff : remainder_sh;
    assign o_quotient_bit = gte;
    assign o_dividend = i_dividend << 1;
    assign o_quotient = (i_quotient << 1) | {31'b0, gte};
endmodule
