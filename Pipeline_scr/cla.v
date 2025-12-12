`timescale 1ns / 1ps

/**
 * @param a first 1-bit input
 * @param b second 1-bit input
 * @param g whether a and b generate a carry
 * @param p whether a and b would propagate an incoming carry
 */
module gp1(input wire a, b,
           output wire g, p);
   assign g = a & b;
   assign p = a | b;
endmodule

/**
 * Computes aggregate generate/propagate signals over a 4-bit window.
 * @param gin incoming generate signals
 * @param pin incoming propagate signals
 * @param cin the incoming carry
 * @param gout whether these 4 bits internally would generate a carry-out (independent of cin)
 * @param pout whether these 4 bits internally would propagate an incoming carry from cin
 * @param cout the carry outs for the low-order 3 bits
 */
module gp4(
    input  wire [3:0] gin,   // G3 G2 G1 G0
    input  wire [3:0] pin,   // P3 P2 P1 P0
    input  wire       cin,   // C0
    output wire       gout,  // window generate G3-0
    output wire       pout,  // window propagate P3-0
    output wire [2:0] cout   // {C3, C2, C1}
);

    // --------------------------------------------------
    // 1. Internal bit-level carries
    // --------------------------------------------------
    // C1 = G0 | P0*C0
    wire c1 = gin[0] | (pin[0] & cin);

    // C2 = G1 | P1*C1
    wire c2 = gin[1] | (pin[1] & c1);

    // C3 = G2 | P2*C2
    wire c3 = gin[2] | (pin[2] & c2);

    // Export carries for low-order 3 bits
    // cout[0] = C1, cout[1] = C2, cout[2] = C3
    assign cout = {c3, c2, c1};

    // --------------------------------------------------
    // 2. Window propagate: all 4 bits must propagate
    //     P3-0 = P3 & P2 & P1 & P0
    //     Use reduction AND (&pin) for compactness.
    // --------------------------------------------------
    assign pout = &pin;

    // --------------------------------------------------
    // 3. Window generate: internal carry-out when C0 = 0
    //
    //   C1' = G0
    //   C2' = G1 | P1*G0
    //   C3' = G2 | P2*G1 | P2*P1*G0
    //   C4' = G3
    //       | P3*G2
    //       | P3*P2*G1
    //       | P3*P2*P1*G0
    //
    //   => gout = C4'
    // --------------------------------------------------
    assign gout =
        gin[3] |
        (pin[3] & gin[2]) |
        (pin[3] & pin[2] & gin[1]) |
        (pin[3] & pin[2] & pin[1] & gin[0]);

endmodule

//** Same as gp4 but for an 8-bit window instead */
module gp8(
    input  wire [7:0] gin,   // G7 G6 G5 G4 G3 G2 G1 G0
    input  wire [7:0] pin,   // P7 P6 P5 P4 P3 P2 P1 P0
    input  wire       cin,   // C0
    output wire       gout,  // window generate G7-0 (carry-out when C0 = 0)
    output wire       pout,  // window propagate P7-0 (all bits propagate)
    output wire [6:0] cout   // {C7, C6, C5, C4, C3, C2, C1}
);

    // --------------------------------------------------
    // 1. Carry chain for REAL cin: C1..C7
    // --------------------------------------------------
    wire c1 = gin[0] | (pin[0] & cin);
    wire c2 = gin[1] | (pin[1] & c1);
    wire c3 = gin[2] | (pin[2] & c2);
    wire c4 = gin[3] | (pin[3] & c3);
    wire c5 = gin[4] | (pin[4] & c4);
    wire c6 = gin[5] | (pin[5] & c5);
    wire c7 = gin[6] | (pin[6] & c6);
    // C8 (final carry) depends on bit 7 and is not exported as cout

    assign cout = {c7, c6, c5, c4, c3, c2, c1};

    // --------------------------------------------------
    // 2. Window propagate: all 8 bits propagate
    // --------------------------------------------------
    assign pout = &pin;

    // --------------------------------------------------
    // 3. Window generate: carry-out when C0 = 0
    //
    //   Compute "primed" carries assuming C0' = 0:
    //   C1' = G0
    //   C2' = G1 | P1*G0
    //   ...
    //   C8' = G7 | P7*C7'
    //
    //   gout = C8'
    // --------------------------------------------------
    wire c1_0 = gin[0];
    wire c2_0 = gin[1] | (pin[1] & c1_0);
    wire c3_0 = gin[2] | (pin[2] & c2_0);
    wire c4_0 = gin[3] | (pin[3] & c3_0);
    wire c5_0 = gin[4] | (pin[4] & c4_0);
    wire c6_0 = gin[5] | (pin[5] & c5_0);
    wire c7_0 = gin[6] | (pin[6] & c6_0);
    wire c8_0 = gin[7] | (pin[7] & c7_0);

    assign gout = c8_0;

endmodule


module cla
  (input  wire [31:0] a, b,
   input  wire        cin,
   output wire [31:0] sum);

    // --------------------------------------------------
    // 1. Per-bit generate / propagate using gp1
    // --------------------------------------------------
    wire [31:0] g;
    wire [31:0] p;

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : gp_bits
            gp1 u_gp1 (
                .a(a[i]),
                .b(b[i]),
                .g(g[i]),
                .p(p[i])
            );
        end
    endgenerate

    // --------------------------------------------------
    // 2. Carry chain: C0..C31
    //    C0 = cin
    //    C_(i+1) = g[i] | p[i]*C_i
    // --------------------------------------------------
    wire [31:0] c;   // carry *into* bit i
    assign c[0] = cin;

    generate
        for (i = 0; i < 31; i = i + 1) begin : carry
            assign c[i+1] = g[i] | (p[i] & c[i]);
        end
    endgenerate

    // --------------------------------------------------
    // 3. Sum bits: sum[i] = a[i] ^ b[i] ^ C_i
    // --------------------------------------------------
    assign sum = a ^ b ^ c;

endmodule
