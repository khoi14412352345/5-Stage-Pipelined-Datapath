`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31
// inst are 32 bits in RV32IM
`define INST_SIZE 31
// RV opcodes are 7 bits
`define OPCODE_SIZE 6
/* verilator lint_off DEFOVERRIDE */
`define DIVIDER_STAGES 8
/* verilator lint_on DEFOVERRIDE */
`include "cla.v"
`include "DividerUnsignedPipelined.v"
`include "../hw5-pipelined/cycle_status.v"

wire        rf_we;
wire [4:0]  rf_rd;
wire [31:0] rf_rd_data;
module RegFile (
    input      [        4:0] rd,
    input      [`REG_SIZE:0] rd_data,
    input      [        4:0] rs1,
    output reg [`REG_SIZE:0] rs1_data,
    input      [        4:0] rs2,
    output reg [`REG_SIZE:0] rs2_data,
    input                    clk,
    input                    we,
    input                    rst
);
  localparam NumRegs = 32;
  wire [`REG_SIZE:0] regs[0:NumRegs-1];
  // TODO: your code here
  //patch: state-holding real register file
  reg [`REG_SIZE:0] shadow_regs[0:NumRegs-1];
  integer i;
  //reset: zero all registers except x0 (which must always remain 0 anyway)
  always @(posedge clk) begin
    if (rst) begin
      for (i=0; i<NumRegs; i=i+1)
        shadow_regs[i] <= 32'd0;
    end else begin
      if (we && rd != 0)  //this guarantee x0 is always zero
        shadow_regs[rd] <= rd_data;
    end
  end
  //aync read (the cpu can read registers instantly)
  always @(*) begin
    rs1_data = (rs1 == 0) ? 32'd0 : shadow_regs[rs1];
    rs2_data = (rs2 == 0) ? 32'd0 : shadow_regs[rs2];
  end
  // drive skeleton wires
  generate
    genvar k;
    for (k=0; k<NumRegs; k=k+1) begin : MAP
      assign regs[k] = shadow_regs[k];
    end
  endgenerate
endmodule


module DatapathPipelined (
  input                     clk,
  input                     rst,
  output     [ `REG_SIZE:0] pc_to_imem,
  input      [`INST_SIZE:0] inst_from_imem,
  output reg [ `REG_SIZE:0] addr_to_dmem,
  input      [ `REG_SIZE:0] load_data_from_dmem,
  output reg [ `REG_SIZE:0] store_data_to_dmem,
  output reg [         3:0] store_we_to_dmem,
  output reg                halt,
  output reg [ `REG_SIZE:0] trace_writeback_pc,
  output reg [`INST_SIZE:0] trace_writeback_inst,
  output reg [31:0]         trace_writeback_cycle_status
);
  localparam [`OPCODE_SIZE:0] OpcodeLoad    = 7'b00_000_11;
  localparam [`OPCODE_SIZE:0] OpcodeStore   = 7'b01_000_11;
  localparam [`OPCODE_SIZE:0] OpcodeBranch  = 7'b11_000_11;
  localparam [`OPCODE_SIZE:0] OpcodeJalr    = 7'b11_001_11;
  localparam [`OPCODE_SIZE:0] OpcodeJal     = 7'b11_011_11;
  localparam [`OPCODE_SIZE:0] OpcodeRegImm  = 7'b00_100_11;
  localparam [`OPCODE_SIZE:0] OpcodeRegReg  = 7'b01_100_11;
  localparam [`OPCODE_SIZE:0] OpcodeEnviron = 7'b11_100_11;
  localparam [`OPCODE_SIZE:0] OpcodeAuipc   = 7'b00_101_11;
  localparam [`OPCODE_SIZE:0] OpcodeLui     = 7'b01_101_11;

  reg [`REG_SIZE:0] cycles_current;
  always @(posedge clk or posedge rst) begin
    if (rst) cycles_current <= 0;
    else cycles_current <= cycles_current + 1;
  end

  // --- Pipeline Registers ---
  // F -> D
  reg [`REG_SIZE:0]  d_pc;
  reg [`INST_SIZE:0] d_inst;
  reg                d_valid;

  // D -> X
  reg [`REG_SIZE:0]  x_pc;
  reg [`INST_SIZE:0] x_inst;
  reg                x_valid;
  reg [4:0]          x_rs1_idx, x_rs2_idx, x_rd_idx;
  reg [`REG_SIZE:0]  x_rs1_val_base, x_rs2_val_base;
  reg                x_is_branch, x_is_jal, x_is_jalr;
  reg                x_is_lui, x_is_auipc, x_is_load, x_is_store;
  reg                x_is_regimm, x_is_regreg, x_is_ecall;
  reg [2:0]          x_funct3;
  reg [6:0]          x_funct7;
  reg [`REG_SIZE:0]  x_imm_i_sext, x_imm_s_sext, x_imm_b_sext, x_imm_j_sext;
  reg [4:0]          x_imm_shamt;

  // Divider Shadow Pipeline (8 Stages: 0..7)
  reg [4:0]          div_p_rd      [0:7];
  reg                div_p_valid   [0:7];
  reg                div_p_rem     [0:7];
  reg                div_p_regwrite[0:7];
  reg                div_p_quot_inv[0:7];
  reg                div_p_rem_inv [0:7];
  reg [`REG_SIZE:0]  div_p_pc      [0:7];
  reg [`INST_SIZE:0] div_p_inst    [0:7];

  // X -> M
  reg [`REG_SIZE:0]  m_pc;
  reg [`INST_SIZE:0] m_inst;
  reg                m_valid;
  reg [4:0]          m_rd_idx;
  reg                m_regwrite, m_memread, m_memwrite;
  reg [3:0]          m_mem_we;
  reg [`REG_SIZE:0]  m_alu_result, m_store_data, m_result;

  // M -> W
  reg [`REG_SIZE:0]  w_pc;
  reg [`INST_SIZE:0] w_inst;
  reg                w_valid;
  reg [4:0]          w_rd_idx_r;
  reg                w_regwrite_r, w_memread;
  reg [`REG_SIZE:0]  w_alu_result;
  reg [2:0]          w_funct3;
  reg [`REG_SIZE:0]  w_result;

  // --- Forward Declarations ---
  wire [`REG_SIZE:0] div_q_u_final, div_r_u_final;
  reg x_branch_taken;
  
  // --- DIVIDER LOGIC DETECTION ---
  wire x_is_div  = x_is_regreg && (x_funct7 == 7'd1) && (x_funct3 == 3'b100);
  wire x_is_divu = x_is_regreg && (x_funct7 == 7'd1) && (x_funct3 == 3'b101);
  wire x_is_rem  = x_is_regreg && (x_funct7 == 7'd1) && (x_funct3 == 3'b110);
  wire x_is_remu = x_is_regreg && (x_funct7 == 7'd1) && (x_funct3 == 3'b111);
  wire x_is_div_instr = x_is_div || x_is_divu || x_is_rem || x_is_remu;

  //---------------------------------
  // --- HAZARD CONTROL LOGIC -------
  //---------------------------------

  // 1. Load-Use Hazard
  // If X-stage has a LOAD that writes rd, and Decode (D) is reading
  // that same register as rs1 or rs2 in the *very next* cycle,
  // we must stall one cycle so data can be forwarded from M/W.
  wire x_load_use_hazard =
    x_valid && x_is_load && (x_rd_idx != 0) && d_valid &&
    ((x_rd_idx == d_rs1_idx) || (x_rd_idx == d_rs2_idx));

  wire stall_load = x_load_use_hazard;
  
  // 2. Divider Data Hazard 
  // Stall if Decode is trying to read a register that will be written
  // by an in-flight DIV/REM in the divider pipeline.
  reg stall_div_data;
  integer h;
  always @(*) begin
    stall_div_data = 1'b0;
    if (d_valid) begin
      // Check all divider stages except last 2 (those will be handled
      // by forwarding / structural logic later).
      for (h = 0; h < `DIVIDER_STAGES-2; h = h + 1) begin
        if (div_p_valid[h] && div_p_regwrite[h] && (div_p_rd[h] != 0)) begin
          if ((div_p_rd[h] == d_rs1_idx) || (div_p_rd[h] == d_rs2_idx)) begin
            stall_div_data = 1'b1;  // RAW hazard with DIV result
          end
        end
      end
      // Also check the X-stage DIV that has just started but not yet
      // inserted into the divider pipeline (write-after-read at Decode).
      if (x_valid && x_is_div_instr && (x_rd_idx != 0)) begin
        if ((x_rd_idx == d_rs1_idx) || (x_rd_idx == d_rs2_idx)) begin
          stall_div_data = 1'b1;
        end
      end
    end
  end

  // 3. Structural Hazard: Divider result ready at Stage 7.
  // When the divider's last stage produces a result (div_result_ready),
  // we may need to give priority to its writeback.
  wire div_result_ready     = div_p_valid[7];
  wire struct_hazard_div_wb = div_result_ready && x_valid && m_regwrite;
  // (Interpretation: in this design we treat "DIV result wants WB while
  // another instruction also writes" as a structural hazard and stall.)

  // Check if DIV currently in the divider pipeline before WB
  integer dh;
  reg div_busy_before_wb;

  always @(*) begin
    div_busy_before_wb = 1'b0;
    // Stages 0..DIVIDER_STAGES-3: DIV is still computing, not at output yet
    for (dh = 0; dh < `DIVIDER_STAGES-3; dh = dh + 1) begin
      if (div_p_valid[dh] && div_p_regwrite[dh] && (div_p_rd[dh] != 5'd0))
        div_busy_before_wb = 1'b1;
    end
    // Also count the DIV that just started in X (not yet shifted into pipe)
    if (x_valid && x_is_div_instr && (x_rd_idx != 5'd0))
      div_busy_before_wb = 1'b1;
  end

  // --- DIV writeback latch for forwarding ---
  // We keep a one-cycle-late copy of DIV result at W stage so that
  // EX-stage forwarding can see it even if DIV is finishing.
  reg               div_w_valid;
  reg [4:0]         div_w_rd;
  reg [`REG_SIZE:0] div_w_data;

  // 2b. Divider "ordering" hazard for independent (non-DIV) instructions
  // If Decode has a non-DIV instruction, but there is *any* DIV still
  // in-flight before the writeback stage, stall to preserve ordering
  // (avoid younger instruction committing before DIV does).
  wire stall_div_indep =
    d_valid && !d_is_div_instr && !stall_div_data && div_busy_before_wb;

  // 4. Global Stall Signal:
  // Any of these conditions will freeze PC and F→D, and also cause
  // D→X to insert bubbles as needed.
  wire stall_wb =
    stall_load      ||  // classic load-use hazard
    stall_div_data  ||  // RAW hazard with DIV result
    stall_div_indep ||  // ordering constraint w.r.t. DIV
    struct_hazard_div_wb; // structural DIV writeback hazard

  // Enable PC and F->D pipeline register:
  // If there's any stall, do NOT advance PC nor latch new instruction.
  wire pc_en = !stall_wb;
  wire fd_en = !stall_wb;

  // X->M and M stage only frozen by DIV writeback structural hazard.
  // So normal stalls (load-use / data hazards) are resolved between
  // F/D/X, while M keeps going unless DIV WB conflicts.
  wire dx_en = !struct_hazard_div_wb;
  wire xm_en = !struct_hazard_div_wb;

  // Flush D->X only when:
  //  - inserting a bubble for load-use hazard
  //  - inserting a bubble for DIV data / independent hazard
  //  - redirect due to taken branch or jump
  wire dx_clear =
    stall_load ||
    stall_div_data ||
    stall_div_indep || 
    (x_valid && (x_is_jal || x_is_jalr || (x_is_branch && x_branch_taken)));

  // --- FETCH STAGE ---
  reg  [`REG_SIZE:0]  f_pc;
  wire [`INST_SIZE:0] f_inst;
  reg                 redirect_pc;
  reg  [`REG_SIZE:0]  redirect_pc_target;

  always @(posedge clk or posedge rst) begin
    if (rst)
      f_pc <= 32'd0;
    else if (pc_en) begin
      // On redirect (branch/jump), override PC with target.
      if (redirect_pc)
        f_pc <= redirect_pc_target;
      else
        f_pc <= f_pc + 4;  // normal sequential fetch
    end
  end

  assign pc_to_imem = f_pc;
  assign f_inst     = inst_from_imem;

  // --- F -> D REGISTER ---
  reg flush_fd; 
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      d_pc <= 0; d_inst <= 0; d_valid <= 0;
    end else if (flush_fd) begin
      d_pc <= 0; d_inst <= 0; d_valid <= 0;
    end else if (fd_en) begin
      d_pc <= f_pc; d_inst <= f_inst; d_valid <= 1'b1;
    end
  end

  // --- DECODE STAGE ---
  // Writeback interface coming from W stage:
  //   w_rd_idx   : destination register index
  //   w_regwrite : whether W will write to RF
  //   w_wdata    : data to be written into RF
  wire [4:0]         w_rd_idx;
  wire               w_regwrite;
  wire [`REG_SIZE:0] w_wdata;

  // Break the decode-stage instruction (d_inst) into fields:
  //   [31:25] funct7
  //   [24:20] rs2
  //   [19:15] rs1
  //   [14:12] funct3
  //   [11:7]  rd
  //   [6:0]   opcode
  wire [6:0] d_funct7;
  wire [4:0] d_rs2_idx, d_rs1_idx, d_rd_idx;
  wire [2:0] d_funct3;
  wire [`OPCODE_SIZE:0] d_opcode;
  assign {d_funct7, d_rs2_idx, d_rs1_idx, d_funct3, d_rd_idx, d_opcode} = d_inst;

  // Immediate fields (raw bit-slices) for different instruction types
  wire [11:0] d_imm_i     = d_inst[31:20];               // I-type
  wire [4:0]  d_imm_shamt = d_inst[24:20];               // shift amount
  wire [11:0] d_imm_s     = {d_funct7, d_rd_idx};        // S-type (store)
  wire [12:0] d_imm_b;                                   // B-type (branch)
  assign {d_imm_b[12], d_imm_b[10:1], d_imm_b[11], d_imm_b[0]} =
         {d_funct7, d_rd_idx, 1'b0};

  wire [20:0] d_imm_j;                                   // J-type (JAL)
  assign {d_imm_j[20], d_imm_j[10:1], d_imm_j[11], d_imm_j[19:12], d_imm_j[0]} =
         {d_inst[31:12], 1'b0};

  // Sign-extended immediates up to 32 bits
  wire [`REG_SIZE:0] d_imm_i_sext =
         {{20{d_imm_i[11]}}, d_imm_i[11:0]};             // I-type
  wire [`REG_SIZE:0] d_imm_s_sext =
         {{20{d_imm_s[11]}}, d_imm_s[11:0]};             // S-type
  wire [`REG_SIZE:0] d_imm_b_sext =
         {{19{d_imm_b[12]}}, d_imm_b[12:0]};             // B-type
  wire [`REG_SIZE:0] d_imm_j_sext =
         {{11{d_imm_j[20]}}, d_imm_j[20:0]};             // J-type

  // High-level instruction type decoding based on opcode
  wire d_is_branch = (d_opcode == OpcodeBranch);
  wire d_is_jal    = (d_opcode == OpcodeJal);
  wire d_is_jalr   = (d_opcode == OpcodeJalr);
  wire d_is_lui    = (d_opcode == OpcodeLui);
  wire d_is_auipc  = (d_opcode == OpcodeAuipc);
  wire d_is_load   = (d_opcode == OpcodeLoad);
  wire d_is_store  = (d_opcode == OpcodeStore);
  wire d_is_regimm = (d_opcode == OpcodeRegImm);  // op-imm
  wire d_is_regreg = (d_opcode == OpcodeRegReg);  // op
  wire d_is_ecall  = (d_opcode == OpcodeEnviron) & (d_inst[31:7] == 0);

  // --------------------------------------------------
  // Detect DIV / REM family at Decode
  //   d_is_regreg + funct7 == 1 + specific funct3
  // --------------------------------------------------
  wire d_is_div  = d_is_regreg && (d_funct7 == 7'd1) && (d_funct3 == 3'b100); // DIV
  wire d_is_divu = d_is_regreg && (d_funct7 == 7'd1) && (d_funct3 == 3'b101); // DIVU
  wire d_is_rem  = d_is_regreg && (d_funct7 == 7'd1) && (d_funct3 == 3'b110); // REM
  wire d_is_remu = d_is_regreg && (d_funct7 == 7'd1) && (d_funct3 == 3'b111); // REMU
  wire d_is_div_instr = d_is_div || d_is_divu || d_is_rem || d_is_remu;

  // --------------------------------------------------
  // Register File Read + Writeback Bypass
  // --------------------------------------------------
  // rf_rs1_data / rf_rs2_data : raw outputs from the RegFile
  // d_rs1_val / d_rs2_val     : actual values used in Decode,
  //                             possibly overridden by W-stage forwarding.
  wire [`REG_SIZE:0] rf_rs1_data;
  wire [`REG_SIZE:0] rf_rs2_data;

  // If W stage is writing the same register that Decode is reading (rs1),
  // bypass W data directly (classic WB→ID forwarding).
  wire [`REG_SIZE:0] d_rs1_val =
    ((w_regwrite & w_valid) && (w_rd_idx != 0) && (w_rd_idx == d_rs1_idx))
      ? w_wdata
      : rf_rs1_data;

  // Same logic for rs2
  wire [`REG_SIZE:0] d_rs2_val =
    ((w_regwrite & w_valid) && (w_rd_idx != 0) && (w_rd_idx == d_rs2_idx))
      ? w_wdata
      : rf_rs2_data;

  // Central register file:
  //   - Write: from W stage (w_rd_idx, w_wdata, w_regwrite & w_valid)
  //   - Read : at Decode (d_rs1_idx, d_rs2_idx)
  RegFile rf (
    .rd       (w_rd_idx),
    .rd_data  (w_wdata),
    .rs1      (d_rs1_idx),
    .rs1_data (rf_rs1_data),
    .rs2      (d_rs2_idx),
    .rs2_data (rf_rs2_data),
    .clk      (clk),
    .we       (w_regwrite & w_valid),
    .rst      (rst)
  );

  // --- D -> X REGISTER ---
  // Latches all decode outputs into the Execute stage (X) on dx_en.
  // If dx_clear is asserted (e.g., due to hazard), we inject a bubble.
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      // On reset: clear all X-stage registers
      x_pc <= 0; x_inst <= 0; x_valid <= 0;
      x_rs1_idx <= 0; x_rs2_idx <= 0; x_rd_idx <= 0;
      x_rs1_val_base <= 0; x_rs2_val_base <= 0;
      x_is_branch <= 0; x_is_jal <= 0; x_is_jalr <= 0;
      x_is_lui <= 0; x_is_auipc <= 0; x_is_load <= 0; x_is_store <= 0;
      x_is_regimm <= 0; x_is_regreg <= 0; x_is_ecall <= 0;
      x_funct3 <= 0; x_funct7 <= 0;
      x_imm_i_sext <= 0; x_imm_s_sext <= 0;
      x_imm_b_sext <= 0; x_imm_j_sext <= 0; x_imm_shamt <= 0;
    end 
    // Only update D→X if pipeline allows (dx_en asserted)
    else if (dx_en) begin 
      if (dx_clear) begin
        // Insert bubble: instruction becomes invalid in X
        x_pc <= 0; x_inst <= 0; x_valid <= 0;
        x_rs1_idx <= 0; x_rs2_idx <= 0; x_rd_idx <= 0;
        x_is_branch <= 0; x_is_jal <= 0; x_is_jalr <= 0;
        x_is_load <= 0; x_is_store <= 0;
        x_is_regimm <= 0; x_is_regreg <= 0; x_is_ecall <= 0;
        // (We don’t strictly need to clear all imm / funct fields here,
        //  but clearing is safer and keeps waveforms clean.)
      end else begin
        // Normal D→X register transfer
        x_pc        <= d_pc;
        x_inst      <= d_inst;
        x_valid     <= d_valid;

        x_rs1_idx   <= d_rs1_idx;
        x_rs2_idx   <= d_rs2_idx;
        x_rd_idx    <= d_rd_idx;

        // Base RS1/RS2 values, before EX-stage forwarding
        x_rs1_val_base <= d_rs1_val;
        x_rs2_val_base <= d_rs2_val;
        
        // Pass along decoded instruction type flags
        x_is_branch <= d_is_branch;
        x_is_jal    <= d_is_jal;
        x_is_jalr   <= d_is_jalr;
        x_is_lui    <= d_is_lui;
        x_is_auipc  <= d_is_auipc;
        x_is_load   <= d_is_load;
        x_is_store  <= d_is_store;
        x_is_regimm <= d_is_regimm;
        x_is_regreg <= d_is_regreg;
        x_is_ecall  <= d_is_ecall;

        // Pass along funct3/funct7 and all sign-extended immediates
        x_funct3      <= d_funct3;
        x_funct7      <= d_funct7;
        x_imm_i_sext  <= d_imm_i_sext;
        x_imm_s_sext  <= d_imm_s_sext;
        x_imm_b_sext  <= d_imm_b_sext;
        x_imm_j_sext  <= d_imm_j_sext;
        x_imm_shamt   <= d_imm_shamt;
      end
    end
  end

  // --- EXECUTE STAGE ---
  // x_rs1_val / x_rs2_val are the *final* source operand values at EX stage,
  // after applying forwarding from:
  //   - divider pipeline stage 7 (div_p_*[7])
  //   - divider writeback shadow (div_w_*)
  //   - normal W stage
  //   - M stage (when result is already computed and not a load)
  reg [`REG_SIZE:0] x_rs1_val, x_rs2_val;

  // Final result of divider at stage 7:
  //   - if div_p_rem[7] == 1 → use remainder
  //   - else                  → use quotient
  wire [`REG_SIZE:0] div_result_stage7 =
         div_p_rem[7] ? div_r_u_final : div_q_u_final;

  //====================================================
  // FORWARDING LOGIC FOR RS1 / RS2 INTO EX STAGE
  //====================================================
  always @(*) begin
    // Default: use base values from RF or ID stage
    x_rs1_val = x_rs1_val_base;
    x_rs2_val = x_rs2_val_base;

    // ---------- RS1 forwarding priority chain ----------
    // 1) From divider pipeline stage 7 (just before WB)
    if (div_p_valid[7] && div_p_regwrite[7] &&
        div_p_rd[7] == x_rs1_idx && div_p_rd[7] != 0) begin
      x_rs1_val = div_result_stage7;

    // 2) From divider writeback shadow (div_w_*)
    end else if (div_w_valid &&
                 div_w_rd == x_rs1_idx && div_w_rd != 0) begin
      x_rs1_val = div_w_data;

    // 3) From normal W stage result
    end else if (w_valid && w_regwrite_r &&
                 w_rd_idx_r == x_rs1_idx && w_rd_idx_r != 0) begin
      x_rs1_val = w_result;

    // 4) From M stage ALU result (when not a load → data already available)
    end else if (m_valid && m_regwrite &&
                 m_rd_idx == x_rs1_idx && m_rd_idx != 0 && !m_memread) begin
      x_rs1_val = m_result;
    end

    // ---------- RS2 forwarding priority chain ----------
    // Same structure as RS1 but for x_rs2_idx
    if (div_p_valid[7] && div_p_regwrite[7] &&
        div_p_rd[7] == x_rs2_idx && div_p_rd[7] != 0) begin
      x_rs2_val = div_result_stage7;

    end else if (div_w_valid &&
                 div_w_rd == x_rs2_idx && div_w_rd != 0) begin
      x_rs2_val = div_w_data;

    end else if (w_valid && w_regwrite_r &&
                 w_rd_idx_r == x_rs2_idx && w_rd_idx_r != 0) begin
      x_rs2_val = w_result;

    end else if (m_valid && m_regwrite &&
                 m_rd_idx == x_rs2_idx && m_rd_idx != 0 && !m_memread) begin
      x_rs2_val = m_result;
    end
  end

  //====================================================
  // ALU ADD/SUB (carry lookahead adder)
  //====================================================
  reg  [`REG_SIZE:0] alu_a, alu_b;
  reg                alu_cin;
  wire [`REG_SIZE:0] alu_sum;

  // Reusable CLA module for all additions/subtractions
  cla alu_addsub (
    .a   (alu_a),
    .b   (alu_b),
    .cin (alu_cin),
    .sum (alu_sum)
  );

  //====================================================
  // UNSIGNED Pipelined Divider Core
  //   - div_a / div_b are driven in EX
  //   - outputs div_q_u / div_r_u are unsigned
  //   - sign handling is done externally using rs*_abs and div_p_* flags
  //====================================================
  reg  [`REG_SIZE:0] div_a, div_b;
  wire [`REG_SIZE:0] div_q_u, div_r_u;

  DividerUnsignedPipelined u_divu (
    .clk        (clk),
    .rst        (rst),
    .stall      (1'b0),        // always running, we handle hazards separately
    .i_dividend (div_a),
    .i_divisor  (div_b),
    .o_remainder(div_r_u),
    .o_quotient (div_q_u)
  );

  //====================================================
  // SIGN / ABSOLUTE VALUE HANDLING FOR SIGNED DIV/REM
  //   rs*_abs = |rs*|
  //   rs1_sign / rs2_sign indicate negativity of original operands
  //====================================================
  wire rs1_sign = x_rs1_val[31];
  wire rs2_sign = x_rs2_val[31];

  wire [`REG_SIZE:0] rs1_abs =
        rs1_sign ? (~x_rs1_val + 1) : x_rs1_val;
  wire [`REG_SIZE:0] rs2_abs =
        rs2_sign ? (~x_rs2_val + 1) : x_rs2_val;

  //====================================================
  // DIV Start Logic
  // Only start divider when:
  //   - EX stage instruction is valid
  //   - It is some DIV/REM instruction (x_is_div_instr)
  //   - There is no structural hazard with divider writeback
  //====================================================
  wire div_start =
       x_valid && x_is_div_instr && !struct_hazard_div_wb;

  //====================================================
  // 8-STAGE DIVIDER PIPELINE META-DATA
  // div_p_*[k] carry:
  //   - valid      : whether stage k holds a DIV/REM op
  //   - rd         : destination register index
  //   - rem        : 1 if this op writes remainder, 0 for quotient
  //   - regwrite   : whether we should write back to rd
  //   - quot_inv   : whether quotient must be negated at stage 7
  //   - rem_inv    : whether remainder must be negated at stage 7
  //   - pc, inst   : original PC / instruction (for debugging or WB info)
  //====================================================
  integer k;
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      // On reset, clear all 0..6 stages of the DIV pipeline
      for (k = 0; k < 7; k = k + 1) begin
        div_p_valid[k]    <= 0;
        div_p_rd[k]       <= 0;
        div_p_rem[k]      <= 0;
        div_p_regwrite[k] <= 0;
        div_p_quot_inv[k] <= 0;
        div_p_rem_inv[k]  <= 0;
        div_p_pc[k]       <= 0;
        div_p_inst[k]     <= 0;
      end
    end else begin
      // Shift pipeline: stage k takes contents from stage k-1
      for (k = 7; k > 0; k = k - 1) begin
        div_p_valid[k]    <= div_p_valid[k-1];
        div_p_rd[k]       <= div_p_rd[k-1];
        div_p_rem[k]      <= div_p_rem[k-1];
        div_p_regwrite[k] <= div_p_regwrite[k-1];
        div_p_quot_inv[k] <= div_p_quot_inv[k-1];
        div_p_rem_inv[k]  <= div_p_rem_inv[k-1];
        div_p_pc[k]       <= div_p_pc[k-1];
        div_p_inst[k]     <= div_p_inst[k-1];
      end

      // Stage 0: inject a new DIV/REM op when div_start is asserted
      if (div_start) begin
        div_p_valid[0]    <= 1'b1;
        div_p_rd[0]       <= x_rd_idx;
        div_p_rem[0]      <= (x_is_rem || x_is_remu);  // 1 = REM/REMU, 0 = DIV/DIVU
        div_p_regwrite[0] <= 1'b1;

        // For signed DIV/REM, decide if quotient needs negation:
        //   - if divisor == 0 → special case, hardware will handle
        //   - else use rs1_sign ^ rs2_sign for quotient sign
        div_p_quot_inv[0] <= (x_is_div || x_is_rem) ?
                             ((x_rs2_val == 0) ? 1'b0 : (rs1_sign ^ rs2_sign)) : 1'b0;

        // For signed DIV/REM, remainder sign is same as dividend sign
        div_p_rem_inv[0]  <= (x_is_div || x_is_rem) ? rs1_sign : 1'b0;

        div_p_pc[0]       <= x_pc;
        div_p_inst[0]     <= x_inst;
      end
      // If no new DIV op this cycle, stage 0 is cleared (bubble)
      else begin
        div_p_valid[0]    <= 1'b0;
        div_p_rd[0]       <= 5'd0;
        div_p_rem[0]      <= 1'b0;
        div_p_regwrite[0] <= 1'b0;
        div_p_quot_inv[0] <= 1'b0;
        div_p_rem_inv[0]  <= 1'b0;
        div_p_pc[0]       <= {(`REG_SIZE+1){1'b0}};
        div_p_inst[0]     <= {(`REG_SIZE+1){1'b0}};
      end
    end
  end

  //====================================================
  //   - div_q_u / div_r_u are always unsigned from core
  //   - div_p_quot_inv[7] / div_p_rem_inv[7] tell us to negate
  //     (two’s complement) for signed DIV/REM cases
  //====================================================
  assign div_q_u_final =
         div_p_quot_inv[7] ? (~div_q_u + 32'd1) : div_q_u;

  assign div_r_u_final =
         div_p_rem_inv[7]  ? (~div_r_u + 32'd1) : div_r_u;

  //====================================================
  // ALU / EX STAGE CONTROL
  // - Computes ALU result, branch decision/target
  // - Sets regwrite / memread / memwrite / mem_we / store_data
  // - Prepares operands for MUL/DIV
  //====================================================
  reg [`REG_SIZE:0] x_alu_result;
  reg [`REG_SIZE:0] x_branch_target;
  reg               x_regwrite;
  reg               x_memread;
  reg               x_memwrite;
  reg [3:0]         x_mem_we;
  reg [`REG_SIZE:0] x_store_data;
  reg [63:0]        mul_res;

  always @(*) begin
    // ---------- Default values (safe NOP behaviour) ----------
    x_alu_result    = 0;
    x_branch_taken  = 0;
    x_branch_target = 0;
    x_regwrite      = 0;
    x_memread       = 0;
    x_memwrite      = 0;
    x_mem_we        = 0;
    x_store_data    = 0;

    // Default ALU / DIV operands
    alu_a           = 0;
    alu_b           = 0;
    alu_cin         = 0;
    div_a           = 0;
    div_b           = 1;      // avoid div-by-0 by default

    mul_res         = 64'd0;  // temporary for MULH/MULHSU/MULHU

    // Only decode / drive outputs when EX stage holds a valid instruction
    if (x_valid) begin
      // ------------------------------
      // LUI: x_rd = imm[31:12] << 12
      // ------------------------------
      if (x_is_lui) begin
        x_alu_result = {x_inst[31:12], 12'd0};
        x_regwrite   = 1;
      end

      // ------------------------------
      // AUIPC: x_rd = x_pc + imm << 12
      // ------------------------------
      else if (x_is_auipc) begin
        x_alu_result = x_pc + {x_inst[31:12], 12'd0};
        x_regwrite   = 1;
      end

      // ------------------------------
      // JAL: x_rd = x_pc + 4; jump target = x_pc + imm_j
      // ------------------------------
      else if (x_is_jal) begin
        x_alu_result    = x_pc + 4;              // link
        x_regwrite      = 1;
        x_branch_taken  = 1;                     // control-flow change
        x_branch_target = x_pc + x_imm_j_sext;   // JAL target
      end

      // ------------------------------
      // JALR: x_rd = x_pc + 4; jump to (rs1 + imm_i) & ~1
      // ------------------------------
      else if (x_is_jalr) begin
        x_alu_result    = x_pc + 4;                                // link
        x_regwrite      = 1;
        x_branch_taken  = 1;
        x_branch_target = (x_rs1_val + x_imm_i_sext) & ~1;         // aligned target
      end

      // ------------------------------
      // BRANCH instructions (BEQ, BNE, BLT, BGE, BLTU, BGEU)
      // ------------------------------
      else if (x_is_branch) begin
        // Decide if branch is taken based on funct3 and operands
        if      (x_funct3 == 3'b000) x_branch_taken = (x_rs1_val ==  x_rs2_val);                  // BEQ
        else if (x_funct3 == 3'b001) x_branch_taken = (x_rs1_val !=  x_rs2_val);                  // BNE
        else if (x_funct3 == 3'b100) x_branch_taken = ($signed(x_rs1_val) <  $signed(x_rs2_val)); // BLT
        else if (x_funct3 == 3'b101) x_branch_taken = ($signed(x_rs1_val) >= $signed(x_rs2_val)); // BGE
        else if (x_funct3 == 3'b110) x_branch_taken = (x_rs1_val <  x_rs2_val);                   // BLTU
        else if (x_funct3 == 3'b111) x_branch_taken = (x_rs1_val >= x_rs2_val);                   // BGEU
        else                          x_branch_taken = 1'b0;

        // If taken, branch target = PC + B-immediate
        if (x_branch_taken)
          x_branch_target = x_pc + x_imm_b_sext;
      end

      // ------------------------------
      // LOAD (LB/LH/LW/LBU/LHU)
      // ------------------------------
      else if (x_is_load) begin
        x_memread   = 1;           // read from memory
        x_regwrite  = 1;           // write back loaded value in W-stage
        alu_a       = x_rs1_val;   // base address
        alu_b       = x_imm_i_sext;// offset
        alu_cin     = 0;
        x_alu_result= alu_sum;     // effective address
      end

      // ------------------------------
      // STORE (SB/SH/SW)
      // ------------------------------
      else if (x_is_store) begin
        x_memwrite  = 1;                       // this is a store
        alu_a       = x_rs1_val;               // base address
        alu_b       = x_imm_s_sext;            // offset
        alu_cin     = 0;
        x_alu_result= alu_sum;                 // effective address

        // Align store data to the correct byte lane
        x_store_data = x_rs2_val << (x_alu_result[1:0] * 8);

        // Set byte-enable (write enable) based on size and address alignment
        if      (x_funct3 == 3'b000) // SB
          x_mem_we = 4'b0001 << x_alu_result[1:0];
        else if (x_funct3 == 3'b001) // SH
          x_mem_we = 4'b0011 << (x_alu_result[1] * 2);
        else if (x_funct3 == 3'b010) // SW
          x_mem_we = 4'b1111;
        else
          x_mem_we = 4'b0000;       // illegal / unsupported store
      end

      // ------------------------------
      // OP-IMM (Register–Immediate ALU operations)
      // ------------------------------
      else if (x_is_regimm) begin
        x_regwrite = 1; // all OP-IMM instructions write rd

        // ADDI
        if (x_funct3 == 3'b000) begin
          alu_a        = x_rs1_val;
          alu_b        = x_imm_i_sext;
          alu_cin      = 0;
          x_alu_result = alu_sum;
        end
        // SLTI
        else if (x_funct3 == 3'b010)
          x_alu_result = ($signed(x_rs1_val) < $signed(x_imm_i_sext)) ? 1 : 0;
        // SLTIU
        else if (x_funct3 == 3'b011)
          x_alu_result = (x_rs1_val < x_imm_i_sext) ? 1 : 0;
        // XORI
        else if (x_funct3 == 3'b100)
          x_alu_result = x_rs1_val ^ x_imm_i_sext;
        // ORI
        else if (x_funct3 == 3'b110)
          x_alu_result = x_rs1_val | x_imm_i_sext;
        // ANDI
        else if (x_funct3 == 3'b111)
          x_alu_result = x_rs1_val & x_imm_i_sext;
        // SLLI
        else if (x_funct3 == 3'b001)
          x_alu_result = x_rs1_val << x_imm_shamt;
        // SRLI / SRAI (funct7 distinguishes)
        else if (x_funct3 == 3'b101) begin
          if (x_funct7[5]) // SRAI
            x_alu_result = $signed(x_rs1_val) >>> x_imm_shamt;
          else             // SRLI
            x_alu_result = x_rs1_val >> x_imm_shamt;
        end
      end

      // ------------------------------
      // OP (Register–Register ALU & M-extension)
      // ------------------------------
      else if (x_is_regreg) begin
        x_regwrite = 1; // all ALU/M instructions write rd

        // ---------- RV32I ALU operations ----------
        // ADD
        if (x_funct7 == 0 && x_funct3 == 3'b000) begin
          alu_a        = x_rs1_val;
          alu_b        = x_rs2_val;
          alu_cin      = 0;
          x_alu_result = alu_sum;
        end
        // SUB (ADD with ~rs2 + 1)
        else if (x_funct7 == 7'b0100000 && x_funct3 == 3'b000) begin
          alu_a        = x_rs1_val;
          alu_b        = ~x_rs2_val;
          alu_cin      = 1;
          x_alu_result = alu_sum;
        end
        // SLL
        else if (x_funct7 == 0 && x_funct3 == 3'b001)
          x_alu_result = x_rs1_val << x_rs2_val[4:0];
        // SLT (signed)
        else if (x_funct7 == 0 && x_funct3 == 3'b010)
          x_alu_result = ($signed(x_rs1_val) < $signed(x_rs2_val)) ? 1 : 0;
        // SLTU (unsigned)
        else if (x_funct7 == 0 && x_funct3 == 3'b011)
          x_alu_result = (x_rs1_val < x_rs2_val) ? 1 : 0;
        // XOR
        else if (x_funct7 == 0 && x_funct3 == 3'b100)
          x_alu_result = x_rs1_val ^ x_rs2_val;
        // SRL (logical right)
        else if (x_funct7 == 0 && x_funct3 == 3'b101)
          x_alu_result = x_rs1_val >> x_rs2_val[4:0];
        // SRA (arithmetic right)
        else if (x_funct7 == 7'b0100000 && x_funct3 == 3'b101)
          x_alu_result = $signed(x_rs1_val) >>> x_rs2_val[4:0];
        // OR
        else if (x_funct7 == 0 && x_funct3 == 3'b110)
          x_alu_result = x_rs1_val | x_rs2_val;
        // AND
        else if (x_funct7 == 0 && x_funct3 == 3'b111)
          x_alu_result = x_rs1_val & x_rs2_val;

        // ---------- RV32M (Multiply / Divide) ----------
        // MUL: low 32 bits of rs1 * rs2
        // (for 32-bit values, signed* signed == unsigned*unsigned in low part)
        else if (x_funct7 == 1 && x_funct3 == 3'b000) begin
          x_alu_result = x_rs1_val * x_rs2_val;
        end

        // MULH: high 32 bits of signed × signed
        else if (x_funct7 == 1 && x_funct3 == 3'b001) begin
          mul_res      = $signed({{32{x_rs1_val[31]}}, x_rs1_val}) *
                         $signed({{32{x_rs2_val[31]}}, x_rs2_val});
          x_alu_result = mul_res[63:32];
        end

        // MULHSU: high 32 bits of signed × unsigned
        else if (x_funct7 == 1 && x_funct3 == 3'b010) begin
          mul_res      = $signed({{32{x_rs1_val[31]}}, x_rs1_val}) *
                         {32'd0, x_rs2_val};
          x_alu_result = mul_res[63:32];
        end

        // MULHU: high 32 bits of unsigned × unsigned
        else if (x_funct7 == 1 && x_funct3 == 3'b011) begin
          mul_res      = {32'd0, x_rs1_val} * {32'd0, x_rs2_val};
          x_alu_result = mul_res[63:32];
        end

        // ---------- DIV / REM: set up divider inputs ----------
        // Signed DIV/REM: use absolute values (rs1_abs, rs2_abs) for core
        else if (x_funct7 == 1 && (x_funct3 == 3'b100 || x_funct3 == 3'b110)) begin
          div_a = rs1_abs;
          div_b = rs2_abs;
        end
        // Unsigned DIVU/REMU: use operands as-is
        else if (x_funct7 == 1 && (x_funct3 == 3'b101 || x_funct3 == 3'b111)) begin
          div_a = x_rs1_val;
          div_b = x_rs2_val;
        end
      end
    end
  end


  //====================================================
  // Redirect / Flush Logic (Control Hazards)
  //====================================================
  always @(*) begin
    // Default: no redirect, no flush
    redirect_pc        = 0;
    redirect_pc_target = 0;
    flush_fd           = 0;

    // Only react to control-flow instructions when X-stage holds a valid instr
    if (x_valid) begin
      // If we see: JAL, JALR, or a taken BRANCH in the X stage,
      // then the PC must be redirected and the F/D stages flushed.
      if (x_is_jal || x_is_jalr || (x_is_branch && x_branch_taken)) begin
        redirect_pc        = 1;                 // signal fetch to use new PC
        redirect_pc_target = x_branch_target;   // computed target address
        flush_fd           = 1;                 // kill instructions in F/D
      end
    end
  end

  //====================================================
  // X -> M REGISTER (EXECUTE STAGE → MEMORY STAGE)
  //====================================================
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      // On reset, clear all M-stage pipeline registers
      m_pc         <= 0;
      m_inst       <= 0;
      m_valid      <= 0;
      m_rd_idx     <= 0;
      m_regwrite   <= 0;
      m_memread    <= 0;
      m_memwrite   <= 0;
      m_mem_we     <= 0;
      m_alu_result <= 0;
      m_store_data <= 0;
    end
    // Update X→M pipeline register only when enabled (no stall)
    else if (xm_en) begin
      // Special case: if current X-stage instruction is a DIV,
      // we inject a BUBBLE into M (DIV is handled by separate pipeline).
      if (x_valid && x_is_div_instr) begin
        m_pc       <= x_pc;      // preserve PC for debug / tracing
        m_inst     <= x_inst;    // preserve instruction word
        m_valid    <= 0;         // bubble (no useful work in M this cycle)
        m_regwrite <= 0;
        m_memread  <= 0;
        m_memwrite <= 0;
        m_mem_we   <= 0;
        // Note: ALU result & store data are irrelevant when bubble
      end else begin
        // Normal path: pass all relevant X-stage signals into M-stage
        m_pc         <= x_pc;
        m_inst       <= x_inst;
        m_valid      <= x_valid;
        m_rd_idx     <= x_rd_idx;
        m_regwrite   <= x_regwrite;
        m_memread    <= x_memread;
        m_memwrite   <= x_memwrite;
        m_mem_we     <= x_mem_we;
        m_alu_result <= x_alu_result;  // effective address or ALU result
        m_store_data <= x_store_data;  // value to be written to memory
      end
    end
  end

  //====================================================
  // MEMORY STAGE: Drive Data Memory Interface
  //====================================================
  always @(*) begin
    // Default outputs from M stage to data memory
    addr_to_dmem      = m_alu_result;  // address computed in X-stage
    store_we_to_dmem  = 0;             // default: no write
    store_data_to_dmem= 0;             // default: no data
    m_result          = m_alu_result;  // default result passed forward (ALU)

    if (m_valid) begin
      // If this is a store instruction, drive write-enable and write data
      if (m_memwrite) begin
        store_we_to_dmem   = m_mem_we;       // byte-enable signals for store
        store_data_to_dmem = m_store_data;   // data to be written
      end
      // For loads, read-data handling typically happens in W-stage using
      // load_data_from_dmem + funct3 (depending on your overall design).
    end
  end

  //====================================================
  // M -> W REGISTER (MEMORY STAGE → WRITEBACK STAGE)
  //====================================================

  // Expose internal writeback register fields to the rest of the core
  assign w_rd_idx   = w_rd_idx_r;    // destination register index (rd)
  assign w_regwrite = w_regwrite_r;  // writeback enable for this instruction

  // Pipeline register updating on clock / reset
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      // --- Global reset: clear all W-stage pipeline state ---
      w_pc         <= 0;
      w_inst       <= 0;
      w_valid      <= 0;
      w_rd_idx_r   <= 0;
      w_regwrite_r <= 0;
      w_memread    <= 0;
      w_alu_result <= 0;
      w_funct3     <= 0;

      // Reset divider writeback-tracking registers
      div_w_valid  <= 1'b0;
      div_w_rd     <= 5'd0;
      div_w_data   <= {(`REG_SIZE+1){1'b0}};
    end

    // -------------------------------------------------
    // 1) Divider result has finished: give it priority
    // -------------------------------------------------
    else if (div_result_ready) begin
      // Drive normal W-stage pipeline outputs from the divider pipeline
      w_valid      <= 1;
      w_regwrite_r <= div_p_regwrite[7];                  // writeback enable from last stage of div pipe
      w_rd_idx_r   <= div_p_rd[7];                        // destination register from div pipe
      w_alu_result <= div_p_rem[7] ?                      // select quotient or remainder
                      div_r_u_final : div_q_u_final;
      w_memread    <= 0;                                  // divider does not read memory
      w_pc         <= div_p_pc[7];                        // PC associated with this div op
      w_inst       <= div_p_inst[7];                      // original instruction word
      w_funct3     <= 0;                                  // not a load/store in this path

      // Also latch a “shadow” copy for WB hazard / forwarding logic
      div_w_valid  <= div_p_regwrite[7];
      div_w_rd     <= div_p_rd[7];
      div_w_data   <= div_p_rem[7] ? 
                      div_r_u_final : div_q_u_final;
    end

    // -------------------------------------------------
    // 2) Structural hazard: DIV is trying to writeback
    //    and we must hold W-stage (do nothing this cycle)
    // -------------------------------------------------
    else if (struct_hazard_div_wb) begin
      // Intentionally empty: hold current W-stage registers
      // (no update, effectively stalling writeback stage)
    end

    // -------------------------------------------------
    // 3) Normal M → W pipeline transfer
    // -------------------------------------------------
    else begin
      // Pass values from M-stage into W-stage pipeline registers
      w_pc         <= m_pc;
      w_inst       <= m_inst;
      w_valid      <= m_valid;
      w_rd_idx_r   <= m_rd_idx;
      w_regwrite_r <= m_regwrite;
      w_memread    <= m_memread;
      w_alu_result <= m_alu_result;
      w_funct3     <= m_inst[14:12];   // keep funct3 for load sign/zero extension, etc.

      // If both a divider writeback and a normal writeback would target
      // the same register in this cycle, and this instruction actually
      // writes a register, then clear the pending divider WB flag.
      // (Avoid double-writing the same rd.)
      if (div_w_valid && w_valid && w_regwrite_r && (w_rd_idx_r == div_w_rd))
        div_w_valid <= 1'b0;
    end
  end

  // --- WRITEBACK STAGE ---
  // Raw 32-bit word loaded from data memory (already aligned to word boundary
  // by the memory system). We will slice/sign-extend/zero-extend it below
  // based on funct3 and the low bits of the effective address.
  wire [`REG_SIZE:0] w_load_data = load_data_from_dmem;

  always @(*) begin
    // Default: for ALU-type instructions, the result is simply the ALU output.
    w_result = w_alu_result;

    // If this instruction was a LOAD, override w_result with properly
    // aligned/sign-extended data from memory.
    if (w_memread) begin
      // -------- Byte loads --------
      if (w_funct3 == 3'b000) begin // LB (signed byte)
        // Use address[1:0] to select which byte inside the 32-bit word
        if      (w_alu_result[1:0] == 2'b00) w_result = {{24{w_load_data[7]}},  w_load_data[7:0]};
        else if (w_alu_result[1:0] == 2'b01) w_result = {{24{w_load_data[15]}}, w_load_data[15:8]};
        else if (w_alu_result[1:0] == 2'b10) w_result = {{24{w_load_data[23]}}, w_load_data[23:16]};
        else                                 w_result = {{24{w_load_data[31]}}, w_load_data[31:24]};
      end

      // -------- Halfword loads --------
      else if (w_funct3 == 3'b001) begin // LH (signed half-word)
        // Use address[1] to pick low or high half of the 32-bit word
        if (w_alu_result[1])
          w_result = {{16{w_load_data[31]}}, w_load_data[31:16]};
        else
          w_result = {{16{w_load_data[15]}}, w_load_data[15:0]};
      end

      // -------- Word load --------
      else if (w_funct3 == 3'b010) begin // LW (full word)
        w_result = w_load_data;
      end

      // -------- Unsigned byte loads --------
      else if (w_funct3 == 3'b100) begin // LBU
        if      (w_alu_result[1:0] == 2'b00) w_result = {24'd0, w_load_data[7:0]};
        else if (w_alu_result[1:0] == 2'b01) w_result = {24'd0, w_load_data[15:8]};
        else if (w_alu_result[1:0] == 2'b10) w_result = {24'd0, w_load_data[23:16]};
        else                                 w_result = {24'd0, w_load_data[31:24]};
      end

      // -------- Unsigned halfword loads --------
      else if (w_funct3 == 3'b101) begin // LHU
        if (w_alu_result[1])
          w_result = {16'd0, w_load_data[31:16]};
        else
          w_result = {16'd0, w_load_data[15:0]};
      end

      // Fallback (should not really happen for valid RV32I loads)
      else begin
        w_result = w_load_data;
      end
    end
  end

  // Data actually written back into the register file.
  assign w_wdata = w_result;

  // --- HALT / ECALL detection ---
  // Set `halt` when a valid ECALL (environment call) reaches W-stage.
  // OpcodeEnviron with all upper bits zero corresponds to "ecall".
  always @(posedge clk or posedge rst) begin
    if (rst)
      halt <= 0;
    else if (w_valid && (w_inst[6:0] == OpcodeEnviron) && (w_inst[31:7] == 0))
      halt <= 1;
  end

  // --- Trace signals for testbench ---
  // These are used by the cocotb testbench to compare retired instructions
  // against the reference model. Only updated when W-stage has a valid inst.
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      trace_writeback_pc   <= 0;
      trace_writeback_inst <= 0;
    end else begin
      if (w_valid) begin
        trace_writeback_pc   <= w_pc;
        trace_writeback_inst <= w_inst;
      end else begin
        trace_writeback_pc   <= 0;
        trace_writeback_inst <= 0;
      end
    end
  end

  // --- TRACE WRITEBACK CYCLE STATUS ---
  always @(posedge clk or posedge rst) begin
    if (rst) begin
        trace_writeback_cycle_status <= CYCLE_STATUS_NONE;
    end else begin
        if (w_valid) begin
            // Normal committed instruction
            trace_writeback_cycle_status <= CYCLE_STATUS_VALID;
        end else begin
            // Bubble (either flush or stall or startup)
            trace_writeback_cycle_status <= CYCLE_STATUS_NONE;
        end
    end
  end

endmodule

module MemorySingleCycle #(
    parameter NUM_WORDS = 512
) (
    input                    rst,
    input                    clk,
    input      [`REG_SIZE:0] pc_to_imem,
    output reg [`REG_SIZE:0] inst_from_imem,
    input      [`REG_SIZE:0] addr_to_dmem,
    output reg [`REG_SIZE:0] load_data_from_dmem,
    input      [`REG_SIZE:0] store_data_to_dmem,
    input      [         3:0] store_we_to_dmem
);

  reg [`REG_SIZE:0] mem_array[0:NUM_WORDS-1];

  initial begin
    $readmemh("mem_initial_contents.hex", mem_array);
  end

  localparam AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam AddrLsb = 2;

  always @(negedge clk) begin
    inst_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
  end

  always @(negedge clk) begin
    if (store_we_to_dmem[0]) begin
      mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
    end
    if (store_we_to_dmem[1]) begin
      mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
    end
    if (store_we_to_dmem[2]) begin
      mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
    end
    if (store_we_to_dmem[3]) begin
      mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
    end
    load_data_from_dmem <= mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];
  end
endmodule

module Processor (
    input                 clk,
    input                 rst,
    output                halt,
    output [ `REG_SIZE:0] trace_writeback_pc,
    output [`INST_SIZE:0] trace_writeback_inst,
    output [31:0]         trace_writeback_cycle_status
);

  wire [`INST_SIZE:0] inst_from_imem;
  wire [ `REG_SIZE:0] pc_to_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [         3:0] mem_data_we;

  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) memory (
    .rst                 (rst),
    .clk                 (clk),
    .pc_to_imem          (pc_to_imem),
    .inst_from_imem      (inst_from_imem),
    .addr_to_dmem        (mem_data_addr),
    .load_data_from_dmem (mem_data_loaded_value),
    .store_data_to_dmem  (mem_data_to_write),
    .store_we_to_dmem    (mem_data_we)
  );

  DatapathPipelined datapath (
    .clk                  (clk),
    .rst                  (rst),
    .pc_to_imem           (pc_to_imem),
    .inst_from_imem       (inst_from_imem),
    .addr_to_dmem         (mem_data_addr),
    .store_data_to_dmem   (mem_data_to_write),
    .store_we_to_dmem     (mem_data_we),
    .load_data_from_dmem  (mem_data_loaded_value),
    .halt                 (halt),
    .trace_writeback_pc   (trace_writeback_pc),
    .trace_writeback_inst (trace_writeback_inst),
    .trace_writeback_cycle_status (trace_writeback_cycle_status)
  );

endmodule

