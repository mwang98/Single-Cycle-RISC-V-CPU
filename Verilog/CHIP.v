`include "Muldiv.v"

module CONST();
    // Constants
    // RISC-V format
    parameter R_TYPE = 7'b0110011; // arithmatic/logical ops
    parameter I_TYPE = 7'b0010011; // immediates
    parameter I_JALR = 7'b1100111;
    parameter I_LOAD = 7'b0000011;
    parameter S_TYPE = 7'b0100011; // store
    parameter B_TYPE = 7'b1100011; // branch
    parameter U_TYPE = 7'b0010111; // upper immediates
    parameter UJ_JAL = 7'b1101111;

    // ALUCtrl signal
    parameter ADD  = 4'b0000;
    parameter SUB  = 4'b0001;
    parameter SLL  = 4'b0010;
    parameter SLT  = 4'b0011;
    parameter SLTU = 4'b0100;
    parameter XOR  = 4'b0101;
    parameter SRL  = 4'b0110;
    parameter SRA  = 4'b0111;
    parameter OR   = 4'b1000;
    parameter AND  = 4'b1001;
    parameter MUL  = 4'b1010;
    parameter DIV  = 4'b1011;

    // Branch
    parameter ISN_BRANCH = 0;
    parameter IS_BRANCH = 1;

    // MemRead
    parameter ISN_MEMREAD = 0;
    parameter IS_MEMREAD = 1;
    
    // MemWrite
    parameter ISN_MEMWRITE = 0;
    parameter IS_MEMWRITE = 1;

    // RegWrite
    parameter ISN_REGWRITE = 0;
    parameter IS_REGWRITE = 1;

    // MemtoReg
    parameter MEM2REG_PC_PLUS_4 = 2'b00;
    parameter MEM2REG_ALU = 2'b01;
    parameter MEM2REG_MEM = 2'b10;
    parameter MEM2REG_PC_PLUS_IMM = 2'b11;

    // ALUSrc
    parameter FROM_IMM = 1;
    parameter FROM_RS2 = 0;

    // PCCtrl
    parameter PCCTRL_PC_PLUS_IMM = 2'b00;
    parameter PCCTRL_RS1_PLUS_IMM = 2'b01;
    parameter PCCTRL_PC_PLUS_4 = 2'b10;

endmodule

// Your code
module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I);

    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;
    
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    reg    [31:0] PC_nxt      ;              //
    wire          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    wire   [31:0] rd_data     ;              //
    //---------------------------------------//

    // parse instruction
    wire    [24:0]  imm;
    wire    [6:0]   opcode;
    wire    [2:0]   funct3;
    wire    [6:0]   funct7;
    assign opcode = mem_rdata_I[6:0];
    assign rd = mem_rdata_I[7:11];
    assign funct3 = mem_rdata_I[14:12];
    assign rs1 = mem_rdata_I[19:15];
    assign rs2 = mem_rdata_I[24:20];
    assign funct7 = mem_rdata_I[25:31];
    assign imm = mem_rdata_I[31:7];

    // control singal
    reg is_branch;
    reg [1:0] mem_to_reg;
    reg [1:0] pc_ctrl;
    reg mem_read;
    reg mem_write;
    reg alu_src;
    reg reg_write;

    assign regWrite = reg_write;

    // var
    reg [4:0] alu_ctrl;
    reg [31:0] extended_imm;
    wire alu_zero;
    wire alu_ready;
    wire [31:0] alu_result;
    wire [31:0] alu_input2;

    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//

    // compute control signal
    Control control(
        .opcode(opcode),
        .is_branch(is_branch),
        .mem_to_reg(mem_to_reg),
        .pc_ctrl(pc_ctrl),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .alu_src(alu_src),
        .reg_write(reg_write)
    );
    ALUControl alu_control(
        .opcode(opcode),
        .funct3(funct3),
        .funct7(funct7),
        .alu_ctrl(alu_ctrl)
    );

    // sign-extended immediate
    IMMGEN imm_gen(
        .instruc(imm),
        .opcode(opcode),
        .imm(extended_imm)
    );

    // select alu input2
    case(alu_src)
        CONST.FROM_IMM: assign alu_input2 = extended_imm;
        CONST.FROM_RS2: assign alu_input2 = rs2_data;
    endcase

    ALU alu(
        .clk(clk),
        .rst_n(rst_n),
        .input1(rs1_data),
        .input2(alu_input2),
        .alu_ctrl(alu_ctrl),
        .result(alu_result),
        .alu_zero(alu_zero),
        .alu_ready(alu_ready),
    );

    // select data written to reg
    case(mem_to_reg)
        MEM2REG_PC_PLUS_4: assign rd_data = PC + 4;
        MEM2REG_ALU: assign rd_data = alu_result;
        MEM2REG_MEM: assign rd_data = mem_rdata_D;
        MEM2REG_PC_PLUS_IMM: assign rd_data = PC + extended_imm;
    endcase

    // select next-state PC
    always @(*) begin
        if(alu_ready) begin
            case(pc_ctrl)
                PCCTRL_PC_PLUS_IMM: PC_nxt = PC + extended_imm << 1;
                PCCTRL_RS1_PLUS_IMM: PC_nxt = rs1_data + extended_imm;
                PCCTRL_PC_PLUS_4: PC_nxt = PC + 4;
            endcase
        end
        else PC_nxt = PC;
    end

    // output
    assign mem_wen_D = mem_write;
    assign mem_addr_D = alu_result;
    assign mem_wdata_D = mem_write ? rs2_data : 0;
    assign mem_addr_I = PC;

    // Update PC
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) PC <= 32'h00010000; // Do not modify this value!!!
        else PC <= PC_nxt;
    end
endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module IMMGEN(
    input   [24:0]  instruc,
    input   [6:0]   opcode,
    output  [31:0]  imm
);
    reg [31:0] ext_imm;
    assign imm = ext_imm;

    always @(*) begin
        case(opcode)
            CONST.R_TYPE: ext_imm = 0;
            CONST.I_TYPE: ext_imm = {{20{instruc[24]}}, instruc[24:13]};
            CONST.I_JALR: ext_imm = {{20{instruc[24]}}, instruc[24:13]};
            CONST.I_LOAD: ext_imm = {{20{instruc[24]}}, instruc[24:13]};
            CONST.S_TYPE: ext_imm = {{20{instruc[24]}}, instruc[24:18], instruc[4:0]};
            CONST.B_TYPE: ext_imm = {{20{instruc[24]}}, instruc[24], instruc[0], instruc[23:18], instruc[4:1]};
            CONST.U_TYPE: ext_imm = {instruc[24:5] ,12'b0};
            CONST.UJ_JAL: ext_imm = {{12{instruc[24]}}, instruc[24], instruc[12:5], instruc[13], instruc[23:14]};
            default: ext_imm = 0;
        endcase
    end
endmodule

module MUX4();
endmodule 


module ALU(
    input   clk,
    input   rst_n,
    input   [31:0]  input1,
    input   [31:0]  input2,
    input   [3:0]   alu_ctrl,
    output  [31:0]  result,
    output  alu_zero,
    output  alu_ready
);
    // Definition of states
    parameter OUT  = 0;
    parameter COMP = 1;

    reg [63:0] muldiv_result;
    reg [31:0] alu_result;
    reg state, state_nxt;
    wire valid;
    wire mode;
    wire ready;

    // output logic
    assign alu_ready = state == OUT;
    assign result = state == OUT ? alu_result : 0;
    assign alu_zero = state == OUT ? alu_result == 0 : 0;

    // MulDiv input
    assign valid = (alu_ctrl == CONST.MUL || alu_ctrl == CONST.DIV);
    assign mode = alu_ctrl == CONST.MUL;

    MulDiv muldiv(
        .clk(clk),
        .rst_n(rst_n),
        .valid(valid),
        .mode(mode),
        .in_A(input1),
        .in_B(input2),
        .ready(ready),
        .output(muldiv_result)
    );

    // next state logic
    always @(*) begin
        case(state)
            OUT: begin
                if(alu_ctrl == CONST.MUL or alu_ctrl == CONST.DIV)
                    state_nxt = COMP;
                else
                    state_nxt = OUT;
            end
            COMP: state_nxt = ready == 1 ? OUT : COMP;
    end

    // combinational logic: ALU
    always @(*) begin
        case(alu_ctrl):
            CONST.ADD: alu_result = input1 + input2;
            CONST.SUB: alu_result = input1 - input2;
            CONST.SLL: alu_result = input1 << input2;
            CONST.SLT: begin
                if(input1[31] ^ input2[31]) alu_result = input1[31] == 0;
                else alu_result = input1[31] == 0 ? input1 < input2 : input1 > input2;
            end
            CONST.SLIU: alu_result = input1 < input2;
            CONST.XOR: alu_result = input1 ^ input2;
            CONST.SRL: alu_result = input1 >>> input2;
            CONST.SRA: alu_result = input1 >> input2;
            CONST.OR: alu_result = input1 | input2;
            CONST.AND: alu_result = input1 & input2;
            CONST.MUL: alu_result = muldiv_result[31:0];
            default: alu_result = 0;
        endcase
    end

    // sequential logic
    always @(posedge clk) begin
        if (!rst_n) state <= OUT;
        else state <= state_nxt;
    end
endmodule

module Control(
    input   [6:0]   opcode,
    output  reg   is_branch,
    output  reg   [1:0] mem_to_reg,
    output  reg   [1:0] pc_ctrl,
    output  reg   mem_read,
    output  reg   mem_write,
    output  reg   alu_src,
    output  reg   reg_write
);
    case(opcode)
        CONST.R_TYPE : begin
            is_branch   = CONST.ISN_BRANCH;
            mem_to_reg  = CONST.MEM2REG_ALU;
            pc_ctrl     = CONST.PCCTRL_PC_PLUS_4;
            mem_read    = CONST.ISN_MEMREAD;
            mem_write   = CONST.ISN_MEMWRITE;
            alu_src     = CONST.FROM_RS2;
            reg_write   = CONST.IS_REGWRITE;
        end
        CONST.I_TYPE : begin
            is_branch   = CONST.ISN_BRANCH;
            mem_to_reg  = CONST.MEM2REG_ALU;
            pc_ctrl     = CONST.PCCTRL_PC_PLUS_4;
            mem_read    = CONST.ISN_MEMREAD;
            mem_write   = CONST.ISN_MEMWRITE;
            alu_src     = CONST.FROM_IMM;
            reg_write   = CONST.IS_REGWRITE;
        end
        CONST.I_JALR : begin
            is_branch   = CONST.ISN_BRANCH;
            mem_to_reg  = CONST.MEM2REG_PC_PLUS_4;
            pc_ctrl     = CONST.PCCTRL_RS1_PLUS_IMM;
            mem_read    = CONST.ISN_MEMREAD;
            mem_write   = CONST.ISN_MEMWRITE;
            alu_src     = CONST.FROM_IMM;
            reg_write   = CONST.IS_REGWRITE;
        end
        CONST.I_LOAD : begin
            is_branch   = CONST.ISN_BRANCH;
            mem_to_reg  = CONST.MEM2REG_MEM;
            pc_ctrl     = CONST.PCCTRL_PC_PLUS_4;
            mem_read    = CONST.IS_MEMREAD;
            mem_write   = CONST.ISN_MEMWRITE;
            alu_src     = CONST.FROM_IMM;
            reg_write   = CONST.IS_REGWRITE;
        end
        CONST.S_TYPE : begin
            is_branch   = CONST.ISN_BRANCH;
            mem_to_reg  = CONST.MEM2REG_MEM;
            pc_ctrl     = CONST.PCCTRL_PC_PLUS_4;
            mem_read    = CONST.ISN_MEMREAD;
            mem_write   = CONST.IS_MEMWRITE;
            alu_src     = CONST.FROM_IMM;
            reg_write   = CONST.ISN_REGWRITE;
        end
        CONST.B_TYPE : begin
            is_branch   = CONST.IS_BRANCH;
            mem_to_reg  = CONST.MEM2REG_ALU;
            pc_ctrl     = CONST.PCCTRL_PC_PLUS_IMM;
            mem_read    = CONST.ISN_MEMREAD;
            mem_write   = CONST.ISN_MEMWRITE;
            alu_src     = CONST.FROM_RS2;
            reg_write   = CONST.ISN_REGWRITE;
        end
        CONST.U_TYPE : begin
            is_branch   = CONST.ISN_BRANCH;
            mem_to_reg  = CONST.MEM2REG_PC_PLUS_IMM;
            pc_ctrl     = CONST.PCCTRL_PC_PLUS_4;
            mem_read    = CONST.ISN_MEMREAD;
            mem_write   = CONST.ISN_MEMWRITE;
            alu_src     = CONST.FROM_RS2;
            reg_write   = CONST.IS_REGWRITE;
        end
        CONST.UJ_JAL : begin
            is_branch   = CONST.ISN_BRANCH;
            mem_to_reg  = CONST.MEM2REG_PC_PLUS_4;
            pc_ctrl     = CONST.PCCTRL_PC_PLUS_IMM;
            mem_read    = CONST.ISN_MEMREAD;
            mem_write   = CONST.ISN_MEMWRITE;
            alu_src     = CONST.FROM_IMM;
            reg_write   = CONST.IS_REGWRITE;
        end
    endcase

endmodule


module ALUControl(
    input   [6:0]   opcode,
    input   [2:0]   funct3,
    input   [6:0]   funct7,
    output  reg [4:0]   alu_ctrl,
);
    always @(*) begin 
        case(opcode)
            CONST.R_TYPE : begin
                if(funct7 == 7'b0000001) alu_ctrl = CONST.MUL;
                else begin
                    case(funct3):
                        3'b000: alu_ctrl = funct7 == 0 ? CONST.ADD : CONST.SUB;
                        3'b001: alu_ctrl = CONST.SLL;
                        3'b010: alu_ctrl = CONST.SLT;
                        3'b011: alu_ctrl = CONST.SLTU;
                        3'b100: alu_ctrl = CONST.XOR;
                        3'b101: alu_ctrl = funct7 == 0 ? CONST.SRL : CONST.SRA;
                        3'b110: alu_ctrl = CONST.OR;
                        3'b111: alu_ctrl = CONST.AND;
                    endcase
                end
            end
            CONST.I_TYPE : begin
                case(funct3):
                    3'b000: alu_ctrl = CONST.ADD;    // addi
                    3'b001: alu_ctrl = CONST.SSL;    // slli
                    3'b010: alu_ctrl = CONST.SLT;    // slti
                    3'b011: alu_ctrl = CONST.SLTT;   // sltiu
                    3'b100: alu_ctrl = CONST.XOR;    // xori
                    3'b101: alu_ctrl = funct7 == 0 ? CONST.SRL : CONST.SRA; // srli, srai
                    3'b110: alu_ctrl = CONST.OR;     // or
                    3'b111: alu_ctrl = CONST.AND;    // andi
                endcase
            end
            CONST.B_TYPE : alu_ctrl = CONST.SUB; // beq
            default: alu_ctrl = CONST.ADD;
            
        endcase
    end
endmodule
