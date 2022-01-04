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
    wire   [31:0] PC_nxt      ;              //
    wire          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    wire   [31:0] rd_data     ;              //
    //---------------------------------------//

    // Todo: other wire/reg
    

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
    
    // Todo: any combinational/sequential circuit

    // TODO: Decode instruction, determine instruction format (R, I, S, B)
    // ALUOpSelector: parse instruction, retrieve needed operation

    // TODO: Define state: regular operation, multiplication

    // TODO: Design operations: ALU, jump, ...
    // ALU: perform ALU operation


    // Update PC
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            
        end
        else begin
            PC <= PC_nxt;
            
        end
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


module ALU(
    input   clk,
    input   [31:0]  input1,
    input   [31:0]  input2,
    input   [4:0]   alu_ctrl,
    output  [31:0]  result,
    output  reg     state
);

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
