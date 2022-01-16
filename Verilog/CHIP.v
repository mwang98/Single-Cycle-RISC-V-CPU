`include "MulDiv.v"

// Constants
// RISC-V format
`define R_TYPE 7'b0110011 // arithmatic/logical ops
`define I_TYPE 7'b0010011 // immediates
`define I_JALR 7'b1100111
`define I_LOAD 7'b0000011
`define S_TYPE 7'b0100011 // store
`define B_TYPE 7'b1100011 // branch
`define U_TYPE 7'b0010111 // upper immediates
`define UJ_JAL 7'b1101111

// ALUCtrl signal
`define ADD  4'b0000
`define SUB  4'b0001
`define SLL  4'b0010
`define SLT  4'b0011
`define SLTU 4'b0100
`define XOR  4'b0101
`define SRL  4'b0110
`define SRA  4'b0111
`define OR   4'b1000
`define AND  4'b1001
`define MUL  4'b1010
`define DIV  4'b1011

// Branch
`define ISN_BRANCH 1'b0
`define IS_BRANCH 1'b1

// MemRead
`define ISN_MEMREAD 1'b0
`define IS_MEMREAD 1'b1

// MemWrite
`define ISN_MEMWRITE 1'b0
`define IS_MEMWRITE 1'b1

// RegWrite
`define ISN_REGWRITE 1'b0
`define IS_REGWRITE 1'b1

// MemtoReg
`define MEM2REG_PC_PLUS_4 2'b00
`define MEM2REG_ALU 2'b01
`define MEM2REG_MEM 2'b10
`define MEM2REG_PC_PLUS_IMM 2'b11

// ALUSrc
`define FROM_IMM 1'b1
`define FROM_RS2 1'b0

// PCCtrl
`define PCCTRL_PC_PLUS_IMM 2'b00
`define PCCTRL_RS1_PLUS_IMM 2'b01
`define PCCTRL_PC_PLUS_4 2'b10


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
    reg    [31:0] rd_data     ;              //
    //---------------------------------------//

    // parse instruction
    wire    [24:0]  imm;
    wire    [6:0]   opcode;
    wire    [2:0]   funct3;
    wire    [6:0]   funct7;
    wire    [31:0] PC_plus_4;
    assign opcode = mem_rdata_I[6:0];
    assign rd = mem_rdata_I[11:7];
    assign funct3 = mem_rdata_I[14:12];
    assign rs1 = mem_rdata_I[19:15];
    assign rs2 = mem_rdata_I[24:20];
    assign funct7 = mem_rdata_I[31:25];
    assign imm = mem_rdata_I[31:7];
    assign PC_plus_4 = PC + 4;

    // control signal
    wire is_branch;
    wire [1:0] mem_to_reg;
    wire [1:0] pc_ctrl;
    wire mem_read;
    wire mem_write;
    wire alu_src;
    wire reg_write;

    assign regWrite = reg_write;

    // var
    wire [3:0] alu_ctrl;
    wire [31:0] extended_imm;
    wire alu_zero;
    wire alu_ready;
    wire [31:0] alu_result;
    reg [31:0] alu_input2;

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
    always @(*) begin
        case(alu_src)
            `FROM_IMM: alu_input2 = extended_imm;
            `FROM_RS2: alu_input2 = rs2_data;
        endcase
    end

    ALU alu(
        .clk(clk),
        .rst_n(rst_n),
        .input1(rs1_data),
        .input2(alu_input2),
        .alu_ctrl(alu_ctrl),
        .result(alu_result),
        .alu_zero(alu_zero),
        .alu_ready(alu_ready)
    );

    

    // select data written to reg
    always @(*) begin
        case(mem_to_reg)
            `MEM2REG_PC_PLUS_4: rd_data = PC_plus_4;
            `MEM2REG_ALU: rd_data = alu_result;
            `MEM2REG_MEM: rd_data = mem_rdata_D;
            `MEM2REG_PC_PLUS_IMM: rd_data = PC + extended_imm;
        endcase
    end


    // select next-state PC
    always @(*) begin
        if(alu_ready)begin
            case(pc_ctrl)
                `PCCTRL_PC_PLUS_IMM: PC_nxt = (is_branch && !alu_zero) ?  PC_plus_4 : (PC + (extended_imm << 1));
                `PCCTRL_RS1_PLUS_IMM: PC_nxt = alu_result;
                `PCCTRL_PC_PLUS_4: PC_nxt = PC_plus_4;
                default : PC_nxt = PC ;
            endcase
        end
        else PC_nxt = PC ;
    end

    // output
    assign mem_wen_D = mem_write;
    assign mem_addr_D = alu_result;
    assign mem_wdata_D =  rs2_data;
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
            `R_TYPE: ext_imm = 0;
            `I_TYPE: ext_imm = {{20{instruc[24]}}, instruc[24:13]};
            `I_JALR: ext_imm = {{20{instruc[24]}}, instruc[24:13]};
            `I_LOAD: ext_imm = {{20{instruc[24]}}, instruc[24:13]};
            `S_TYPE: ext_imm = {{20{instruc[24]}}, instruc[24:18], instruc[4:0]};
            `B_TYPE: ext_imm = {{20{instruc[24]}}, instruc[24], instruc[0], instruc[23:18], instruc[4:1]};
            `U_TYPE: ext_imm = {instruc[24:5] ,12'b0};
            `UJ_JAL: ext_imm = {{12{instruc[24]}}, instruc[24], instruc[12:5], instruc[13], instruc[23:14]};
            default: ext_imm = 0;
        endcase
    end

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


    reg [31:0] alu_result;
    wire [63:0] muldiv_result;
    wire valid;
    wire mode;
    wire ready;


    // output logic
    assign alu_ready = alu_ctrl == `MUL ? ready : 1;
    assign result = alu_ready ? alu_result : 0;
    assign alu_zero = alu_ctrl == `SUB ? (alu_result == 0) : 1;

    // MulDiv input
    assign valid = (alu_ctrl == `MUL || alu_ctrl == `DIV);
    assign mode = alu_ctrl == `MUL ? 0 : 1;

    MulDiv muldiv(
        .clk(clk),
        .rst_n(rst_n),
        .valid(valid),
        .mode(mode),
        .in_A(input1),
        .in_B(input2),
        .ready(ready),
        .out(muldiv_result)
    );



    // combinational logic: ALU
    always @(*) begin
        case(alu_ctrl)
            `ADD: alu_result = input1 + input2;
            `SUB: alu_result = input1 - input2;
            `SLL: alu_result = input1 << input2;
            `SLT: begin
                if(input1[31] ^ input2[31]) alu_result = input1[31] == 1;
                else alu_result = input1[31] == 0 ? input1 < input2 : input1 > input2;
            end
            `SLTU: alu_result = input1 < input2;
            `XOR: alu_result = input1 ^ input2;
            `SRL: alu_result = input1 >>> input2;
            `SRA: alu_result = input1 >> input2;
            `OR: alu_result = input1 | input2;
            `AND: alu_result = input1 & input2;
            `MUL: alu_result = muldiv_result[31:0];
            `DIV: alu_result = muldiv_result[31:0];
            default: alu_result = 0;
        endcase
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
    always @(*) begin 
        case(opcode)
            `R_TYPE : begin
                is_branch   = `ISN_BRANCH;
                mem_to_reg  = `MEM2REG_ALU;
                pc_ctrl     = `PCCTRL_PC_PLUS_4;
                mem_read    = `ISN_MEMREAD;
                mem_write   = `ISN_MEMWRITE;
                alu_src     = `FROM_RS2;
                reg_write   = `IS_REGWRITE;
            end
            `I_TYPE : begin
                is_branch   = `ISN_BRANCH;
                mem_to_reg  = `MEM2REG_ALU;
                pc_ctrl     = `PCCTRL_PC_PLUS_4;
                mem_read    = `ISN_MEMREAD;
                mem_write   = `ISN_MEMWRITE;
                alu_src     = `FROM_IMM;
                reg_write   = `IS_REGWRITE;
            end
            `I_JALR : begin
                is_branch   = `ISN_BRANCH;
                mem_to_reg  = `MEM2REG_PC_PLUS_4;
                pc_ctrl     = `PCCTRL_RS1_PLUS_IMM;
                mem_read    = `ISN_MEMREAD;
                mem_write   = `ISN_MEMWRITE;
                alu_src     = `FROM_IMM;
                reg_write   = `IS_REGWRITE;
            end
            `I_LOAD : begin
                is_branch   = `ISN_BRANCH;
                mem_to_reg  = `MEM2REG_MEM;
                pc_ctrl     = `PCCTRL_PC_PLUS_4;
                mem_read    = `IS_MEMREAD;
                mem_write   = `ISN_MEMWRITE;
                alu_src     = `FROM_IMM;
                reg_write   = `IS_REGWRITE;
            end
            `S_TYPE : begin
                is_branch   = `ISN_BRANCH;
                mem_to_reg  = `MEM2REG_MEM;
                pc_ctrl     = `PCCTRL_PC_PLUS_4;
                mem_read    = `ISN_MEMREAD;
                mem_write   = `IS_MEMWRITE;
                alu_src     = `FROM_IMM;
                reg_write   = `ISN_REGWRITE;
            end
            `B_TYPE : begin
                is_branch   = `IS_BRANCH;
                mem_to_reg  = `MEM2REG_ALU;
                pc_ctrl     = `PCCTRL_PC_PLUS_IMM;
                mem_read    = `ISN_MEMREAD;
                mem_write   = `ISN_MEMWRITE;
                alu_src     = `FROM_RS2;
                reg_write   = `ISN_REGWRITE;
            end
            `U_TYPE : begin
                is_branch   = `ISN_BRANCH;
                mem_to_reg  = `MEM2REG_PC_PLUS_IMM;
                pc_ctrl     = `PCCTRL_PC_PLUS_4;
                mem_read    = `ISN_MEMREAD;
                mem_write   = `ISN_MEMWRITE;
                alu_src     = `FROM_RS2;
                reg_write   = `IS_REGWRITE;
            end
            `UJ_JAL : begin
                is_branch   = `ISN_BRANCH;
                mem_to_reg  = `MEM2REG_PC_PLUS_4;
                pc_ctrl     = `PCCTRL_PC_PLUS_IMM;
                mem_read    = `ISN_MEMREAD;
                mem_write   = `ISN_MEMWRITE;
                alu_src     = `FROM_IMM;
                reg_write   = `IS_REGWRITE;
            end
            default : begin
                is_branch   = 0;
                mem_to_reg  = 0;
                pc_ctrl     = 0;
                mem_read    = 0;
                mem_write   = 0;
                alu_src     = 0;
                reg_write   = 0;
            end
        endcase
    end

endmodule

module ALUControl(
    input   [6:0]   opcode,
    input   [2:0]   funct3,
    input   [6:0]   funct7,
    output  reg [3:0]   alu_ctrl
);
    always @(*) begin 
        case(opcode)
            `R_TYPE : begin
                if(funct7 == 7'b0000001)
                    alu_ctrl = (funct3==3'b000) ? `MUL : `DIV;
                else begin
                    case(funct3)
                        3'b000: alu_ctrl = funct7 == 0 ? `ADD : `SUB;
                        3'b001: alu_ctrl = `SLL;
                        3'b010: alu_ctrl = `SLT;
                        3'b011: alu_ctrl = `SLTU;
                        3'b100: alu_ctrl = `XOR;
                        3'b101: alu_ctrl = funct7 == 0 ? `SRL : `SRA;
                        3'b110: alu_ctrl = `OR;
                        3'b111: alu_ctrl = `AND;
                    endcase
                end
            end
            `I_TYPE : begin
                case(funct3)
                    3'b000: alu_ctrl = `ADD;    // addi
                    3'b001: alu_ctrl = `SLL;    // slli
                    3'b010: alu_ctrl = `SLT;    // slti
                    3'b011: alu_ctrl = `SLTU;   // sltiu
                    3'b100: alu_ctrl = `XOR;    // xori
                    3'b101: alu_ctrl = funct7 == 0 ? `SRL : `SRA; // srli, srai
                    3'b110: alu_ctrl = `OR;     // or
                    3'b111: alu_ctrl = `AND;    // andi
                endcase
            end
            `B_TYPE : alu_ctrl = `SUB; // beq
            default: alu_ctrl = `ADD;
        endcase
    end
    
endmodule