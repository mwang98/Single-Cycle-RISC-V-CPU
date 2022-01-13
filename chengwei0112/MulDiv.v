module MulDiv(
    input clk,
    input rst_n,
    input valid,
    input mode,
    input  [31:0] in_A,
    output ready,
    input  [31:0] in_B,
    output [63:0] out
);
    // Definition of states
    parameter IDLE = 3'd0;
    parameter MUL  = 3'd1;
    parameter DIV  = 3'd2;
    parameter OUT  = 3'd3;

    // Todo: Wire and reg if needed
    reg  [ 2:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;

    // Todo: Instatiate any primitives if needed

    // Todo 5: Wire assignments
    assign out = shreg;
    assign ready = (state == OUT) ? 1 : 0;
    
    // Combinational always block
    // Todo 1: Next-state logic of state machine
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) state_nxt = mode == 0 ? MUL : DIV;
                else state_nxt = IDLE;
            end
            MUL : state_nxt = counter == 5'd31 ? OUT : MUL;
            DIV : state_nxt = counter == 5'd31 ? OUT : DIV;
            OUT : state_nxt = IDLE;
            default : state_nxt = IDLE;
        endcase
    end
    // Todo 2: Counter
    always @(*) begin
        case(state)
            MUL: counter_nxt = (counter < 6'd32) ? counter + 1'b1 : 0;
            DIV: counter_nxt = (counter < 6'd32) ? counter + 1'b1 : 0;
            default: counter_nxt = 0;
        endcase
    end
    
    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) alu_in_nxt = in_B;
                else       alu_in_nxt = 0;
            end
            OUT : alu_in_nxt = 0;
            default: alu_in_nxt = alu_in;
        endcase
    end

    // Todo 3: ALU output
    always @(*) begin
        case(state)
            MUL : alu_out = {1'b0, shreg[63:32]} + (shreg[0] ? alu_in : 0);
            DIV : alu_out = (shreg[62:31] >= alu_in) ? (shreg[62:31] - alu_in) : shreg[62:31];
            default: alu_out = 0;
        endcase
    end
    
    // Todo 4: Shift register
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) shreg_nxt = {32'b0, in_A};
                else shreg_nxt = 0;
            end
            MUL : begin
                shreg_nxt[30:0] = shreg[31:1]; // right Shift
                shreg_nxt[63:31] = alu_out; // update (handle overflow)
            end
            DIV : begin
                shreg_nxt[63:32] = alu_out[31:0];
                shreg_nxt[31:1] = shreg[30:0];
                shreg_nxt[0] = (alu_out[31:0] == shreg[62:31]) ? 0 : 1;
            end
            default: shreg_nxt = 0;
        endcase
    end

    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end
        else begin
            state <= state_nxt;
            counter <= counter_nxt;
            shreg <= shreg_nxt;
            alu_in <= alu_in_nxt;
        end
    end

endmodule