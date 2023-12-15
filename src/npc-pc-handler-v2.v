// NPC/PC Handler Selector
module NPC_PC_Handler_Selector(
    input branch,
    input jump,
    output reg [1:0] pc_source_select
);
    always @(*) begin
        if (jump)                        pc_source_select = 2'b10; // Jump instruction, use jump target
        else if (branch)                 pc_source_select = 2'b01; // Branch instruction, use branch target
        else                             pc_source_select = 2'b00; // Normal execution, use sequential address
    end
endmodule

// PC Adder
module PC_Adder(
    input [31:0] pc_in,
    output [31:0] pc_out
);
    assign pc_out = pc_in + 4; // Increment PC by 4 for sequential execution
endmodule

// PC and NPC Registers ( Hacer modulos apartes.)
module NPC_Register(
    input clk,
    input reset,
    input load_enable,
    input [31:0] data_in,
    output reg [31:0] data_out
);
    always @(posedge clk) begin
        if (reset) data_out <= 32'd4;
        else if (load_enable) data_out <= data_in;
    end
endmodule

module PC_Register(
    input clk,
    input reset,
    input load_enable,
    input [31:0] data_in,
    output reg [31:0] data_out

);
    always @(posedge clk) begin
        if (reset) data_out <= 32'd0;
        else if (load_enable) data_out <= data_in;
    end
endmodule

module PC_MUX(
    input [31:0] nPC,
    input [31:0] TA,
    input [31:0] jump_target,
    input [1:0] select,
    output reg [31:0] Q
);
    always @(*) begin
        case (select)
            2'b00: Q = nPC;
            2'b01: Q = TA;
            2'b10: Q = jump_target;
            default: Q = 32'b0;
        endcase
    end
endmodule
