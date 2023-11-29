// NPC/PC Handler Selector
module NPC_PC_Handler_Selector(
    input branch,
    input jump,
    output reg [1:0] pc_source_selector
);
    always @(*) begin
        if (jump)                        pc_source_selector = 2'b10; // Jump instruction, use jump target
        else if (branch)                 pc_source_selector = 2'b01; // Branch instruction, use branch target
        else                             pc_source_selector = 2'b00; // Normal execution, use sequential address
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
module PC_NPC_Register(
    input clk,
    input reset,
    input load_enable,
    input [31:0] data_in,
    output reg [31:0] data_out,
    input is_npc // Flag to differentiate between PC and NPC
);
    always @(posedge clk) begin
        if (reset) data_out <= (is_npc ? 32'd4 : 32'b0); // NPC initializes to 4, PC to 0
        else if (load_enable) data_out <= data_in;
    end
endmodule

// PC MUX (BORRAR YA!)
module PC_MUX(
    input [31:0] sequential_pc,
    input [31:0] branch_target,
    input [31:0] jump_target,
    input [1:0] select,
    output reg [31:0] next_pc
);
    always @(*) begin
        case (select)
            2'b00: next_pc = sequential_pc;
            2'b01: next_pc = branch_target;
            2'b10: next_pc = jump_target;
            default: next_pc = 32'b0;
        endcase
    end
endmodule
