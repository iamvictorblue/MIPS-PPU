`include "control-unit-v2.v"
`include "mips-definitions.v"

module ControlUnit_tb;

reg [31:0] instruction;
reg reset;
wire ALUOp, Load, RegFileEnable, HiEnable, LoEnable, JalAdder, TaMux, RsAddrMux, WriteDestination, RegDst;
reg select; // Select signal for ControlUnitMUX

// Instantiate the ControlUnit
ControlUnit cu(
    .instruction(instruction),
    .reset(reset),
    .ALUOp(ALUOp),
    .Load(Load),
    .RegFileEnable(RegFileEnable),
    .HiEnable(HiEnable),
    .LoEnable(LoEnable),
    .JalAdder(JalAdder),
    .TaMux(TaMux),
    .RsAddrMux(RsAddrMux),
    .WriteDestination(WriteDestination),
    .RegDst(RegDst)
);

// Control signals for the MUX
wire [8:0] control_signals_from_cu = {ALUOp, Load, RegFileEnable, HiEnable, LoEnable, JalAdder, TaMux, RsAddrMux, WriteDestination, RegDst};
wire [8:0] control_signals_muxed;

// Instantiate the ControlUnitMUX
ControlUnitMUX cu_mux(
    .select(select),
    .control_signals_in(control_signals_from_cu),
    .control_signals_out(control_signals_muxed)
);

// File handling variables
integer file;
integer scan_file;
reg [31:0] binary_instruction;
reg [8*32:1] line; // Assuming a maximum of 32 characters per line

initial begin
    // Open the file
    file = $fopen("p3.txt", "r");
    if (file == 0) begin
        $display("File not found!");
        $finish;
    end

    // Apply reset
    reset = 1;
    #10 reset = 0;
    select = 0; // Pass control signals through

    // Read and apply instructions from the file
    while (!$feof(file)) begin
        scan_file = $fgets(line, file); // Read a line from the file
        scan_file = $sscanf(line, "%b", binary_instruction); // Convert to binary
        instruction = binary_instruction; // Apply the instruction
        #10; // Wait for a time period

        // Display the type of instruction
        $display("Instruction: %b, Type: %s", instruction, getInstructionType(instruction));
    end

    $fclose(file);
    $finish;
end

// Function to determine the instruction type
function [79:0] getInstructionType;
    input [31:0] inst;
    begin
        if (inst[31:26] == 6'b000000) begin  // Check for R-type opcodes
            // Further check for specific R-type instructions if needed
            getInstructionType = "R-type";
        end else if (inst[31:26] == 6'b000010 || inst[31:26] == 6'b000011) begin  // Check for J-type opcodes
            getInstructionType = "J-type";
        end else begin
            getInstructionType = "I-type";
        end
    end
endfunction

// ...

endmodule
