`timescale 1ns / 1ns

// Include MIPS modules
`include "npc-pc-handler-v2.v"
`include "pipeline-registers-v2.v"
`include "control-unit-v2.v"

module MIPS_Pipeline_tb;

    // Clock, Reset, and Control Signals
    reg Clk, Reset;
    reg S; // Control signal for the Control Unit MUX

    // Wire declarations for interconnecting modules
    wire [31:0] PC, nPC; // PC and NPC values
    // ... (Other necessary wire declarations)

    // Instantiate your MIPS modules here
    // Example: NPC_PC_Handler npc_pc_handler(...);
    // Example: Pipeline_Registers pipeline_registers(...);
    // Example: Control_Unit control_unit(...);

    // Clock generation
    initial begin
        Clk = 0;
        forever #2 Clk = ~Clk; // Toggle clock every 2 time units
    end

    // Reset and control signal handling
    initial begin
        Reset = 1; // Active high reset
        S = 0; // Initial state of the Control Unit MUX signal
        #3 Reset = 0; // Deactivate reset at time 3
        #40 S = 1; // Change state of S at time 40
        #48 $finish; // End simulation at time 48
    end

    // Monitor statement for displaying required information
    initial begin
        $monitor("Time: %0t, PC: %d, nPC: %d, Instruction: %b, EX Control Signals: %b, MEM Control Signals: %b, WB Control Signals: %b",
                 $time, PC, nPC, /* instruction */, /* EX control signals */, /* MEM control signals */, /* WB control signals */);
        // Note: Replace placeholders with actual signals from your modules
    end

    // Read and apply instructions from the provided text file
    integer file, code;
    reg [31:0] instruction;

    initial begin
        file = $fopen("p3.txt", "r");
        if (file == 0) begin
            $display("Error: File not found.");
            $finish;
        end

        while (!$feof(file)) begin
            code = $fscanf(file, "%b\n", instruction);
            // Apply the instruction to your pipeline
            // ...
        end
        $fclose(file);
    end

endmodule
