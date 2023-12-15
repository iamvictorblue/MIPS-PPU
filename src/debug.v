`timescale 1ns / 1ns

`include "control-unit-v2.v"
`include "alu-v2.v"
`include "pipeline-registers-v2.v"
`include "muxes.v"
`include "reset-handler.v"
`include "operand2_handler.v"
`include "condition-handler.v"
`include "instruction-memory.v"
`include "register-file.v"
`include "data-memory.v"
`include "npc-pc-handler-v2.v"
`include "output-handler.v"
`include "hazard-forwarding-unit.v" 

module debug(

    // Instruction Memory 
    integer fi, fo, code, i; 
    reg [7:0] data;
    reg [7:0] Addr; 
    wire [31:0] instruction;
    wire [31:0] instruction_out;

   // Clock, Reset, and Control Signals
    reg LE = 1;
    reg clk, clr;
    reg S; // Control signal for the Control Unit MUX

    // Counters
    wire [31:0] nPC;     // The Next Program Counter
    wire [31:0] nPC4;    // Next Program Counter (Modified)
    wire [31:0] PC;      // The actual Program Counter. This counter state is at the Fetch State
    wire [31:0] instruction_id;
    wire [31:0] PC_ID;   // The Program Counter state at the Decode Stage
    wire [31:0] PC_EX;   // The Program Counter state at the Execute State
    wire [31:0] PC_MEM;  // The Program Counter state at the Memory State

    // Used to calculate the instruction that should be executed by modifying the Program Counter (PC)
    wire [31:0] TA;         // The Target Address

    // HI/LO registers
    reg hi_enable;              // Hi enable signal
    reg lo_enable;              // Lo enable signal
    reg [31:0] pw_signal;       // Signal connected to PW input
    wire [31:0] hi_out_signal;  // Signal to receive HiRegister output
    wire [31:0] lo_out_signal;  // Signal to receive LoRegister output


    // Multiplexer outputs
    wire [1:0] forwardMX1;              // Multiplexer selection for Mux 1 of ID Stage
    wire [1:0] forwardMX2;              // Multiplexer selection for Mux 2 of ID Stage

    wire [1:0] forwardOutputHandler;    // Selects an option from the MUX connected to the output handler
    wire [1:0] forwardPC;               // Selects an option from the MUX that inside a nPC/PC logic box
    
    wire forwardCU;
);

 // Precharging the Instruction Memory
    initial begin
        fi = $fopen("p3.txt","r");
        Addr = 9'b00000000;
        // $display("Precharging Instruction Memory...\n---------------------------------------------\n");
        while (!$feof(fi)) begin
            // if (Addr % 4 == 0 && !$feof(fi)) $display("\n\nLoading Next Instruction...\n-------------------------------------------------------------------------");
            code = $fscanf(fi, "%b", data);
            // $display("---- %b ----     Address: %d\n", data, Addr);
            ROM.Mem[Addr] = data;
            RAM.Mem[Addr] = data;
            Addr = Addr + 1;
        end
        $fclose(fi);
        Addr = 9'b00000000;
    end


    // // Clock generator
    initial begin
        clr <= 1'b1;
        clk <= 1'b0;
        #2 clk <= ~clk;
        #1 clr <= 1'b0;
        #1 clk <= ~clk; 
        forever #2 clk = ~clk;
    end


endmodule
