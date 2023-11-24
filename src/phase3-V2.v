`timescale 1ns / 1ns

// Include MIPS modules
`include "npc-pc-handler-v2.v"
`include "pipeline-registers-v2.v"
`include "control-unit-v2.v"

module rom_512x8 (output reg [31:0] DataOut, input [7:0] Address);
    reg [7:0] Mem[0:511];       //512 8bit locations
    always@(Address)            //Loop when Address changes
        begin 
            DataOut = {Mem[Address], Mem[Address+1], Mem[Address+2], Mem[Address+3]};
        end
endmodule

module MIPS_Pipeline_tb;

    // Instruction Memory 
    integer fi, fo, code, i; 
    reg [7:0] data;
    reg [7:0] Addr; 
    wire [31:0] instruction;

    // Clock, Reset, and Control Signals
    reg Clk, Reset;
    reg S; // Control signal for the Control Unit MUX

    // Wire declarations for interconnecting modules
    wire [31:0] PC, nPC; // PC and NPC values
    wire [31:0] instruction_out;
    wire [3:0] ALU_OP;
    wire [31:0] nPC4;

    // Instantiate the PC Adder
    PC_Adder pc_adder(
        .pc_in(PC),       // Current Program Counter
        .pc_out(nPC4)     // Output: Next Program Counter (PC + 4)
    );

    // Instantiate the PC Register
    PC_NPC_Register pc_reg(
        .clk(Clk),
        .reset(Reset),
        .load_enable(1'b1), // Assuming always enabled for simplicity
        .data_in(nPC),      // Input data (could be from PC_MUX)
        .data_out(PC),      // Current Program Counter
        .is_npc(1'b0)       // Flag indicating this is the PC register
    );

    // Instantiate the NPC Register
    PC_NPC_Register npc_reg(
        .clk(Clk),
        .reset(Reset),
        .load_enable(1'b1), // Assuming always enabled
        .data_in(nPC4),
        .data_out(nPC),     // Next Program Counter
        .is_npc(1'b1)       // Flag indicating this is the NPC register
    );

    // Instantiate the PC MUX
    PC_MUX pc_mux(
        .sequential_pc(sequential_pc),        // Input: Sequential PC (PC + 4)
        .branch_target(branch_target),
        .jump_target(jump_target),
        .select(select),
        .next_pc(next_pc)
    );


    // Instantiate the Control Unit
    ControlUnit control_unit(
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



    // Instantiate the Pipeline Registers
    IF_ID_Register if_id_register(
        .clk(Clk),
        .reset(Reset),
        .instruction_in(instruction),
        .pc_in(PC)
        // ... (Other connections)
    );

    ID_EX_Register id_ex_register(
        .clk(Clk),
        .reset(Reset)
        // ... (Other connections)
    );

    EX_MEM_Register ex_mem_register(
        .clk(Clk),
        .reset(Reset)
        // ... (Other connections)
    );

    MEM_WB_Register mem_wb_register(
        .clk(Clk),
        .reset(Reset)
        // ... (Other connections)
    );

     // Instruction Memory
    rom_512x8 ram1 (
        instruction, // OUT
        PC[7:0]      // IN
    );

    // Precharging the Instruction Memory
    initial begin
        fi = $fopen("p3.txt","r");
        Addr = 8'b00000000;
        // $display("Precharging Instruction Memory...\n---------------------------------------------\n");
        while (!$feof(fi)) begin
            // if (Addr % 4 == 0 && !$feof(fi)) $display("\n\nLoading Next Instruction...\n-------------------------------------------------------------------------");
            code = $fscanf(fi, "%b", data);
            // $display("---- %b ----\n", data);
            ram1.Mem[Addr] = data;
            Addr = Addr + 1;
        end
        $fclose(fi);
        Addr = 8'b00000000;
    end


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
                 $time, PC, nPC, instruction, /* EX control signals */, /* MEM control signals */, /* WB control signals */);
        // Note: Replace placeholders with actual signals from your modules
    end



endmodule
