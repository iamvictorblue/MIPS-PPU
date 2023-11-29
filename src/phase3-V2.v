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

    // Declaration of wires for Control Unit output signals
    wire Cond_Mux, Jump, Branch, JalAdder, TaMux, Base_Addr_MUX, 
    RsAddrMux, Data_Mem_RW, Data_Mem_Enable, RegDst, HiEnable, 
    RegFileEnable, Jump_Addr_MUX_Enable, LoEnable, MemtoReg, Load;
    wire [1:0] CMU, WriteDestination, Data_Mem_Size;
    wire [2:0] S0_S2;
    wire [3:0] ALUOp;


    // PC Adder
    PC_Adder pc_adder(
        .pc_in(PC),       // Current Program Counter
        .pc_out(nPC4)     // Output: Next Program Counter (PC + 4)
    );

    //  PC Register
    PC_NPC_Register pc_reg(
        .clk(Clk),
        .reset(Reset),
        .load_enable(1'b1), // Assuming always enabled for simplicity
        .data_in(nPC),      // Input data (could be from PC_MUX)
        .data_out(PC),      // Current Program Counter
        .is_npc(1'b0)       // Flag indicating this is the PC register
    );

    //  NPC Register
    PC_NPC_Register npc_reg(
        .clk(Clk),
        .reset(Reset),
        .load_enable(1'b1), // Assuming always enabled
        .data_in(nPC4),
        .data_out(nPC),     // Next Program Counter
        .is_npc(1'b1)       // Flag indicating this is the NPC register
    );

    //  PC MUX
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
        .reset(Reset), // Assuming 'Reset' is a wire or reg in your testbench
        .Cond_Mux(Cond_Mux),
        .Jump(Jump),
        .Branch(Branch),
        .JalAdder(JalAdder),
        .CMU(CMU),
        .TaMux(TaMux),
        .WriteDestination(WriteDestination),
        .S0_S2(S0_S2),
        .Base_Addr_MUX(Base_Addr_MUX),
        .RsAddrMux(RsAddrMux),
        .ALUOp(ALUOp),
        .Data_Mem_RW(Data_Mem_RW),
        .Data_Mem_Enable(Data_Mem_Enable),
        .Data_Mem_Size(Data_Mem_Size),
        .RegDst(RegDst),
        .HiEnable(HiEnable),
        .RegFileEnable(RegFileEnable),
        .Jump_Addr_MUX_Enable(Jump_Addr_MUX_Enable),
        .LoEnable(LoEnable),
        .MemtoReg(MemtoReg),
        .Load(Load)
    );



    // Instantiate the Pipeline Registers
    IF_ID_Register if_id_register(
        .clk(Clk),
        .reset(Reset),
        .instruction_in(instruction),
        .pc_in(PC)
       
    );

    ID_EX_Register id_ex_register(
        .clk(Clk),
        .reset(Reset)
        
    );

    EX_MEM_Register ex_mem_register(
        .clk(Clk),
        .reset(Reset)
      
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
            
            code = $fscanf(fi, "%b", data);
            // $display("---- %b ----\n", data);
            ram1.Mem[Addr] = data;
            Addr = Addr + 1;
        end
        $fclose(fi);
        Addr = 8'b00000000;
    end

    // Function to get instruction keyword
    function [63:0] get_instruction_keyword;
        input [31:0] instruction;
        reg [5:0] opcode;
        reg [5:0] func_code;
        begin
            opcode = instruction[31:26]; // Extract the opcode
            func_code = instruction[5:0]; // Extract the function code for R-type instructions

            case(opcode)
                6'b001001: get_instruction_keyword = "ADDIU"; // ADDIU
                6'b100100: get_instruction_keyword = "LBU";   // LBU
                6'b101000: get_instruction_keyword = "SB";    // SB
                6'b000111: get_instruction_keyword = "BGTZ";  // BGTZ
                6'b001111: get_instruction_keyword = "LUI";   // LUI
                6'b000011: get_instruction_keyword = "JAL";   // JAL

                6'b000000: begin // for R-type instructions
                    case(func_code)
                        6'b100011: get_instruction_keyword = "SUBU"; // SUBU
                        6'b001000: get_instruction_keyword = "JR";    // JR
                        // Add other R-type instructions 
                        default:   get_instruction_keyword = "R-TYPE_UNKNOWN";
                    endcase
                end
                // Add other I-type and J-type instructions 
                default:   get_instruction_keyword = "NOP"; // Default NOP for unrecognized instructions
            endcase
        end
    endfunction



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
        always (posedge clk) begin
            //fork and join dentro de un initial     
            $display("ID - Time: %0t, Instruction Keyword: %s PC: %d, nPC: %d, ",
                    $time, instruction_keyword, PC, nPC);
            //EX
            $display("EX - Time: %0t, PC: %d, nPC: %d, Instruction: %b, EX Control Signals: %b, , , , , ",
                    $time,  /* EX control signals */);
            //MEM
            $display("MEM - Time: %0t, PC: %d, nPC: %d, Instruction: %b, MEM Control Signals: %b, , , , ,  ,  ",
                    $time, /* MEM control signals */);
            //WB
            $display("WB - Time: %0t, PC: %d, nPC: %d, Instruction: %b, WB Control Signals: %b, , , , , , , ",
                    $time,  /* WB control signals */);
            
            //if/id:
            //id/ex: //poner señal enable de tal cosa, 
            //ex/mem:
            //mem/wb:
            //4 displays apartes 
        end
    end



endmodule
