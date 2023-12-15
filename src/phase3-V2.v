`timescale 1s / 1s

// Include MIPS modules
`include "npc-pc-handler-v2.v"
`include "pipeline-registers-v2.v"
`include "control-unit-v2.v"

module rom_512x8 (output reg [31:0] DataOut, input [8:0] Address);
    reg [7:0] Mem[0:511];       //512 9bit locations
    always@(Address)            //Loop when Address changes
        begin 
            DataOut = {Mem[Address], Mem[Address+1], Mem[Address+2], Mem[Address+3]};
        end
endmodule

module MIPS_Pipeline_tb;

    // Instruction Memory 
    integer fi, fo, code, i; 
    reg [7:0] data;
    reg [8:0] Addr; 
    wire [31:0] instruction;

    // Clock, Reset, and Control Signals
    reg LE = 1;
    reg Clk, Reset;
    reg S; // Control signal for the Control Unit MUX

    // Wire declarations for interconnecting modules
    wire [31:0] PC, nPC; // PC and NPC values
    wire [31:0] instruction_id;
    wire [31:0] nPC4;

    // Declaration of wires for Control Unit output signals
    wire Cond_Mux, Jump, Branch, JalAdder, TaMux, Base_Addr_MUX, 
    RsAddrMux, Data_Mem_RW, Data_Mem_Enable, Data_Mem_SE, HiEnable, 
    RegFileEnable, Jump_Addr_MUX_Enable, LoEnable, MemtoReg, Load,CMUX;
    wire [1:0] WriteDestination, Data_Mem_Size;
    wire [2:0] S0_S2;
    wire [3:0] ALUOp;

    // Testbench signal declarations
    wire [18:0] control_signals_from_cu; 
    wire [18:0] control_signals_to_registers; 

    // PC Adder
    PC_Adder pc_adder(
        .pc_in(nPC),       // Current Program Counter
        .pc_out(nPC4)     // Output: Next Program Counter (PC + 4)
    );

    //  NPC Register
    NPC_Register npc_reg(
        .clk(Clk),
        .reset(Reset),
        .load_enable(1'b1), // Assuming always enabled for simplicity
        .data_in(nPC4),      
        .data_out(nPC)     // Current Program Counter
        
    );

    //  PC Register
    PC_Register pc_reg(
        .clk(Clk),
        .reset(Reset),
        .load_enable(1'b1), // Assuming always enabled for simplicity
        .data_in(nPC),     
        .data_out(PC)      // Current Program Counter
        
    );

    // //  PC MUX
    // PC_MUX pc_mux(
    //     .sequential_pc(sequential_pc),        // Input: Sequential PC (PC + 4)
    //     .branch_target(branch_target),
    //     .jump_target(jump_target),
    //     .select(select),
    //     .next_pc(next_pc)
    // );

     // Instantiate the Control Unit
    ControlUnit control_unit(
        .instruction(instruction_id),
        .instr_singals(.instr)
    );

    // ControlUnitMUX instantiation
    ControlUnitMUX control_unit_mux_inst (
        .S(S),
        .control_signals_in(control_signals_from_cu),
        .control_signals_out(control_signals_to_registers)
    );

    // Instantiate the Pipeline Registers
    IF_ID_Register if_id_register(
        .clk(Clk),
        .reset(Reset),
        .LE(LE),
        .instruction_in(instruction),
        .instruction_out(instruction_id)
        // .pc_in(PC)
       
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
        .DataOut(instruction), // OUT
        .Address(PC[8:0])      // IN
    );

    // Precharging the Instruction Memory
    initial begin
        fi = $fopen("p3.txt","r");
            Addr = 9'b00000000;
            // $display("Precharging Instruction Memory...\n---------------------------------------------\n");
            while (!$feof(fi)) begin
                
                code = $fscanf(fi, "%b", data);
                // $display("---- %b ----\n", data);
                ram1.Mem[Addr] = data;
                Addr = Addr + 1;
            end
        $fclose(fi);
        
    end

    // Function to get instruction keyword
    function [63:0] get_instruction_keyword;
        input [31:0] instruction_id;
        reg [5:0] opcode;
        reg [5:0] func_code;
        begin
            opcode = instruction_id[31:26]; // Extract the opcode
            func_code = instruction_id[5:0]; // Extract the function code for R-type instructions

            case(opcode)
                6'b001001: get_instruction_keyword = "ADDIU"; // ADDIU
                6'b100100: get_instruction_keyword = "LBU";   // LBU
                6'b101000: get_instruction_keyword = "SB";    // SB
                6'b000111: get_instruction_keyword = "BGTZ";  // BGTZ
                6'b001111: get_instruction_keyword = "LUI";   // LUI
                6'b000011: get_instruction_keyword = "JAL";   // JAL

                6'b000000, 6'b011100: begin // for R-type instructions
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

    // Display statement for displaying required information
    
    always @(posedge Clk) begin
     //Display time at the beginning of each clock cycle
         $display("\nTime: %0t s", $time);
        //  $display("Inst: %b | Inst_ID: %b | PC: %d|nPC: %d|",instruction[31:26],instruction_id[31:26], PC, nPC);
     //ID stage
         $display("ID Stage - Inst: %-6s | PC: %d | nPC: %d | Jump: %b | Branch: %b | JalAdder: %b | CMUX: %b | TaMux: %b | ALUOp: %b | Data_Mem_RW: %b | Data_Mem_Enable: %b | Data_Mem_Size: %b | Data_Mem_SE: %b | Base_Addr_MUX: %b | RsAddrMux: %b | HiEnable: %b | RegFileEnable: %b | Jump_Addr_MUX_Enable: %b | LoEnable: %b | MemtoReg: %b | Load: %b | S0-S2: %b ", 
         get_instruction_keyword(instruction_id), PC, nPC, Jump, Branch, JalAdder, CMUX, TaMux, ALUOp, Data_Mem_RW, Data_Mem_Enable, Data_Mem_Size, Data_Mem_SE, Base_Addr_MUX, RsAddrMux, HiEnable, RegFileEnable, Jump_Addr_MUX_Enable, LoEnable, MemtoReg, Load, S0_S2);
         $display("\n");
     // EX stage
         $display("EX Stage -  WriteDestination: %b | Cond_Mux: %b | Jump: %b | Branch: %b | TaMux: %b | ALUOp: %b | HiEnable: %b | LoEnable: %b | Data_Mem_RW: %b | Data_Mem_Enable: %b | Data_Mem_Size: %b | Data_Mem_SE: %b | RegFileEnable: %b | S0-S2: %b   ", WriteDestination, Cond_Mux, Jump, Branch, TaMux, ALUOp, HiEnable, LoEnable, Data_Mem_RW, Data_Mem_Enable, Data_Mem_Size,Data_Mem_SE, RegFileEnable, S0_S2 );
         $display("\n");
     // MEM stage
         $display("MEM Stage - WriteDestination: %b | Data_Mem_RW: %b | Data_Mem_Enable: %b | Data_Mem_Size: %b | Data_Mem_SE: %b | MemtoReg: %b | HiEnable: %b | LoEnable: %b | RegFileEnable: %b  ", WriteDestination, Data_Mem_RW, Data_Mem_Enable, Data_Mem_Size, Data_Mem_SE, MemtoReg, HiEnable, LoEnable, RegFileEnable);
         $display("\n");
     // WB stage
         $display("WB Stage - WriteDestination: %b | HiEnable: %b | RegFileEnable: %b | LoEnable: %b | MemtoReg: %b ", WriteDestination, HiEnable, RegFileEnable, LoEnable, MemtoReg);
         $display("\n"); 
    end

endmodule


