`timescale 1ns / 1ns

// Phase 4 

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


module phase4_tb;

// -------------- R E G I S T E R S -------------------- //

    // Instruction Memory 
    integer fi, fo, code, i; 
    reg [7:0] data;
    reg [7:0] Addr; 
    wire [31:0] instruction;
    wire [31:0] instruction_out;

   // Clock, Reset, and Control Signals
    
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
    // reg forwardCU;

    // Instruction Signals from the Control Unit
    wire [19:0] instr_signals;     // Unslized Control Unit instructions between CU and CU_MUX
    wire [18:0] ID_CU;      // Unslized Control Unit instructions at the ID Stage
    wire [9:0]  EX_CU;      // Unslized Control Unit instructions at the EX Stage
    wire  MEM_CU;     // Unslized Control Unit instructions at the MEM Stage

    wire Cond_Mux, Jump, Branch, JalAdder, TaMux, Base_Addr_MUX, 
    RsAddrMux, Data_Mem_RW, Data_Mem_Enable, Data_Mem_SE, HiEnable, 
    RegFileEnable, Jump_Addr_MUX_Enable, LoEnable, MemtoReg, Load,CMUX;
    wire [1:0] Data_Mem_Size;
    wire [2:0] S0_S2;
    wire [3:0] ALUOp;

    //IF/ID register outputs
    wire [15:0] imm16;           // Corrected bit width
    wire [25:0] addr26;          // Corrected bit width
    wire [15:0] imm16Handler_ID;    // Corrected bit width
    wire [4:0] rs, rt, rd, WriteDestination_ID;       // Corrected bit width for register addresses
   
    wire [31:0] Base_Addr_SE;      // 32-bit signal for mux_2x1_base_addr
    wire [31:0] multiplierBy4_out; // 32-bit signal for multiplierBy4
    wire [31:0] adder32Bit_out;    // 32-bit signal for adder32Bit
    wire [31:0] sign_extended_out;
    wire [31:0] Base_Addr_A;
    wire [31:0] hi_signal_ID, lo_signal_ID, JalAdder_ID, JalAdder_EX, JalAdder_MEM,JalAdder_WB;


    // Testbench signal declarations
    wire [26:0] control_signals_from_cu; 
    wire [18:0] control_signals_to_registers; 

    // Outputs of Components
    wire [31:0] ALU_OUT;        // ALU Output, used for the PC/nPC system
    wire [31:0] MEM_OUT;        // This is the output of a MUX located in MEM stage
    wire [31:0] WB_OUT;         // Writeback instruction. Goe to PW of the Register File and to the Muxes of ID Stage
    // reg [31:0] WB_OUT;
    wire [31:0] PC_MUX_OUT;     // Output between the PC and the PC_MUX
    wire [21:0] extend_signal;  // 22-bit signal for extend port of SignExtender
    wire [15:0] extend_signal_imm16;  // 22-bit signal for extend port of SignExtender


    // Controls when the flow will flow and when it should stop
    wire PC_LE;
    wire nPC_LE;
    wire IF_ID_Pipeline_LE;

    wire [31:0] PC_dummy;

    // reg cond_branch_OUT = 0; // Goes to nPC/PC handler, from condition handler
    wire cond_branch_OUT;
    wire [3:00] CC_COND;


    // -------  Other Outputs -------- /
    wire [31:0] addr26_SE;
    wire [31:0] imm16_SE;

    // Register File Output
    wire [31:0] pa;
    wire [31:0] pb;


    // Multiplexer Output (ID)
    wire [31:0] ID_MX1;
    wire [31:0] ID_MX2;



    // EX Stage
    wire [4:0] WriteDestination_EX;
    wire [3:0] ALU_OP; // ALU OPCODE Instructions
    wire [31:0] hi_signal_EX, lo_signal_EX;
    wire [15:0] imm16Handler_EX;  // 16-bit signal
    wire [4:0] rs_EX, rt_EX, rd_EX;  // 5-bit signals for register addresses
    wire [19:0] control_signals_out_ID_EX;  // 19-bit signal
    wire [9:0] control_signals_out_EX_MEM;  // 19-bit signal
    wire [4:0] control_signals_out_MEM_WB;  // 19-bit signal
    wire CC_Enable;     // Condition Code Enable
    wire [31:0] operand2_handler_out;

    // Multiplexer Output (EX)
    wire [31:0] EX_MX1;
    wire [31:0] EX_MX2;
    wire [31:0] MEM_MX2;

    wire [31:0] ID_TA;
    wire [31:0] EX_TA;

    // Declaration of signals to be connected to the Condition_Handler
    wire if_id_reset_condition_handler;
    wire CH_Out_condition_handler;
    wire [31:0] instruction_condition_handler; // Connect this to the source of the instruction
    wire Z_condition_handler; // Zero flag, connect this to the source
    wire N_condition_handler; // Negative flag, connect this to the source

    // Declaration of signals to be connected to the Hazard_Forwarding_Unit

    wire [4:0] operandA_hazard_forwarding_unit; // Connect to source operand A in ID stage
    wire [4:0] operandB_hazard_forwarding_unit; // Connect to source operand B in ID stage

    wire [31:0] N; // Source2 Operand Handler output
    wire [3:0] ALU_FLAGS;

    wire [4:0] RD_EX; // Destination Register filtered by a MUX (EX)

    // MEM Stage
    // --------------------------------------------------------------------------------------------
    wire [4:0] RD_MEM; // Destination Register filtered by a MUX (MEM)
    wire [4:0] DataMemInstructions;         // This goes on the Data Memory
    wire [2:0] OutputHandlerInstructions;   // This goes on the output handler
    wire [31:0] MEM_ALU_OUT_Address;
    wire [31:0] DataMemory_OUT;
    wire [4:0] WriteDestination_MEM;


    //  WB Stage
    // --------------------------------------------------------------------------------------------
    wire WB_Register_File_Enable;
    wire [4:0] RD_WB;
    wire [4:0] WriteDestination_WB;
    // reg WB_Register_File_Enable;
    // reg [4:0] RD_WB;
    // reg [31:0] TempR5;


// -------------- M O D U L E  I N S T A N C I A T I O N -------------------- //
    
    // Adder, updates the PC
     PC_Adder pc_adder(
        .pc_in(nPC),       // Current Program Counter
        .pc_out(nPC4)     // Output: Next Program Counter (PC + 4)
    );

    // nPC register
    NPC_Register npc_reg(
        .clk(clk),
        .reset(clr),
        .load_enable(nPC_LE), // Assuming always enabled for simplicity
        .data_in(nPC4),      
        .data_out(nPC)     // Current Program Counter
        
    );

    // PC register
    PC_Register pc_reg(
        .clk(clk),
        .reset(clr),
        .load_enable(nPC_LE), // Assuming always enabled for simplicity
        .data_in(PC_MUX_OUT),     
        .data_out(PC_dummy)      // Current Program Counter
        
    );

    NPC_PC_Handler_Selector nPC_PC_Handler (
        .branch              (cond_branch_OUT),
        .jump                (instr_signals[1]),
        .pc_source_select  (forwardPC)
    );

    // Multiplexer, filters which value to update the PC
    PC_MUX PC_MUX (
        .nPC        (nPC),
        .TA         (TA),
        .select     (forwardPC),
        .Q          (PC_MUX_OUT)
    );

    

    // Instruction Memory
    rom_512x8 ROM (
        .DataOut(instruction), // OUT
        .Address(PC_dummy[8:0])      // IN
    );


    // Data Memory
    ram_512x8 RAM (
        .DataOut                        (DataMemory_OUT),
        .SignExtend                     (DataMemInstructions[0]),   
        .ReadWrite                      (DataMemInstructions[1]),      
        .Enable                         (DataMemInstructions[2]), 
        .Size                           (DataMemInstructions[4:3]),
        .Address                        (MEM_ALU_OUT_Address[7:0]),
        .DataIn                         (MEM_MX2)
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


    // Clock generator
    initial begin
        clr <= 1'b1;
        clk <= 1'b0;
        forever #2 clk = ~clk;
    end

    initial begin
        #3 clr <= 1'b0;
    end

    // -|-|-|-|-|-|-|-|----- I D  S T A G E -----|-|-|-|-|-|-|-|- //

    IF_ID_Register if_id_register(
        .clk(clk),
        .reset(clr),
        .instruction_in(instruction),
        .PC(PC_dummy),
        .LE(LE),
        .instruction_out(instruction_id),
        .pc_out(PC_ID),
        .imm16(imm16),
        .addr26(addr26),
        .imm16Handler(imm16Handler_ID),
        .rs(rs),
        .rt(rt),
        .rd(rd)
    );

    SignExtender_imm16 SignExtender_imm16 (
        .extended               (imm16_SE),
        .extend                 (imm16)
    );

    SignExtender SignExtender_addr26 (
        .extended               (addr26_SE),
        .extend                 (addr26)
    );

    mux_2x1_base_addr Base_Addr_mux (
        .Y                       (Base_Addr_SE),
        .I0                      (imm16_SE),
        .I1                      (addr26_SE),
        .S                       (Base_Addr_MUX)
    );

    multiplierBy4 multiplierBy4 (
        .multipliedOut          (Base_Addr_A),
        .in                     (Base_Addr_SE)
    );

    adder32Bit adder32Bit (
        .out (TA),
        .a   (Base_Addr_A),
        .b   (PC_ID)
    );

    adder32Bit_jal adder32Bit_jal (
        .out (JalAdder_ID),
        .a   (PC_ID),
        .b   (4'd8)
    );
    
    mux_2x1 RS_Addr_MUX (
        .Y                       (ID_TA),
        .I0                      (TA),
        .I1                      (pa),
        .S                       (RsAddrMux)
    );

    mux_2x1 TA_MUX(
        .Y(TA), 
        .S(TaMux),
        .I0(ID_TA),
        .I1(EX_TA)
    );

    ControlUnit control_unit(
        .instruction(instruction_id),
        .instr_signals(control_signals_from_cu)
    );

    ControlUnitMUX control_unit_mux_inst (
        .CMUX(control_signals_from_cu[4]),
        .control_signals_in(control_signals_from_cu),
        .control_signals_out(control_signals_to_registers)
    );

    reset_handler reset_handler (
        .reset_out                  (Reset),
        .system_reset               (clr),
        .condition_handler_instr    (cond_branch_OUT)
       
    );

    mux_3x1_wd WriteDestination_MUX(
        .Y                       (WriteDestination_ID),
        .I0                      (rs),
        .I1                      (rt),
        .I2                      (5'd31),
        .S                       (control_signals_from_cu[7:6])
    );

    // Register File, saves operand and destiny registers
    RegisterFile register_file (
        .PA                             (pa),
        .PB                             (pb),
        .PW                             (WB_OUT),
        .RW                             (RD_WB),
        .RA                             (rs),
        .RB                             (rt),
        .LE                             (WB_Register_File_Enable),
        .Clk                            (clk)
    );

    

    // Instantiation of HiRegister
    HiRegister hi_reg_inst (
        .clk(clk),             // Connect to clock signal
        .HiEnable(hi_enable),  // Connect to hi enable signal
        .PW(pw_signal),        // Connect to PW input signal
        .HiSignal(hi_out_signal) // Connect to output signal
    );

    // Instantiation of LoRegister
    LoRegister lo_reg_inst (
        .clk(clk),             // Connect to clock signal
        .LoEnable(lo_enable),  // Connect to lo enable signal
        .PW(pw_signal),        // Connect to PW input signal
        .LoSignal(lo_out_signal) // Connect to output signal
    );


    mux_4x1 MX1 (
        .S                              (forwardMX1),
        .I0                             (pa),               // File Register value selected by rs1
        .I1                             (ALU_OUT),          // EX_RD
        .I2                             (MEM_OUT),          // MEM_RD
        .I3                             (WB_OUT),           // WB_RD 
        .Y                              (ID_MX1)            // MUX OUTPUT
    );

    mux_4x1 MX2 (
        .S                              (forwardMX2),
        .I0                             (pb),
        .I1                             (ALU_OUT),
        .I2                             (MEM_OUT),
        .I3                             (WB_OUT),
        .Y                              (ID_MX2)
    );

  
    // -|-|-|-|-|-|-|-|----- E X  S T A G E -----|-|-|-|-|-|-|-|- //

    ID_EX_Register id_ex_register(
        .clk(clk),
        .reset(clr),
        .instruction_in(instruction),
        .PC(PC_ID),
        .control_signals_in(control_signals_from_cu),
        .rs_ID(rs),
        .rt_ID(rt),
        .rd_ID(rd),
        .ID_TA(ID_TA),
        .hi_signal_ID(hi_out_signal),
        .lo_signal_ID(lo_out_signal),
        .imm16Handler_ID(imm16Handler_ID),
        .ID_MX1(ID_MX1),
        .ID_MX2(ID_MX2),
        .WriteDestination_ID(WriteDestination_ID),
        .JalAdder_ID(JalAdder_ID),

        .JalAdder_EX(JalAdder_EX),
        .WriteDestination_EX(WriteDestination_EX),
        .hi_signal_EX(hi_signal_EX),
        .lo_signal_EX(lo_signal_EX),
        .imm16Handler_EX(imm16Handler_EX),

        .EX_MX1(EX_MX1),
        .EX_MX2(EX_MX2),

        .rs_EX(rs_EX),
        .rt_EX(rt_EX),
        .rd_EX(rd_EX),
        .EX_TA(TA),
        .PC_EX(PC_EX),
        .control_signals_out(control_signals_out_ID_EX)
    
    );

    // Operand2 Handler instantiation
    Operand2_Handler uut (
        .PB(EX_MX2),
        .HI(hi_out_signal),
        .LO(lo_out_signal),
        .PC(PC_EX),
        .imm16(imm16Handler_EX),
        .S0_S2(control_signals_from_cu[10:8]),
        .N(operand2_handler_out)
    );
  
    // ALU instantiation
    ALU alu (
        .A(EX_MX1),
        .B(operand2_handler_out),
        .opcode(ALUOp),
        .Out(ALU_OUT),
        .Z(Z)
    );


    // Instantiation of Condition_Handler
    Condition_Handler condition_handler_instance (
        .if_id_reset(if_id_reset_condition_handler),
        .CH_Out(CH_Out_condition_handler),
        .instruction(instruction_condition_handler),
        .Z(Z_condition_handler),
        .N(N_condition_handler)
    );

    // Instantiation of Hazard_Forwarding_Unit
    hazard_forwarding_unit hazard_forwarding_unit_instance (
        .forwardMX1                     (forwardMX1),
        .forwardMX2                     (forwardMX2),
        
        .nPC_LE                         (nPC_LE),
        .PC_LE                          (PC_LE),
        .IF_ID_LE                       (IF_ID_Pipeline_LE),

        .CU_S                           (forwardCU), 

        .EX_Register_File_Enable        (EX_CU[4]),
        .MEM_Register_File_Enable       (MEM_CU),
        .WB_Register_File_Enable        (WB_Register_File_Enable),

       
        .EX_RD                          (RD_EX),
        .MEM_RD                         (RD_MEM),
        .WB_RD                          (RD_WB),

        .operandA(operandA_hazard_forwarding_unit),
        .operandB(operandB_hazard_forwarding_unit)
    );

    // -|-|-|-|-|-|-|-|----- M E M  S T A G E -----|-|-|-|-|-|-|-|- //

   
    EX_MEM_Register ex_mem_register(
        .clk(clk),
        .reset(clr),
        .PC(PC_EX),
        .control_signals_in(control_signals_out_EX_MEM), // Connect only the relevant 10 bits
        .WriteDestination_EX(WriteDestination_EX),
        .JalAdder_EX(JalAdder_EX),
        .EX_MX2(EX_MX2),
        .EX_ALU_OUT(ALU_OUT),
        .MEM_ALU_OUT(MEM_ALU_OUT_Address),
        .MEM_MX2(MEM_MX2),
        .JalAdder_MEM(JalAdder_MEM),
        .WriteDestination_MEM(WriteDestination_MEM),
        .PC_MEM(PC_MEM),
        .control_signals_out(control_signals_out_MEM_WB)
    );

    mux_2x1 MEM_MUX (
        .Y                       (MEM_OUT),
        .I0                      (MEM_ALU_OUT_Address),
        .I1                      (DataMemory_OUT),
        .S                       (MEM_MUX)
    );
    
    MEM_WB_Register mem_wb_register(
        .clk(clk),
        .reset(clr),
        .control_signals_in(control_signals_out_MEM_WB),
        .MEM_MUX(MEM_MUX),
        .WriteDestination_MEM(WriteDestination_MEM),
        .JalAdder_MEM(JalAdder_MEM),
        .MEM_OUT(MEM_OUT),
        .JalAdder_WB(JalAdder_WB),
        .WriteDestination_WB(RD_WB)
        
    );

    mux_2x1 MemtoReg_MUX (
        .Y                       (WB_OUT),
        .I0                      (MEM_OUT),
        .I1                      (JalAdder_WB),
        .S                       (control_signals_from_cu[25])
    );



    // initial begin
    //     $monitor($time, " PC=%d, DataAddress=%d", PC, instruction);
    // end
// // llamar registros de manera indirecta
    // Clock generation
    // Clock generator
    // initial begin
    //     clr <= 1'b1;
    //     clk <= 1'b0;
    //     #2 clk <= ~clk;
    //     #1 clr <= 1'b0;
    //     #1 clk <= ~clk; 
    //    forever #2 clk = ~clk;
    // end

    // Clock generation
    // initial begin
    //     clk = 0;
    //     forever #2 clk = ~clk; // Toggle clock every 2 time units
    // end

    // initial begin
    //     $monitor("\n\n\nTIME: %d\n---------------------------------\
    //     \nPC: %d\n--------------------------------------\
    //     \nR5: %d | R6: %d\
    //     \nR16: %d | R17: %d\
    //     \nR18: %d\
    //     \n--------------------------------------------------",
    //     $time,
    //     PC,
    //     register_file.Q5, register_file.Q6, register_file.Q16, register_file.Q17, register_file.Q18);
    // end

   
   

    initial begin
        #90;
        $display("---------->>>>>> LOC 52", RAM.Mem[52]);
    end
    // Clock generation (for simulation)
    // always begin
    //     #2 clk = ~clk;
    // end
    // initial begin
    // // Initialize the reset
    // // clr = 1;
    // #5 clr = 0;  // Release the reset after a brief period
    // // Initialize the clock
    // clk = 0;
    // forever #2 clk = ~clk; // Generate clock with a period of 4 time units
    // #37 S = 1;
    // end
  // Test vector application

    // initial begin
        
    //     $dumpfile("test.vcd"); // pass this to GTK Wave to visualize better wtf is going on
    //     $dumpvars(0, phase4_tb);
    //     #100;
    //     $display("\n----------------------------------------------------------\nSimmulation Complete!");
    //     $finish;
    // end

    initial begin
        #100;
        $display("\n----------------------------------------------------------\nSimmulation Complete!");
        $finish;
    end 

    // initial begin
    //     $monitor("|TIME: %d|Clk: %b | PC_dummy: %d| nPC: %d | Clr: %b | PC_MUX_OUT: %d |  R5: %d| R6: %d| R16: %d | R17: %d| R18: %d",
    //     $time,clk, PC_dummy, nPC, clr, PC_MUX_OUT, register_file.Q5, register_file.Q6, register_file.Q16, register_file.Q17, register_file.Q18);
        
    // end
    initial begin
        // $monitor("|TIME: %d|Clk: %b | PC_dummy: %d| nPC: %d| ALU_A: %d|ALU_B: %d|ALU_OUT: %d",
        // $time,clk, PC_dummy, nPC, pa , pb ,ALU_OUT );
        $monitor("TIME: %d | Clk: %b | PC_dummy: %d|pb: %b   |  hi_out_signal: %b  | lo_out_signal: %b  | PC_EX: %b  | op2_h_out: %b  | imm16Handler_EX: %b  | S0_S2: %b  | instruction: %b ",
        $time,clk,PC_dummy,pb,hi_out_signal,lo_out_signal, PC_EX, operand2_handler_out, imm16Handler_EX,S0_S2,instruction);
    end
    


endmodule