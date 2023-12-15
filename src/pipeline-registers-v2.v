module IF_ID_Register(
    input wire clk,
    input wire reset,
   
    input wire [31:0] instruction_in,
    input wire [31:0] PC, 
    input wire LE,
    output reg [31:0] instruction_out,
    output reg [31:0] pc_out,
    output reg [15:0] imm16,                // Imm16
    output reg [25:0] addr26,            // address26
    output reg [15:0] imm16Handler,
    output reg [4:0]  rs,            // rs
    output reg [4:0]  rt,              // rt
    output reg [4:0]  rd            // rd
);
//es reset sincronico
    always @(posedge clk) 
        if (reset) begin
            // Reset the registers when reset signal is asserted
            instruction_out <= 32'b0;       
            pc_out <= 32'b0; 
            imm16 <= 16'b0;
            addr26 <= 26'b0;
            imm16Handler <= 16'b0;
            rs <= 5'b0;
            rt <= 5'b0;
            rd <= 5'b0;
        end else begin
            instruction_out <= instruction_in;
            pc_out <= PC;
            imm16 <= instruction_in[15:0];
            addr26 <= instruction_in[25:0];
            imm16Handler <= instruction_in[15:0];
            rs <= instruction_in[25:21];
            rt <= instruction_in[20:16];
            rd <= instruction_in[15:11];
    end
endmodule

module ID_EX_Register(
    input clk, 
    input reset,

    input [31:0] instruction_in,
    input [31:0] PC, // fase 4
    input [26:0] control_signals_in, //control signals in
    input wire [4:0]  rs_ID,            // rs
    input wire [4:0]  rt_ID,              // rt
    input wire [4:0]  rd_ID,

    input wire [31:0] hi_signal_ID,
    input wire [31:0] lo_signal_ID,
    input wire [15:0] imm16Handler_ID,

    input wire [31:0] ID_MX1,
    input wire [31:0] ID_MX2,

    input wire [4:0] WriteDestination_ID,
    input wire [31:0] JalAdder_ID,
    input wire [31:0] ID_TA,

    output reg [4:0]  EX_Data_MEM_instr,
    output reg [3:0]  EX_ALU_OP_instr,
    output reg [12:0]  EX_control_unit_instr, //control signals out

    output reg [31:0] JalAdder_EX,
    output reg [4:0] WriteDestination_EX,
    output reg [31:0] hi_signal_EX,
    output reg [31:0] lo_signal_EX,
    output reg [15:0] imm16Handler_EX,


    output reg [31:0] EX_MX1,
    output reg [31:0] EX_MX2,
    output reg [4:0]  rs_EX,            // rs
    output reg [4:0]  rt_EX,              // rt
    output reg [4:0]  rd_EX,
    output reg [31:0] EX_TA,
    output reg [31:0] PC_EX// para source operand2 handler
    
);

always @(posedge clk) 
    if (reset) begin
        // Reset the registers and control signals when reset signal is asserted
        JalAdder_EX <= 32'b0;
        WriteDestination_EX <= 5'b0;
        hi_signal_EX <= 32'b0;
        lo_signal_EX <= 32'b0;
        imm16Handler_EX <= 16'b0;
        EX_MX1 <= 32'b0;
        EX_MX2 <= 32'b0; 
        rs_EX <= 5'b0;
        rt_EX <= 5'b0;
        rd_EX <= 5'b0;
        PC_EX <= 32'b0;
        EX_ALU_OP_instr <= 4'b0;
        EX_control_unit_instr <= 14'b0;
        EX_TA <= 32'b0;
    end else begin
        // Copy input values to respective output registers and control signals
        JalAdder_EX <= JalAdder_ID;
        WriteDestination_EX <= WriteDestination_ID;
        hi_signal_EX <= hi_signal_ID;
        lo_signal_EX <= lo_signal_ID;
        imm16Handler_EX <= imm16Handler_ID;
        EX_MX1 <= ID_MX1;
        EX_MX2 <= ID_MX2;
        rs_EX <= instruction_in[25:21];   
        rt_EX <=  instruction_in[20:16];
        rd_EX <=  instruction_in[15:11];
        EX_TA <= ID_TA;
        EX_ALU_OP_instr <= control_signals_in[15:13];
        EX_control_unit_instr <= control_signals_in[15:2]; //EX control signals
        PC_EX <= PC;
    end



endmodule    

// prueba EX/MEM
module EX_MEM_Register(
    input clk,
    input reset,
    input wire [31:0] PC,
    input wire [4:0] WriteDestination_EX,
    input wire [31:0] JalAdder_EX,
    input wire [31:0] EX_MX2,
    input wire [31:0] EX_ALU_OUT,
    input wire [13:0] EX_control_signals_in, // Include relevant control signals from EX stage

    output reg [31:0] MEM_ALU_OUT,
    output reg [31:0] MEM_MX2,
    output reg [31:0] JalAdder_MEM,
    output reg [4:0] WriteDestination_MEM,
    output reg [31:0] PC_MEM,
    output reg [5:0] EX_MEM_control_signals, // Output relevant control signals for MEM stage
    output reg [4:0] Data_Mem_instructions
);

always @(posedge clk) 
    if (reset) begin
        // Reset the registers and control signals when reset signal is asserted
        MEM_ALU_OUT <= 32'b0;
        MEM_MX2 <= 32'b0;
        JalAdder_MEM <= 32'b0;
        WriteDestination_MEM <= 5'b0;
        PC_MEM <= 32'b0;
        EX_MEM_control_signals <= 6'b0;
        Data_Mem_instructions <= 5'b0;
    end else begin
        // Copy input values to respective output registers and control signals
        MEM_ALU_OUT <= EX_ALU_OUT;
        MEM_MX2 <= EX_MX2;
        Data_Mem_instructions <= EX_control_signals_in[20:16];
        JalAdder_MEM <= JalAdder_EX;
        WriteDestination_MEM <= WriteDestination_EX;
        PC_MEM <= PC;
        EX_MEM_control_signals <= EX_control_signals_in[9:4]; // Relevant signals for MEM stage
    end
endmodule

module MEM_WB_Register(

    input clk,
    input reset,
    input wire [5:0] MEM_control_signals_in, // Include relevant control signals from MEM stage
    input  MEM_MUX,
    input wire [4:0] WriteDestination_MEM,
    input wire [31:0] JalAdder_MEM,

    output reg [31:0] MEM_OUT,
    output reg [31:0] JalAdder_WB,
    output reg [4:0] WriteDestination_WB,
    output reg [4:0] MEM_WB_control_signals// Output relevant control signals for WB stage

    
);

always @(posedge clk) 
    if (reset) begin
        // Reset the registers and control signals when reset signal is asserted
        MEM_OUT <= 32'b0;
        JalAdder_WB <= 32'b0;
        WriteDestination_WB <= 5'b0;
        MEM_WB_control_signals <= 5'b0;

        
    end else begin
        // Copy input values to respective output registers and control signals
        MEM_OUT <= MEM_MUX;
        JalAdder_WB <= JalAdder_MEM;
        WriteDestination_WB <= WriteDestination_MEM;
        MEM_WB_control_signals <= MEM_control_signals_in[4:0]; // Relevant signals for WB stage

    end
endmodule



// endmodule

// module MEM_WB_Register(
//     input clk,
//     input reset,
//     input [31:0] alu_result_in,
//     input [31:0] mem_data_in, // Data read from memory
//     input [4:0] write_reg_in,
//     input [8:0] control_signals_in,
//     output reg [31:0] alu_result_out,
//     output reg [31:0] mem_data_out,
//     output reg [4:0] write_reg_out,
//     output reg [8:0] control_signals_out
// );

// always @(posedge clk or posedge reset) begin
//     if (reset) begin
//         alu_result_out <= 32'b0;
//         mem_data_out <= 32'b0;
//         write_reg_out <= 5'b0;
//         control_signals_out <= 9'b0;
//     end else begin
//         alu_result_out <= alu_result_in;
//         mem_data_out <= mem_data_in;
//         write_reg_out <= write_reg_in;
//         control_signals_out <= control_signals_in;
//     end
// end

// endmodule

// prueba EX/MEM