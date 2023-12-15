`timescale 1ns / 1ns

module hazard_forwarding_unit (
    output reg [1:0] forwardMX1,
    output reg [1:0] forwardMX2,

    output reg nPC_LE,
    output reg PC_LE,
    output reg IF_ID_LE,

    output reg CU_S,

    input wire EX_Register_File_Enable,
    input wire MEM_Register_File_Enable,
    input wire WB_Register_File_Enable,

    input wire [4:0] EX_RD,
    input wire [4:0] MEM_RD,
    input wire [4:0] WB_RD,

    input wire [4:0] operandA, // Source operand A in ID stage
    input wire [4:0] operandB, // Source operand B in ID stage
    input wire [4:0] ID_rd,
    input wire EX_load_instr,
    input wire ID_store_instr
);

    always @* begin
        /*//FORWARDING PA (ID_mux1)\\*/
        if (EX_Register_File_Enable && (operandA == EX_RD))         // EX forwarding
                forwardMX1 <= 2'b01;   
            else if (MEM_Register_File_Enable && (operandA == MEM_RD))    // MEM forwarding
                forwardMX1 <= 2'b10; 
            else if (WB_Register_File_Enable && (operandA == WB_RD)) 
                forwardMX1 <= 2'b11; 
            else forwardMX1 <= 2'b00;                                   // Not forwarding (passing PA from the register file)

        /*//FORWARDING PB (ID_mux2)\\*/
        if (EX_Register_File_Enable && (operandB == EX_RD))           // EX forwarding
            forwardMX2 <= 2'b01; 
            else if (MEM_Register_File_Enable && (operandB == MEM_RD))    // MEM forwarding
                forwardMX2 <= 2'b10; 
            else if (WB_Register_File_Enable && (operandB == WB_RD))      // WB forwarding
                forwardMX2 <= 2'b11;                                    // Forwarding [HFU_WB_Rd] to ID
            else forwardMX2 <= 2'b00;                                   // Not forwarding (passing PB from the register file)

        if (EX_load_instr && ((operandA == EX_RD) || (operandB == EX_RD))) begin // Hazard asserted
            nPC_LE          <= 1'b0; // Disable Load Enable of the Next Program Counter (nPC)
            PC_LE           <= 1'b0; // Disable Load Enable of the Program Counter (PC)
            IF_ID_LE        <= 1'b0; // Disable IF/ID Pipeline Register from loading
            CU_S            <= 1'b1; // Forward Control Signals corresponding to a NOP instruction
        end else begin               // Hazard not asserted
            nPC_LE          <= 1'b1; // Program Counter is Load Enable
            PC_LE           <= 1'b1; // Next Program Counter is Load Enable
            IF_ID_LE        <= 1'b1; // IF/ID Pipeline Register is enabled
            CU_S            <= 1'b0; // Dont Forward Control Signals corresponding to a NOP instruction
        end
        // $display("PC_LE: %b, EX_load_instr: %b", PC_LE, EX_load_instr);
    end
endmodule