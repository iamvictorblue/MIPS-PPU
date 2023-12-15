`include "mips-definitions.v"

module Condition_Handler (
    output reg if_id_reset,
    output reg CH_Out,
    input wire [31:0] instruction,
    input wire Z,
    input wire N
); 

// Define opcodes for the instructions
localparam [5:0] 
    OPCODE_RTYPE = 6'b000000,
    OPCODE_SPECIAL = 6'b011100, // For CLO and CLZ
    OPCODE_J    = 6'b000010,
    OPCODE_JAL  = 6'b000011,
    OPCODE_ADDI  = 6'b001000,
    OPCODE_ADDIU = 6'b001001,
    OPCODE_SLTI  = 6'b001010,
    OPCODE_SLTIU = 6'b001011,
    OPCODE_ANDI  = 6'b001100,
    OPCODE_ORI   = 6'b001101,
    OPCODE_XORI  = 6'b001110,
    OPCODE_LUI   = 6'b001111,
    OPCODE_LB    = 6'b100000,
    OPCODE_LH    = 6'b100001,
    OPCODE_LW    = 6'b100011,
    OPCODE_LBU   = 6'b100100,
    OPCODE_LHU   = 6'b100101,
    OPCODE_SB    = 6'b101000,
    OPCODE_SH    = 6'b101001,
    OPCODE_SW    = 6'b101011,
    OPCODE_BEQ   = 6'b000100,
    OPCODE_BNE   = 6'b000101,
    OPCODE_BLEZ  = 6'b000110,
    OPCODE_BGTZ  = 6'b000111,
    OPCODE_REGIMM = 6'b000001; // Opcode for BLTZ, BGEZ, BLTZAL, BGEZAL, and BAL

// Define function codes for R-type instructions
localparam [5:0]
    FUNC_ADD  = 6'b100000,
    FUNC_ADDU = 6'b100001,
    FUNC_SUB  = 6'b100010,
    FUNC_SUBU = 6'b100011,
    FUNC_JR   = 6'b001000,
    FUNC_JALR = 6'b001001,
    FUNC_MFHI = 6'b010000,
    FUNC_MFLO = 6'b010010,
    FUNC_MOVN = 6'b001011,
    FUNC_MOVZ = 6'b001010,
    FUNC_MTHI = 6'b010001,
    FUNC_MTLO = 6'b010011,
    FUNC_SLT  = 6'b101010,
    FUNC_SLTU = 6'b101011,
    FUNC_AND  = 6'b100100,
    FUNC_OR   = 6'b100101,
    FUNC_XOR  = 6'b100110,
    FUNC_NOR  = 6'b100111,
    FUNC_SLL  = 6'b000000,
    FUNC_SLLV = 6'b000100,
    FUNC_SRA  = 6'b000011,
    FUNC_SRAV = 6'b000111,
    FUNC_SRL  = 6'b000010,
    FUNC_SRLV = 6'b000110;

// Define 'rt' field for special REGIMM instructions
localparam [4:0]
    RT_BLTZ   = 5'b00000,
    RT_BGEZ   = 5'b00001,
    RT_BLTZAL = 5'b10000,
    RT_BGEZAL = 5'b10001,
    RT_BAL    = 5'b10001; // Same as BGEZAL


always @* begin
    
    case (instruction[31:26])
        OPCODE_BNE: begin //not equal
            if(Z == 0) CH_Out <= 1'b1;
            else CH_Out <= 1'b0;
        end

        OPCODE_BEQ: begin //equal
            if(Z == 1) CH_Out <= 1'b1;
            else CH_Out <= 1'b0;
        end
        OPCODE_BGTZ: begin //greater than zero
            if(instruction[20:16] == 5'b00000) begin
                if((Z == 0) && (N == 0)) CH_Out <= 1'b1;
                else CH_Out <= 1'b0;
            end
        end 
        OPCODE_BLEZ: begin //lesser or equal than zero
            if(instruction[20:16] == 5'b00000) begin
                if((Z == 1) || (N == 1)) CH_Out <= 1'b1;
                else CH_Out <= 1'b0;
            end
        end // acomodar para chequiar rt con BGTZ y BLEZ
        OPCODE_REGIMM: begin
            case (instruction[20:16])
              RT_BAL: begin //BAL (BGEZAL) //branch and link
                    CH_Out <= 1'b1;
              end
              RT_BGEZ: begin //greater or equal than zero
                    if((Z == 1) || (N == 0)) CH_Out <= 1'b1;
                    else CH_Out <= 1'b0;
              end
              RT_BLTZ: begin //lesser than zero
                    if((Z == 0) && (N == 1)) CH_Out <= 1'b1;
                    else CH_Out <= 1'b0;
              end
              RT_BLTZAL: begin //lesser than zero and link
                    if(N == 1) CH_Out <= 1'b1;
                    else CH_Out <= 1'b0;
              end
            endcase
        end
    endcase

end

endmodule