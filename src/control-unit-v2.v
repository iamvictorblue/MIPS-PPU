`timescale 1ns / 1ps

module ControlUnitMUX(
    input S,
    input [18:0] control_signals_in, // Input control signals
    output reg [18:0] control_signals_out // Output control signals
);

always @(*) begin
    if (S == 1'b0) begin
        control_signals_out = control_signals_in;
    end else begin
        control_signals_out = 18'b0; // Output zeros for all control signals
    end
end

endmodule


module ControlUnit(
    input [31:0] instruction,
    input reset,
    output reg Cond_Mux,           // condition check, e.g., for branches
    output reg Jump,
    output reg Branch,       
    output reg JalAdder,          
    output reg [1:0] CMU,
    output reg TaMux,              
    output reg [1:0] WriteDestination,  
    output reg [2:0] S0_S2,         // Assuming this is part of ALUOp or similar control
    output reg Base_Addr_MUX,       // This could be equivalent to something like RegFileEnable but for base address
    output reg RsAddrMux,         
    output reg [3:0] ALUOp, 
    output reg Data_Mem_RW,         
    output reg Data_Mem_Enable,     
    output reg [1:0] Data_Mem_Size, 
    output reg RegDst,              // Equivalent to WriteDestination, selects destination register
    output reg HiEnable,      
    output reg RegFileEnable,// Equivalent to RegFileEnable, enables writing to the register file
    output reg Jump_Addr_MUX_Enable,
    output reg LoEnable,        // Equivalent to LoEnable, enables LO register operations
    output reg MemtoReg,
    output reg Load           
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


always @(*) begin
    // Default control signals for NOP (no operation) and all other control signals
    ALUOp = 0;
    Load = 0;
    RegFileEnable = 0;
    HiEnable = 0;
    LoEnable = 0;
    JalAdder = 0;
    TaMux = 0;
    RsAddrMux = 0; // 1 does PC=rs, 0 passes normal TA
    WriteDestination = 0; // 1 saves to rt, 0 saves to rd
    RegDst = 0; // 1 saves to $r31


    if (!reset) begin
        // Decode the instruction based on opcode and function code
        case (instruction[31:26]) // Check the opcode
            OPCODE_RTYPE, OPCODE_SPECIAL: begin
                case (instruction[5:0]) // Check the function code for R-type instructions
                    FUNC_ADD, FUNC_ADDU, FUNC_SUB, FUNC_SUBU,
                    FUNC_SLT, FUNC_SLTU, FUNC_AND, FUNC_OR, 
                    FUNC_XOR, FUNC_NOR, FUNC_SLL, FUNC_SLLV, 
                    FUNC_SRA, FUNC_SRAV, FUNC_SRL, FUNC_SRLV: begin
                        // ALUOp = 1;/ hacerlo para 4 bits y hacer logica correspondiente a cada
                        // instruccion tipo R
                        RegFileEnable = 1;
                        WriteDestination = 00b'0; // Select 'rd' field as the destination // cambiar a señal de 2 bis
                    end
                    FUNC_JR, FUNC_JALR: begin
                        RsAddrMux = 1; // Control signal for jump register instructions
                        RegFileEnable = 1;
                        if (instruction[5:0] == FUNC_JALR) begin
                            RegFileEnable = 1; // For JALR, link address is stored in register
                            RsAddrMux = 1;
                            WriteDestination = 0; // Select 'rd' field as the destination register.
                            JalAdder = 1;
                        end
                    end
                    FUNC_MFHI: begin
                        RegFileEnable = 1;
                        HiEnable = 1; // Move from HI register to a register file destination
                        // Set signals for HI register read
                    end
                    FUNC_MFLO: begin
                        RegFileEnable = 1; // Move from LO register to a register file destination
                        LoEnable = 1; // Set signals for LO register read
                    end
                    FUNC_MTHI: begin
                        HiEnable = 1; 
                    end    
                    FUNC_MTLO: begin
                        LoEnable = 1;
                    end
                    // ... (Other cases for MOVN, MOVZ as required)
                endcase
            end
            OPCODE_J: begin
                // For J, no control signals are set except for those affecting the PC
            end
            OPCODE_JAL: begin
                RegFileEnable = 1;
                JalAdder = 1; // PC+8 is calculated
                RegDst = 1; // Link address is stored in $ra ($31)
            end
            // Immediate (I-type) instructions
            OPCODE_ADDI, OPCODE_ADDIU, OPCODE_SLTI, OPCODE_SLTIU, 
            OPCODE_ANDI, OPCODE_ORI, OPCODE_XORI, OPCODE_LUI: begin
                ALUOp = 1; // ALU operation for arithmetic, logical, or immediate value calculation
                RegFileEnable = 1; // Set for instructions that write back to a register
                WriteDestination = 1; // Immediate instructions write to 'rt' (I-type)
            end
            // Load instructions
            OPCODE_LB, OPCODE_LH, OPCODE_LW, OPCODE_LBU, OPCODE_LHU: begin
                ALUOp = 1; // For address calculation
                RegFileEnable = 1; // Data from memory will be written back to the register file
                Load = 1; // Enable signal for indicating a load from memory
                WriteDestination = 1; // Load instructions write to 'rt' (I-type)
            end
            // Store instructions
            OPCODE_SB, OPCODE_SH, OPCODE_SW: begin
                ALUOp = 1; // For address calculation
                // No need to enable register file write, data is written to memory
                // No need for Load or RegFileEnable for store instructions
            end
            // Branch instructions
            OPCODE_BEQ, OPCODE_BNE, OPCODE_BLEZ, OPCODE_BGTZ, OPCODE_REGIMM: begin
                ALUOp = 1; // Branch instructions involve ALU operations
                // No need to enable RegFileEnable, WriteDestination, or Load for branch instructions
                case (instruction[20:16]) // Additional case for REGIMM instructions
                    RT_BLTZ, RT_BGEZ: begin
                        // Set control signals for BLTZ and BGEZ
                    end
                    RT_BLTZAL, RT_BGEZAL: begin
                        RegFileEnable = 1; // Write back the return address for link instructions
                        RegDst = 1; // Write to $r31 for link instructions (JAL-type)
                        JalAdder = 1;
                    end
                endcase
            end

            default: begin
                // Handle undefined instruction or set to NOP
            end
        endcase
    end
end

endmodule
