`timescale 1s / 1s

module ControlUnitMUX(
    input CMUX,
    input [24:0] control_signals_in, // Input control signals
    output reg [24:0] control_signals_out //  control signals
);

    always @(*) begin
        if (CMUX == 1'b1) begin
            control_signals_out <= control_signals_in[23:0];
        end else begin
            control_signals_out = 24'b0; //  zeros for all control signals
        end
    end

endmodule


module ControlUnit(
    input [31:0] instruction,
    output reg [24:0] instr_signals      
);
     reg Cond_Mux;    //1     // condition check, e.g., for branches
     reg Jump; //2
     reg Branch;  //3  
     reg JalAdder;    //4   
     reg CMUX; //5
     reg TaMux;    //6        
     reg [1:0] WriteDestination; //7,8 //2b01 = rd,2b10 = rt,2b11 = r31 
     reg [2:0] S0_S2;         //9,10,11
     reg Base_Addr_MUX;     //12  // This could be equivalent to something like RegFileEnable but for base address
     reg RsAddrMux;         //13
     reg [3:0] ALUOp; //14,15,16
     reg Data_Mem_RW;        //17 
     reg Data_Mem_Enable;  //18   
     reg [1:0] Data_Mem_Size; //19,20
     reg Data_Mem_SE; //21
     reg HiEnable;     //22
     reg RegFileEnable;//23 // Equivalent to RegFileEnable, enables writing to the register file
     reg Jump_Addr_MUX_Enable;//24
     reg LoEnable;    //25    // Equivalent to LoEnable, enables LO register operations
     reg MemtoReg;    //26
     reg Load;   
     reg MEM_MUX;     //27


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

    Cond_Mux = 0;
    Jump = 0;
    Branch = 0;
    JalAdder = 0;
    CMUX = 1;
    TaMux = 0;
    WriteDestination = 2'b00;// 3 for r31, 2 saves to rt, 1 saves to rd
    S0_S2 = 3'b000;
    Base_Addr_MUX = 0;
    RsAddrMux = 0;
    ALUOp = 4'b0000;
    Data_Mem_RW = 0;
    Data_Mem_Enable = 0;
    Data_Mem_Size = 2'b00;
    Data_Mem_SE = 0;
    HiEnable = 0;
    RegFileEnable = 0;
    Jump_Addr_MUX_Enable = 0;
    LoEnable = 0;
    MemtoReg = 0;
    Load = 0; 
    instr_signals = 27'b0;
    MEM_MUX = 0;

        // Decode the instruction based on opcode and function code
        case (instruction[31:26]) // Check the opcode

            OPCODE_RTYPE, OPCODE_SPECIAL: begin
                case (instruction[5:0]) // Check the function code for R-type instructions
                    // Add cases for specific R-type instructions here
                    FUNC_SUBU: begin
                        ALUOp = 4'b0001; // Example ALU operation code for SUBU
                        RegFileEnable = 1;
                        WriteDestination = 2'b11; // Writes to 'rd'
                        S0_S2 = 3'b100; 
                    end
                    FUNC_JR: begin
                        Jump = 1;
                        RsAddrMux = 1;
                        
                    end
                    FUNC_MFHI: begin
                        // Control signals specific to the MFHI instruction
                        S0_S2 = 3'b001; // Select HI register
                        HiEnable = 1;
                        
                    end
                    FUNC_MFLO: begin
                        // Control signals specific to the MFLO instruction
                        S0_S2 = 3'b010; // Select LO register
                        LoEnable = 1;
                        
                    end
                    // ... (Other R-type)
                endcase
            end

            OPCODE_ADDIU: begin
                ALUOp = 4'b0000; // Example ALU operation code for ADDIU
                RegFileEnable = 1;
                WriteDestination = 2'b01; // Writes to 'rt'
                S0_S2 = 3'b100; // Select imm16 || sign extended
            end
            OPCODE_LBU: begin
                ALUOp = 4'b0000; // Example ALU operation code for LBU
                RegFileEnable = 1;
                Load = 1;
                WriteDestination = 2'b01; // Writes to 'rt'
                Data_Mem_Enable = 1;
                Data_Mem_RW = 0; //load
                Data_Mem_Size = 2'b01;
                Data_Mem_SE = 1; // sign_extension16 || Mem[A] || Mem[A+1]
                S0_S2 = 3'b100; // Select imm16 || se
                MEM_MUX = 1;
            end
            OPCODE_SB: begin
                ALUOp = 4'b0000; // Example ALU operation code for SB
                Data_Mem_RW = 1; //store
                Data_Mem_Enable = 1;
                Data_Mem_Size = 2'b01;
                Data_Mem_SE = 0; // sign_extension16 || Mem[A] || Mem[A+1]
                WriteDestination = 2'b01; // Writes to 'rt'
                MEM_MUX = 0;
            end
            OPCODE_BGTZ: begin
                ALUOp = 4'b1001; // Example ALU operation code for BGTZ
                Branch = 1;
                RsAddrMux = 0;
                Base_Addr_MUX = 0;
            end
            OPCODE_LUI: begin
                ALUOp = 4'b0110; // Example ALU operation code for LUI
                RegFileEnable = 1;
                WriteDestination = 2'b01; // Writes to 'rt'
                S0_S2 = 3'b100; // Select imm16 || 0x0000
            end
            OPCODE_JAL: begin
                Jump = 1;
                JalAdder = 1;// PC+8 is calculated
                RegFileEnable = 1;
                WriteDestination = 2'b10; //r31
                MemtoReg = 1;
            end

            OPCODE_J: begin
                // For J, no control signals are set except for those affecting the PC
            end
            
            // Immediate (I-type) instructions
            OPCODE_ADDI, OPCODE_SLTI, OPCODE_SLTIU, 
            OPCODE_ANDI, OPCODE_ORI, OPCODE_XORI: begin
                ALUOp = 1; // ALU operation for arithmetic, logical, or immediate value calculation
                RegFileEnable = 1; // Set for instructions that write back to a register
                WriteDestination = 2'b01; // Immediate instructions write to 'rt' (I-type)
            end
            // Load instructions
            OPCODE_LB, OPCODE_LH, OPCODE_LW, OPCODE_LBU, OPCODE_LHU: begin
                ALUOp = 1; // For address calculation
                RegFileEnable = 1; // Data from memory will be written back to the register file
                Load = 1; // Enable signal for indicating a load from memory
                WriteDestination = 2'b01; // Load instructions write to 'rt' (I-type)
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
                        
                        // Set ALUOp for branch condition check, might need a specific operation
                         // Set appropriate ALU operation code
                        // If needed for selecting the source register for ALU
                        // Ensure other control signals like Data_Mem_RW, WriteDestination are not set
                        //...
                    end
                    RT_BLTZAL, RT_BGEZAL: begin
                        RegFileEnable = 1; // Write back the return address for link instructions
                        WriteDestination = 2'b10; //r31
                        JalAdder = 1;
                    end
                endcase
            end

            default: begin
                // Handle undefined instruction or set to NOP
            end
        endcase

    // instr_signals[0] = Cond_Mux;
    // instr_signals[1] = Jump;
    // instr_signals[2] = Branch;
    // instr_signals[3] = JalAdder;
    // instr_signals[4] = CMUX;
    // instr_signals[5] = TaMux;
    // instr_signals[7:6] = WriteDestination;
    // instr_signals[10:8] = S0_S2;
    // instr_signals[11] = Base_Addr_MUX;
    // instr_signals[12] = RsAddrMux;
    // instr_signals[15:13] = ALUOp;
    // instr_signals[16] = Data_Mem_RW;
    // instr_signals[17] = Data_Mem_Enable;
    // instr_signals[19:18] = Data_Mem_Size;
    // instr_signals[20] = Data_Mem_SE;
    // instr_signals[21] = HiEnable;
    // instr_signals[22] = RegFileEnable;
    // instr_signals[23] = Jump_Addr_MUX_Enable;
    // instr_signals[24] = LoEnable;
    // instr_signals[25] = MemtoReg;
    // instr_signals[26] = Load;

    instr_signals[0] = Load;             // WB Stage
    instr_signals[1] = MemtoReg;         // WB Stage
    instr_signals[2] = LoEnable;         // WB Stage
    instr_signals[3] = RegFileEnable;    // WB Stage
    instr_signals[4] = HiEnable;         // WB Stage
    instr_signals[5] = MEM_MUX;      // MEM Stage
    instr_signals[6] = Data_Mem_SE;      // MEM Stage
    instr_signals[8:7] = Data_Mem_Size; // MEM Stage
    instr_signals[9] = Data_Mem_Enable;  // MEM Stage
    instr_signals[10] = Data_Mem_RW;     // MEM Stage
    instr_signals[14:11] = ALUOp;        // EX Stage
    instr_signals[17:15] = S0_S2;        // EX Stage
    instr_signals[18] = RsAddrMux;       // ID
    instr_signals[19] = Base_Addr_MUX;  // ID
    instr_signals[21:20] = WriteDestination; // ID Stage
    instr_signals[22] = CMUX;            // ID Stage
    instr_signals[23] = JalAdder;        // ID Stage
    instr_signals[24] = Jump;            // IF Stage
    instr_signals[25] = Cond_Mux;        // IF Stage


    
    end
endmodule
