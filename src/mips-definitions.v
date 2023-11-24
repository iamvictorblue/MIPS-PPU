// Function codes for R-type instructions
`define FUNC_ADD  6'b100000
`define FUNC_ADDU 6'b100001
`define FUNC_SUB  6'b100010
`define FUNC_SUBU 6'b100011
`define FUNC_SLT  6'b101010
`define FUNC_SLTU 6'b101011
`define FUNC_AND  6'b100100
`define FUNC_OR   6'b100101
`define FUNC_XOR  6'b100110
`define FUNC_NOR  6'b100111
`define FUNC_SLL  6'b000000
`define FUNC_SRL  6'b000010
`define FUNC_SRA  6'b000011
`define FUNC_SLLV 6'b000100
`define FUNC_SRLV 6'b000110
`define FUNC_SRAV 6'b000111
`define FUNC_JR   6'b001000
`define FUNC_JALR 6'b001001
`define FUNC_MFHI 6'b010000
`define FUNC_MTHI 6'b010001
`define FUNC_MFLO 6'b010010
`define FUNC_MTLO 6'b010011
`define FUNC_MOVN 6'b001011
`define FUNC_MOVZ 6'b001010

// Opcodes for I-type and J-type instructions
`define OPCODE_J     6'b000010
`define OPCODE_JAL   6'b000011
`define OPCODE_BEQ   6'b000100
`define OPCODE_BNE   6'b000101
`define OPCODE_BLEZ  6'b000110
`define OPCODE_BGTZ  6'b000111
`define OPCODE_ADDI  6'b001000
`define OPCODE_ADDIU 6'b001001
`define OPCODE_SLTI  6'b001010
`define OPCODE_SLTIU 6'b001011
`define OPCODE_ANDI  6'b001100
`define OPCODE_ORI   6'b001101
`define OPCODE_XORI  6'b001110
`define OPCODE_LUI   6'b001111
`define OPCODE_LB    6'b100000
`define OPCODE_LH    6'b100001
`define OPCODE_LW    6'b100011
`define OPCODE_LBU   6'b100100
`define OPCODE_LHU   6'b100101
`define OPCODE_SB    6'b101000
`define OPCODE_SH    6'b101001
`define OPCODE_SW    6'b101011
`define OPCODE_REGIMM 6'b000001

// Special 'rt' field for REGIMM instructions
`define RT_BLTZ   5'b00000
`define RT_BGEZ   5'b00001
`define RT_BLTZAL 5'b10000
`define RT_BGEZAL 5'b10001
