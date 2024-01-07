module Operand2_Handler (
    input [31:0] PB, HI, LO, PC,
    input [15:0] imm16,
    input [2:0] S0_S2,
    output reg [31:0] N
);

always @(*) begin  // Changed from @(S0_S2) to @(*) for sensitivity to all inputs
    case(S0_S2)
        3'b000: N = PB;
        3'b001: N = HI;
        3'b010: N = LO;
        3'b011: N = PC;
        3'b100: N = {{16{imm16[15]}}, imm16};  // Sign extension
        3'b101: N = {16'b0000000000000000, imm16}; // Zero extension
        default: N = 32'b0;  // Default case
    endcase
end

endmodule


/*
module Operand2_Handler_tb;

    reg [31:0] PB = 32'b00010010001101000101011001111000;
    reg [31:0] HI = 32'b10000111011001010100001100100001;
    reg [31:0] LO = 32'b10101011110011011110111110101011;
    reg [31:0] PC = 32'b11111110110111001011101010011000;
    reg [15:0] imm16;
    reg [2:0] S0_S2;
    wire [31:0] N;

    // Instantiate the Operand2 Handler module
    Operand2_Handler uut (
        .PB(PB),
        .HI(HI),
        .LO(LO),
        .PC(PC),
        .imm16(imm16),
        .S0_S2(S0_S2),
        .N(N)
    );

    initial begin
        $display("%s                  %s                  %s                       %s", "S2", "S1", "S0", "N");
        
        // Test with imm16 = 1110110001000100
        imm16 = 16'b1110110001000100;
        S0_S2 = 3'b000;
        #10;
        $display("%b                    %b                   %b           %b", S0_S2[2], S0_S2[1], S0_S2[0], N);

        S0_S2 = 3'b001;
        #10;
        $display("%b                    %b                   %b           %b", S0_S2[2], S0_S2[1], S0_S2[0], N);

        S0_S2 = 3'b010;
        #10;
        $display("%b                    %b                   %b           %b", S0_S2[2], S0_S2[1], S0_S2[0], N);

        S0_S2 = 3'b011;
        #10;
        $display("%b                    %b                   %b           %b", S0_S2[2], S0_S2[1], S0_S2[0], N);

        S0_S2 = 3'b100;
        #10;
        $display("%b                    %b                   %b           %b", S0_S2[2], S0_S2[1], S0_S2[0], N);

        S0_S2 = 3'b101;
        #10;
        $display("%b                    %b                   %b           %b", S0_S2[2], S0_S2[1], S0_S2[0], N);

        // Test with imm16 = 0110110001000100
        imm16 = 16'b0110110001000100;
        S0_S2 = 3'b100;
        #10;
        $display("%b                    %b                   %b           %b", S0_S2[2], S0_S2[1], S0_S2[0], N);
    end


 endmodule
*/