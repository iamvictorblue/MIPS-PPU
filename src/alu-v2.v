module ALU(
    input [31:0] A,
    input [31:0] B,
    input [3:0] opcode,
    output reg [31:0] Out,
    output reg Z,
    output reg N
);

always @ (*) begin
    case(opcode)
        4'b0000: begin // A + B, Z N
            Out = A + B;
            Z = (Out == 32'b0);
            N = (Out[31] == 1);
        end
        4'b0001: begin // A - B, Z N
            Out = A - B;
            Z = (Out == 32'b0);
            N = (Out[31] == 1);
        end
        4'b0010: begin // A and B ,none
            Out = A & B;
            Z = 1'b0;
            N = 1'b0;
        end
        4'b0011: begin // A or B ,none
            Out = A | B;
            Z = 1'b0;
            N = 1'b0;
        end
        4'b0100: begin // A xor B,none
            Out = A ^ B;
            Z = 1'b0;
            N = 1'b0;
        end
        4'b0101: begin // A nor B ,none
            Out = ~(A | B);
            Z = 1'b0;
            N = 1'b0;
        end
        4'b0110: begin // shift left logical (B) A positions 
            Out = B << A;
            Z = 1'b0;
            N = 1'b0;
        end
        4'b0111: begin // shift right logical (B) A positions 
            Out = B >> A;
            Z = 1'b0;
            N = 1'b0;
        end
        4'b1000: begin // shift right arithmetic (B) A positions 
            Out = $signed(B) >>> A;
            Z = 1'b0;
            N = 1'b0;
        end
        4'b1001: begin // If (A < B) then Out=1, else Out=0 ,Z N
            Out = (A < B) ? 1 : 0;
            Z = (Out == 0); 
            N = 1'b0;     
        end


        4'b1010: begin // A 
            Out = A;
            Z = (Out == 32'b0);
            N = (Out[31] == 1);
        end
        4'b1011: begin // B 
            Out = B;
            Z = (Out == 32'b0);
            N = (Out[31] == 1);
        end
        4'b1100: begin // B + 8 
            Out = B + 8;
            Z = (Out == 32'b0);
            N = (Out[31] == 1);
        end
        default: begin
            Out = 32'b0;
            Z = 1'b0;
            N = 1'b0;
        end
    endcase
end

endmodule

// `timescale 1ns / 1ps

// module ALU_Testbench;

// reg [31:0] A, B;
// reg [3:0] opcode;
// wire [31:0] Out;
// wire Z, N;

// // Instantiate the ALU module
// ALU uut (
//     .A(A),
//     .B(B),
//     .opcode(opcode),
//     .Out(Out),
//     .Z(Z),
//     .N(N)
// );


// reg clk = 0;
// always #5 clk = ~clk;


// initial begin
//     $display("Opcode    A                                B                                Result                                         A(dec)           B(dec)      Result(dec)            Z    N ");

//     // Test Case 1
//     A = 32'b10;
//     B = 32'b11;
//     opcode = 4'b0000; // A + B, Z N
//     #10;
//     $display("%b   %b   %b   %b        %d         %d         %d           %b   %b", opcode, A, B, Out, A, B, Out, Z, N);

//     // Test Case 2
//     A = 32'b10;
//     B = 32'b10;
//     opcode = 4'b0001; // A - B, Z N
//     #10;
//     $display("%b   %b   %b   %b        %d         %d         %d           %b   %b", opcode, A, B, Out, A, B, Out, Z, N);

//     // Test Case 3
//     A = 32'b101;
//     B = 32'b11;
//     opcode = 4'b0010; // A and B
//     #10;
//     $display("%b   %b   %b   %b        %d         %d         %d           %b   %b", opcode, A, B, Out, A, B, Out, Z, N);

//     // Test Case 4
//     A = 32'b101;
//     B = 32'b11;
//     opcode = 4'b0011; // A or B
//     #10;
//     $display("%b   %b   %b   %b        %d         %d         %d           %b   %b", opcode, A, B, Out, A, B, Out, Z, N);

//     // Test Case 5
//     A = 32'b101;
//     B = 32'b11;
//     opcode = 4'b0100; // A xor B
//     #10;
//     $display("%b   %b   %b   %b        %d         %d         %d           %b   %b", opcode, A, B, Out, A, B, Out, Z, N);

//     // Test Case 6
//     A = 32'b101;
//     B = 32'b11;
//     opcode = 4'b0101; // A nor B
//     #10;
//     $display("%b   %b   %b   %b        %d         %d         %d           %b   %b", opcode, A, B, Out, A, B, Out, Z, N);

//     // Test Case 7
//     A = 32'b10;
//     B = 32'b1;
//     opcode = 4'b0110; // Shift Left Logical (B) A positions
//     #10;
//     $display("%b   %b   %b   %b        %d         %d         %d           %b   %b", opcode, A, B, Out, A, B, Out, Z, N);

//     // Test Case 8
//     A = 32'b100;
//     B = 32'b10;
//     opcode = 4'b0111; // Shift Right Logical (B) A positions
//     #10;
//     $display("%b   %b   %b   %b        %d         %d         %d           %b   %b", opcode, A, B, Out, A, B, Out, Z, N);

//     // Test Case 9
//     A = 32'b1001;
//     B = 32'b1;
//     opcode = 4'b1000; // Shift Right Arithmetic (B) A positions
//     #10;
//     $display("%b   %b   %b   %b        %d         %d         %d           %b   %b", opcode, A, B, Out, A, B, Out, Z, N);

//     // Test Case 10
//     A = 32'b11;
//     B = 32'b11;
//     opcode = 4'b1001; // If (A < B) then Out=1, else Out=0
//     #10;
//     $display("%b   %b   %b   %b        %d         %d         %d           %b   %b", opcode, A, B, Out, A, B, Out, Z, N);

//     // Test Case 11
//     A = 32'b10;
//     B = 32'b11;
//     opcode = 4'b1010; // A
//     #10;
//     $display("%b   %b   %b   %b        %d         %d         %d           %b   %b", opcode, A, B, Out, A, B, Out, Z, N);

//     // Test Case 12
//     A = 32'b10;
//     B = 32'b11;
//     opcode = 4'b1011; // B
//     #10;
//     $display("%b   %b   %b   %b        %d         %d         %d           %b   %b", opcode, A, B, Out, A, B, Out, Z, N);

//     // Test Case 13
//     A = 32'b10;
//     B = 32'b1010;
//     opcode = 4'b1100; // B + 8
//     #10;
//     $display("%b   %b   %b   %b        %d         %d         %d           %b   %b", opcode, A, B, Out, A, B, Out, Z, N);

//     $finish;
// end

// endmodule
