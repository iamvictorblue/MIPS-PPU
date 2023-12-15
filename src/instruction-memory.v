
`timescale 1ns / 1ps

module rom_512x8 (output reg [31:0] DataOut, input [8:0] Address);
    reg [7:0] Mem[0:511];       //512 9bit locations
    always@(Address)            //Loop when Address changes
        begin 
            DataOut = {Mem[Address], Mem[Address+1], Mem[Address+2], Mem[Address+3]};
        end
endmodule