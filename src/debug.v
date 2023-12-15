`timescale 1ns / 1ns

module debug(


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


    // // Clock generator
    initial begin
        clr <= 1'b1;
        clk <= 1'b0;
        #2 clk <= ~clk;
        #1 clr <= 1'b0;
        #1 clk <= ~clk; 
        forever #2 clk = ~clk;
    end


end
