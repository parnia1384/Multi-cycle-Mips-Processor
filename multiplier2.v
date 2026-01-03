`timescale 1ns/1ns
module multiplier(
   input clk,  
   input start,
   input [31:0] A, 
   input [31:0] B, 
   output wire [31:0] lo,
   output wire [31:0] hi,
   output wire ready
    );
    reg[63:0] Product;
    reg [5:0] counter;
    reg [31:0] accumulative;
    
    wire [32:0] Modifier;
    assign ready = counter[5];
    assign Modifier = (!Product[0]) ? ({1'b0,Product[63:32]}) : ({1'b0,Product[63:32]} + accumulative);
    always @ (posedge clk) begin
        if(start) begin
            counter <= 0;
            Product <= {32'b0, A};
            accumulative <= B;
        end
        else if(!ready) begin
            counter <= counter + 1;
            Product <= {Modifier, Product[31:1]};
        end
    end
    assign lo = (ready) ? Product[31:0] : 0;
    assign hi = (ready) ? Product[63:32] : 0;
endmodule