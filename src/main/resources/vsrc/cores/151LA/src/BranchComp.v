module BranchComp #(
    parameter size = 32
) (
    //input
    RS1_data,
    RS2_data,
    BrUn,

    //output
    BrEq,
    BrLT
);

  //I/O ports
  input BrUn;
  input [size-1:0] RS1_data;
  input [size-1:0] RS2_data;
  output BrEq;
  output BrLT;

  //Internal Signals
  wire [size-1:0] RS1_data_m, RS2_data_m;

  //Main function
  assign BrEq = (RS1_data == RS2_data) ? 1 : 0;
  assign RS1_data_m = (BrUn) ? -RS1_data : RS1_data;
  assign RS2_data_m = (BrUn) ? -RS2_data : RS2_data;
  assign BrLT = (~BrUn && ($signed(RS1_data) < $signed(RS2_data))) ? 1 : 
                (BrUn && ((RS1_data) < (RS2_data))) ? 1 : 0;





    
endmodule