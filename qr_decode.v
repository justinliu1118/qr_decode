module qr_decode(
    input clk,                          //clock input
    input srstn,                        //synchronous reset (active low)
    input qr_decode_start,              //start decoding for one QR code
                                        //1: start (one-cycle pulse)
    input [15:0] sram_rdata,            //read data from SRAM
    output reg [9:0] sram_raddr,            //read address to SRAM

    output decode_valid,                //decoded code is valid
    output [7:0] decode_jis8_code,      //decoded JIS8 code
    output qr_decode_finish             //1: decoding one QR code is finished
);
parameter IDLE=2'b00, SEARCH=2'b01, ROTATE=2'b10, GETCODE=2'b11;

wire search_rotate;
wire rotate_getcode;
wire getcode_demask;
wire demask_decode;

wire [9:0] search_raddr, rotate_raddr, getcode_raddr;
wire [624:0] search_out;
wire scale;
wire [3:0] rotate;
wire [3:0] find_pos;
wire [9:0] find_addr;
wire [3:0] get_pos;
wire [9:0] get_addr;
wire[624:0] qrcode;
wire [223:0] demask_out;

reg [1:0] state, state_nxt;

always @(posedge clk) state <= state_nxt;

always @(*) begin
    case(state)
    IDLE: state_nxt = (qr_decode_start) ? SEARCH:IDLE;
    SEARCH: state_nxt = (search_rotate) ? ROTATE:SEARCH;
    ROTATE: state_nxt = (rotate_getcode) ? GETCODE:ROTATE;
    GETCODE: state_nxt = (getcode_demask) ? IDLE : GETCODE;
    default: state_nxt = IDLE;
    endcase
end

always @(*) begin
    case(state) //synopsys parallel_case
    IDLE: sram_raddr=0;
    SEARCH: sram_raddr = search_raddr;
    ROTATE: sram_raddr = rotate_raddr;
    GETCODE: sram_raddr = getcode_raddr;
    endcase
end

search search(
    .clk(clk),                         
    .srstn(srstn),
    .start(qr_decode_start),              
                                        
    .sram_rdata(sram_rdata),          
    .sram_raddr(search_raddr),            

    .scale(scale),
    .find_pos(find_pos),
    .find_addr(find_addr),
    .finish(search_rotate)
);

rotate rotate_m(
    .clk(clk),                          
    .srstn(srstn),
    .start(search_rotate),            
                                        
    .sram_rdata(sram_rdata),            
    .scale(scale),
    .find_pos(find_pos),
    .find_addr(find_addr),
    .sram_raddr(rotate_raddr),
    .rotate(rotate),         
    .get_addr(get_addr),
    .get_pos(get_pos),
    .finish(rotate_getcode)
);

getcode getcode(
    .clk(clk),                        
    .start(rotate_getcode),
    .rotate(rotate),              
    .scale(scale),
    .get_pos(get_pos),
    .get_addr(get_addr),
                                        
    .sram_rdata(sram_rdata),           
    .sram_raddr(getcode_raddr),
    .load_finish(getcode_demask),
    .out(qrcode)  
);

demask demask(
    .clk(clk),
    .srstn(srstn),
    .start(getcode_demask),
    .in(qrcode),
    .finish(demask_decode),
    .demask_out(demask_out)
);

decode decode(
    .clk(clk),
    .start(demask_decode),
    .in(demask_out),
    .code_out(decode_jis8_code),
    .valid(decode_valid),
    .finish(qr_decode_finish)
);

endmodule



