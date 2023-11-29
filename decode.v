module decode(
    input clk,
    input start,
    input [223:0] in,
    output reg [7:0] code_out,
    output reg valid,
    output reg finish
);

parameter IDLE = 2'b00, DECODE = 2'b01, FIN = 2'b10;

reg [1:0] state, state_nxt;
reg [211:0] code_buf, code_buf_nxt;
reg [4:0] cnt, cnt_nxt;
wire [7:0] wordnum;

assign wordnum = in[219:212];

always @(posedge clk) begin
    state <= state_nxt;
    code_buf <= code_buf_nxt;
    cnt <= cnt_nxt;
end

always @(*) begin
    case(state) //synopsys parallel_case
    IDLE: state_nxt = (start) ? DECODE : IDLE;
    DECODE: state_nxt = (cnt == wordnum-1) ? FIN : DECODE;
    FIN: state_nxt = IDLE;
    default: state_nxt = IDLE;
    endcase
end

always @(*) begin
    code_buf_nxt = (start) ? in[211:0] : (state == DECODE) ? {code_buf[203:0], code_buf[211:204]} : 0;
    cnt_nxt = (state == DECODE) ? (cnt == wordnum-1) ? 0 : cnt+1'b1 : 0;
    code_out = (state == DECODE) ? code_buf[211:204] : 0;
    finish = (state == FIN) ? 1 : 0 ;
    valid = (state == DECODE) ? 1 : 0;
end

endmodule