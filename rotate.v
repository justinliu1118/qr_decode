module rotate(
    input clk,                          //clock input
    input srstn,
    input start,              //start decoding for one QR code
                                        //1: start (one-cycle pulse)
    input [15:0] sram_rdata,            //read data from SRAM
    input scale,
    input [3:0] find_pos,
    input [9:0] find_addr,
    output reg [9:0] sram_raddr,
    output reg [3:0] rotate,          //read address to SRAM
    output reg [9:0] get_addr,
    output reg [3:0] get_pos,
    output reg finish
);

parameter IDLE=1'b0, FIND_ROTATE=1'b1;

reg [9:0] sram_raddr_nxt;
reg [9:0] get_addr_nxt;
reg [3:0] get_pos_nxt;
reg [3:0] rotate_nxt; // [3] 270 [2] 180 [1] 90 [0] 0
reg finish_nxt;

reg state, state_nxt;
reg [3:0] find_cnt, find_cnt_nxt;
reg [63:0] sram_buf, sram_buf_nxt;
reg start_buf;
wire [9:0] read_left, read_right, read_diag_right, read_diag_left, read_down;
wire [24:0] small_pattern;
wire [24:0] large_pattern;

assign small_pattern = 25'b11111_00001_11101_11101_11101;
assign large_pattern = 25'b11111_11111_00011_00011_10011;
assign read_left  = (scale) ? (find_pos[1]) ? find_addr-3'd4 : find_addr-3'd5 : find_addr-4'd9;
assign read_right = (scale) ? (find_pos[1]) ? find_addr+3'd5 : find_addr+3'd4 : find_addr+4'd9;
assign read_down = (scale) ? (find_pos[3]) ? find_addr+8'd160 : find_addr+8'd128 : find_addr+9'd288;
assign read_diag_left = read_down+read_left-find_addr;
assign read_diag_right = read_down+read_right-find_addr;

always @(posedge clk) begin
    if(~srstn) rotate <= 0;
    else rotate <= rotate_nxt;
    state <= state_nxt;
    sram_buf <= sram_buf_nxt;
    find_cnt <= find_cnt_nxt;
    start_buf <= start;
    finish <= finish_nxt;
    get_addr <= get_addr_nxt;
    get_pos <= get_pos_nxt;
    sram_raddr <= sram_raddr_nxt;
end

always @(*) begin
    case(state) //synopsys parallel_case
    IDLE: state_nxt = (start_buf) ? FIND_ROTATE : IDLE;
    FIND_ROTATE: state_nxt = (find_cnt == 4'd15) ? IDLE : FIND_ROTATE;
    default: state_nxt = IDLE;
    endcase
end

always @(*) begin
    find_cnt_nxt = (state == FIND_ROTATE) ? find_cnt + 1'b1 : 0;
    sram_buf_nxt = {sram_rdata, sram_buf[63:16]};
    finish_nxt =   (find_cnt == 4'd15) ? 1 : 0; 
    get_addr_nxt = (find_cnt == 4'd15)?(rotate == 4'b1100) ? read_left : find_addr : get_addr;
    get_pos_nxt  = (find_cnt == 4'd15)?(rotate == 4'b1100) ? (scale) ? {find_pos[3:2], ~find_pos[1], find_pos[0]} : find_pos: find_pos : get_pos;
end

always @(*) begin
    if(state == FIND_ROTATE)
        case(find_cnt) //synopsys parallel_case
        4'd0: sram_raddr_nxt  = read_diag_left+6'd32;
        4'd1: sram_raddr_nxt  = read_diag_left+6'd33;
        4'd2: sram_raddr_nxt  = read_down;
        4'd3: sram_raddr_nxt  = read_down+1'b1;
        4'd4: sram_raddr_nxt  = read_down+6'd32; 
        4'd5: sram_raddr_nxt  = read_down+6'd33; 
        4'd6: sram_raddr_nxt  = read_diag_right;
        4'd7: sram_raddr_nxt  = read_diag_right+1'b1;
        4'd8: sram_raddr_nxt  = read_diag_right+6'd32;
        4'd9: sram_raddr_nxt  = read_diag_right+6'd33;
        4'd10: sram_raddr_nxt = read_right;
        4'd11: sram_raddr_nxt = read_right+1'b1;
        4'd12: sram_raddr_nxt = read_right+6'd32;
        4'd13: sram_raddr_nxt = read_right+6'd33;
        4'd14: sram_raddr_nxt = read_left;
        4'd15: sram_raddr_nxt = read_left;
        endcase
    else if(start_buf)
        sram_raddr_nxt = read_diag_left+1'b1;
    else
        sram_raddr_nxt = read_diag_left;
end

always @(*) begin
    if(state == FIND_ROTATE)
        case(find_cnt[3:2]) //synopsys parallel_case
        2'b00: begin // diag_left
                case({scale, find_pos}) //synopsys parallel_case
                5'b00000:   if({sram_buf[16], sram_buf[3:0], sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32]} == large_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]};
                            else rotate_nxt = rotate;
                5'b00001:   if({sram_buf[17:16], sram_buf[3:1], sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33]} == large_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]};
                            else rotate_nxt = rotate;
                5'b00010:   if({sram_buf[18:16], sram_buf[3:2], sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34]} == large_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]};
                            else rotate_nxt = rotate;
                5'b00011:   if({sram_buf[19:16], sram_buf[3], sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35]} == large_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]};
                            else rotate_nxt = rotate;
                5'b00100:   if({sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36]} == large_pattern) 
                                rotate_nxt = {1'b1, rotate[2:0]};
                            else rotate_nxt = rotate;
                5'b00101:   if({sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37]} == large_pattern) 
                                rotate_nxt = {1'b1, rotate[2:0]};
                            else rotate_nxt = rotate;
                5'b00110:   if({sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38]} == large_pattern) 
                                rotate_nxt = {1'b1, rotate[2:0]};
                            else rotate_nxt = rotate;
                5'b00111:   if({sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39]} == large_pattern) 
                                rotate_nxt = {1'b1, rotate[2:0]};
                            else rotate_nxt = rotate;
                5'b01000:   if({sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40]} == large_pattern) 
                                rotate_nxt = {1'b1, rotate[2:0]};
                            else rotate_nxt = rotate;
                5'b01001:   if({sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41]} == large_pattern) 
                                rotate_nxt = {1'b1, rotate[2:0]};
                            else rotate_nxt = rotate;
                5'b01010:   if({sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42]} == large_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]};
                            else rotate_nxt = rotate;
                5'b01011:   if({sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43]} == large_pattern) 
                                rotate_nxt = {1'b1, rotate[2:0]};
                            else rotate_nxt = rotate;
                5'b01100:   if({sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40], sram_buf[60], sram_buf[47:44]} == large_pattern) 
                                rotate_nxt = {1'b1, rotate[2:0]};
                            else rotate_nxt = rotate;
                5'b01101:   if({sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41], sram_buf[61:60], sram_buf[47:45]} == large_pattern) 
                                rotate_nxt = {1'b1, rotate[2:0]};
                            else rotate_nxt = rotate;
                5'b01110:   if({sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42], sram_buf[62:60], sram_buf[47:46]} == large_pattern) 
                                rotate_nxt = {1'b1, rotate[2:0]};
                            else rotate_nxt = rotate;
                5'b01111:   if({sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43], sram_buf[63:60], sram_buf[47]} == large_pattern) 
                                rotate_nxt = {1'b1, rotate[2:0]};
                            else rotate_nxt = rotate;
                5'b10000:   if({sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42]} == small_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]}; // 2 - 10
                            else rotate_nxt = rotate;
                5'b10001:   if({sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43]} == small_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]}; // 3 - 11
                            else rotate_nxt = rotate;
                5'b10010:   if({sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40]} == small_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]}; // 0 - 8
                            else rotate_nxt = rotate;
                5'b10011:   if({sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41]} == small_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]}; // 1 - 9
                            else rotate_nxt = rotate;
                5'b10100:   if({sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42], sram_buf[62:60], sram_buf[47:46]} == small_pattern) 
                                rotate_nxt = {1'b1, rotate[2:0]}; // 6 - 14
                            else rotate_nxt = rotate;
                5'b10101:   if({sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43], sram_buf[63:60], sram_buf[47]} == small_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]}; // 7 - 15
                            else rotate_nxt = rotate;
                5'b10110:   if({sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40], sram_buf[60], sram_buf[47:44]} == small_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]}; // 4 - 12
                            else rotate_nxt = rotate;
                5'b10111:   if({sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41], sram_buf[61:60], sram_buf[47:45]} == small_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]}; // 5 - 13
                            else rotate_nxt = rotate;
                5'b11000:   if({sram_buf[18:16], sram_buf[3:2], sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34]} == small_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]}; // 10 - 2
                            else rotate_nxt = rotate;
                5'b11001:   if({sram_buf[19:16], sram_buf[3], sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35]} == small_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]}; // 11 - 3
                            else rotate_nxt = rotate;
                5'b11010:   if({sram_buf[16], sram_buf[3:0], sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32]} == small_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]}; // 8 - 0
                            else rotate_nxt = rotate;
                5'b11011:   if({sram_buf[17:16], sram_buf[3:1], sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33]} == small_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]}; // 9 - 1
                            else rotate_nxt = rotate;
                5'b11100:   if({sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38]} == small_pattern) 
                                rotate_nxt = {1'b1, rotate[2:0]}; // 14 - 6
                            else rotate_nxt = rotate;
                5'b11101:   if({sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39]} == small_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]}; // 15 - 7
                            else rotate_nxt = rotate;
                5'b11110:   if({sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36]} == small_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]}; // 12 - 4
                            else rotate_nxt = rotate;
                5'b11111:   if({sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37]} == small_pattern)
                                rotate_nxt = {1'b1, rotate[2:0]}; // 13 - 5
                            else rotate_nxt = rotate;
                endcase
            
        end
        2'b01: begin // down
        case({scale, find_pos}) //synopsys parallel_case
                5'b00000:   if({sram_buf[16], sram_buf[3:0], sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32]} == large_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]};
                            else rotate_nxt = rotate;
                5'b00001:   if({sram_buf[17:16], sram_buf[3:1], sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33]} == large_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]};
                            else rotate_nxt = rotate;
                5'b00010:   if({sram_buf[18:16], sram_buf[3:2], sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34]} == large_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]};
                            else rotate_nxt = rotate;
                5'b00011:   if({sram_buf[19:16], sram_buf[3], sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35]} == large_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]};
                            else rotate_nxt = rotate;
                5'b00100:   if({sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36]} == large_pattern) 
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]};
                            else rotate_nxt = rotate;
                5'b00101:   if({sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37]} == large_pattern) 
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]};
                            else rotate_nxt = rotate;
                5'b00110:   if({sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38]} == large_pattern) 
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]};
                            else rotate_nxt = rotate;
                5'b00111:   if({sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39]} == large_pattern) 
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]};
                            else rotate_nxt = rotate;
                5'b01000:   if({sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40]} == large_pattern) 
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]};
                            else rotate_nxt = rotate;
                5'b01001:   if({sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41]} == large_pattern) 
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]};
                            else rotate_nxt = rotate;
                5'b01010:   if({sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42]} == large_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]};
                            else rotate_nxt = rotate;
                5'b01011:   if({sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43]} == large_pattern) 
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]};
                            else rotate_nxt = rotate;
                5'b01100:   if({sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40], sram_buf[60], sram_buf[47:44]} == large_pattern) 
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]};
                            else rotate_nxt = rotate;
                5'b01101:   if({sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41], sram_buf[61:60], sram_buf[47:45]} == large_pattern) 
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]};
                            else rotate_nxt = rotate;
                5'b01110:   if({sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42], sram_buf[62:60], sram_buf[47:46]} == large_pattern) 
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]};
                            else rotate_nxt = rotate;
                5'b01111:   if({sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43], sram_buf[63:60], sram_buf[47]} == large_pattern) 
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]};
                            else rotate_nxt = rotate;
                5'b10000:   if({sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40]} == small_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]}; // 8
                            else rotate_nxt = rotate;
                5'b10001:   if({sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41]} == small_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]}; // 9
                            else rotate_nxt = rotate;
                5'b10010:   if({sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42]} == small_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]}; // 10
                            else rotate_nxt = rotate;
                5'b10011:   if({sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43]} == small_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]}; // 11
                            else rotate_nxt = rotate;
                5'b10100:   if({sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40], sram_buf[60], sram_buf[47:44]} == small_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]}; // 12
                            else rotate_nxt = rotate;
                5'b10101:   if({sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41], sram_buf[61:60], sram_buf[47:45]} == small_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]}; // 13
                            else rotate_nxt = rotate;
                5'b10110:   if({sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42], sram_buf[62:60], sram_buf[47:46]} == small_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]}; // 14
                            else rotate_nxt = rotate;
                5'b10111:    if({sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43], sram_buf[63:60], sram_buf[47]} == small_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]}; // 15
                            else rotate_nxt = rotate;
                5'b11000:   if({sram_buf[16], sram_buf[3:0], sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32]} == small_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]}; // 0
                            else rotate_nxt = rotate;
                5'b11001:   if({sram_buf[17:16], sram_buf[3:1], sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33]} == small_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]}; // 1
                            else rotate_nxt = rotate;
                5'b11010:   if({sram_buf[18:16], sram_buf[3:2], sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34]} == small_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]}; // 2
                            else rotate_nxt = rotate;
                5'b11011:   if({sram_buf[19:16], sram_buf[3], sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35]} == small_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]}; // 3
                            else rotate_nxt = rotate;
                5'b11100:   if({sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36]} == small_pattern) 
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]}; // 4
                            else rotate_nxt = rotate;
                5'b11101:   if({sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37]} == small_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]}; // 5
                            else rotate_nxt = rotate;
                5'b11110:   if({sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38]} == small_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]}; // 6
                            else rotate_nxt = rotate;
                5'b11111:   if({sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39]} == small_pattern)
                                rotate_nxt = {rotate[3], 1'b1, rotate[1:0]}; // 7
                            else rotate_nxt = rotate;
                endcase
        end
        2'b10: begin // diag_right
        case({scale, find_pos}) //synopsys parallel_case
                5'b00000:   if({sram_buf[16], sram_buf[3:0], sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32]} == large_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]};
                            else rotate_nxt = rotate;
                5'b00001:   if({sram_buf[17:16], sram_buf[3:1], sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33]} == large_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]};
                            else rotate_nxt = rotate;
                5'b00010:   if({sram_buf[18:16], sram_buf[3:2], sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34]} == large_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]};
                            else rotate_nxt = rotate;
                5'b00011:   if({sram_buf[19:16], sram_buf[3], sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35]} == large_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]};
                            else rotate_nxt = rotate;
                5'b00100:   if({sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36]} == large_pattern) 
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]};
                            else rotate_nxt = rotate;
                5'b00101:   if({sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37]} == large_pattern) 
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]};
                            else rotate_nxt = rotate;
                5'b00110:   if({sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38]} == large_pattern) 
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]};
                            else rotate_nxt = rotate;
                5'b00111:   if({sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39]} == large_pattern) 
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]};
                            else rotate_nxt = rotate;
                5'b01000:   if({sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40]} == large_pattern) 
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]};
                            else rotate_nxt = rotate;
                5'b01001:   if({sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41]} == large_pattern) 
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]};
                            else rotate_nxt = rotate;
                5'b01010:   if({sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42]} == large_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]};
                            else rotate_nxt = rotate;
                5'b01011:   if({sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43]} == large_pattern) 
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]};
                            else rotate_nxt = rotate;
                5'b01100:   if({sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40], sram_buf[60], sram_buf[47:44]} == large_pattern) 
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]};
                            else rotate_nxt = rotate;
                5'b01101:   if({sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41], sram_buf[61:60], sram_buf[47:45]} == large_pattern) 
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]};
                            else rotate_nxt = rotate;
                5'b01110:   if({sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42], sram_buf[62:60], sram_buf[47:46]} == large_pattern) 
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]};
                            else rotate_nxt = rotate;
                5'b01111:   if({sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43], sram_buf[63:60], sram_buf[47]} == large_pattern) 
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]};
                            else rotate_nxt = rotate;
                5'b10000:   if({sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42]} == small_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]}; // 2 - 10
                            else rotate_nxt = rotate;
                5'b10001:   if({sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43]} == small_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]}; // 3 - 11
                            else rotate_nxt = rotate;
                5'b10010:   if({sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40]} == small_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]}; // 0 - 8
                            else rotate_nxt = rotate;
                5'b10011:   if({sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41]} == small_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]}; // 1 - 9
                            else rotate_nxt = rotate;
                5'b10100:   if({sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42], sram_buf[62:60], sram_buf[47:46]} == small_pattern) 
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]}; // 6 - 14
                            else rotate_nxt = rotate;
                5'b10101:   if({sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43], sram_buf[63:60], sram_buf[47]} == small_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]}; // 7 - 15
                            else rotate_nxt = rotate;
                5'b10110:   if({sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40], sram_buf[60], sram_buf[47:44]} == small_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]}; // 4 - 12
                            else rotate_nxt = rotate;
                5'b10111:   if({sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41], sram_buf[61:60], sram_buf[47:45]} == small_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]}; // 5 - 13
                            else rotate_nxt = rotate;
                5'b11000:   if({sram_buf[18:16], sram_buf[3:2], sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34]} == small_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]}; // 10 - 2
                            else rotate_nxt = rotate;
                5'b11001:   if({sram_buf[19:16], sram_buf[3], sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35]} == small_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]}; // 11 - 3
                            else rotate_nxt = rotate;
                5'b11010:   if({sram_buf[16], sram_buf[3:0], sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32]} == small_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]}; // 8 - 0
                            else rotate_nxt = rotate;
                5'b11011:   if({sram_buf[17:16], sram_buf[3:1], sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33]} == small_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]}; // 9 - 1
                            else rotate_nxt = rotate;
                5'b11100:   if({sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38]} == small_pattern) 
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]}; // 14 - 6
                            else rotate_nxt = rotate;
                5'b11101:   if({sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39]} == small_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]}; // 15 - 7
                            else rotate_nxt = rotate;
                5'b11110:   if({sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36]} == small_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]}; // 12 - 4
                            else rotate_nxt = rotate;
                5'b11111:   if({sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37]} == small_pattern)
                                rotate_nxt = {rotate[3:2], 1'b1, rotate[0]}; // 13 - 5
                            else rotate_nxt = rotate;
                endcase
        end
        2'b11: begin // right
                case({scale, find_pos}) //synopsys parallel_case
                5'b00000:   if({sram_buf[16], sram_buf[3:0], sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32]} == large_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1};
                            else rotate_nxt = rotate;
                5'b00001:   if({sram_buf[17:16], sram_buf[3:1], sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33]} == large_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1};
                            else rotate_nxt = rotate;
                5'b00010:   if({sram_buf[18:16], sram_buf[3:2], sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34]} == large_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1};
                            else rotate_nxt = rotate;
                5'b00011:   if({sram_buf[19:16], sram_buf[3], sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35]} == large_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1};
                            else rotate_nxt = rotate;
                5'b00100:   if({sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36]} == large_pattern) 
                                rotate_nxt = {rotate[3:1], 1'b1};
                            else rotate_nxt = rotate;
                5'b00101:   if({sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37]} == large_pattern) 
                                rotate_nxt = {rotate[3:1], 1'b1};
                            else rotate_nxt = rotate;
                5'b00110:   if({sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38]} == large_pattern) 
                                rotate_nxt = {rotate[3:1], 1'b1};
                            else rotate_nxt = rotate;
                5'b00111:   if({sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39]} == large_pattern) 
                                rotate_nxt = {rotate[3:1], 1'b1};
                            else rotate_nxt = rotate;
                5'b01000:   if({sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40]} == large_pattern) 
                                rotate_nxt = {rotate[3:1], 1'b1};
                            else rotate_nxt = rotate;
                5'b01001:   if({sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41]} == large_pattern) 
                                rotate_nxt = {rotate[3:1], 1'b1};
                            else rotate_nxt = rotate;
                5'b01010:   if({sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42]} == large_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1};
                            else rotate_nxt = rotate;
                5'b01011:   if({sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43]} == large_pattern) 
                                rotate_nxt = {rotate[3:1], 1'b1};
                            else rotate_nxt = rotate;
                5'b01100:   if({sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40], sram_buf[60], sram_buf[47:44]} == large_pattern) 
                                rotate_nxt = {rotate[3:1], 1'b1};
                            else rotate_nxt = rotate;
                5'b01101:   if({sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41], sram_buf[61:60], sram_buf[47:45]} == large_pattern) 
                                rotate_nxt = {rotate[3:1], 1'b1};
                            else rotate_nxt = rotate;
                5'b01110:   if({sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42], sram_buf[62:60], sram_buf[47:46]} == large_pattern) 
                                rotate_nxt = {rotate[3:1], 1'b1};
                            else rotate_nxt = rotate;
                5'b01111:   if({sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43], sram_buf[63:60], sram_buf[47]} == large_pattern) 
                                rotate_nxt = {rotate[3:1], 1'b1};
                            else rotate_nxt = rotate;
                5'b10000:   if({sram_buf[18:16], sram_buf[3:2], sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34]} == small_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1}; // 2
                            else rotate_nxt = rotate;
                5'b10001:   if({sram_buf[19:16], sram_buf[3], sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35]} == small_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1}; // 3
                            else rotate_nxt = rotate;
                5'b10010:   if({sram_buf[16], sram_buf[3:0], sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32]} == small_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1}; // 0
                            else rotate_nxt = rotate;
                5'b10011:   if({sram_buf[17:16], sram_buf[3:1], sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33]} == small_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1}; // 1
                            else rotate_nxt = rotate;
                5'b10100:   if({sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38]} == small_pattern) 
                                rotate_nxt = {rotate[3:1], 1'b1}; // 6
                            else rotate_nxt = rotate;
                5'b10101:   if({sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39]} == small_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1}; // 7
                            else rotate_nxt = rotate;
                5'b10110:   if({sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36]} == small_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1}; // 4
                            else rotate_nxt = rotate;
                5'b10111:   if({sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37]} == small_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1}; // 5
                            else rotate_nxt = rotate;
                5'b11000:   if({sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42]} == small_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1}; // 10
                            else rotate_nxt = rotate;
                5'b11001:   if({sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43]} == small_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1}; // 11
                            else rotate_nxt = rotate;
                5'b11010:   if({sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40]} == small_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1}; // 8
                            else rotate_nxt = rotate;
                5'b11011:   if({sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41]} == small_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1}; // 9
                            else rotate_nxt = rotate;
                5'b11100:   if({sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42], sram_buf[62:60], sram_buf[47:46]} == small_pattern) 
                                rotate_nxt = {rotate[3:1], 1'b1}; // 14
                            else rotate_nxt = rotate;
                5'b11101:   if({sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43], sram_buf[63:60], sram_buf[47]} == small_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1}; // 15
                            else rotate_nxt = rotate;
                5'b11110:   if({sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40], sram_buf[60], sram_buf[47:44]} == small_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1}; // 12
                            else rotate_nxt = rotate;
                5'b11111:   if({sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41], sram_buf[61:60], sram_buf[47:45]} == small_pattern)
                                rotate_nxt = {rotate[3:1], 1'b1}; // 13
                            else rotate_nxt = rotate;
                endcase
        end
        endcase
    else
        rotate_nxt = rotate;
    
end




endmodule