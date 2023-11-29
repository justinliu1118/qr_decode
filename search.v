module search(
    input clk,                          //clock input
    input srstn,
    input start,              //start decoding for one QR code
                                        //1: start (one-cycle pulse)
    input [15:0] sram_rdata,            //read data from SRAM
    output reg [9:0] sram_raddr,            //read address to SRAM

    output reg scale, // 1: 25x25 0 : 50x50
    output reg [3:0] find_pos,
    output reg [9:0] find_addr,
    output reg finish
);

parameter IDLE = 2'b00, READ = 2'b01, FIND = 2'b10;

reg finish_nxt;
reg scale_nxt;
reg [9:0] sram_raddr_nxt;
reg [3:0] find_pos_nxt;
reg [9:0] find_addr_nxt;

reg [1:0] state, state_nxt;
reg [1:0] read_cnt, read_cnt_nxt;
reg [9:0] read_offset, read_offset_nxt;
reg [63:0] sram_buf, sram_buf_nxt;
wire [24:0] small_pattern;
wire [24:0] large_pattern;

assign small_pattern = 25'b11111_00001_11101_11101_11101;
assign large_pattern = 25'b11111_11111_00011_00011_10011;

always@(posedge clk) begin
    if(~srstn) begin
        read_offset <= 0;
        read_cnt <= 0;
    end
    else begin
        read_offset <= read_offset_nxt;
        read_cnt <= read_cnt_nxt;
    end
    state <= state_nxt;
    sram_raddr <= sram_raddr_nxt;
    scale <= scale_nxt;
    find_addr <= find_addr_nxt;
    finish <= finish_nxt;
    sram_buf <= sram_buf_nxt;
    find_pos <= find_pos_nxt;
    
end

always @(*) begin
    case(state) //synopsys parallel_case
    IDLE: state_nxt = (start) ? READ : IDLE;
    READ: state_nxt = (finish) ? IDLE: (read_cnt == 2'd3) ? FIND : READ;
    FIND: state_nxt = READ;
    default: state_nxt = IDLE;
    endcase
end

always @(*) begin
    read_offset_nxt = ((read_cnt == 2'b00) && (sram_rdata == 0)) ? read_offset + 1'b1 : (read_cnt == 2'd3) ? read_offset + 1'b1 : read_offset;
    sram_buf_nxt = (state == READ) ? {sram_rdata, sram_buf[63:16]} : 0;
    read_cnt_nxt = (state == READ) ? ((read_cnt == 2'b00) && (sram_rdata == 0)) ? 0 : read_cnt + 1'b1 : 0;
end

always @(*) begin
    if(state == READ)
        case(read_cnt) // synopsys parallel_case
        2'b00: sram_raddr_nxt = read_offset + 1'b1;
        2'b01: sram_raddr_nxt = read_offset + 6'd32;
        2'b10: sram_raddr_nxt = read_offset + 6'd33;
        default: sram_raddr_nxt = read_offset;
        endcase
    else
        sram_raddr_nxt = read_offset;
end

always @(*) begin
    if({sram_buf[16], sram_buf[3:0], sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32]} == small_pattern) begin
        scale_nxt = 1;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 0;
        finish_nxt = 1;
    end
    else if({sram_buf[17:16], sram_buf[3:1], sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33]} == small_pattern) begin
        scale_nxt = 1;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd1;
        finish_nxt = 1;
    end
    else if({sram_buf[18:16], sram_buf[3:2], sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34]} == small_pattern) begin
        scale_nxt = 1;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd2;
        finish_nxt = 1;
    end
    else if({sram_buf[19:16], sram_buf[3], sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35]} == small_pattern) begin
        scale_nxt = 1;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd3;
        finish_nxt = 1;
    end
    else if({sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36]} == small_pattern) begin
        scale_nxt = 1;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd4;
        finish_nxt = 1;
    end
    else if({sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37]} == small_pattern) begin
        scale_nxt = 1;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd5;
        finish_nxt = 1;
    end
    else if({sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38]} == small_pattern) begin
        scale_nxt = 1;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd6;
        finish_nxt = 1;
    end
    else if({sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39]} == small_pattern) begin
        scale_nxt = 1;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd7;
        finish_nxt = 1;
    end
    else if({sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40]} == small_pattern) begin
        scale_nxt = 1;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd8;
        finish_nxt = 1;
    end
    else if({sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41]} == small_pattern) begin
        scale_nxt = 1;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd9;
        finish_nxt = 1;
    end
    else if({sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42]} == small_pattern) begin
        scale_nxt = 1;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd10;
        finish_nxt = 1;
    end
    else if({sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43]} == small_pattern) begin
        scale_nxt = 1;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd11;
        finish_nxt = 1;
    end
    else if({sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40], sram_buf[60], sram_buf[47:44]} == small_pattern) begin
        scale_nxt = 1;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd12;
        finish_nxt = 1;
    end
    else if({sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41], sram_buf[61:60], sram_buf[47:45]} == small_pattern) begin
        scale_nxt = 1;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd13;
        finish_nxt = 1;
    end
    else if({sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42], sram_buf[62:60], sram_buf[47:46]} == small_pattern) begin
        scale_nxt = 1;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd14;
        finish_nxt = 1;
    end
    else if({sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43], sram_buf[63:60], sram_buf[47]} == small_pattern) begin
        scale_nxt = 1;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd15;
        finish_nxt = 1;
    end
    else if({sram_buf[16], sram_buf[3:0], sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32]} == large_pattern) begin
        scale_nxt = 0;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 0;
        finish_nxt = 1;
    end
    else if({sram_buf[17:16], sram_buf[3:1], sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33]} == large_pattern) begin
        scale_nxt = 0;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd1;
        finish_nxt = 1;
    end
    else if({sram_buf[18:16], sram_buf[3:2], sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34]} == large_pattern) begin
        scale_nxt = 0;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd2;
        finish_nxt = 1;
    end
    else if({sram_buf[19:16], sram_buf[3], sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35]} == large_pattern) begin
        scale_nxt = 0;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd3;
        finish_nxt = 1;
    end
    else if({sram_buf[20], sram_buf[7:4], sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36]} == large_pattern) begin
        scale_nxt = 0;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd4;
        finish_nxt = 1;
    end
    else if({sram_buf[21:20], sram_buf[7:5], sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37]} == large_pattern) begin
        scale_nxt = 0;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd5;
        finish_nxt = 1;
    end
    else if({sram_buf[22:20], sram_buf[7:6], sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38]} == large_pattern) begin
        if(sram_raddr[4:0] == 4'd5) begin
            scale_nxt = scale;
            find_addr_nxt = find_addr;
            find_pos_nxt = find_pos;
            finish_nxt = 0;
        end
        else begin
            scale_nxt = 0;
            find_addr_nxt = sram_raddr;
            find_pos_nxt = 4'd6;
            finish_nxt = 1;
        end
    end
    else if({sram_buf[23:20], sram_buf[7], sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39]} == large_pattern) begin
        scale_nxt = 0;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd7;
        finish_nxt = 1;
    end
    else if({sram_buf[24], sram_buf[11:8], sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40]} == large_pattern) begin
        scale_nxt = 0;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd8;
        finish_nxt = 1;
    end
    else if({sram_buf[25:24], sram_buf[11:9], sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41]} == large_pattern) begin
        scale_nxt = 0;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd9;
        finish_nxt = 1;
    end
    else if({sram_buf[26:24], sram_buf[11:10], sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42]} == large_pattern) begin
        scale_nxt = 0;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd10;
        finish_nxt = 1;
    end
    else if({sram_buf[27:24], sram_buf[11], sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43]} == large_pattern) begin
        scale_nxt = 0;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd11;
        finish_nxt = 1;
    end
    else if({sram_buf[28], sram_buf[15:12], sram_buf[48], sram_buf[35:32], sram_buf[52], sram_buf[39:36], sram_buf[56], sram_buf[43:40], sram_buf[60], sram_buf[47:44]} == large_pattern) begin
        scale_nxt = 0;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd12;
        finish_nxt = 1;
    end
    else if({sram_buf[29:28], sram_buf[15:13], sram_buf[49:48], sram_buf[35:33], sram_buf[53:52], sram_buf[39:37], sram_buf[57:56], sram_buf[43:41], sram_buf[61:60], sram_buf[47:45]} == large_pattern) begin
        scale_nxt = 0;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd13;
        finish_nxt = 1;
    end
    else if({sram_buf[30:28], sram_buf[15:14], sram_buf[50:48], sram_buf[35:34], sram_buf[54:52], sram_buf[39:38], sram_buf[58:56], sram_buf[43:42], sram_buf[62:60], sram_buf[47:46]} == large_pattern) begin
        scale_nxt = 0;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd14;
        finish_nxt = 1;
    end
    else if({sram_buf[31:28], sram_buf[15], sram_buf[51:48], sram_buf[35], sram_buf[55:52], sram_buf[39], sram_buf[59:56], sram_buf[43], sram_buf[63:60], sram_buf[47]} == large_pattern) begin
        scale_nxt = 0;
        find_addr_nxt = sram_raddr;
        find_pos_nxt = 4'd15;
        finish_nxt = 1;
    end
    else begin
        scale_nxt = scale;
        find_addr_nxt = find_addr;
        find_pos_nxt = find_pos;
        finish_nxt = 0;
    end
end

endmodule