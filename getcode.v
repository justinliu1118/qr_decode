module getcode(
    input clk,                          //clock input
    input start,
    input [3:0] rotate,              //start decoding for one QR code
    input scale,
    input [3:0] get_pos,
    input [9:0] get_addr,
                                        //1: start (one-cycle pulse)
    input [15:0] sram_rdata,            //read data from SRAM
    output reg [9:0] sram_raddr,
    output reg load_finish,
    output reg [624:0] out         //read address to SRAM
);

parameter IDLE=1'b0, READ=1'b1;

reg [9:0] sram_raddr_nxt;
reg load_finish_nxt;

reg start_buf;
reg state, state_nxt;
reg [8:0] read_block_cnt, read_block_cnt_nxt;
reg [1:0] read_row_cnt, read_row_cnt_nxt;
reg [3:0] data;
reg [624:0] sram_buf, sram_buf_nxt;

wire small_last, small_init, large_last, large_init;
assign small_last = (read_block_cnt == 7'd6) | (read_block_cnt == 7'd13) | (read_block_cnt == 7'd20) | (read_block_cnt == 7'd27) | (read_block_cnt == 7'd34) | (read_block_cnt == 7'd41) | (read_block_cnt == 7'd48);
assign large_last = (read_block_cnt == 9'd12) | (read_block_cnt == 9'd25) | (read_block_cnt == 9'd38) | (read_block_cnt == 9'd51) | (read_block_cnt == 9'd64) | (read_block_cnt == 9'd77) | (read_block_cnt == 9'd90) | (read_block_cnt == 9'd103) | (read_block_cnt == 9'd116)| (read_block_cnt == 9'd129)| (read_block_cnt == 9'd142)| (read_block_cnt == 9'd155)| (read_block_cnt == 9'd168);
assign small_init = (read_block_cnt == 7'd0) | (read_block_cnt == 7'd7) | (read_block_cnt == 7'd14) | (read_block_cnt == 7'd21) | (read_block_cnt == 7'd28) | (read_block_cnt == 7'd35) | (read_block_cnt == 7'd42);
assign large_init = (read_block_cnt == 9'd0) | (read_block_cnt == 9'd13) | (read_block_cnt == 9'd26) | (read_block_cnt == 9'd39) | (read_block_cnt == 9'd52) | (read_block_cnt == 9'd65) | (read_block_cnt == 9'd78) | (read_block_cnt == 9'd91) | (read_block_cnt == 9'd104)| (read_block_cnt == 9'd117)| (read_block_cnt == 9'd130)| (read_block_cnt == 9'd143)| (read_block_cnt == 9'd156);

always @(posedge clk) begin
    state <= state_nxt;
    sram_raddr <= sram_raddr_nxt;
    read_block_cnt <= read_block_cnt_nxt;
    read_row_cnt <= read_row_cnt_nxt;
    start_buf <= start;
    sram_buf <= sram_buf_nxt;
    load_finish <= load_finish_nxt;
end

always @(*) begin
    case(state) //synopsys parallel_case
    IDLE: state_nxt = (start_buf) ? READ : IDLE;
    READ : state_nxt = (load_finish) ? IDLE : READ;
    default: state_nxt = IDLE;
    endcase
end

always @(*) begin
    sram_raddr_nxt = (state == IDLE) ? (start) ? sram_raddr+1'b1 : get_addr : (scale) ? 
                     (small_last)? (read_row_cnt == 2'd3) ? sram_raddr+6'd26 : sram_raddr-3'd6 : sram_raddr + 1'b1 :
                     (large_last)? (read_row_cnt[1]) ? sram_raddr+7'd20 : sram_raddr-4'd12 : sram_raddr + 1'b1;

    read_block_cnt_nxt = (state == READ) ? (scale) ? 
                         (small_last)? (read_row_cnt == 2'd3) ? read_block_cnt + 1'b1 : read_block_cnt-3'd6 : read_block_cnt + 1'b1 :
                         (large_last)? (read_row_cnt[1]) ? read_block_cnt + 1'b1 : read_block_cnt-4'd12 : read_block_cnt + 1'b1 : 0;

    read_row_cnt_nxt   = (state == READ) ? (scale) ? 
                         (small_last)? read_row_cnt + 1'b1 : read_row_cnt : 
                         (large_last)? read_row_cnt + 2'd2 : read_row_cnt : get_pos[3:2];

    load_finish_nxt = (scale) ? (read_block_cnt == 7'd48) ? (get_pos[3:2] == read_row_cnt) ? 1 : 0 : 0 : 
                                (read_block_cnt == 9'd168) ? (get_pos[3:2] == read_row_cnt) ? 1 : 0 : 0;                         
end

always @(*) begin
    case(read_row_cnt) //synopsys parallel_case
    2'b00: data = {sram_rdata[0], sram_rdata[1], sram_rdata[2], sram_rdata[3]};
    2'b01: data = {sram_rdata[4], sram_rdata[5], sram_rdata[6], sram_rdata[7]};
    2'b10: data = {sram_rdata[8], sram_rdata[9], sram_rdata[10], sram_rdata[11]};
    2'b11: data = {sram_rdata[12], sram_rdata[13], sram_rdata[14], sram_rdata[15]};
    endcase
end

always @(*) begin
        case({scale, get_pos[1:0]}) //synopsys parallel_case
        3'b100:  if(small_last)
                    sram_buf_nxt = {sram_buf[623:0], data[3]};
                else
                    sram_buf_nxt = {sram_buf[620:0], data};
        3'b101:  if(small_init)
                    sram_buf_nxt = {sram_buf[621:0], data[2:0]};
                else if(small_last)
                    sram_buf_nxt = {sram_buf[622:0], data[3:2]};
                else
                    sram_buf_nxt = {sram_buf[620:0], data};
        3'b110:  if(small_init)
                    sram_buf_nxt = {sram_buf[622:0], data[1:0]};
                else if(small_last)
                    sram_buf_nxt = {sram_buf[621:0], data[3:1]};
                else
                    sram_buf_nxt = {sram_buf[620:0], data};
        3'b111:  if(small_init)
                    sram_buf_nxt = {sram_buf[623:0], data[0]};
                else
                    sram_buf_nxt = {sram_buf[620:0], data};
        3'b000:  if(large_last)
                    sram_buf_nxt = {sram_buf[623:0], data[3]};
                else
                    sram_buf_nxt = {sram_buf[622:0], data[3], data[1]};
        3'b001:  if(large_last)
                    sram_buf_nxt = {sram_buf[623:0], data[2]};
                else
                    sram_buf_nxt = {sram_buf[622:0], data[2], data[0]};
        3'b010:  if(large_init)
                    sram_buf_nxt = {sram_buf[623:0], data[1]};
                else if(large_last)
                    sram_buf_nxt = {sram_buf[622:0], data[3], data[1]};
                else
                    sram_buf_nxt = {sram_buf[622:0], data[3], data[1]};
        3'b011:  if(large_init)
                    sram_buf_nxt = {sram_buf[623:0], data[0]};
                else if(large_last)
                    sram_buf_nxt = {sram_buf[622:0], data[2], data[0]};
                else
                    sram_buf_nxt = {sram_buf[622:0], data[2], data[0]};
        endcase
end

always @(*) begin
    if(load_finish)
        case(rotate) //synopsys parallel_case
        // 0
        4'b0101: out = sram_buf;
        // 90
        4'b0110: out = {sram_buf[24], sram_buf[49], sram_buf[74], sram_buf[99], sram_buf[124], sram_buf[149], sram_buf[174], sram_buf[199], sram_buf[224], sram_buf[249], sram_buf[274], sram_buf[299], sram_buf[324], sram_buf[349], sram_buf[374], sram_buf[399], sram_buf[424], sram_buf[449], sram_buf[474], sram_buf[499], sram_buf[524], sram_buf[549], sram_buf[574], sram_buf[599], sram_buf[624], 
                            sram_buf[23], sram_buf[48], sram_buf[73], sram_buf[98], sram_buf[123], sram_buf[148], sram_buf[173], sram_buf[198], sram_buf[223], sram_buf[248], sram_buf[273], sram_buf[298], sram_buf[323], sram_buf[348], sram_buf[373], sram_buf[398], sram_buf[423], sram_buf[448], sram_buf[473], sram_buf[498], sram_buf[523], sram_buf[548], sram_buf[573], sram_buf[598], sram_buf[623], 
                            sram_buf[22], sram_buf[47], sram_buf[72], sram_buf[97], sram_buf[122], sram_buf[147], sram_buf[172], sram_buf[197], sram_buf[222], sram_buf[247], sram_buf[272], sram_buf[297], sram_buf[322], sram_buf[347], sram_buf[372], sram_buf[397], sram_buf[422], sram_buf[447], sram_buf[472], sram_buf[497], sram_buf[522], sram_buf[547], sram_buf[572], sram_buf[597], sram_buf[622], 
                            sram_buf[21], sram_buf[46], sram_buf[71], sram_buf[96], sram_buf[121], sram_buf[146], sram_buf[171], sram_buf[196], sram_buf[221], sram_buf[246], sram_buf[271], sram_buf[296], sram_buf[321], sram_buf[346], sram_buf[371], sram_buf[396], sram_buf[421], sram_buf[446], sram_buf[471], sram_buf[496], sram_buf[521], sram_buf[546], sram_buf[571], sram_buf[596], sram_buf[621], 
                            sram_buf[20], sram_buf[45], sram_buf[70], sram_buf[95], sram_buf[120], sram_buf[145], sram_buf[170], sram_buf[195], sram_buf[220], sram_buf[245], sram_buf[270], sram_buf[295], sram_buf[320], sram_buf[345], sram_buf[370], sram_buf[395], sram_buf[420], sram_buf[445], sram_buf[470], sram_buf[495], sram_buf[520], sram_buf[545], sram_buf[570], sram_buf[595], sram_buf[620], 
                            sram_buf[19], sram_buf[44], sram_buf[69], sram_buf[94], sram_buf[119], sram_buf[144], sram_buf[169], sram_buf[194], sram_buf[219], sram_buf[244], sram_buf[269], sram_buf[294], sram_buf[319], sram_buf[344], sram_buf[369], sram_buf[394], sram_buf[419], sram_buf[444], sram_buf[469], sram_buf[494], sram_buf[519], sram_buf[544], sram_buf[569], sram_buf[594], sram_buf[619], 
                            sram_buf[18], sram_buf[43], sram_buf[68], sram_buf[93], sram_buf[118], sram_buf[143], sram_buf[168], sram_buf[193], sram_buf[218], sram_buf[243], sram_buf[268], sram_buf[293], sram_buf[318], sram_buf[343], sram_buf[368], sram_buf[393], sram_buf[418], sram_buf[443], sram_buf[468], sram_buf[493], sram_buf[518], sram_buf[543], sram_buf[568], sram_buf[593], sram_buf[618], 
                            sram_buf[17], sram_buf[42], sram_buf[67], sram_buf[92], sram_buf[117], sram_buf[142], sram_buf[167], sram_buf[192], sram_buf[217], sram_buf[242], sram_buf[267], sram_buf[292], sram_buf[317], sram_buf[342], sram_buf[367], sram_buf[392], sram_buf[417], sram_buf[442], sram_buf[467], sram_buf[492], sram_buf[517], sram_buf[542], sram_buf[567], sram_buf[592], sram_buf[617], 
                            sram_buf[16], sram_buf[41], sram_buf[66], sram_buf[91], sram_buf[116], sram_buf[141], sram_buf[166], sram_buf[191], sram_buf[216], sram_buf[241], sram_buf[266], sram_buf[291], sram_buf[316], sram_buf[341], sram_buf[366], sram_buf[391], sram_buf[416], sram_buf[441], sram_buf[466], sram_buf[491], sram_buf[516], sram_buf[541], sram_buf[566], sram_buf[591], sram_buf[616], 
                            sram_buf[15], sram_buf[40], sram_buf[65], sram_buf[90], sram_buf[115], sram_buf[140], sram_buf[165], sram_buf[190], sram_buf[215], sram_buf[240], sram_buf[265], sram_buf[290], sram_buf[315], sram_buf[340], sram_buf[365], sram_buf[390], sram_buf[415], sram_buf[440], sram_buf[465], sram_buf[490], sram_buf[515], sram_buf[540], sram_buf[565], sram_buf[590], sram_buf[615], 
                            sram_buf[14], sram_buf[39], sram_buf[64], sram_buf[89], sram_buf[114], sram_buf[139], sram_buf[164], sram_buf[189], sram_buf[214], sram_buf[239], sram_buf[264], sram_buf[289], sram_buf[314], sram_buf[339], sram_buf[364], sram_buf[389], sram_buf[414], sram_buf[439], sram_buf[464], sram_buf[489], sram_buf[514], sram_buf[539], sram_buf[564], sram_buf[589], sram_buf[614], 
                            sram_buf[13], sram_buf[38], sram_buf[63], sram_buf[88], sram_buf[113], sram_buf[138], sram_buf[163], sram_buf[188], sram_buf[213], sram_buf[238], sram_buf[263], sram_buf[288], sram_buf[313], sram_buf[338], sram_buf[363], sram_buf[388], sram_buf[413], sram_buf[438], sram_buf[463], sram_buf[488], sram_buf[513], sram_buf[538], sram_buf[563], sram_buf[588], sram_buf[613], 
                            sram_buf[12], sram_buf[37], sram_buf[62], sram_buf[87], sram_buf[112], sram_buf[137], sram_buf[162], sram_buf[187], sram_buf[212], sram_buf[237], sram_buf[262], sram_buf[287], sram_buf[312], sram_buf[337], sram_buf[362], sram_buf[387], sram_buf[412], sram_buf[437], sram_buf[462], sram_buf[487], sram_buf[512], sram_buf[537], sram_buf[562], sram_buf[587], sram_buf[612], 
                            sram_buf[11], sram_buf[36], sram_buf[61], sram_buf[86], sram_buf[111], sram_buf[136], sram_buf[161], sram_buf[186], sram_buf[211], sram_buf[236], sram_buf[261], sram_buf[286], sram_buf[311], sram_buf[336], sram_buf[361], sram_buf[386], sram_buf[411], sram_buf[436], sram_buf[461], sram_buf[486], sram_buf[511], sram_buf[536], sram_buf[561], sram_buf[586], sram_buf[611], 
                            sram_buf[10], sram_buf[35], sram_buf[60], sram_buf[85], sram_buf[110], sram_buf[135], sram_buf[160], sram_buf[185], sram_buf[210], sram_buf[235], sram_buf[260], sram_buf[285], sram_buf[310], sram_buf[335], sram_buf[360], sram_buf[385], sram_buf[410], sram_buf[435], sram_buf[460], sram_buf[485], sram_buf[510], sram_buf[535], sram_buf[560], sram_buf[585], sram_buf[610], 
                            sram_buf[9], sram_buf[34], sram_buf[59], sram_buf[84], sram_buf[109], sram_buf[134], sram_buf[159], sram_buf[184], sram_buf[209], sram_buf[234], sram_buf[259], sram_buf[284], sram_buf[309], sram_buf[334], sram_buf[359], sram_buf[384], sram_buf[409], sram_buf[434], sram_buf[459], sram_buf[484], sram_buf[509], sram_buf[534], sram_buf[559], sram_buf[584], sram_buf[609], 
                            sram_buf[8], sram_buf[33], sram_buf[58], sram_buf[83], sram_buf[108], sram_buf[133], sram_buf[158], sram_buf[183], sram_buf[208], sram_buf[233], sram_buf[258], sram_buf[283], sram_buf[308], sram_buf[333], sram_buf[358], sram_buf[383], sram_buf[408], sram_buf[433], sram_buf[458], sram_buf[483], sram_buf[508], sram_buf[533], sram_buf[558], sram_buf[583], sram_buf[608], 
                            sram_buf[7], sram_buf[32], sram_buf[57], sram_buf[82], sram_buf[107], sram_buf[132], sram_buf[157], sram_buf[182], sram_buf[207], sram_buf[232], sram_buf[257], sram_buf[282], sram_buf[307], sram_buf[332], sram_buf[357], sram_buf[382], sram_buf[407], sram_buf[432], sram_buf[457], sram_buf[482], sram_buf[507], sram_buf[532], sram_buf[557], sram_buf[582], sram_buf[607], 
                            sram_buf[6], sram_buf[31], sram_buf[56], sram_buf[81], sram_buf[106], sram_buf[131], sram_buf[156], sram_buf[181], sram_buf[206], sram_buf[231], sram_buf[256], sram_buf[281], sram_buf[306], sram_buf[331], sram_buf[356], sram_buf[381], sram_buf[406], sram_buf[431], sram_buf[456], sram_buf[481], sram_buf[506], sram_buf[531], sram_buf[556], sram_buf[581], sram_buf[606], 
                            sram_buf[5], sram_buf[30], sram_buf[55], sram_buf[80], sram_buf[105], sram_buf[130], sram_buf[155], sram_buf[180], sram_buf[205], sram_buf[230], sram_buf[255], sram_buf[280], sram_buf[305], sram_buf[330], sram_buf[355], sram_buf[380], sram_buf[405], sram_buf[430], sram_buf[455], sram_buf[480], sram_buf[505], sram_buf[530], sram_buf[555], sram_buf[580], sram_buf[605], 
                            sram_buf[4], sram_buf[29], sram_buf[54], sram_buf[79], sram_buf[104], sram_buf[129], sram_buf[154], sram_buf[179], sram_buf[204], sram_buf[229], sram_buf[254], sram_buf[279], sram_buf[304], sram_buf[329], sram_buf[354], sram_buf[379], sram_buf[404], sram_buf[429], sram_buf[454], sram_buf[479], sram_buf[504], sram_buf[529], sram_buf[554], sram_buf[579], sram_buf[604], 
                            sram_buf[3], sram_buf[28], sram_buf[53], sram_buf[78], sram_buf[103], sram_buf[128], sram_buf[153], sram_buf[178], sram_buf[203], sram_buf[228], sram_buf[253], sram_buf[278], sram_buf[303], sram_buf[328], sram_buf[353], sram_buf[378], sram_buf[403], sram_buf[428], sram_buf[453], sram_buf[478], sram_buf[503], sram_buf[528], sram_buf[553], sram_buf[578], sram_buf[603], 
                            sram_buf[2], sram_buf[27], sram_buf[52], sram_buf[77], sram_buf[102], sram_buf[127], sram_buf[152], sram_buf[177], sram_buf[202], sram_buf[227], sram_buf[252], sram_buf[277], sram_buf[302], sram_buf[327], sram_buf[352], sram_buf[377], sram_buf[402], sram_buf[427], sram_buf[452], sram_buf[477], sram_buf[502], sram_buf[527], sram_buf[552], sram_buf[577], sram_buf[602], 
                            sram_buf[1], sram_buf[26], sram_buf[51], sram_buf[76], sram_buf[101], sram_buf[126], sram_buf[151], sram_buf[176], sram_buf[201], sram_buf[226], sram_buf[251], sram_buf[276], sram_buf[301], sram_buf[326], sram_buf[351], sram_buf[376], sram_buf[401], sram_buf[426], sram_buf[451], sram_buf[476], sram_buf[501], sram_buf[526], sram_buf[551], sram_buf[576], sram_buf[601], 
                            sram_buf[0], sram_buf[25], sram_buf[50], sram_buf[75], sram_buf[100], sram_buf[125], sram_buf[150], sram_buf[175], sram_buf[200], sram_buf[225], sram_buf[250], sram_buf[275], sram_buf[300], sram_buf[325], sram_buf[350], sram_buf[375], sram_buf[400], sram_buf[425], sram_buf[450], sram_buf[475], sram_buf[500], sram_buf[525], sram_buf[550], sram_buf[575], sram_buf[600]};
        //180:
        4'b1100: out = {sram_buf[0], sram_buf[1], sram_buf[2], sram_buf[3], sram_buf[4], sram_buf[5], sram_buf[6], sram_buf[7], sram_buf[8], sram_buf[9], sram_buf[10], sram_buf[11], sram_buf[12], sram_buf[13], sram_buf[14], sram_buf[15], sram_buf[16], sram_buf[17], sram_buf[18], sram_buf[19], sram_buf[20], sram_buf[21], sram_buf[22], sram_buf[23], sram_buf[24], 
                            sram_buf[25], sram_buf[26], sram_buf[27], sram_buf[28], sram_buf[29], sram_buf[30], sram_buf[31], sram_buf[32], sram_buf[33], sram_buf[34], sram_buf[35], sram_buf[36], sram_buf[37], sram_buf[38], sram_buf[39], sram_buf[40], sram_buf[41], sram_buf[42], sram_buf[43], sram_buf[44], sram_buf[45], sram_buf[46], sram_buf[47], sram_buf[48], sram_buf[49], 
                            sram_buf[50], sram_buf[51], sram_buf[52], sram_buf[53], sram_buf[54], sram_buf[55], sram_buf[56], sram_buf[57], sram_buf[58], sram_buf[59], sram_buf[60], sram_buf[61], sram_buf[62], sram_buf[63], sram_buf[64], sram_buf[65], sram_buf[66], sram_buf[67], sram_buf[68], sram_buf[69], sram_buf[70], sram_buf[71], sram_buf[72], sram_buf[73], sram_buf[74], 
                            sram_buf[75], sram_buf[76], sram_buf[77], sram_buf[78], sram_buf[79], sram_buf[80], sram_buf[81], sram_buf[82], sram_buf[83], sram_buf[84], sram_buf[85], sram_buf[86], sram_buf[87], sram_buf[88], sram_buf[89], sram_buf[90], sram_buf[91], sram_buf[92], sram_buf[93], sram_buf[94], sram_buf[95], sram_buf[96], sram_buf[97], sram_buf[98], sram_buf[99], 
                            sram_buf[100], sram_buf[101], sram_buf[102], sram_buf[103], sram_buf[104], sram_buf[105], sram_buf[106], sram_buf[107], sram_buf[108], sram_buf[109], sram_buf[110], sram_buf[111], sram_buf[112], sram_buf[113], sram_buf[114], sram_buf[115], sram_buf[116], sram_buf[117], sram_buf[118], sram_buf[119], sram_buf[120], sram_buf[121], sram_buf[122], sram_buf[123], sram_buf[124], 
                            sram_buf[125], sram_buf[126], sram_buf[127], sram_buf[128], sram_buf[129], sram_buf[130], sram_buf[131], sram_buf[132], sram_buf[133], sram_buf[134], sram_buf[135], sram_buf[136], sram_buf[137], sram_buf[138], sram_buf[139], sram_buf[140], sram_buf[141], sram_buf[142], sram_buf[143], sram_buf[144], sram_buf[145], sram_buf[146], sram_buf[147], sram_buf[148], sram_buf[149], 
                            sram_buf[150], sram_buf[151], sram_buf[152], sram_buf[153], sram_buf[154], sram_buf[155], sram_buf[156], sram_buf[157], sram_buf[158], sram_buf[159], sram_buf[160], sram_buf[161], sram_buf[162], sram_buf[163], sram_buf[164], sram_buf[165], sram_buf[166], sram_buf[167], sram_buf[168], sram_buf[169], sram_buf[170], sram_buf[171], sram_buf[172], sram_buf[173], sram_buf[174], 
                            sram_buf[175], sram_buf[176], sram_buf[177], sram_buf[178], sram_buf[179], sram_buf[180], sram_buf[181], sram_buf[182], sram_buf[183], sram_buf[184], sram_buf[185], sram_buf[186], sram_buf[187], sram_buf[188], sram_buf[189], sram_buf[190], sram_buf[191], sram_buf[192], sram_buf[193], sram_buf[194], sram_buf[195], sram_buf[196], sram_buf[197], sram_buf[198], sram_buf[199], 
                            sram_buf[200], sram_buf[201], sram_buf[202], sram_buf[203], sram_buf[204], sram_buf[205], sram_buf[206], sram_buf[207], sram_buf[208], sram_buf[209], sram_buf[210], sram_buf[211], sram_buf[212], sram_buf[213], sram_buf[214], sram_buf[215], sram_buf[216], sram_buf[217], sram_buf[218], sram_buf[219], sram_buf[220], sram_buf[221], sram_buf[222], sram_buf[223], sram_buf[224], 
                            sram_buf[225], sram_buf[226], sram_buf[227], sram_buf[228], sram_buf[229], sram_buf[230], sram_buf[231], sram_buf[232], sram_buf[233], sram_buf[234], sram_buf[235], sram_buf[236], sram_buf[237], sram_buf[238], sram_buf[239], sram_buf[240], sram_buf[241], sram_buf[242], sram_buf[243], sram_buf[244], sram_buf[245], sram_buf[246], sram_buf[247], sram_buf[248], sram_buf[249], 
                            sram_buf[250], sram_buf[251], sram_buf[252], sram_buf[253], sram_buf[254], sram_buf[255], sram_buf[256], sram_buf[257], sram_buf[258], sram_buf[259], sram_buf[260], sram_buf[261], sram_buf[262], sram_buf[263], sram_buf[264], sram_buf[265], sram_buf[266], sram_buf[267], sram_buf[268], sram_buf[269], sram_buf[270], sram_buf[271], sram_buf[272], sram_buf[273], sram_buf[274], 
                            sram_buf[275], sram_buf[276], sram_buf[277], sram_buf[278], sram_buf[279], sram_buf[280], sram_buf[281], sram_buf[282], sram_buf[283], sram_buf[284], sram_buf[285], sram_buf[286], sram_buf[287], sram_buf[288], sram_buf[289], sram_buf[290], sram_buf[291], sram_buf[292], sram_buf[293], sram_buf[294], sram_buf[295], sram_buf[296], sram_buf[297], sram_buf[298], sram_buf[299], 
                            sram_buf[300], sram_buf[301], sram_buf[302], sram_buf[303], sram_buf[304], sram_buf[305], sram_buf[306], sram_buf[307], sram_buf[308], sram_buf[309], sram_buf[310], sram_buf[311], sram_buf[312], sram_buf[313], sram_buf[314], sram_buf[315], sram_buf[316], sram_buf[317], sram_buf[318], sram_buf[319], sram_buf[320], sram_buf[321], sram_buf[322], sram_buf[323], sram_buf[324], 
                            sram_buf[325], sram_buf[326], sram_buf[327], sram_buf[328], sram_buf[329], sram_buf[330], sram_buf[331], sram_buf[332], sram_buf[333], sram_buf[334], sram_buf[335], sram_buf[336], sram_buf[337], sram_buf[338], sram_buf[339], sram_buf[340], sram_buf[341], sram_buf[342], sram_buf[343], sram_buf[344], sram_buf[345], sram_buf[346], sram_buf[347], sram_buf[348], sram_buf[349], 
                            sram_buf[350], sram_buf[351], sram_buf[352], sram_buf[353], sram_buf[354], sram_buf[355], sram_buf[356], sram_buf[357], sram_buf[358], sram_buf[359], sram_buf[360], sram_buf[361], sram_buf[362], sram_buf[363], sram_buf[364], sram_buf[365], sram_buf[366], sram_buf[367], sram_buf[368], sram_buf[369], sram_buf[370], sram_buf[371], sram_buf[372], sram_buf[373], sram_buf[374], 
                            sram_buf[375], sram_buf[376], sram_buf[377], sram_buf[378], sram_buf[379], sram_buf[380], sram_buf[381], sram_buf[382], sram_buf[383], sram_buf[384], sram_buf[385], sram_buf[386], sram_buf[387], sram_buf[388], sram_buf[389], sram_buf[390], sram_buf[391], sram_buf[392], sram_buf[393], sram_buf[394], sram_buf[395], sram_buf[396], sram_buf[397], sram_buf[398], sram_buf[399], 
                            sram_buf[400], sram_buf[401], sram_buf[402], sram_buf[403], sram_buf[404], sram_buf[405], sram_buf[406], sram_buf[407], sram_buf[408], sram_buf[409], sram_buf[410], sram_buf[411], sram_buf[412], sram_buf[413], sram_buf[414], sram_buf[415], sram_buf[416], sram_buf[417], sram_buf[418], sram_buf[419], sram_buf[420], sram_buf[421], sram_buf[422], sram_buf[423], sram_buf[424], 
                            sram_buf[425], sram_buf[426], sram_buf[427], sram_buf[428], sram_buf[429], sram_buf[430], sram_buf[431], sram_buf[432], sram_buf[433], sram_buf[434], sram_buf[435], sram_buf[436], sram_buf[437], sram_buf[438], sram_buf[439], sram_buf[440], sram_buf[441], sram_buf[442], sram_buf[443], sram_buf[444], sram_buf[445], sram_buf[446], sram_buf[447], sram_buf[448], sram_buf[449], 
                            sram_buf[450], sram_buf[451], sram_buf[452], sram_buf[453], sram_buf[454], sram_buf[455], sram_buf[456], sram_buf[457], sram_buf[458], sram_buf[459], sram_buf[460], sram_buf[461], sram_buf[462], sram_buf[463], sram_buf[464], sram_buf[465], sram_buf[466], sram_buf[467], sram_buf[468], sram_buf[469], sram_buf[470], sram_buf[471], sram_buf[472], sram_buf[473], sram_buf[474], 
                            sram_buf[475], sram_buf[476], sram_buf[477], sram_buf[478], sram_buf[479], sram_buf[480], sram_buf[481], sram_buf[482], sram_buf[483], sram_buf[484], sram_buf[485], sram_buf[486], sram_buf[487], sram_buf[488], sram_buf[489], sram_buf[490], sram_buf[491], sram_buf[492], sram_buf[493], sram_buf[494], sram_buf[495], sram_buf[496], sram_buf[497], sram_buf[498], sram_buf[499], 
                            sram_buf[500], sram_buf[501], sram_buf[502], sram_buf[503], sram_buf[504], sram_buf[505], sram_buf[506], sram_buf[507], sram_buf[508], sram_buf[509], sram_buf[510], sram_buf[511], sram_buf[512], sram_buf[513], sram_buf[514], sram_buf[515], sram_buf[516], sram_buf[517], sram_buf[518], sram_buf[519], sram_buf[520], sram_buf[521], sram_buf[522], sram_buf[523], sram_buf[524], 
                            sram_buf[525], sram_buf[526], sram_buf[527], sram_buf[528], sram_buf[529], sram_buf[530], sram_buf[531], sram_buf[532], sram_buf[533], sram_buf[534], sram_buf[535], sram_buf[536], sram_buf[537], sram_buf[538], sram_buf[539], sram_buf[540], sram_buf[541], sram_buf[542], sram_buf[543], sram_buf[544], sram_buf[545], sram_buf[546], sram_buf[547], sram_buf[548], sram_buf[549], 
                            sram_buf[550], sram_buf[551], sram_buf[552], sram_buf[553], sram_buf[554], sram_buf[555], sram_buf[556], sram_buf[557], sram_buf[558], sram_buf[559], sram_buf[560], sram_buf[561], sram_buf[562], sram_buf[563], sram_buf[564], sram_buf[565], sram_buf[566], sram_buf[567], sram_buf[568], sram_buf[569], sram_buf[570], sram_buf[571], sram_buf[572], sram_buf[573], sram_buf[574], 
                            sram_buf[575], sram_buf[576], sram_buf[577], sram_buf[578], sram_buf[579], sram_buf[580], sram_buf[581], sram_buf[582], sram_buf[583], sram_buf[584], sram_buf[585], sram_buf[586], sram_buf[587], sram_buf[588], sram_buf[589], sram_buf[590], sram_buf[591], sram_buf[592], sram_buf[593], sram_buf[594], sram_buf[595], sram_buf[596], sram_buf[597], sram_buf[598], sram_buf[599], 
                            sram_buf[600], sram_buf[601], sram_buf[602], sram_buf[603], sram_buf[604], sram_buf[605], sram_buf[606], sram_buf[607], sram_buf[608], sram_buf[609], sram_buf[610], sram_buf[611], sram_buf[612], sram_buf[613], sram_buf[614], sram_buf[615], sram_buf[616], sram_buf[617], sram_buf[618], sram_buf[619], sram_buf[620], sram_buf[621], sram_buf[622], sram_buf[623], sram_buf[624]};
        4'b0111: out = {sram_buf[0], sram_buf[1], sram_buf[2], sram_buf[3], sram_buf[4], sram_buf[5], sram_buf[6], sram_buf[7], sram_buf[8], sram_buf[9], sram_buf[10], sram_buf[11], sram_buf[12], sram_buf[13], sram_buf[14], sram_buf[15], sram_buf[16], sram_buf[17], sram_buf[18], sram_buf[19], sram_buf[20], sram_buf[21], sram_buf[22], sram_buf[23], sram_buf[24], 
                            sram_buf[25], sram_buf[26], sram_buf[27], sram_buf[28], sram_buf[29], sram_buf[30], sram_buf[31], sram_buf[32], sram_buf[33], sram_buf[34], sram_buf[35], sram_buf[36], sram_buf[37], sram_buf[38], sram_buf[39], sram_buf[40], sram_buf[41], sram_buf[42], sram_buf[43], sram_buf[44], sram_buf[45], sram_buf[46], sram_buf[47], sram_buf[48], sram_buf[49], 
                            sram_buf[50], sram_buf[51], sram_buf[52], sram_buf[53], sram_buf[54], sram_buf[55], sram_buf[56], sram_buf[57], sram_buf[58], sram_buf[59], sram_buf[60], sram_buf[61], sram_buf[62], sram_buf[63], sram_buf[64], sram_buf[65], sram_buf[66], sram_buf[67], sram_buf[68], sram_buf[69], sram_buf[70], sram_buf[71], sram_buf[72], sram_buf[73], sram_buf[74], 
                            sram_buf[75], sram_buf[76], sram_buf[77], sram_buf[78], sram_buf[79], sram_buf[80], sram_buf[81], sram_buf[82], sram_buf[83], sram_buf[84], sram_buf[85], sram_buf[86], sram_buf[87], sram_buf[88], sram_buf[89], sram_buf[90], sram_buf[91], sram_buf[92], sram_buf[93], sram_buf[94], sram_buf[95], sram_buf[96], sram_buf[97], sram_buf[98], sram_buf[99], 
                            sram_buf[100], sram_buf[101], sram_buf[102], sram_buf[103], sram_buf[104], sram_buf[105], sram_buf[106], sram_buf[107], sram_buf[108], sram_buf[109], sram_buf[110], sram_buf[111], sram_buf[112], sram_buf[113], sram_buf[114], sram_buf[115], sram_buf[116], sram_buf[117], sram_buf[118], sram_buf[119], sram_buf[120], sram_buf[121], sram_buf[122], sram_buf[123], sram_buf[124], 
                            sram_buf[125], sram_buf[126], sram_buf[127], sram_buf[128], sram_buf[129], sram_buf[130], sram_buf[131], sram_buf[132], sram_buf[133], sram_buf[134], sram_buf[135], sram_buf[136], sram_buf[137], sram_buf[138], sram_buf[139], sram_buf[140], sram_buf[141], sram_buf[142], sram_buf[143], sram_buf[144], sram_buf[145], sram_buf[146], sram_buf[147], sram_buf[148], sram_buf[149], 
                            sram_buf[150], sram_buf[151], sram_buf[152], sram_buf[153], sram_buf[154], sram_buf[155], sram_buf[156], sram_buf[157], sram_buf[158], sram_buf[159], sram_buf[160], sram_buf[161], sram_buf[162], sram_buf[163], sram_buf[164], sram_buf[165], sram_buf[166], sram_buf[167], sram_buf[168], sram_buf[169], sram_buf[170], sram_buf[171], sram_buf[172], sram_buf[173], sram_buf[174], 
                            sram_buf[175], sram_buf[176], sram_buf[177], sram_buf[178], sram_buf[179], sram_buf[180], sram_buf[181], sram_buf[182], sram_buf[183], sram_buf[184], sram_buf[185], sram_buf[186], sram_buf[187], sram_buf[188], sram_buf[189], sram_buf[190], sram_buf[191], sram_buf[192], sram_buf[193], sram_buf[194], sram_buf[195], sram_buf[196], sram_buf[197], sram_buf[198], sram_buf[199], 
                            sram_buf[200], sram_buf[201], sram_buf[202], sram_buf[203], sram_buf[204], sram_buf[205], sram_buf[206], sram_buf[207], sram_buf[208], sram_buf[209], sram_buf[210], sram_buf[211], sram_buf[212], sram_buf[213], sram_buf[214], sram_buf[215], sram_buf[216], sram_buf[217], sram_buf[218], sram_buf[219], sram_buf[220], sram_buf[221], sram_buf[222], sram_buf[223], sram_buf[224], 
                            sram_buf[225], sram_buf[226], sram_buf[227], sram_buf[228], sram_buf[229], sram_buf[230], sram_buf[231], sram_buf[232], sram_buf[233], sram_buf[234], sram_buf[235], sram_buf[236], sram_buf[237], sram_buf[238], sram_buf[239], sram_buf[240], sram_buf[241], sram_buf[242], sram_buf[243], sram_buf[244], sram_buf[245], sram_buf[246], sram_buf[247], sram_buf[248], sram_buf[249], 
                            sram_buf[250], sram_buf[251], sram_buf[252], sram_buf[253], sram_buf[254], sram_buf[255], sram_buf[256], sram_buf[257], sram_buf[258], sram_buf[259], sram_buf[260], sram_buf[261], sram_buf[262], sram_buf[263], sram_buf[264], sram_buf[265], sram_buf[266], sram_buf[267], sram_buf[268], sram_buf[269], sram_buf[270], sram_buf[271], sram_buf[272], sram_buf[273], sram_buf[274], 
                            sram_buf[275], sram_buf[276], sram_buf[277], sram_buf[278], sram_buf[279], sram_buf[280], sram_buf[281], sram_buf[282], sram_buf[283], sram_buf[284], sram_buf[285], sram_buf[286], sram_buf[287], sram_buf[288], sram_buf[289], sram_buf[290], sram_buf[291], sram_buf[292], sram_buf[293], sram_buf[294], sram_buf[295], sram_buf[296], sram_buf[297], sram_buf[298], sram_buf[299], 
                            sram_buf[300], sram_buf[301], sram_buf[302], sram_buf[303], sram_buf[304], sram_buf[305], sram_buf[306], sram_buf[307], sram_buf[308], sram_buf[309], sram_buf[310], sram_buf[311], sram_buf[312], sram_buf[313], sram_buf[314], sram_buf[315], sram_buf[316], sram_buf[317], sram_buf[318], sram_buf[319], sram_buf[320], sram_buf[321], sram_buf[322], sram_buf[323], sram_buf[324], 
                            sram_buf[325], sram_buf[326], sram_buf[327], sram_buf[328], sram_buf[329], sram_buf[330], sram_buf[331], sram_buf[332], sram_buf[333], sram_buf[334], sram_buf[335], sram_buf[336], sram_buf[337], sram_buf[338], sram_buf[339], sram_buf[340], sram_buf[341], sram_buf[342], sram_buf[343], sram_buf[344], sram_buf[345], sram_buf[346], sram_buf[347], sram_buf[348], sram_buf[349], 
                            sram_buf[350], sram_buf[351], sram_buf[352], sram_buf[353], sram_buf[354], sram_buf[355], sram_buf[356], sram_buf[357], sram_buf[358], sram_buf[359], sram_buf[360], sram_buf[361], sram_buf[362], sram_buf[363], sram_buf[364], sram_buf[365], sram_buf[366], sram_buf[367], sram_buf[368], sram_buf[369], sram_buf[370], sram_buf[371], sram_buf[372], sram_buf[373], sram_buf[374], 
                            sram_buf[375], sram_buf[376], sram_buf[377], sram_buf[378], sram_buf[379], sram_buf[380], sram_buf[381], sram_buf[382], sram_buf[383], sram_buf[384], sram_buf[385], sram_buf[386], sram_buf[387], sram_buf[388], sram_buf[389], sram_buf[390], sram_buf[391], sram_buf[392], sram_buf[393], sram_buf[394], sram_buf[395], sram_buf[396], sram_buf[397], sram_buf[398], sram_buf[399], 
                            sram_buf[400], sram_buf[401], sram_buf[402], sram_buf[403], sram_buf[404], sram_buf[405], sram_buf[406], sram_buf[407], sram_buf[408], sram_buf[409], sram_buf[410], sram_buf[411], sram_buf[412], sram_buf[413], sram_buf[414], sram_buf[415], sram_buf[416], sram_buf[417], sram_buf[418], sram_buf[419], sram_buf[420], sram_buf[421], sram_buf[422], sram_buf[423], sram_buf[424], 
                            sram_buf[425], sram_buf[426], sram_buf[427], sram_buf[428], sram_buf[429], sram_buf[430], sram_buf[431], sram_buf[432], sram_buf[433], sram_buf[434], sram_buf[435], sram_buf[436], sram_buf[437], sram_buf[438], sram_buf[439], sram_buf[440], sram_buf[441], sram_buf[442], sram_buf[443], sram_buf[444], sram_buf[445], sram_buf[446], sram_buf[447], sram_buf[448], sram_buf[449], 
                            sram_buf[450], sram_buf[451], sram_buf[452], sram_buf[453], sram_buf[454], sram_buf[455], sram_buf[456], sram_buf[457], sram_buf[458], sram_buf[459], sram_buf[460], sram_buf[461], sram_buf[462], sram_buf[463], sram_buf[464], sram_buf[465], sram_buf[466], sram_buf[467], sram_buf[468], sram_buf[469], sram_buf[470], sram_buf[471], sram_buf[472], sram_buf[473], sram_buf[474], 
                            sram_buf[475], sram_buf[476], sram_buf[477], sram_buf[478], sram_buf[479], sram_buf[480], sram_buf[481], sram_buf[482], sram_buf[483], sram_buf[484], sram_buf[485], sram_buf[486], sram_buf[487], sram_buf[488], sram_buf[489], sram_buf[490], sram_buf[491], sram_buf[492], sram_buf[493], sram_buf[494], sram_buf[495], sram_buf[496], sram_buf[497], sram_buf[498], sram_buf[499], 
                            sram_buf[500], sram_buf[501], sram_buf[502], sram_buf[503], sram_buf[504], sram_buf[505], sram_buf[506], sram_buf[507], sram_buf[508], sram_buf[509], sram_buf[510], sram_buf[511], sram_buf[512], sram_buf[513], sram_buf[514], sram_buf[515], sram_buf[516], sram_buf[517], sram_buf[518], sram_buf[519], sram_buf[520], sram_buf[521], sram_buf[522], sram_buf[523], sram_buf[524], 
                            sram_buf[525], sram_buf[526], sram_buf[527], sram_buf[528], sram_buf[529], sram_buf[530], sram_buf[531], sram_buf[532], sram_buf[533], sram_buf[534], sram_buf[535], sram_buf[536], sram_buf[537], sram_buf[538], sram_buf[539], sram_buf[540], sram_buf[541], sram_buf[542], sram_buf[543], sram_buf[544], sram_buf[545], sram_buf[546], sram_buf[547], sram_buf[548], sram_buf[549], 
                            sram_buf[550], sram_buf[551], sram_buf[552], sram_buf[553], sram_buf[554], sram_buf[555], sram_buf[556], sram_buf[557], sram_buf[558], sram_buf[559], sram_buf[560], sram_buf[561], sram_buf[562], sram_buf[563], sram_buf[564], sram_buf[565], sram_buf[566], sram_buf[567], sram_buf[568], sram_buf[569], sram_buf[570], sram_buf[571], sram_buf[572], sram_buf[573], sram_buf[574], 
                            sram_buf[575], sram_buf[576], sram_buf[577], sram_buf[578], sram_buf[579], sram_buf[580], sram_buf[581], sram_buf[582], sram_buf[583], sram_buf[584], sram_buf[585], sram_buf[586], sram_buf[587], sram_buf[588], sram_buf[589], sram_buf[590], sram_buf[591], sram_buf[592], sram_buf[593], sram_buf[594], sram_buf[595], sram_buf[596], sram_buf[597], sram_buf[598], sram_buf[599], 
                            sram_buf[600], sram_buf[601], sram_buf[602], sram_buf[603], sram_buf[604], sram_buf[605], sram_buf[606], sram_buf[607], sram_buf[608], sram_buf[609], sram_buf[610], sram_buf[611], sram_buf[612], sram_buf[613], sram_buf[614], sram_buf[615], sram_buf[616], sram_buf[617], sram_buf[618], sram_buf[619], sram_buf[620], sram_buf[621], sram_buf[622], sram_buf[623], sram_buf[624]};
        //270
        4'b0011: out = {sram_buf[600], sram_buf[575], sram_buf[550], sram_buf[525], sram_buf[500], sram_buf[475], sram_buf[450], sram_buf[425], sram_buf[400], sram_buf[375], sram_buf[350], sram_buf[325], sram_buf[300], sram_buf[275], sram_buf[250], sram_buf[225], sram_buf[200], sram_buf[175], sram_buf[150], sram_buf[125], sram_buf[100], sram_buf[75], sram_buf[50], sram_buf[25], sram_buf[0], 
                            sram_buf[601], sram_buf[576], sram_buf[551], sram_buf[526], sram_buf[501], sram_buf[476], sram_buf[451], sram_buf[426], sram_buf[401], sram_buf[376], sram_buf[351], sram_buf[326], sram_buf[301], sram_buf[276], sram_buf[251], sram_buf[226], sram_buf[201], sram_buf[176], sram_buf[151], sram_buf[126], sram_buf[101], sram_buf[76], sram_buf[51], sram_buf[26], sram_buf[1], 
                            sram_buf[602], sram_buf[577], sram_buf[552], sram_buf[527], sram_buf[502], sram_buf[477], sram_buf[452], sram_buf[427], sram_buf[402], sram_buf[377], sram_buf[352], sram_buf[327], sram_buf[302], sram_buf[277], sram_buf[252], sram_buf[227], sram_buf[202], sram_buf[177], sram_buf[152], sram_buf[127], sram_buf[102], sram_buf[77], sram_buf[52], sram_buf[27], sram_buf[2], 
                            sram_buf[603], sram_buf[578], sram_buf[553], sram_buf[528], sram_buf[503], sram_buf[478], sram_buf[453], sram_buf[428], sram_buf[403], sram_buf[378], sram_buf[353], sram_buf[328], sram_buf[303], sram_buf[278], sram_buf[253], sram_buf[228], sram_buf[203], sram_buf[178], sram_buf[153], sram_buf[128], sram_buf[103], sram_buf[78], sram_buf[53], sram_buf[28], sram_buf[3], 
                            sram_buf[604], sram_buf[579], sram_buf[554], sram_buf[529], sram_buf[504], sram_buf[479], sram_buf[454], sram_buf[429], sram_buf[404], sram_buf[379], sram_buf[354], sram_buf[329], sram_buf[304], sram_buf[279], sram_buf[254], sram_buf[229], sram_buf[204], sram_buf[179], sram_buf[154], sram_buf[129], sram_buf[104], sram_buf[79], sram_buf[54], sram_buf[29], sram_buf[4], 
                            sram_buf[605], sram_buf[580], sram_buf[555], sram_buf[530], sram_buf[505], sram_buf[480], sram_buf[455], sram_buf[430], sram_buf[405], sram_buf[380], sram_buf[355], sram_buf[330], sram_buf[305], sram_buf[280], sram_buf[255], sram_buf[230], sram_buf[205], sram_buf[180], sram_buf[155], sram_buf[130], sram_buf[105], sram_buf[80], sram_buf[55], sram_buf[30], sram_buf[5], 
                            sram_buf[606], sram_buf[581], sram_buf[556], sram_buf[531], sram_buf[506], sram_buf[481], sram_buf[456], sram_buf[431], sram_buf[406], sram_buf[381], sram_buf[356], sram_buf[331], sram_buf[306], sram_buf[281], sram_buf[256], sram_buf[231], sram_buf[206], sram_buf[181], sram_buf[156], sram_buf[131], sram_buf[106], sram_buf[81], sram_buf[56], sram_buf[31], sram_buf[6], 
                            sram_buf[607], sram_buf[582], sram_buf[557], sram_buf[532], sram_buf[507], sram_buf[482], sram_buf[457], sram_buf[432], sram_buf[407], sram_buf[382], sram_buf[357], sram_buf[332], sram_buf[307], sram_buf[282], sram_buf[257], sram_buf[232], sram_buf[207], sram_buf[182], sram_buf[157], sram_buf[132], sram_buf[107], sram_buf[82], sram_buf[57], sram_buf[32], sram_buf[7], 
                            sram_buf[608], sram_buf[583], sram_buf[558], sram_buf[533], sram_buf[508], sram_buf[483], sram_buf[458], sram_buf[433], sram_buf[408], sram_buf[383], sram_buf[358], sram_buf[333], sram_buf[308], sram_buf[283], sram_buf[258], sram_buf[233], sram_buf[208], sram_buf[183], sram_buf[158], sram_buf[133], sram_buf[108], sram_buf[83], sram_buf[58], sram_buf[33], sram_buf[8], 
                            sram_buf[609], sram_buf[584], sram_buf[559], sram_buf[534], sram_buf[509], sram_buf[484], sram_buf[459], sram_buf[434], sram_buf[409], sram_buf[384], sram_buf[359], sram_buf[334], sram_buf[309], sram_buf[284], sram_buf[259], sram_buf[234], sram_buf[209], sram_buf[184], sram_buf[159], sram_buf[134], sram_buf[109], sram_buf[84], sram_buf[59], sram_buf[34], sram_buf[9], 
                            sram_buf[610], sram_buf[585], sram_buf[560], sram_buf[535], sram_buf[510], sram_buf[485], sram_buf[460], sram_buf[435], sram_buf[410], sram_buf[385], sram_buf[360], sram_buf[335], sram_buf[310], sram_buf[285], sram_buf[260], sram_buf[235], sram_buf[210], sram_buf[185], sram_buf[160], sram_buf[135], sram_buf[110], sram_buf[85], sram_buf[60], sram_buf[35], sram_buf[10], 
                            sram_buf[611], sram_buf[586], sram_buf[561], sram_buf[536], sram_buf[511], sram_buf[486], sram_buf[461], sram_buf[436], sram_buf[411], sram_buf[386], sram_buf[361], sram_buf[336], sram_buf[311], sram_buf[286], sram_buf[261], sram_buf[236], sram_buf[211], sram_buf[186], sram_buf[161], sram_buf[136], sram_buf[111], sram_buf[86], sram_buf[61], sram_buf[36], sram_buf[11], 
                            sram_buf[612], sram_buf[587], sram_buf[562], sram_buf[537], sram_buf[512], sram_buf[487], sram_buf[462], sram_buf[437], sram_buf[412], sram_buf[387], sram_buf[362], sram_buf[337], sram_buf[312], sram_buf[287], sram_buf[262], sram_buf[237], sram_buf[212], sram_buf[187], sram_buf[162], sram_buf[137], sram_buf[112], sram_buf[87], sram_buf[62], sram_buf[37], sram_buf[12], 
                            sram_buf[613], sram_buf[588], sram_buf[563], sram_buf[538], sram_buf[513], sram_buf[488], sram_buf[463], sram_buf[438], sram_buf[413], sram_buf[388], sram_buf[363], sram_buf[338], sram_buf[313], sram_buf[288], sram_buf[263], sram_buf[238], sram_buf[213], sram_buf[188], sram_buf[163], sram_buf[138], sram_buf[113], sram_buf[88], sram_buf[63], sram_buf[38], sram_buf[13], 
                            sram_buf[614], sram_buf[589], sram_buf[564], sram_buf[539], sram_buf[514], sram_buf[489], sram_buf[464], sram_buf[439], sram_buf[414], sram_buf[389], sram_buf[364], sram_buf[339], sram_buf[314], sram_buf[289], sram_buf[264], sram_buf[239], sram_buf[214], sram_buf[189], sram_buf[164], sram_buf[139], sram_buf[114], sram_buf[89], sram_buf[64], sram_buf[39], sram_buf[14], 
                            sram_buf[615], sram_buf[590], sram_buf[565], sram_buf[540], sram_buf[515], sram_buf[490], sram_buf[465], sram_buf[440], sram_buf[415], sram_buf[390], sram_buf[365], sram_buf[340], sram_buf[315], sram_buf[290], sram_buf[265], sram_buf[240], sram_buf[215], sram_buf[190], sram_buf[165], sram_buf[140], sram_buf[115], sram_buf[90], sram_buf[65], sram_buf[40], sram_buf[15], 
                            sram_buf[616], sram_buf[591], sram_buf[566], sram_buf[541], sram_buf[516], sram_buf[491], sram_buf[466], sram_buf[441], sram_buf[416], sram_buf[391], sram_buf[366], sram_buf[341], sram_buf[316], sram_buf[291], sram_buf[266], sram_buf[241], sram_buf[216], sram_buf[191], sram_buf[166], sram_buf[141], sram_buf[116], sram_buf[91], sram_buf[66], sram_buf[41], sram_buf[16], 
                            sram_buf[617], sram_buf[592], sram_buf[567], sram_buf[542], sram_buf[517], sram_buf[492], sram_buf[467], sram_buf[442], sram_buf[417], sram_buf[392], sram_buf[367], sram_buf[342], sram_buf[317], sram_buf[292], sram_buf[267], sram_buf[242], sram_buf[217], sram_buf[192], sram_buf[167], sram_buf[142], sram_buf[117], sram_buf[92], sram_buf[67], sram_buf[42], sram_buf[17], 
                            sram_buf[618], sram_buf[593], sram_buf[568], sram_buf[543], sram_buf[518], sram_buf[493], sram_buf[468], sram_buf[443], sram_buf[418], sram_buf[393], sram_buf[368], sram_buf[343], sram_buf[318], sram_buf[293], sram_buf[268], sram_buf[243], sram_buf[218], sram_buf[193], sram_buf[168], sram_buf[143], sram_buf[118], sram_buf[93], sram_buf[68], sram_buf[43], sram_buf[18], 
                            sram_buf[619], sram_buf[594], sram_buf[569], sram_buf[544], sram_buf[519], sram_buf[494], sram_buf[469], sram_buf[444], sram_buf[419], sram_buf[394], sram_buf[369], sram_buf[344], sram_buf[319], sram_buf[294], sram_buf[269], sram_buf[244], sram_buf[219], sram_buf[194], sram_buf[169], sram_buf[144], sram_buf[119], sram_buf[94], sram_buf[69], sram_buf[44], sram_buf[19], 
                            sram_buf[620], sram_buf[595], sram_buf[570], sram_buf[545], sram_buf[520], sram_buf[495], sram_buf[470], sram_buf[445], sram_buf[420], sram_buf[395], sram_buf[370], sram_buf[345], sram_buf[320], sram_buf[295], sram_buf[270], sram_buf[245], sram_buf[220], sram_buf[195], sram_buf[170], sram_buf[145], sram_buf[120], sram_buf[95], sram_buf[70], sram_buf[45], sram_buf[20], 
                            sram_buf[621], sram_buf[596], sram_buf[571], sram_buf[546], sram_buf[521], sram_buf[496], sram_buf[471], sram_buf[446], sram_buf[421], sram_buf[396], sram_buf[371], sram_buf[346], sram_buf[321], sram_buf[296], sram_buf[271], sram_buf[246], sram_buf[221], sram_buf[196], sram_buf[171], sram_buf[146], sram_buf[121], sram_buf[96], sram_buf[71], sram_buf[46], sram_buf[21], 
                            sram_buf[622], sram_buf[597], sram_buf[572], sram_buf[547], sram_buf[522], sram_buf[497], sram_buf[472], sram_buf[447], sram_buf[422], sram_buf[397], sram_buf[372], sram_buf[347], sram_buf[322], sram_buf[297], sram_buf[272], sram_buf[247], sram_buf[222], sram_buf[197], sram_buf[172], sram_buf[147], sram_buf[122], sram_buf[97], sram_buf[72], sram_buf[47], sram_buf[22], 
                            sram_buf[623], sram_buf[598], sram_buf[573], sram_buf[548], sram_buf[523], sram_buf[498], sram_buf[473], sram_buf[448], sram_buf[423], sram_buf[398], sram_buf[373], sram_buf[348], sram_buf[323], sram_buf[298], sram_buf[273], sram_buf[248], sram_buf[223], sram_buf[198], sram_buf[173], sram_buf[148], sram_buf[123], sram_buf[98], sram_buf[73], sram_buf[48], sram_buf[23], 
                            sram_buf[624], sram_buf[599], sram_buf[574], sram_buf[549], sram_buf[524], sram_buf[499], sram_buf[474], sram_buf[449], sram_buf[424], sram_buf[399], sram_buf[374], sram_buf[349], sram_buf[324], sram_buf[299], sram_buf[274], sram_buf[249], sram_buf[224], sram_buf[199], sram_buf[174], sram_buf[149], sram_buf[124], sram_buf[99], sram_buf[74], sram_buf[49], sram_buf[24]};
        default: out = 0;
        endcase
    else
        out = 0;
end


endmodule