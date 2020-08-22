module vdma
(
aclk,
rst,
//AXIS
axis_tvalid,
axis_tready,
axis_tdata,
axis_tlast,
axis_tuser,
//AXI R
axi_arvalid,
axi_arready,
axi_id,
axi_addr,
axi_arlen,
axi_arsize,
axi_arburst,
axi_arlock,
axi_arcache,
axi_arprot,
axi_arqos,
axi_rvalid,
axi_rready,
axi_rdata,
axi_rlast,
axi_rid,
axi_rresp,
//CFG
cfg_active,
cfg_frame_addr,
cfg_frame_lines,
cfg_line_stride,
cfg_line_words

);
    `include "vdma_config.vh"
    localparam                          IDLE=3'b0;
    localparam                          INIT=3'b01;
    localparam                          START=3'b010;
    localparam                          TRANS=3'b011;
    localparam                          END=3'b100;


    input   wire                                aclk;
    input   wire                                rst;

    //axi stream
    output  wire                                axis_tvalid;
    input   wire                                axis_tready;
    output  wire    [DATA_WIDTH-1:0]            axis_tdata;
    output  wire                                axis_tlast;
    output  wire                                axis_tuser;

    //axi
    output  wire                                axi_arvalid;
    input   wire                                axi_arready;
    output  wire    [ID_WIDTH-1:0]              axi_id;
    output  wire    [ADDR_WIDTH-1:0]            axi_addr;
    output  wire    [LEN_WIDTH-1:0]             axi_arlen;
    output  wire    [SIZE_WIDTH-1:0]            axi_arsize;
    output  wire    [BURST_WIDTH-1:0]           axi_arburst;
    output  wire                                axi_arlock;
    output  wire    [CACHE_WIDTH-1:0]           axi_arcache;
    output  wire    [PROT_WIDTH-1:0]            axi_arprot;
    output  wire    [QOS_WIDTH-1:0]             axi_arqos;
    //
    input   wire                                axi_rvalid,
    output  wire                                axi_rready;
    input   wire    [DATA_WIDTH-1:0]            axi_rdata;
    input   wire                                axi_rlast;
    input   wire    [ID_WIDTH-1:0]              axi_rid;
    input   wire    [RESP_WIDTH-1:0]            axi_rresp;

    //
    input   wire                                cfg_active;
    input   wire    [ADDR_WIDTH-1:0]            cfg_frame_addr;
    input   wire    [LINE_COUNT-1:0]            cfg_frame_lines;
    input   wire    [STRIDE_COUNT-1:0]          cfg_line_stride;
    input   wire    [LINE_COUNT-ADDRLSB-1:0]    cfg_line_words;

    //fifo
    wire                                        write_to_fifo;
    wire                                        read_from_fifo;
    wire    [DATA_WIDTH-1:0]                    write_data;
    wire                                        fifo_full;
    wire                                        fifo_empty;
    wire    [FIFO_DEPTH-1:0]                    fifo_fill;
    reg     [FIFO_DEPTH-1:0]                    fifo_level;
    wire                                        fifo_is_full;

    reg                                         axi_arvalid_r;
    reg     [ID_WIDTH-1:0]                      axi_id_r;
    reg     [ADDR_WIDTH-1:0]                    axi_addr_r;
    reg     [LEN_WIDTH-1:0]                     axi_arlen_r;

    reg                                         r_none_outstanding;
    reg     [OUTSTANDING_COUNT-1:0]             r_bursts_outstanding;

    reg                                         abort_pending_r;
    reg                                         burst_start;

    wire                                        vlast;
    wire                                        hlast;
    reg        [LINE_COUNT-1:0]                 r_frame_lines;
    reg        [STRIDE_COUNT-1:0]               r_line_stride;
    reg        [LINE_COUNT-ADDRLSB-1:0]         r_line_words;
    reg        [ADDR_WIDTH:0]                   r_frame_addr;

    reg                                         req_hlast;
    reg                                         req_vlast;
    reg        [ADDR_WIDTH-1:0]                 req_addr;
    reg        [ADDR_WIDTH-1:0]                 req_line_addr;
    reg        [LINE_COUNT-ADDRLSB-1:0]         req_line_words;
    reg        [LINE_COUNT-1:0]                 line_counter;

    //
    reg                                         rd_hlast;
    reg                                         rd_vlast;
    reg        [LINE_COUNT-1:0]                 rd_lines;
    reg        [LINE_COUNT-ADDRLSB-1:0]         rd_line_beats;
    
    reg        [MAX_BURST-1:0]                   burst_max;
    reg        [MAX_BURST-1:0]                   till_boundary;

    //fsm
    reg[2:0]                                     fsm_state;
    reg[2:0]                                     fsm_state_nxt;
    reg                                          fsm_init;
    //reg                                            addr_cal;
    reg                                          trans_start;

    reg                                          r_busy;
    reg                                          r_err;
    //reg                                    cfg_active;
    
    always@(posedge aclk or negedge rst)
    begin
        if(!rst)
        begin
            fsm_state <= IDLE;
        end
        else
        begin
            fsm_state <= fsm_state_nxt;
        end
    end
    
    always@(*)
    begin
        fsm_state_nxt = fsm_state;
        case(fsm_state)
        IDLE:
        begin
            fsm_init = 1'b0;
            start_burst = 1'b0;
            trans_start = 1'b0;
            fsm_state_nxt = r_err?IDLE:INIT;
        end
        INIT:
        begin
            fsm_init = 1'b1;
            start_burst = 1'b0;
            trans_start = 1'b0;
            fsm_state_nxt = r_err?IDLE:CAL;
        end
        START:
        begin
            fsm_init = 1'b0;
            start_burst = 1'b1;
            trans_start = 1'b0;
            fsm_state_nxt = r_err?IDLE:TRANS;
        end
        TRANS:
        begin
            fsm_init = 1'b0;
            start_burst = 1'b0;
            trans_start = 1'b1;
            fsm_state_nxt = r_err?IDLE:END;
        end
        END:
        begin
            fsm_init = 1'b0;
            addr_cal = 1'b0;
            trans_start = 1'b0;
            fsm_state_nxt = r_err?IDLE:INIT;
        end
    end

    always@(posedge aclk or negedge rst)
    begin
        if(!rst)
        begin
            r_busy <= 1'b0;
            r_err <= 1'b0;
        end
        else if(!r_busy)
        begin
            if(cfg_active && !r_err)
                r_busy <= 1'b1;
        end
        else
        begin
            if(axi_rready && axi_rvalid && axi_rresp[1])//fixed me rresp
            begin
                r_busy <= 1'b0;
                r_err <= 1'b1;
            end
        end
    end

    //fifo
    assign write_to_fifo = axi_rvalid;
    assign write_data = axi_rdata;
    assign vlast = rd_vlast;
    assign hlast = rd_hlast;
    assign axi_rready = !fifo_full;

    always@(posedge aclk or negedge rst)
    if(!rst)
        fifo_level <= (1<<FIFO_DEPTH);
    else
    case ({trans_start,read_from_fifo&&!fifo_empty})//fixed me
    2'b00 :
    begin
    end
    2'b10:
    fifo_level <=fifo_level-(axi_arlen_r+1'b1);
    2'b11:
    fifo_level <=fifo_level-axi_arlen;
    2'b01:
    fifo_level <=fifo_level+1'b1;
    endcase

    assign axis_tvalid = !fifo_empty;
    assign read_from_fifo = axis_tvalid && axis_tready;

    sfifo#(
    .BW(DATA_WIDTH+2),
    .LGFLEN(FIFO_DEPTH)
    )
    vfifo(
    aclk,
    rst,
    write_to_fifo,
    {vlast&&hlast,hlast,write_data},
    fifo_full,
    fifo_fill,
    read_from_fifo,
    {axis_tlast,axis_tuser,axis_tdata},
    fifo_empty
    );

    assign fifo_is_full = (fifo_level < (1<<MAX_BURST));

    //
    always@(posedge aclk or negedge rst)
    if(!rst)
    begin
        r_frame_addr <=    {1'b0,cfg_frame_addr};
        r_frame_lines<= cfg_frame_lines;
        r_line_words <= cfg_line_words;
        r_line_stride<={{(ADDRLSB){1'b0}},cfg_line_stride[STRIDE_COUNT:ADDRLSB]};
    end
    else
    begin
        if(cfg_active&&trans_start==1'b1&&req_hlast&&req_vlast)
        begin
            r_frame_addr <= {1'b0,cfg_frame_addr};
            r_frame_lines <= cfg_frame_lines;
            r_line_words <= cfg_line_words;
            //align
            r_line_stride <= {{(ADDRLSB){1'b0}},cfg_line_stride[STRIDE_COUNT:ADDRLSB]};
        end
    end
    
    always@(posedge aclk or negedge rst)
    if(!rst)
    begin
        req_addr <= {(ADDR_WIDTH+1'b1){1b'0}};
        req_line_addr <= {(ADDR_WIDTH+1'b0){1b'0}};
        req_line_words <= {(LINE_COUNT-ADDRLSB){1b'0}};
    end
    else
    begin
        if(trans_start == 1'b1)
        begin
            if(req_hlast && req_vlast)
            begin
                if(cfg_active)
                begin
                    req_addr <= {1'b0,cfg_frame_addr};
                    req_line_addr <= {1'b0,cfg_frame_addr};
                    req_line_words <= cfg_line_words;
                end
                else
                begin
                    req_addr <= r_frame_addr;
                    req_line_addr <= r_frame_addr;
                    req_line_words <= r_line_words;
                end
            end
            else if(req_hlast)
            begin
                req_addr <= req_line_addr + (r_line_stride<<axi_arsize);
                req_line_addr <= req_line_addr + (r_line_stride<<axi_arsize);
                req_line_words <= r_line_words;
            end
            else
            begin
                req_addr <= req_addr +((axi_arlen_r+1'b1)<<axi_arsize);
                req_line_words <= req_line_words-(axi_arlen_r + 1'b1);
                req_addr[ADDRLSB-1:0] <= {(ADDRLSB)1'b0};
            end
        end
        
    end
    
    always@(posedge aclk or negedge rst)
    if(!rst)
    begin
        line_counter <= {(LINE_COUNT){1'b0}};
        req_vlast <= 1'b0;
    end
    else if(trans_start == 1'b1 && req_hlast)
    begin
        if(req_vlast)
        begin
            if(cfg_active)
            begin
                line_counter <= cfg_frame_lines - 1'b1;
                req_vlast <= (cfg_frame_lines<=1'b1);
            end
            else//fixed me?
            begin
                line_counter <= r_frame_lines-1'b1;
                req_vlast <= (r_frame_lines<=1'b1);
            end
        end
        else
        begin
            line_counter <= line_counter -1'b1;
            req_vlast <= (line_counter <= 1'b1);
        end
    end

    //rd hlast vlast ->h,v last
    always@(posedge aclk or negedge rst)
    if(!rst)
    begin
        rd_line_beats <= {(LINE_COUNT-ADDRLSB){1b'0}};
        rd_hlast <= 1'b0;
    end
    else if(axi_rvalid && axi_rready)
    begin
        if(rd_hlast)
        begin
            rd_line_beats <= r_line_words -1'b1;
            rd_hlast <= (r_line_words <=1'b1);
        end
        else
        begin
            rd_line_beats <= rd_line_beats -1'b1;
            rd_hlast <= (rd_line_beats <=1'b1);
        end
    end

    always@(posedge aclk or negedge rst)
    if(!rst)
    begin
        rd_lines <= {(LINE_COUNT){1'b0}};
        rd_vlast <= 1'b0;
    end
    else
    begin
        if(axi_rvalid && axi_rready && rd_hlast)
        begin
            if(vlast)
            begin
                rd_lines <= r_frame_lines - 1'b1;
                rd_vlast <= (r_frame_lines <= 1'b1);
            end
            else
            begin
                rd_lines <= rd_lines - 1'b1;
                rd_vlast <= (rd_lines <= 1'b1);
            end
        end
    end

    always@(posedge aclk or negedge rst)
    if(!rst)
    begin
        r_bursts_outstanding < = {(OUTSTANDING_COUNT){1'b0}};
        r_none_outstanding <= 1'b1;
    end
    else
        case ({trans_start,axi_rvalid&&axi_rready&&axi_rlast})
        
        2'b01:
        begin
            r_bursts_outstanding <= r_bursts_outstanding -1'b1;
            r_none_outstanding <= (r_bursts_outstanding==1);
        end
        2'b10:
        begin
            r_bursts_outstanding <= r_bursts_outstanding +1'b1;
            r_none_outstanding <= 1'b0;
        end
        default:
        begin
        end
    endcase

    always@(posedge aclk or negedge rst)
    if(!rst)
        abort_pending_r <= 1'b0;
    else if(axi_rvalid && axi_rready && axi_rresp[1])
        abort_pending_r <= 1'b1;
    else if(!r_busy && !r_none_outstanding)
        abort_pending_r <= 1'b1;

    always@(posedge aclk or negedge rst)
    begin
        //start_burst = 1'b1;
        if(fifo_full)//fixed me ?
            start_burst = 1'b0;
        if(axi_arvalid_r && axi_arready)
            start_burst = 1'b0;
        if(abort_pending_r)
            start_burst = 1'b0;
    end

    always@(posedge aclk or negedge rst)
    if(fsm_init == 1'b1)
    begin
        if(req_line_words > (1<<MAX_BURST))
            burst_max <= (1<<MAX_BURST);
        else
            burst_max <= req_line_words;
    end

    always@(*)
        till_boundary = ~req_addr[ADDRLSB+:MAX_BURST];

    always@(posedge aclk or negedge rst)
    if(!axi_arvalid_r ||axi_arready)
    begin
        if(till_boundary >1'b0 && burst_max <= till_boundary)
            axi_arlen_r <= burst_max -1'b1;
        else
            axi_arlen_r <= till_boundary;
    end

    always@(posedge aclk or negedge rst)
    if(!rst)
        req_hlast <= 1'b0;
    else
    begin
        //fixed me reset?
        if(fsm_init == 1'b1 || start_burst)
        begin
            req_hlast <= 1'b1;
            if(req_line_words > till_boundary + 1'b1)
                req_hlast <= 1'b0;
            if(req_line_words > burst_max)
                req_hlast <= 1'b0;
        end
    end

    always@(posedge aclk or negedge rst)
    if(!rst)
        axi_arvalid_r <= 1'b0;
    else if(!axi_arvalid_r || axi_arready)
        axi_arvalid_r <= start_burst;
        
    always@(posedge aclk or negedge rst)
    if(!rst)
        axi_addr_r <= {(ADDR_WIDTH){1'b0}};
    else
    begin
        if(start_burst)
            axi_addr_r <= req_addr[ADDR_WIDTH-1:0];
            
        axi_addr_r[ADDRLSB-1:0] <= {ADDRLSB{1'b0}};
    end

    assign axi_arvalid = axi_arvalid_r;
    assign axi_id = {(ID_WIDTH){1'b0}};
    assign axi_addr=axi_addr_r;
    assign axi_arlen =axi_arlen_r;
    assign axi_arsize = $clog2(DATA_WIDTH)-3;
    //INCR
    assign axi_arburst = 2'b01;
    assign axi_arlock = 1'b0;
    assign axi_arcache = {(CACHE_WIDTH){1'b0}};
    assign axi_arprot = {(PROT_WIDTH){1'b0}};
    assign axi_arqos = {(QOS_WIDTH){1'b0}};
endmodule