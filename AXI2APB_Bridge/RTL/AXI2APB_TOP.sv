module AXI2APB_TOP #(
    parameter ADDR_WIDTH            = 32,
    parameter DATA_WIDTH            = 32
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // AXI Write Address Channel
    input  wire                     awid_i,    // 쓰기 주소 ID: 쓰기 주소 채널 그룹의 식별 태그
    input  wire [ADDR_WIDTH-1:0]    awaddr_i,  // 쓰기 주소: 버스트 전송의 첫 번째 전송 주소
    input  wire [3:0]               awlen_i,   // 버스트 길이: 버스트 내 전송 수 (정확한 전송 개수)
    input  wire [2:0]               awsize_i,  // 전송 크기: 버스트 내 각 전송의 바이트 단위 크기
    input  wire [1:0]               awburst_i, // 버스트 유형: 버스트 내 주소 증가/고정 방식 결정
    input  wire                     awvalid_i, // 쓰기 주소 유효: 마스터가 유효한 주소 및 제어 정보를 제시
    output wire                     awready_o, // 쓰기 주소 준비: 슬레이브가 주소 및 제어 신호 수락 준비

    // AXI Write Data Channel
    input  wire                     wid_i,     // 쓰기 데이터 ID: 쓰기 데이터 전송의 식별 태그 (AXI3 전용)
    input  wire [DATA_WIDTH-1:0]    wdata_i,   // 쓰기 데이터: 전송할 실제 데이터
    input  wire [3:0]               wstrb_i,   // 쓰기 스트로브: 각 바이트 레인 유효 데이터 표시
    input  wire                     wlast_i,   // 마지막 전송: 버스트 내 마지막 전송 표시
    input  wire                     wvalid_i,  // 쓰기 유효: 유효한 쓰기 데이터 및 스트로브 제공
    output wire                     wready_o,  // 쓰기 준비: 슬레이브가 쓰기 데이터를 수락할 준비 표시

    // AXI Write Response Channel
    output wire                     bid_o,    // 응답 ID 태그: 쓰기 응답 전송의 식별 태그
    output wire [1:0]               bresp_o,  // 쓰기 응답: 쓰기 트랜잭션 상태 표시
    output wire                     bvalid_o, // 쓰기 응답 유효: 슬레이브가 유효한 쓰기 응답 신호 제시
    input  wire                     bready_i, // 응답 준비: 마스터가 쓰기 응답 수락 준비

    // AXI Read Address Channel
    input  wire                     arid_i,    // 읽기 주소 ID: 읽기 주소 채널 그룹의 식별 태그
    input  wire [ADDR_WIDTH-1:0]    araddr_i,  // 읽기 주소: 버스트 전송의 첫 번째 전송 주소
    input  wire [3:0]               arlen_i,   // 버스트 길이: 버스트 내 전송 수 (정확한 전송 개수)
    input  wire [2:0]               arsize_i,  // 전송 크기: 버스트 내 각 전송의 바이트 단위 크기
    input  wire [1:0]               arburst_i, // 버스트 유형: 버스트 내 주소 증가/고정 방식 결정
    input  wire                     arvalid_i, // 읽기 주소 유효: 마스터가 유효한 주소 및 제어 정보를 제시
    output wire                     arready_o, // 읽기 주소 준비: 슬레이브가 주소 및 제어 신호 수락 준비

    // AXI Read Data Channel
    output wire                     rid_o,    // 읽기 데이터 ID: 읽기 데이터 그룹의 식별 태그
    output wire [DATA_WIDTH-1:0]    rdata_o,  // 읽기 데이터: 슬레이브가 제공하는 읽기 데이터
    output wire [1:0]               rresp_o,  // 읽기 응답: 읽기 전송 상태 표시
    output wire                     rlast_o,  // 마지막 전송: 버스트 내 마지막 읽기 전송 표시
    output wire                     rvalid_o, // 읽기 유효: 유효한 읽기 데이터 및 응답 표시
    input  wire                     rready_i, // 읽기 준비: 마스터가 읽기 데이터 및 응답 수락 준비

    // APB Master Interface
    output wire [ADDR_WIDTH-1:0]    paddr_o,   // APB 주소 버스: 주변 장치로 전송할 주소
    output wire [DATA_WIDTH-1:0]    pwdata_o,  // APB 쓰기 데이터 버스: 주변 장치로 전송할 쓰기 데이터
    output wire                     pwrite_o,  // APB 방향 제어: HIGH일 때 쓰기, LOW일 때 읽기
    output wire                     penable_o, // APB 전송 활성화: 두 번째 및 이후 사이클에서 활성화
    output wire [1:0]               psel_o,    // APB 슬레이브 선택: 각 비트마다 슬레이브 선택 신호
    input  wire [DATA_WIDTH-1:0]    prdata_i,  // APB 읽기 데이터 버스: 슬레이브가 제공하는 읽기 데이터
    input  wire                     pready_i,  // APB 준비: 슬레이브가 전송을 수락하거나 연장 준비
    input  wire                     pslverr_i   // APB 에러 표시: 전송 실패 시 HIGH (미지원 시 LOW 고정)
);

    // fill your code.
    // ---------------------------------------------------------------------------
    // Helper : Address decoder (2‑way, 4‑KB windows)
    // ---------------------------------------------------------------------------
    function automatic logic [1:0] decode_psel(input logic [ADDR_WIDTH-1:0] a);
        if (a[31:12] == 20'h0001F)      decode_psel = 2'b01; // 0x0001_F000
        else if (a[31:12] == 20'h0002F) decode_psel = 2'b10; // 0x0002_F000
        else                            decode_psel = 2'b00; // unreachable
    endfunction

    // ---------------------------------------------------------------------------
    // Encoded command packet (48 bits)
    //  [0]      is_write
    //  [1]      is_incr     (burst type)
    //  [5:2]    len_m1      (awlen/arlen)
    //  [37:6]   address     (32 bits)
    // ---------------------------------------------------------------------------
    localparam CMD_W = 48;

    // FIFO depth = 16 (lg2 = 4)
    localparam LG2   = 4;

    // ---------------------------------------------------------------------------
    // FIFOs
    // ---------------------------------------------------------------------------
    // WRITE command FIFO ---------------------------------------------------------
    logic             wcmd_full,  wcmd_empty;
    logic             wcmd_wren,  wcmd_rden;
    logic [CMD_W-1:0] wcmd_wdata, wcmd_rdata;

    BRIDGE_FIFO #(
        .DEPTH_LG2 (LG2),
        .DATA_WIDTH(CMD_W)
    ) u_wcmd_fifo (
        .clk      (clk),
        .rst_n    (rst_n),
        .full_o   (wcmd_full),
        .wren_i   (wcmd_wren),
        .wdata_i  (wcmd_wdata),
        .empty_o  (wcmd_empty),
        .rden_i   (wcmd_rden),
        .rdata_o  (wcmd_rdata)
    );

    // READ command FIFO ----------------------------------------------------------
    logic             rcmd_full,  rcmd_empty;
    logic             rcmd_wren,  rcmd_rden;
    logic [CMD_W-1:0] rcmd_wdata, rcmd_rdata;

    BRIDGE_FIFO #(
        .DEPTH_LG2 (LG2),
        .DATA_WIDTH(CMD_W)
    ) u_rcmd_fifo (
        .clk      (clk),
        .rst_n    (rst_n),
        .full_o   (rcmd_full),
        .wren_i   (rcmd_wren),
        .wdata_i  (rcmd_wdata),
        .empty_o  (rcmd_empty),
        .rden_i   (rcmd_rden),
        .rdata_o  (rcmd_rdata)
    );

    // WRITE‑data FIFO -----------------------------------------------------------
    logic              wdat_full, wdat_empty;
    logic              wdat_wren, wdat_rden;
    logic [DATA_WIDTH-1:0] wdat_rdata;

    BRIDGE_FIFO #(
        .DEPTH_LG2 (LG2),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_wdat_fifo (
        .clk      (clk),
        .rst_n    (rst_n),
        .full_o   (wdat_full),
        .wren_i   (wdat_wren),
        .wdata_i  (wdata_i),        // write side data directly
        .empty_o  (wdat_empty),
        .rden_i   (wdat_rden),
        .rdata_o  (wdat_rdata)
    );

    // BRESP FIFO ---------------------------------------------------------------
    logic            bq_full, bq_empty;
    logic            bq_wren, bq_rden;
    logic [1:0]      bq_wdata, bq_rdata; // just bresp[1:0]

    BRIDGE_FIFO #(
        .DEPTH_LG2 (LG2),
        .DATA_WIDTH(2)
    ) u_bq (
        .clk      (clk),
        .rst_n    (rst_n),
        .full_o   (bq_full),
        .wren_i   (bq_wren),
        .wdata_i  (bq_wdata),
        .empty_o  (bq_empty),
        .rden_i   (bq_rden),
        .rdata_o  (bq_rdata)
    );

    // RDATA FIFO ---------------------------------------------------------------
    localparam RPKT_W = DATA_WIDTH + 3; // {rlast, rresp[1:0], rdata[31:0]}

    logic              rp_full, rp_empty;
    logic              rp_wren, rp_rden;
    logic [RPKT_W-1:0] rp_wdata, rp_rdata;

    BRIDGE_FIFO #(
        .DEPTH_LG2 (LG2),
        .DATA_WIDTH(RPKT_W)
    ) u_rp (
        .clk      (clk),
        .rst_n    (rst_n),
        .full_o   (rp_full),
        .wren_i   (rp_wren),
        .wdata_i  (rp_wdata),
        .empty_o  (rp_empty),
        .rden_i   (rp_rden),
        .rdata_o  (rp_rdata)
    );

    // ---------------------------------------------------------------------------
    // AXI channel handshakes (push into FIFOs)
    // ---------------------------------------------------------------------------
    assign awready_o = ~wcmd_full;   // space in write‑command FIFO
    assign wready_o  = ~wdat_full;   // space in write‑data FIFO
    assign arready_o = ~rcmd_full;   // space in read‑command FIFO

    // Command packet packing -----------------------------------------------------
    assign wcmd_wren           = awvalid_i & awready_o;
    assign wcmd_wdata[0]       = 1'b1;                 // is_write
    assign wcmd_wdata[1]       = (awburst_i == 2'b01); // is_incr
    assign wcmd_wdata[5:2]     = awlen_i;
    assign wcmd_wdata[37:6]    = awaddr_i;
    assign wcmd_wdata[CMD_W-1:38] = '0;               // padding

    assign rcmd_wren           = arvalid_i & arready_o;
    assign rcmd_wdata[0]       = 1'b0;                 // is_read
    assign rcmd_wdata[1]       = (arburst_i == 2'b01);
    assign rcmd_wdata[5:2]     = arlen_i;
    assign rcmd_wdata[37:6]    = araddr_i;
    assign rcmd_wdata[CMD_W-1:38] = '0;

    assign wdat_wren = wvalid_i & wready_o;  // push full‑word data only

    // ---------------------------------------------------------------------------
    // APB BUS MULTIPLEXING
    // WRITE FSM gets priority when active; READ FSM uses bus when WRITE IDLE.
    // ---------------------------------------------------------------------------

    // APB shared outputs (registered)
    logic [ADDR_WIDTH-1:0] paddr_r;  logic [DATA_WIDTH-1:0] pwdata_r;
    logic                  pwrite_r, penable_r; logic [1:0] psel_r;
    assign paddr_o   = paddr_r;
    assign pwdata_o  = pwdata_r;
    assign pwrite_o  = pwrite_r;
    assign penable_o = penable_r;
    assign psel_o    = psel_r;

    // ---------------------------------------------------------------------------
    // Write‑side state machine ---------------------------------------------------
    // ---------------------------------------------------------------------------

    typedef enum logic [1:0] {W_IDLE, W_SETUP, W_ENABLE} wstate_e;
    wstate_e wst, wst_n;
    logic [4:0]       wlen_cnt, wlen_cnt_n;  // up to 16 beats (0‑based)
    logic [ADDR_WIDTH-1:0] waddr_cur, waddr_next;
    logic               wincr, wincr_n;

    // Sequential
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wst        <= W_IDLE;
            wlen_cnt   <= 0;
            waddr_cur  <= 0;
            wincr      <= 1'b0;
        end else begin
            wst        <= wst_n;
            wlen_cnt   <= wlen_cnt_n;
            waddr_cur  <= waddr_next;
            wincr      <= wincr_n;
        end
    end

    // Combinational
    always_comb begin
        // default APB bus outputs (inactive)
        paddr_r   = '0;
        pwdata_r  = '0;
        pwrite_r  = 1'b0;
        penable_r = 1'b0;
        psel_r    = 2'b00;

        // FIFO controls default
        wcmd_rden = 1'b0;
        wdat_rden = 1'b0;
        bq_wren   = 1'b0;
        bq_wdata  = 2'b00;

        // next‑state defaults
        wst_n       = wst;
        wlen_cnt_n  = wlen_cnt;
        waddr_next  = waddr_cur;
        wincr_n     = wincr;

        unique case (wst)
            // -----------------------------------------------------
            W_IDLE: begin
                if (!wcmd_empty) begin            // new write command
                    wcmd_rden   = 1'b1;           // pop immediately
                    wst_n       = W_SETUP;
                    wlen_cnt_n  = wcmd_rdata[5:2];
                    waddr_next  = wcmd_rdata[37:6];
                    wincr_n     = wcmd_rdata[1];
                end
            end

            // -----------------------------------------------------
            W_SETUP: begin
                if (!wdat_empty) begin            // ensure data ready
                    // Drive APB SETUP signals (penable=0)
                    pwrite_r  = 1'b1;
                    paddr_r   = waddr_cur;
                    pwdata_r  = wdat_rdata;
                    psel_r    = decode_psel(waddr_cur);

                    // Prepare to move to ENABLE next cycle
                    wst_n     = W_ENABLE;
                end
            end

            // -----------------------------------------------------
            W_ENABLE: begin
                // Keep same address/data, assert PENABLE
                pwrite_r  = 1'b1;
                paddr_r   = waddr_cur;
                pwdata_r  = wdat_rdata;
                psel_r    = decode_psel(waddr_cur);
                penable_r = 1'b1;

                if (pready_i) begin
                    // Data accepted → pop WDATA beat
                    wdat_rden   = 1'b1;

                    if (wlen_cnt == 0) begin
                        // entire burst done → queue BRESP, return to IDLE
                        bq_wren   = 1'b1;
                        bq_wdata  = 2'b00;      // OKAY
                        wst_n     = W_IDLE;
                    end else begin
                        // next beat
                        wlen_cnt_n = wlen_cnt - 1;
                        waddr_next = wincr ? (waddr_cur + 4) : waddr_cur;
                        wst_n      = W_SETUP;
                    end
                end
            end
        endcase
    end

    // ---------------------------------------------------------------------------
    // Read‑side state machine ----------------------------------------------------
    // ---------------------------------------------------------------------------

    typedef enum logic [1:0] {R_IDLE, R_SETUP, R_ENABLE} rstate_e;
    rstate_e rstt, rstt_n;
    logic [4:0]       rlen_cnt, rlen_cnt_n;
    logic [ADDR_WIDTH-1:0] raddr_cur, raddr_next;
    logic               rincr, rincr_n;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rstt      <= R_IDLE;
            rlen_cnt  <= 0;
            raddr_cur <= 0;
            rincr     <= 1'b0;
        end else begin
            rstt      <= rstt_n;
            rlen_cnt  <= rlen_cnt_n;
            raddr_cur <= raddr_next;
            rincr     <= rincr_n;
        end
    end

    always_comb begin
        // Default FIFO signals
        rcmd_rden = 1'b0;
        rp_wren   = 1'b0;
        rp_wdata  = '0;

        // next‑state defaults
        rstt_n       = rstt;
        rlen_cnt_n   = rlen_cnt;
        raddr_next   = raddr_cur;
        rincr_n      = rincr;

        // APB bus defaults when READ owns bus (overwritten if WRITE active)
        if (wst == W_IDLE) begin
            case (rstt)
                // -------------------------------------------------
                R_IDLE: begin
                    if (!rcmd_empty) begin
                        rcmd_rden   = 1'b1;
                        rstt_n      = R_SETUP;
                        rlen_cnt_n  = rcmd_rdata[5:2];
                        raddr_next  = rcmd_rdata[37:6];
                        rincr_n     = rcmd_rdata[1];
                    end
                end

                // -------------------------------------------------
                R_SETUP: begin
                    // Drive APB SETUP for read
                    pwrite_r  = 1'b0;
                    paddr_r   = raddr_cur;
                    psel_r    = decode_psel(raddr_cur);
                    // penable=0 in SETUP
                    rstt_n    = R_ENABLE;
                end

                // -------------------------------------------------
                R_ENABLE: begin
                    pwrite_r  = 1'b0;
                    paddr_r   = raddr_cur;
                    psel_r    = decode_psel(raddr_cur);
                    penable_r = 1'b1;

                    if (pready_i) begin
                        // capture PRDATA and queue to R FIFO
                        rp_wren       = 1'b1;
                        rp_wdata[DATA_WIDTH-1:0] = prdata_i;
                        rp_wdata[DATA_WIDTH+1:DATA_WIDTH] = 2'b00; // rresp OKAY
                        rp_wdata[DATA_WIDTH+2]           = (rlen_cnt==0); // rlast

                        if (rlen_cnt == 0) begin
                            rstt_n = R_IDLE;
                        end else begin
                            rlen_cnt_n = rlen_cnt - 1;
                            raddr_next = rincr ? (raddr_cur + 4) : raddr_cur;
                            rstt_n = R_SETUP;
                        end
                    end
                end
            endcase
        end
    end

    // ---------------------------------------------------------------------------
    // AXI output channels (B/R)
    // ---------------------------------------------------------------------------
    assign bvalid_o = ~bq_empty;
    assign bresp_o  = bq_empty ? 2'b00 : bq_rdata;
    assign bid_o    = 1'b0;             // single‑ID system
    assign bq_rden  = bvalid_o & bready_i;

    assign rvalid_o = ~rp_empty;
    assign rdata_o  = rp_rdata[DATA_WIDTH-1:0];
    assign rresp_o  = rp_rdata[DATA_WIDTH+1:DATA_WIDTH];
    assign rlast_o  = rp_rdata[DATA_WIDTH+2];
    assign rid_o    = 1'b0;
    assign rp_rden  = rvalid_o & rready_i;

endmodule
