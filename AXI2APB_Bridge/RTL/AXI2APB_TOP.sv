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
    // Address decoder (two 4‑KB windows)
    // ---------------------------------------------------------------------------
    function automatic logic [1:0] decode_psel(input logic [ADDR_WIDTH-1:0] a);
        if (a[31:12] == 20'h0001F)      decode_psel = 2'b01; // 0x0001_F000
        else if (a[31:12] == 20'h0002F) decode_psel = 2'b10; // 0x0002_F000
        else                            decode_psel = 2'b00; // unreachable in spec
    endfunction

    // ---------------------------------------------------------------------------
    // Encoded command : 48 bits
    //   [0]  is_write  | [1] is_incr | [5:2] len_m1 | [37:6] addr
    // ---------------------------------------------------------------------------
    localparam CMD_W = 48;
    localparam LG2   = 4;   // FIFO depth = 16

    // ---------------------------------------------------------------------------
    // FIFO declarations (WRITE‑cmd, READ‑cmd, WDATA, BRESP, RDATA)
    // ---------------------------------------------------------------------------
    logic             wcmd_full, wcmd_empty, wcmd_wren, wcmd_rden;
    logic [CMD_W-1:0] wcmd_wdata, wcmd_rdata;
    logic             rcmd_full, rcmd_empty, rcmd_wren, rcmd_rden;
    logic [CMD_W-1:0] rcmd_wdata, rcmd_rdata;
    logic             wdat_full, wdat_empty, wdat_wren, wdat_rden;
    logic [DATA_WIDTH-1:0] wdat_rdata;
    logic             bq_full,  bq_empty,  bq_wren,  bq_rden;
    logic [1:0]       bq_wdata, bq_rdata;
    localparam RPKT_W = DATA_WIDTH + 3;
    logic             rp_full,  rp_empty,  rp_wren,  rp_rden;
    logic [RPKT_W-1:0] rp_wdata, rp_rdata;

    //  ⮕ WRITE‑cmd FIFO
    BRIDGE_FIFO #(.DEPTH_LG2(LG2), .DATA_WIDTH(CMD_W)) u_wcmd_fifo(
        .* ,   // positional elided for brevity
        .full_o(wcmd_full), .empty_o(wcmd_empty),
        .wren_i(wcmd_wren), .wdata_i(wcmd_wdata),
        .rden_i(wcmd_rden), .rdata_o(wcmd_rdata));
    //  ⮕ READ‑cmd FIFO
    BRIDGE_FIFO #(.DEPTH_LG2(LG2), .DATA_WIDTH(CMD_W)) u_rcmd_fifo(
        .* ,
        .full_o(rcmd_full), .empty_o(rcmd_empty),
        .wren_i(rcmd_wren), .wdata_i(rcmd_wdata),
        .rden_i(rcmd_rden), .rdata_o(rcmd_rdata));
    //  ⮕ WDATA FIFO
    BRIDGE_FIFO #(.DEPTH_LG2(LG2), .DATA_WIDTH(DATA_WIDTH)) u_wdat_fifo(
        .* ,
        .full_o(wdat_full), .empty_o(wdat_empty),
        .wren_i(wdat_wren), .wdata_i(wdata_i),    // push path
        .rden_i(wdat_rden), .rdata_o(wdat_rdata));
    //  ⮕ BRESP FIFO
    BRIDGE_FIFO #(.DEPTH_LG2(LG2), .DATA_WIDTH(2)) u_bq(
        .* , .full_o(bq_full), .empty_o(bq_empty), .wren_i(bq_wren), .wdata_i(bq_wdata),
        .rden_i(bq_rden), .rdata_o(bq_rdata));
    //  ⮕ RDATA FIFO
    BRIDGE_FIFO #(.DEPTH_LG2(LG2), .DATA_WIDTH(RPKT_W)) u_rp(
        .* , .full_o(rp_full), .empty_o(rp_empty), .wren_i(rp_wren), .wdata_i(rp_wdata),
        .rden_i(rp_rden), .rdata_o(rp_rdata));

    // ---------------------------------------------------------------------------
    // AXI -> FIFO handshakes
    // ---------------------------------------------------------------------------
    assign awready_o = ~wcmd_full;
    assign wready_o  = ~wdat_full;
    assign arready_o = ~rcmd_full;

    // pack command payloads
    assign wcmd_wren          = awvalid_i & awready_o;
    assign wcmd_wdata         = {10'b0, awaddr_i, awlen_i, (awburst_i==2'b01), 1'b1};
    assign rcmd_wren          = arvalid_i & arready_o;
    assign rcmd_wdata         = {10'b0, araddr_i, arlen_i, (arburst_i==2'b01), 1'b0};
    assign wdat_wren          = wvalid_i & wready_o;

    // ---------------------------------------------------------------------------
    // WRITE‑side FSM : generates *w_* APB bus signals
    // ---------------------------------------------------------------------------

    typedef enum logic [1:0] {W_IDLE, W_SETUP, W_ENABLE} wstate_e;
    wstate_e wst, wst_n;
    logic [4:0]        wcnt, wcnt_n;  // beats left
    logic [ADDR_WIDTH-1:0] waddr, waddr_n;
    logic              wincr, wincr_n;

    // APB bus (WRITE‑generated)
    logic [ADDR_WIDTH-1:0] w_paddr;
    logic [DATA_WIDTH-1:0] w_pwdata;
    logic [1:0]            w_psel;
    logic                  w_pwrite, w_penable;

    // Sequential
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wst   <= W_IDLE; wcnt <= 0; waddr <= 0; wincr <= 1'b0;
        end else begin
            wst   <= wst_n;  wcnt <= wcnt_n; waddr <= waddr_n; wincr <= wincr_n;
        end
    end

    // Combinational defaults
    always_comb begin
        // default APB‑WRITE bus inactive
        w_paddr   = '0; w_pwdata = '0; w_psel = 2'b00;
        w_pwrite  = 1'b0; w_penable = 1'b0;

        // FIFO default controls
        wcmd_rden = 1'b0; wdat_rden = 1'b0;
        bq_wren   = 1'b0; bq_wdata = 2'b00;

        // next‑state defaults
        wst_n = wst; wcnt_n = wcnt; waddr_n = waddr; wincr_n = wincr;

        unique case (wst)
            W_IDLE: begin
                if (!wcmd_empty) begin
                    wcmd_rden = 1'b1;
                    wcnt_n    = wcmd_rdata[5:2];
                    waddr_n   = wcmd_rdata[37:6];
                    wincr_n   = wcmd_rdata[1];
                    wst_n     = W_SETUP;
                end
            end
            W_SETUP: begin
                if (!wdat_empty) begin
                    w_paddr   = waddr; w_pwdata = wdat_rdata;
                    w_psel    = decode_psel(waddr);
                    w_pwrite  = 1'b1;              // SETUP
                    wst_n     = W_ENABLE;          // advance
                end
            end
            W_ENABLE: begin
                w_paddr   = waddr; w_pwdata = wdat_rdata;
                w_psel    = decode_psel(waddr);
                w_pwrite  = 1'b1; w_penable = 1'b1;
                if (pready_i) begin
                    wdat_rden = 1'b1;              // consume data beat
                    if (wcnt == 0) begin           // burst done
                        bq_wren  = 1'b1; bq_wdata = 2'b00; // OKAY
                        wst_n    = W_IDLE;
                    end else begin
                        wcnt_n   = wcnt - 1;
                        waddr_n  = wincr ? (waddr + 4) : waddr;
                        wst_n    = W_SETUP;
                    end
                end
            end
        endcase
    end

    // ---------------------------------------------------------------------------
    // READ‑side FSM : generates *r_* APB bus signals
    // ---------------------------------------------------------------------------

    typedef enum logic [1:0] {R_IDLE, R_SETUP, R_ENABLE} rstate_e;
    rstate_e rstt, rstt_n;
    logic [4:0]        rcnt, rcnt_n;
    logic [ADDR_WIDTH-1:0] raddr, raddr_n;
    logic              rincr, rincr_n;

    // APB bus (READ‑generated)
    logic [ADDR_WIDTH-1:0] r_paddr;
    logic [1:0]            r_psel;
    logic                  r_penable;

    // Sequential
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rstt <= R_IDLE; rcnt <= 0; raddr <= 0; rincr <= 1'b0;
        end else begin
            rstt <= rstt_n; rcnt <= rcnt_n; raddr <= raddr_n; rincr <= rincr_n;
        end
    end

    // Combinational defaults
    always_comb begin
        r_paddr   = '0; r_psel = 2'b00; r_penable = 1'b0;
        rcmd_rden = 1'b0; rp_wren = 1'b0; rp_wdata = '0;

        rstt_n = rstt; rcnt_n = rcnt; raddr_n = raddr; rincr_n = rincr;

        // NOTE: READ FSM must not steal bus once WRITE is active; arbitration
        // is handled in the bus multiplexer later.

        case (rstt)
            R_IDLE: begin
                if (!rcmd_empty) begin
                    rcmd_rden = 1'b1;
                    rcnt_n    = rcmd_rdata[5:2];
                    raddr_n   = rcmd_rdata[37:6];
                    rincr_n   = rcmd_rdata[1];
                    rstt_n    = R_SETUP;
                end
            end
            R_SETUP: begin
                r_paddr = raddr; r_psel = decode_psel(raddr);
                rstt_n  = R_ENABLE;         // move to ENABLE next cycle
            end
            R_ENABLE: begin
                r_paddr   = raddr; r_psel = decode_psel(raddr);
                r_penable = 1'b1;
                if (pready_i) begin
                    rp_wren  = 1'b1;
                    rp_wdata[DATA_WIDTH-1:0] = prdata_i;
                    rp_wdata[DATA_WIDTH+1:DATA_WIDTH] = 2'b00;   // OKAY
                    rp_wdata[DATA_WIDTH+2] = (rcnt == 0);
                    if (rcnt == 0) begin
                        rstt_n = R_IDLE;
                    end else begin
                        rcnt_n  = rcnt - 1;
                        raddr_n = rincr ? (raddr + 4) : raddr;
                        rstt_n  = R_SETUP;
                    end
                end
            end
        endcase
    end

    // ---------------------------------------------------------------------------
    // APB bus multiplexer (SINGLE procedural driver for *_o signals)
    // ---------------------------------------------------------------------------
    logic [ADDR_WIDTH-1:0] paddr_r;
    logic [DATA_WIDTH-1:0] pwdata_r;
    logic [1:0]            psel_r;
    logic                  pwrite_r, penable_r;

    always_comb begin
        // default all zeros
        paddr_r   = '0; pwdata_r = '0; psel_r = 2'b00; pwrite_r = 1'b0; penable_r = 1'b0;

        if (wst != W_IDLE) begin                        // WRITE owns bus
            paddr_r   = w_paddr; pwdata_r = w_pwdata;
            psel_r    = w_psel;  pwrite_r = w_pwrite; penable_r = w_penable;
        end else if (rstt != R_IDLE) begin              // READ owns bus
            paddr_r   = r_paddr; psel_r = r_psel;
            pwrite_r  = 1'b0;   penable_r = r_penable;
            // pwdata not used on reads
        end
    end

    assign paddr_o   = paddr_r;
    assign pwdata_o  = pwdata_r;
    assign psel_o    = psel_r;
    assign pwrite_o  = pwrite_r;
    assign penable_o = penable_r;

    // ---------------------------------------------------------------------------
    // AXI B/R channel outputs
    // ---------------------------------------------------------------------------
    assign bid_o    = 1'b0;                   // single‑ID
    assign bvalid_o = ~bq_empty;
    assign bresp_o  = bq_rdata;
    assign bq_rden  = bvalid_o & bready_i;

    assign rid_o    = 1'b0;
    assign rvalid_o = ~rp_empty;
    assign rdata_o  = rp_rdata[DATA_WIDTH-1:0];
    assign rresp_o  = rp_rdata[DATA_WIDTH+1:DATA_WIDTH];
    assign rlast_o  = rp_rdata[DATA_WIDTH+2];
    assign rp_rden  = rvalid_o & rready_i;

endmodule
