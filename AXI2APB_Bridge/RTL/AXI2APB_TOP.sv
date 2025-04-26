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
    
    // =====================================================================
    // 1. Command FIFO (AW/AR) : {is_write, burst[1:0], len[3:0], addr[31:0]} = 1+2+4+32 = 39‑bit
    // =====================================================================
    localparam CMD_WIDTH = 1 + 2 + 4 + ADDR_WIDTH; // 39 bits

    wire              cmdfifo_full, cmdfifo_empty;
    wire              cmdfifo_wren, cmdfifo_rden;
    wire [CMD_WIDTH-1:0] cmdfifo_wdata, cmdfifo_rdata;

    BRIDGE_FIFO #(
        .DEPTH_LG2 (CMD_DEPTH_LG2),     // 4‑deep
        .DATA_WIDTH(CMD_WIDTH)
    ) u_cmd_fifo (
        .clk       (clk),
        .rst_n     (rst_n),
        .full_o    (cmdfifo_full),
        .wren_i    (cmdfifo_wren),
        .wdata_i   (cmdfifo_wdata),
        .empty_o   (cmdfifo_empty),
        .rden_i    (cmdfifo_rden),
        .rdata_o   (cmdfifo_rdata)
    );

    // Write address enqueue
    assign cmdfifo_wren  = (awvalid_i & awready_o) | (arvalid_i & arready_o);
    assign cmdfifo_wdata = (awvalid_i) ?
                           {1'b1, awburst_i, awlen_i, awaddr_i} :
                           {1'b0, arburst_i, arlen_i, araddr_i};

    assign awready_o = ~cmdfifo_full & ~awvalid_i & ~arvalid_i; // mutual exclusive push
    assign arready_o = ~cmdfifo_full & ~arvalid_i & ~awvalid_i;

    // FIFO read occurs when state machine moves from IDLE to SETUP

    // =====================================================================
    // 2. Write‑data FIFO (stream) : pure 32‑bit data queue
    // =====================================================================
    wire              wfifo_full, wfifo_empty;
    wire              wfifo_wren, wfifo_rden;

    BRIDGE_FIFO #(
        .DEPTH_LG2 (WFIFO_DEPTH_LG2),   // 16‑deep
        .DATA_WIDTH(DATA_WIDTH)
    ) u_wdata_fifo (
        .clk       (clk),
        .rst_n     (rst_n),
        .full_o    (wfifo_full),
        .wren_i    (wfifo_wren),
        .wdata_i   (wdata_i),
        .empty_o   (wfifo_empty),
        .rden_i    (wfifo_rden),
        .rdata_o   (pwdata_o)          // FIFO output drives APB write data directly
    );

    assign wfifo_wren = wvalid_i & ~wfifo_full;
    assign wready_o   = ~wfifo_full;

    // =====================================================================
    // 3. State machine / datapath
    // =====================================================================

    typedef enum logic [2:0] {
        S_IDLE,
        S_SETUP,
        S_ENABLE,
        S_W_RESP,
        S_R_RESP
    } st_t;

    st_t state, next;

    // Latched fields from command FIFO
    logic            is_write;
    logic [1:0]      burst_type;
    logic [3:0]      burst_len;
    logic [ADDR_WIDTH-1:0] cmd_addr;

    logic [3:0] beat_cnt;

    // Decode function
    function automatic [1:0] decode_sel(input [ADDR_WIDTH-1:0] a);
        decode_sel = a[16] ? 2'b10 : 2'b01;
    endfunction

    // ----- Next‑state logic -----
    always_comb begin
        next = state;
        case (state)
            S_IDLE   : if (!cmdfifo_empty)       next = S_SETUP;
            S_SETUP  :                           next = S_ENABLE;
            S_ENABLE : if (pready_i) begin
                            if (is_write) begin
                                if (beat_cnt == burst_len) next = S_W_RESP;
                                else                        next = S_SETUP;
                            end else begin
                                next = S_R_RESP; // read has single beat response path
                            end
                        end
            S_W_RESP : if (bready_i)             next = S_IDLE;
            S_R_RESP : if (rready_i & rvalid_o & rlast_o) next = S_IDLE;
        endcase
    end

    // ----- State register -----
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= next;
    end

    // ----- FIFO read handshake -----
    assign cmdfifo_rden = (state==S_IDLE) & (next==S_SETUP);

    // ----- Capture command -----
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            is_write   <= 0; burst_type <= 0; burst_len <= 0; cmd_addr <= 0; beat_cnt <= 0;
        end else if (cmdfifo_rden) begin
            {is_write, burst_type, burst_len, cmd_addr} <= cmdfifo_rdata;
            beat_cnt <= 0;
        end else if (state==S_ENABLE && pready_i) begin
            beat_cnt <= beat_cnt + 1;
            if (burst_type==2'b01)   // INCR
                cmd_addr <= cmd_addr + 4;
        end
    end

    // ----- APB control signals -----
    logic penable_r, pwrite_r;
    logic [ADDR_WIDTH-1:0] paddr_r;
    logic [1:0] psel_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            penable_r <= 0; pwrite_r <= 0; paddr_r <= 0; psel_r <= 0;
        end else begin
            case (state)
                S_SETUP: begin
                    penable_r <= 0;
                    pwrite_r  <= is_write;
                    paddr_r   <= cmd_addr;
                    psel_r    <= decode_sel(cmd_addr);
                end
                S_ENABLE: begin
                    penable_r <= 1;
                end
                default: penable_r <= 0;
            endcase
        end
    end

    assign paddr_o   = paddr_r;
    assign pwrite_o  = pwrite_r;
    assign penable_o = penable_r;
    assign psel_o    = psel_r;
    // pwdata_o already driven by FIFO output

    // ----- AXI Write response -----
    logic bvalid_r;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) bvalid_r <= 0;
        else if (state==S_ENABLE && pready_i && is_write && beat_cnt==burst_len) bvalid_r <= 1;
        else if (bready_i) bvalid_r <= 0;
    end

    assign bvalid_o = bvalid_r;
    assign bresp_o  = pslverr_i ? 2'b10 : 2'b00;
    assign bid_o    = 1'b0;

    // ----- AXI Read data/resp -----
    logic rvalid_r, rlast_r;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rvalid_r <= 0; rlast_r <= 0;
        end else if (state==S_ENABLE && pready_i && !is_write) begin
            rvalid_r <= 1;
            rlast_r  <= 1; // read: APB always single data ACK per enable
        end else if (rready_i & rvalid_r) begin
            rvalid_r <= 0; rlast_r <= 0;
        end
    end

    assign rvalid_o = rvalid_r;
    assign rdata_o  = prdata_i;
    assign rresp_o  = pslverr_i ? 2'b10 : 2'b00;
    assign rlast_o  = rlast_r;
    assign rid_o    = 1'b0;

endmodule