module AXI2APB_TOP #(
    parameter ADDR_WIDTH            = 32,
    parameter DATA_WIDTH            = 32
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // AXI Write Address Channel
    input  wire                     awid_i,
    input  wire [ADDR_WIDTH-1:0]    awaddr_i,
    input  wire [3:0]               awlen_i,
    input  wire [2:0]               awsize_i,
    input  wire [1:0]               awburst_i,
    input  wire                     awvalid_i,
    output wire                     awready_o,

    // AXI Write Data Channel
    input  wire                     wid_i,
    input  wire [DATA_WIDTH-1:0]    wdata_i,
    input  wire [3:0]               wstrb_i,
    input  wire                     wlast_i,
    input  wire                     wvalid_i,
    output wire                     wready_o,

    // AXI Write Response Channel
    output wire                     bid_o,
    output wire [1:0]               bresp_o,
    output wire                     bvalid_o,
    input  wire                     bready_i,

    // AXI Read Address Channel
    input  wire                     arid_i,
    input  wire [ADDR_WIDTH-1:0]    araddr_i,
    input  wire [3:0]               arlen_i,
    input  wire [2:0]               arsize_i,
    input  wire [1:0]               arburst_i,
    input  wire                     arvalid_i,
    output wire                     arready_o,

    // AXI Read Data Channel
    output wire                     rid_o,
    output wire [DATA_WIDTH-1:0]    rdata_o,
    output wire [1:0]               rresp_o,
    output wire                     rlast_o,
    output wire                     rvalid_o,
    input  wire                     rready_i,

    // APB Master Interface
    output wire [ADDR_WIDTH-1:0]    paddr_o,
    output wire [DATA_WIDTH-1:0]    pwdata_o,
    output wire                     pwrite_o,
    output wire                     penable_o,
    output wire [1:0]               psel_o,
    input  wire [DATA_WIDTH-1:0]    prdata_i,
    input  wire                     pready_i,
    input  wire                     pslverr_i
);

    // 내부 연결 신호
    wire                        wr_trans;
    wire                        rd_trans;
    wire [ADDR_WIDTH-1:0]       trans_addr;
    wire [DATA_WIDTH-1:0]       trans_data;
    wire [DATA_WIDTH-1:0]       read_data;
    wire                        trans_done;
    wire                        trans_error;
    wire [3:0]                  burst_len;
    wire                        fifo_wren;
    wire                        fifo_rden;
    wire                        fifo_full;
    wire                        fifo_empty;
    wire [DATA_WIDTH-1:0]       fifo_rdata;
    
    // 버스트 관리 신호
    wire                        burst_start;
    wire                        burst_done;
    wire [ADDR_WIDTH-1:0]       burst_addr;
    
    // FIFO 리셋을 위한 엣지 감지
    reg                         wr_trans_d;
    wire                        fifo_reset;
    
    // wr_trans 신호의 상승 엣지 감지
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wr_trans_d <= 1'b0;
        else
            wr_trans_d <= wr_trans;
    end
    
    // 새 쓰기 트랜잭션이 시작될 때 FIFO 리셋
    assign fifo_reset = wr_trans && !wr_trans_d;
    
    // 주소 검증 로직: 유효하지 않은 주소가 들어오면 기본 주소로 변환
    function [ADDR_WIDTH-1:0] validate_address;
        input [ADDR_WIDTH-1:0] addr;
        begin
            if ((addr >= 32'h0001_F000 && addr <= 32'h0001_FFFF) ||
                (addr >= 32'h0002_F000 && addr <= 32'h0002_FFFF))
                validate_address = addr;
            else
                validate_address = 32'h0001_F000; // 기본적으로 SLV1 영역 시작 주소 사용
        end
    endfunction
    
    // 주소 검증 적용
    wire [ADDR_WIDTH-1:0] validated_burst_addr;
    assign validated_burst_addr = validate_address(burst_addr);
    
    // AXI 프로토콜 처리기 인스턴스
    AXI_PROTOCOL_HANDLER #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) axi_handler (
        .clk(clk),
        .rst_n(rst_n),
        
        // AXI 인터페이스
        .awid_i(awid_i),
        .awaddr_i(awaddr_i),
        .awlen_i(awlen_i),
        .awsize_i(awsize_i),
        .awburst_i(awburst_i),
        .awvalid_i(awvalid_i),
        .awready_o(awready_o),
        
        .wid_i(wid_i),
        .wdata_i(wdata_i),
        .wstrb_i(wstrb_i),
        .wlast_i(wlast_i),
        .wvalid_i(wvalid_i),
        .wready_o(wready_o),
        
        .bid_o(bid_o),
        .bresp_o(bresp_o),
        .bvalid_o(bvalid_o),
        .bready_i(bready_i),
        
        .arid_i(arid_i),
        .araddr_i(araddr_i),
        .arlen_i(arlen_i),
        .arsize_i(arsize_i),
        .arburst_i(arburst_i),
        .arvalid_i(arvalid_i),
        .arready_o(arready_o),
        
        .rid_o(rid_o),
        .rdata_o(rdata_o),
        .rresp_o(rresp_o),
        .rlast_o(rlast_o),
        .rvalid_o(rvalid_o),
        .rready_i(rready_i),
        
        // 내부 인터페이스
        .wr_trans_o(wr_trans),
        .rd_trans_o(rd_trans),
        .trans_addr_o(trans_addr),
        .trans_data_o(trans_data),
        .read_data_i(read_data),
        .trans_done_i(trans_done),
        .trans_error_i(trans_error),
        .burst_len_o(burst_len),
        .fifo_wren_o(fifo_wren),
        .fifo_full(fifo_full)
    );
    
    // APB 프로토콜 처리기 인스턴스
    APB_PROTOCOL_HANDLER #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) apb_handler (
        .clk(clk),
        .rst_n(rst_n),
        
        // APB 인터페이스
        .paddr_o(paddr_o),
        .pwdata_o(pwdata_o),
        .pwrite_o(pwrite_o),
        .penable_o(penable_o),
        .psel_o(psel_o),
        .prdata_i(prdata_i),
        .pready_i(pready_i),
        .pslverr_i(pslverr_i),
        
        // 내부 인터페이스
        .wr_trans_i(wr_trans),
        .rd_trans_i(rd_trans),
        .trans_addr_i(validated_burst_addr),  // 검증된 주소 사용
        .trans_data_i(fifo_rdata),  // FIFO로부터 데이터 수신
        .read_data_o(read_data),
        .trans_done_o(trans_done),
        .trans_error_o(trans_error),
        .burst_len_i(burst_len),
        .fifo_rden_o(fifo_rden)
    );
    
    // 버스트 관리자 인스턴스
    BURST_MANAGER #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) burst_mgr (
        .clk(clk),
        .rst_n(rst_n),
        
        // 버스트 정보
        .burst_len_i(burst_len),
        .burst_size_i(wr_trans ? awsize_i : arsize_i),
        .burst_type_i(wr_trans ? awburst_i : arburst_i),
        .start_addr_i(trans_addr),
        
        // 제어 및 상태
        .burst_start_i(burst_start),
        .transfer_done_i(trans_done),
        .burst_done_o(burst_done),
        .curr_addr_o(burst_addr)
    );
    
    // FIFO 인스턴스
    BRIDGE_FIFO #(
        .DEPTH_LG2(4),           // 16개의 항목
        .DATA_WIDTH(DATA_WIDTH)
    ) fifo_inst (
        .clk(clk),
        .rst_n(rst_n && !fifo_reset),  // 새 버스트 시작 시 FIFO 리셋
        .full_o(fifo_full),
        .wren_i(fifo_wren),
        .wdata_i(wdata_i),
        .empty_o(fifo_empty),
        .rden_i(fifo_rden),
        .rdata_o(fifo_rdata)
    );
    
    // 버스트 시작 신호 생성
    assign burst_start = wr_trans || rd_trans;

endmodule 