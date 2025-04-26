module AXI_PROTOCOL_HANDLER #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input  wire                     clk,
    input  wire                     rst_n,
    
    // AXI Write Address Channel
    input  wire                     awid_i,
    input  wire [ADDR_WIDTH-1:0]    awaddr_i,
    input  wire [3:0]               awlen_i,
    input  wire [2:0]               awsize_i,
    input  wire [1:0]               awburst_i,
    input  wire                     awvalid_i,
    output reg                      awready_o,
    
    // AXI Write Data Channel
    input  wire                     wid_i,
    input  wire [DATA_WIDTH-1:0]    wdata_i,
    input  wire [3:0]               wstrb_i,
    input  wire                     wlast_i,
    input  wire                     wvalid_i,
    output reg                      wready_o,
    
    // AXI Write Response Channel
    output reg                      bid_o,
    output reg [1:0]                bresp_o,
    output reg                      bvalid_o,
    input  wire                     bready_i,
    
    // AXI Read Address Channel
    input  wire                     arid_i,
    input  wire [ADDR_WIDTH-1:0]    araddr_i,
    input  wire [3:0]               arlen_i,
    input  wire [2:0]               arsize_i,
    input  wire [1:0]               arburst_i,
    input  wire                     arvalid_i,
    output reg                      arready_o,
    
    // AXI Read Data Channel
    output reg                      rid_o,
    output reg [DATA_WIDTH-1:0]     rdata_o,
    output reg [1:0]                rresp_o,
    output reg                      rlast_o,
    output reg                      rvalid_o,
    input  wire                     rready_i,
    
    // 내부 인터페이스 - APB와 통신
    output reg                      wr_trans_o,      // 쓰기 트랜잭션 표시
    output reg                      rd_trans_o,      // 읽기 트랜잭션 표시
    output reg [ADDR_WIDTH-1:0]     trans_addr_o,    // 트랜잭션 주소
    output reg [DATA_WIDTH-1:0]     trans_data_o,    // 트랜잭션 데이터 (쓰기)
    input  wire [DATA_WIDTH-1:0]    read_data_i,     // 읽기 데이터
    input  wire                     trans_done_i,    // 트랜잭션 완료 신호
    input  wire                     trans_error_i,   // 트랜잭션 에러 신호
    output reg [3:0]                burst_len_o,     // 버스트 길이
    output reg                      fifo_wren_o      // FIFO 쓰기 활성화
);

    // AXI FSM 상태 정의
    typedef enum logic [2:0] {
        AXI_IDLE = 3'b000,
        READ_ADDR = 3'b001,
        READ_DATA = 3'b010,
        WRITE_ADDR = 3'b011,
        WRITE_DATA = 3'b100,
        WRITE_RESP = 3'b101
    } axi_state_t;
    
    // 내부 레지스터
    reg [2:0]                   axi_state, axi_next_state;
    reg                         arid_reg;
    reg                         awid_reg;
    reg [ADDR_WIDTH-1:0]        addr_reg;
    reg [3:0]                   burst_len;
    reg [3:0]                   burst_cnt;
    reg                         trans_active;
    
    // AXI 상태 머신
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            axi_state <= AXI_IDLE;
        else
            axi_state <= axi_next_state;
    end
    
    // AXI 상태 전이 로직
    always_comb begin
        axi_next_state = axi_state;
        awready_o = 1'b0;
        wready_o = 1'b0;
        arready_o = 1'b0;
        fifo_wren_o = 1'b0;
        
        case (axi_state)
            AXI_IDLE: begin
                // 읽기 요청 우선 처리
                if (arvalid_i) begin
                    axi_next_state = READ_ADDR;
                end
                // 쓰기 요청 처리
                else if (awvalid_i) begin
                    axi_next_state = WRITE_ADDR;
                end
            end
            
            READ_ADDR: begin
                arready_o = 1'b1;
                if (arvalid_i && arready_o) begin
                    axi_next_state = READ_DATA;
                end
            end
            
            READ_DATA: begin
                if (rvalid_o && rready_i && rlast_o) begin
                    axi_next_state = AXI_IDLE;
                end
            end
            
            WRITE_ADDR: begin
                awready_o = 1'b1;
                if (awvalid_i && awready_o) begin
                    axi_next_state = WRITE_DATA;
                end
            end
            
            WRITE_DATA: begin
                wready_o = 1'b1;  // FIFO가 가득 차지 않았다고 가정
                if (wvalid_i && wready_o) begin
                    fifo_wren_o = 1'b1;
                    if (wlast_i) begin
                        axi_next_state = WRITE_RESP;
                    end
                end
            end
            
            WRITE_RESP: begin
                if (bvalid_o && bready_i) begin
                    axi_next_state = AXI_IDLE;
                end
            end
            
            default: axi_next_state = AXI_IDLE;
        endcase
    end
    
    // 레지스터 업데이트 로직
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 레지스터 초기화
            addr_reg <= {ADDR_WIDTH{1'b0}};
            burst_len <= 4'b0000;
            burst_cnt <= 4'b0000;
            arid_reg <= 1'b0;
            awid_reg <= 1'b0;
            
            bid_o <= 1'b0;
            bresp_o <= 2'b00;
            bvalid_o <= 1'b0;
            
            rid_o <= 1'b0;
            rdata_o <= {DATA_WIDTH{1'b0}};
            rresp_o <= 2'b00;
            rlast_o <= 1'b0;
            rvalid_o <= 1'b0;
            
            wr_trans_o <= 1'b0;
            rd_trans_o <= 1'b0;
            trans_addr_o <= {ADDR_WIDTH{1'b0}};
            trans_data_o <= {DATA_WIDTH{1'b0}};
            burst_len_o <= 4'b0000;
            trans_active <= 1'b0;
        end else begin
            // 기본값으로 트랜잭션 신호 리셋
            if (!trans_active) begin
                wr_trans_o <= 1'b0;
                rd_trans_o <= 1'b0;
            end
            
            case (axi_state)
                AXI_IDLE: begin
                    // 응답 신호 초기화
                    bvalid_o <= 1'b0;
                    rvalid_o <= 1'b0;
                    rlast_o <= 1'b0;
                    trans_active <= 1'b0;
                end
                
                READ_ADDR: begin
                    if (arvalid_i && arready_o) begin
                        // 유효한 주소 범위인지 확인
                        if ((araddr_i >= 32'h0001_F000 && araddr_i <= 32'h0001_FFFF) ||
                            (araddr_i >= 32'h0002_F000 && araddr_i <= 32'h0002_FFFF)) begin
                            addr_reg <= araddr_i;
                            trans_addr_o <= araddr_i;
                        end else begin
                            // 유효하지 않은 주소인 경우 기본 주소 사용
                            addr_reg <= 32'h0001_F000;
                            trans_addr_o <= 32'h0001_F000;
                        end
                        
                        burst_len <= arlen_i;
                        burst_len_o <= arlen_i;
                        arid_reg <= arid_i;
                        burst_cnt <= 4'b0000;
                        
                        // 읽기 트랜잭션 시작
                        rd_trans_o <= 1'b1;
                        trans_active <= 1'b1;
                    end
                end
                
                READ_DATA: begin
                    if (trans_done_i) begin
                        rvalid_o <= 1'b1;
                        rid_o <= arid_reg;
                        rdata_o <= read_data_i;
                        rresp_o <= trans_error_i ? 2'b10 : 2'b00;  // SLVERR or OKAY
                        
                        if (burst_cnt == burst_len) begin
                            rlast_o <= 1'b1;
                            trans_active <= 1'b0;
                        end else begin
                            rlast_o <= 1'b0;
                            addr_reg <= addr_reg + 4;  // 다음 주소로 이동
                            trans_addr_o <= addr_reg + 4;
                            burst_cnt <= burst_cnt + 1;
                        end
                    end
                    
                    if (rvalid_o && rready_i) begin
                        rvalid_o <= 1'b0;
                    end
                end
                
                WRITE_ADDR: begin
                    if (awvalid_i && awready_o) begin
                        // 유효한 주소 범위인지 확인
                        if ((awaddr_i >= 32'h0001_F000 && awaddr_i <= 32'h0001_FFFF) ||
                            (awaddr_i >= 32'h0002_F000 && awaddr_i <= 32'h0002_FFFF)) begin
                            addr_reg <= awaddr_i;
                            trans_addr_o <= awaddr_i;
                        end else begin
                            // 유효하지 않은 주소인 경우 기본 주소 사용
                            addr_reg <= 32'h0001_F000;
                            trans_addr_o <= 32'h0001_F000;
                        end
                        
                        burst_len <= awlen_i;
                        burst_len_o <= awlen_i;
                        awid_reg <= awid_i;
                        burst_cnt <= 4'b0000;
                        
                        // 쓰기 트랜잭션 시작
                        wr_trans_o <= 1'b1;
                        trans_active <= 1'b1;
                    end
                end
                
                WRITE_DATA: begin
                    if (wvalid_i && wready_o) begin
                        trans_data_o <= wdata_i;
                        
                        if (wlast_i) begin
                            trans_active <= 1'b0;
                        end
                    end
                end
                
                WRITE_RESP: begin
                    if (trans_done_i) begin
                        bvalid_o <= 1'b1;
                        bid_o <= awid_reg;
                        bresp_o <= trans_error_i ? 2'b10 : 2'b00;  // SLVERR or OKAY
                    end
                    
                    if (bvalid_o && bready_i) begin
                        bvalid_o <= 1'b0;
                    end
                end
            endcase
        end
    end

endmodule 