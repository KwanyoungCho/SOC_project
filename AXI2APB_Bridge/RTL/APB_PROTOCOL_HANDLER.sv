module APB_PROTOCOL_HANDLER #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input  wire                     clk,
    input  wire                     rst_n,
    
    // APB 마스터 인터페이스
    output reg [ADDR_WIDTH-1:0]     paddr_o,
    output reg [DATA_WIDTH-1:0]     pwdata_o,
    output reg                      pwrite_o,
    output reg                      penable_o,
    output reg [1:0]                psel_o,
    input  wire [DATA_WIDTH-1:0]    prdata_i,
    input  wire                     pready_i,
    input  wire                     pslverr_i,
    
    // 내부 인터페이스 - AXI와 통신
    input  wire                     wr_trans_i,      // 쓰기 트랜잭션 표시
    input  wire                     rd_trans_i,      // 읽기 트랜잭션 표시
    input  wire [ADDR_WIDTH-1:0]    trans_addr_i,    // 트랜잭션 주소
    input  wire [DATA_WIDTH-1:0]    trans_data_i,    // 트랜잭션 데이터 (쓰기)
    output reg [DATA_WIDTH-1:0]     read_data_o,     // 읽기 데이터
    output reg                      trans_done_o,    // 트랜잭션 완료 신호
    output reg                      trans_error_o,   // 트랜잭션 에러 신호
    input  wire [3:0]               burst_len_i,     // 버스트 길이
    output reg                      fifo_rden_o      // FIFO 읽기 활성화
);

    // APB 상태 정의
    typedef enum logic [2:0] {
        IDLE = 3'b000,
        FIFO_READ = 3'b001,
        SETUP = 3'b010,
        ACCESS = 3'b011,
        DONE = 3'b100
    } apb_state_t;
    
    // 내부 레지스터
    reg [2:0]                   apb_state, apb_next_state;
    reg [3:0]                   burst_cnt;
    reg                         wr_trans_reg;
    reg                         rd_trans_reg;
    reg [ADDR_WIDTH-1:0]        addr_reg;
    reg [DATA_WIDTH-1:0]        wdata_reg;
    
    // APB 슬레이브 선택 로직
    // SLV1 영역: 0x0001_F000 - 0x0001_FFFF
    // SLV2 영역: 0x0002_F000 - 0x0002_FFFF
    function [1:0] slave_select;
        input [ADDR_WIDTH-1:0] addr;
        begin
            if ((addr >= 32'h0001_F000) && (addr <= 32'h0001_FFFF))
                slave_select = 2'b01;  // Slave 1
            else if ((addr >= 32'h0002_F000) && (addr <= 32'h0002_FFFF))
                slave_select = 2'b10;  // Slave 2
            else
                slave_select = 2'b01;  // 유효하지 않은 주소는 기본적으로 Slave 1 선택
        end
    endfunction
    
    // APB 상태 머신
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            apb_state <= IDLE;
        else
            apb_state <= apb_next_state;
    end
    
    // APB 상태 전이 로직
    always_comb begin
        apb_next_state = apb_state;
        fifo_rden_o = 1'b0;  // 기본적으로 FIFO 읽기 비활성화
        
        case (apb_state)
            IDLE: begin
                if (wr_trans_reg || rd_trans_reg) begin
                    if (wr_trans_reg) 
                        apb_next_state = FIFO_READ;  // 쓰기 전에 FIFO에서 데이터 읽기
                    else
                        apb_next_state = SETUP;  // 읽기는 바로 SETUP으로
                end
            end
            
            FIFO_READ: begin
                // FIFO 읽기를 적극적으로 활성화
                fifo_rden_o = 1'b1;
                
                // 다음 상태로 전환
                apb_next_state = SETUP;
            end
            
            SETUP: begin
                apb_next_state = ACCESS;
            end
            
            ACCESS: begin
                if (pready_i) begin
                    if (burst_cnt < burst_len_i) begin
                        // 버스트 진행 중 - 다음 단계로 이동
                        if (wr_trans_reg) begin
                            // 쓰기인 경우 바로 다음 데이터를 읽음
                            apb_next_state = FIFO_READ;
                        end else begin
                            apb_next_state = SETUP;
                        end
                    end else begin
                        // 트랜잭션 완료
                        apb_next_state = DONE;
                    end
                end
            end
            
            DONE: begin
                apb_next_state = IDLE;
            end
            
            default: apb_next_state = IDLE;
        endcase
    end
    
    // 레지스터 업데이트 로직
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 레지스터 초기화
            burst_cnt <= 4'b0000;
            wr_trans_reg <= 1'b0;
            rd_trans_reg <= 1'b0;
            addr_reg <= {ADDR_WIDTH{1'b0}};
            wdata_reg <= {DATA_WIDTH{1'b0}};
            
            paddr_o <= {ADDR_WIDTH{1'b0}};
            pwdata_o <= {DATA_WIDTH{1'b0}};
            pwrite_o <= 1'b0;
            penable_o <= 1'b0;
            psel_o <= 2'b00;
            
            read_data_o <= {DATA_WIDTH{1'b0}};
            trans_done_o <= 1'b0;
            trans_error_o <= 1'b0;
        end else begin
            // 트랜잭션 요청 저장
            if (wr_trans_i) begin
                wr_trans_reg <= 1'b1;
                rd_trans_reg <= 1'b0;
                addr_reg <= trans_addr_i;
                burst_cnt <= 4'b0000;
            end else if (rd_trans_i) begin
                rd_trans_reg <= 1'b1;
                wr_trans_reg <= 1'b0;
                addr_reg <= trans_addr_i;
                burst_cnt <= 4'b0000;
            end
            
            // 트랜잭션 완료 신호 기본적으로 비활성화
            trans_done_o <= 1'b0;
            
            // 상태에 따른 APB 신호 업데이트
            case (apb_state)
                IDLE: begin
                    penable_o <= 1'b0;
                    psel_o <= 2'b00;
                    trans_error_o <= 1'b0;
                end
                
                FIFO_READ: begin
                    // FIFO 데이터를 읽어서 wdata_reg에 저장
                    wdata_reg <= trans_data_i;
                end
                
                SETUP: begin
                    // APB 셋업 단계
                    // 유효한 주소 범위 검증
                    if ((addr_reg >= 32'h0001_F000 && addr_reg <= 32'h0001_FFFF) ||
                        (addr_reg >= 32'h0002_F000 && addr_reg <= 32'h0002_FFFF)) begin
                        paddr_o <= addr_reg;
                    end else begin
                        paddr_o <= 32'h0001_F000; // 기본 주소
                    end
                    
                    pwrite_o <= wr_trans_reg;
                    psel_o <= slave_select(addr_reg);
                    
                    if (wr_trans_reg) begin
                        // 쓰기 데이터 설정 - FIFO에서 읽은 데이터 사용
                        pwdata_o <= wdata_reg;
                    end
                    
                    penable_o <= 1'b0;
                end
                
                ACCESS: begin
                    // APB 액세스 단계
                    penable_o <= 1'b1;
                    
                    if (pready_i) begin
                        // 트랜잭션 완료 처리
                        trans_done_o <= 1'b1;
                        trans_error_o <= pslverr_i;
                        
                        if (!wr_trans_reg) begin
                            // 읽기 데이터 저장
                            read_data_o <= prdata_i;
                        end
                        
                        if (burst_cnt < burst_len_i) begin
                            // 버스트 계속 - 주소 업데이트
                            addr_reg <= addr_reg + 4;  // 4바이트씩 증가 (32비트 전송)
                            burst_cnt <= burst_cnt + 1;
                        end
                    end
                end
                
                DONE: begin
                    // 트랜잭션 완료
                    wr_trans_reg <= 1'b0;
                    rd_trans_reg <= 1'b0;
                end
            endcase
        end
    end

endmodule 