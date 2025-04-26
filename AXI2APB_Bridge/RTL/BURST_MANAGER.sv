module BURST_MANAGER #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input  wire                     clk,
    input  wire                     rst_n,
    
    // AXI 버스트 정보
    input  wire [3:0]               burst_len_i,    // 버스트 길이 (AXI AWLEN/ARLEN)
    input  wire [2:0]               burst_size_i,   // 버스트 크기 (AXI AWSIZE/ARSIZE)
    input  wire [1:0]               burst_type_i,   // 버스트 타입 (AXI AWBURST/ARBURST)
    input  wire [ADDR_WIDTH-1:0]    start_addr_i,   // 시작 주소
    
    // 버스트 제어 및 상태
    input  wire                     burst_start_i,  // 버스트 시작 신호
    input  wire                     transfer_done_i, // 하나의 전송 완료 신호
    output reg                      burst_done_o,   // 버스트 완료 신호
    output reg [ADDR_WIDTH-1:0]     curr_addr_o     // 현재 전송 주소
);

    // 버스트 타입 상수
    localparam BURST_FIXED = 2'b00;  // 고정 주소
    localparam BURST_INCR  = 2'b01;  // 증가 주소
    localparam BURST_WRAP  = 2'b10;  // 랩 어라운드 주소
    
    // 내부 레지스터
    reg [3:0]                   transfer_count;
    reg [ADDR_WIDTH-1:0]        next_addr;
    reg [2:0]                   addr_incr_bytes;
    
    // 버스트 크기에 따른 주소 증가량 계산
    always_comb begin
        case (burst_size_i)
            3'b000: addr_incr_bytes = 3'd1;  // 1 바이트
            3'b001: addr_incr_bytes = 3'd2;  // 2 바이트
            3'b010: addr_incr_bytes = 3'd4;  // 4 바이트
            3'b011: addr_incr_bytes = 3'd4;  // 4 바이트 (대신 8 바이트)
            default: addr_incr_bytes = 3'd4; // 기본값
        endcase
    end
    
    // 버스트 상태 관리
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            transfer_count <= 4'b0000;
            curr_addr_o <= {ADDR_WIDTH{1'b0}};
            next_addr <= {ADDR_WIDTH{1'b0}};
            burst_done_o <= 1'b0;
        end else begin
            // 기본 상태
            burst_done_o <= 1'b0;
            
            if (burst_start_i) begin
                // 새 버스트 시작
                transfer_count <= 4'b0000;
                // 유효한 주소인지 확인하고 설정
                if ((start_addr_i >= 32'h0001_F000 && start_addr_i <= 32'h0001_FFFF) ||
                    (start_addr_i >= 32'h0002_F000 && start_addr_i <= 32'h0002_FFFF)) begin
                    curr_addr_o <= start_addr_i;
                    next_addr <= start_addr_i;
                end else begin
                    // 유효하지 않은 주소인 경우 기본 슬레이브 1 주소 사용
                    curr_addr_o <= 32'h0001_F000;
                    next_addr <= 32'h0001_F000;
                end
            end else if (transfer_done_i) begin
                // 전송 완료 처리
                transfer_count <= transfer_count + 1;
                
                // 다음 주소 계산
                case (burst_type_i)
                    BURST_FIXED: begin
                        // 고정 주소 - 변경 없음
                        curr_addr_o <= start_addr_i;
                    end
                    
                    BURST_INCR: begin
                        // 증가 주소
                        next_addr <= next_addr + addr_incr_bytes;
                        curr_addr_o <= next_addr;
                    end
                    
                    BURST_WRAP: begin
                        // 랩 어라운드 주소
                        // 랩 경계 계산
                        reg [ADDR_WIDTH-1:0] wrap_boundary;
                        reg [ADDR_WIDTH-1:0] wrap_mask;
                        
                        // 랩 크기 = (burst_len + 1) * 전송 크기
                        wrap_mask = ((burst_len_i + 1) * addr_incr_bytes) - 1;
                        wrap_boundary = start_addr_i & ~wrap_mask;
                        
                        // 다음 주소 계산 (랩 경계 내에서 순환)
                        next_addr <= wrap_boundary | ((next_addr + addr_incr_bytes) & wrap_mask);
                        curr_addr_o <= next_addr;
                    end
                    
                    default: begin
                        // 증가 주소 (기본값)
                        next_addr <= next_addr + addr_incr_bytes;
                        curr_addr_o <= next_addr;
                    end
                endcase
                
                // 버스트 완료 확인
                if (transfer_count == burst_len_i) begin
                    burst_done_o <= 1'b1;
                end
            end
        end
    end

endmodule 