module ADDRESS_DECODER #(
    parameter ADDR_WIDTH = 32
)(
    input  wire [ADDR_WIDTH-1:0]    addr_i,
    output reg  [1:0]               psel_o
);

    // APB 슬레이브 주소 범위 정의
    // SLV1 영역: 0x0001_F000 - 0x0001_FFFF (4KB)
    // SLV2 영역: 0x0002_F000 - 0x0002_FFFF (4KB)
    localparam [ADDR_WIDTH-1:0] SLV1_START = 32'h0001_F000;
    localparam [ADDR_WIDTH-1:0] SLV1_END   = 32'h0001_FFFF;
    localparam [ADDR_WIDTH-1:0] SLV2_START = 32'h0002_F000;
    localparam [ADDR_WIDTH-1:0] SLV2_END   = 32'h0002_FFFF;
    
    // 주소 디코딩 로직
    always_comb begin
        if ((addr_i >= SLV1_START) && (addr_i <= SLV1_END)) begin
            psel_o = 2'b01;  // Slave 1 선택
        end 
        else if ((addr_i >= SLV2_START) && (addr_i <= SLV2_END)) begin
            psel_o = 2'b10;  // Slave 2 선택
        end 
        else begin
            psel_o = 2'b01;  // 유효하지 않은 주소는 기본적으로 Slave 1 선택
        end
    end

endmodule 