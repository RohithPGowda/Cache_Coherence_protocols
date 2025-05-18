//------------------------------------------------------------------------------
// Single-Core Cache Hierarchy: L1 (MOESI) & L2 Stub
// Verilog-2001, 32-bit data, 15-bit address
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Top-Level: Connects Core -> L1 -> L2 -> Memory
//------------------------------------------------------------------------------
module top_cache_system #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 15
)(
    input  wire                   clk,
    input  wire                   rst,

    // Core interface
    input  wire                   core_read,
    input  wire                   core_write,
    input  wire [ADDR_WIDTH-1:0]  core_addr,
    input  wire [DATA_WIDTH-1:0]  core_wdata,
    output wire [DATA_WIDTH-1:0]  core_rdata,

    // Memory interface
    output wire                   mem_read,
    output wire                   mem_write,
    output wire [ADDR_WIDTH-1:0]  mem_addr,
    output wire [DATA_WIDTH-1:0]  mem_wdata,
    input  wire [DATA_WIDTH-1:0]  mem_rdata
);

    // Signals between L1 and L2
    wire        busRd;
    wire        busRdX;
    wire        supplyL2;

    // L1 instance
    l1_moesi #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) L1 (
        .clk        (clk),
        .rst        (rst),
        // Core side
        .local_read (core_read),
        .local_write(core_write),
        .addr_in    (core_addr),
        .wdata_in   (core_wdata),
        .rdata_out  (core_rdata),
        // L2 side
        .busRd      (busRd),
        .busRdX     (busRdX),
        .shared     (supplyL2),
        .supplyData (),
        .invalidate (),
        // Memory passthrough
        .mem_read   (/unused/),  
        .mem_write  (/unused/),  
        .mem_addr   (/unused/),  
        .mem_wdata  (/unused/)   
    );

    // L2 instance (stub)
    l2_stub #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) L2 (
        .clk        (clk),
        .rst        (rst),
        .busRd_in   (busRd),
        .busRdX_in  (busRdX),
        .addr_in    (core_addr),
        .supplyData (supplyL2),
        .mem_read   (mem_read),
        .mem_write  (mem_write),
        .mem_addr   (mem_addr),
        .mem_wdata  (mem_wdata),
        .mem_rdata  (mem_rdata)
    );

endmodule

//------------------------------------------------------------------------------
// L1 Cache: MOESI protocol
//------------------------------------------------------------------------------
module l1_moesi #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 15
)(
    input  wire                   clk,
    input  wire                   rst,
    // Core side
    input  wire                   local_read,
    input  wire                   local_write,
    input  wire [ADDR_WIDTH-1:0]  addr_in,
    input  wire [DATA_WIDTH-1:0]  wdata_in,
    output reg  [DATA_WIDTH-1:0]  rdata_out,
    // L2 side
    output wire                   busRd,
    output wire                   busRdX,
    input  wire                   shared,
    output wire                   supplyData,
    output wire                   invalidate,
    // Memory passthrough (unused in L1)
    output wire                   mem_read,
    output wire                   mem_write,
    output wire [ADDR_WIDTH-1:0]  mem_addr,
    output wire [DATA_WIDTH-1:0]  mem_wdata
);
    wire issueRd, issueRdX;
    wire [2:0] state;
    // FSM
    moesi_datapath dp(
        .clk(clk), .rst(rst),
        .next_state(),
        .state(state)
    );
    moesi_controller ctrl(
        .local_read(local_read),
        .local_write(local_write),
        .busRd(issueRd),
        .busRdX(issueRdX),
        .busUpgr(1'b0),
        .shared(shared),
        .state(state),
        .next_state(),
        .issueBusRd(busRd),
        .issueBusRdX(busRdX),
        .issueBusUpgr(),
        .supplyData(supplyData),
        .invalidate(invalidate)
    );
    // Data path (registers)
    always @(posedge clk) begin
        if (rst) rdata_out <= {DATA_WIDTH{1'b0}};
        else if (supplyData)
            rdata_out <= /* from L2/memory */ rdata_out;
    end
    assign mem_read  = 1'b0;
    assign mem_write = 1'b0;
    assign mem_addr  = {ADDR_WIDTH{1'b0}};
    assign mem_wdata = {DATA_WIDTH{1'b0}};
endmodule

//------------------------------------------------------------------------------
// L2 Stub: forwards all requests to memory
//------------------------------------------------------------------------------
module l2_stub #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 15
)(
    input  wire                   clk,
    input  wire                   rst,
    input  wire                   busRd_in,
    input  wire                   busRdX_in,
    input  wire [ADDR_WIDTH-1:0]  addr_in,
    output reg                    supplyData,
    output reg                    invalidate,
    output reg                    mem_read,
    output reg                    mem_write,
    output reg  [ADDR_WIDTH-1:0]  mem_addr,
    output reg  [DATA_WIDTH-1:0]  mem_wdata,
    input  wire [DATA_WIDTH-1:0]  mem_rdata
);
    always @(posedge clk) begin
        if (rst) begin
            supplyData <= 0; invalidate <= 0;
            mem_read   <= 0; mem_write <= 0;
            mem_addr   <= {ADDR_WIDTH{1'b0}};
            mem_wdata  <= {DATA_WIDTH{1'b0}};
        end else begin
            supplyData <= 0; invalidate <= 0;
            mem_read   <= 0; mem_write <= 0;
            mem_addr   <= addr_in;
            if (busRd_in) begin
                mem_read     <= 1;
                supplyData   <= 1;
            end else if (busRdX_in) begin
                mem_read     <= 1;
                supplyData   <= 1;
                invalidate   <= 1;
            end
        end
    end
endmodule

//------------------------------------------------------------------------------
// MOESI FSM modules
//------------------------------------------------------------------------------
module moesi_datapath(
    input  wire        clk,
    input  wire        rst,
    output reg [2:0]   state,
    input  wire [2:0]  next_state
);
    localparam I=3'b000, S=3'b001, O=3'b010, E=3'b011, M=3'b100;
    always @(posedge clk) begin
        if (rst) state <= I;
        else     state <= next_state;
    end
endmodule

module moesi_controller(
    input  wire        local_read,
    input  wire        local_write,
    input  wire        busRd,
    input  wire        busRdX,
    input  wire        busUpgr,
    input  wire        shared,
    input  wire [2:0]  state,
    output reg [2:0]   next_state,
    output reg         issueBusRd,
    output reg         issueBusRdX,
    output reg         issueBusUpgr,
    output reg         supplyData,
    output reg         invalidate
);
    localparam I=3'b000, S=3'b001, O=3'b010, E=3'b011, M=3'b100;
    always @(*) begin
        next_state     = state;
        issueBusRd     = 0;
        issueBusRdX    = 0;
        issueBusUpgr   = 0;
        supplyData     = 0;
        invalidate     = 0;
        case(state)
            M: if (local_write) next_state=M;
               else if (busRd)  begin next_state=O; supplyData=1; end
               else if (busRdX|busUpgr) begin next_state=I; invalidate=1; end
            O: if (local_write) begin next_state=M; issueBusUpgr=1; end
               else if (busRd) next_state=S;
               else if (busRdX|busUpgr) begin next_state=I; invalidate=1; end
            E: if (local_write) next_state=M;
               else if (busRd)  next_state=S;
               else if (busRdX|busUpgr) begin next_state=I; invalidate=1; end
            S: if (local_write) begin next_state=M; issueBusUpgr=1; end
               else if (local_read|busRd) next_state=S;
               else if (busRdX|busUpgr) begin next_state=I; invalidate=1; end
            I: if (local_read) begin issueBusRd=1; next_state=shared?S:E; end
               else if (local_write) begin issueBusRdX=1; next_state=M; end
        endcase
    end
endmodule