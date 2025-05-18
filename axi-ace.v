typedef struct packed {
  logic [31:0] ARADDR;
  logic        ARVALID;
  logic        ARREADY;
  logic [31:0] RDATA;
  logic        RVALID;
  logic        RREADY;
  logic [31:0] AWADDR;
  logic        AWVALID;
  logic        AWREADY;
  logic [31:0] WDATA;
  logic        WVALID;
  logic        WREADY;
  logic        BVALID;
  logic        BREADY;
} axi_ace_if_t;

module ace_controller (
  input  logic        clk, rst,
  input  logic        cpu_rd_req, cpu_wr_req,
  input  logic [31:0] cpu_addr,
  output logic [2:0]  curr_state,
  inout  axi_ace_if_t axi,
  input  logic        Shared_line,
  output logic        bus_trn
);
  typedef enum logic [2:0] {NP, SC, M, SM, E} state_t;
  state_t state, next_state;

  always_ff @(posedge clk or posedge rst) begin
    if (rst) state <= NP;
    else     state <= next_state;
  end

  always_comb begin
    next_state = state;
    bus_trn     = 0;
    axi.ARVALID = 0;
    axi.AWVALID = 0;
    axi.WVALID  = 0;

    case (state)
      NP: if (cpu_rd_req) begin
            axi.ARADDR  = cpu_addr;
            axi.ARVALID = 1;
            if (axi.ARREADY) next_state = Shared_line ? SC : E;
            bus_trn = axi.ARREADY;
          end else if (cpu_wr_req) begin
            axi.AWADDR  = cpu_addr;
            axi.AWVALID = 1;
            axi.WDATA   = 32'hDEAD;
            axi.WVALID  = 1;
            if (axi.AWREADY && axi.WREADY) next_state = Shared_line ? SM : M;
            bus_trn = axi.AWREADY && axi.WREADY;
          end
      SC: if (cpu_wr_req) begin
            axi.AWADDR  = cpu_addr;
            axi.AWVALID = 1;
            axi.WDATA   = 32'hDEAD;
            axi.WVALID  = 1;
            if (axi.AWREADY && axi.WREADY) begin
              next_state = Shared_line ? SM : M;
              bus_trn = 1;
            end
          end
      E: if (cpu_wr_req) begin
            next_state = M;
          end else if (0) begin
            next_state = SC;
            bus_trn    = 1;
          end
      M: if (0) begin
            next_state = SM;
            bus_trn    = 1;
          end
      SM: if (cpu_wr_req && Shared_line) begin
            axi.AWADDR  = cpu_addr;
            axi.AWVALID = 1;
            axi.WDATA   = 32'hDEAD;
            axi.WVALID  = 1;
            if (axi.AWREADY && axi.WREADY) bus_trn = 1;
          end else if (0) begin
            next_state = SC;
          end
    endcase
  end

  assign curr_state = state;
endmodule

module ace_datapath #(
  parameter ADDR_WIDTH = 15
)(
  input  logic                   clk, rst,
  input  logic [2:0]             next_state,
  input  logic                   cpu_rd_req, cpu_wr_req,
  input  logic [31:0]            cpu_addr,
  inout  axi_ace_if_t            axi,
  input  logic                   bus_trn,
  output logic [2:0]             curr_state
);
  localparam WORDS = (1 << (ADDR_WIDTH-2));
  logic [31:0] mem_array [0:WORDS-1];
  logic [ADDR_WIDTH-1:0] idx = cpu_addr;

  always_ff @(posedge clk or posedge rst) begin
    if (rst) curr_state <= NP;
    else     curr_state <= next_state;
  end

  always_ff @(posedge clk) begin
    if (axi.RVALID && axi.RREADY)
      mem_array[idx[ADDR_WIDTH-1:2]] <= axi.RDATA;
  end

  always_ff @(posedge clk) begin
    if (axi.BVALID && axi.BREADY)
      ;
  end
endmodule

module ace_system (
  input  logic        clk, rst,
  input  logic        cpu_rd, cpu_wr,
  input  logic [31:0] cpu_addr,
  input  logic        Shared_L1, Shared_L2
);
  axi_ace_if_t axi_L1, axi_L2;

  logic [2:0] state_L1;
  logic       bus_trn_L1;
  ace_controller ctrl_L1 (
    .clk(clk), .rst(rst),
    .cpu_rd_req(cpu_rd), .cpu_wr_req(cpu_wr), .cpu_addr(cpu_addr),
    .axi(axi_L1), .Shared_line(Shared_L1), .bus_trn(bus_trn_L1),
    .curr_state(state_L1)
  );
  ace_datapath #(.ADDR_WIDTH(15)) dp_L1 (
    .clk(clk), .rst(rst), .next_state(state_L1),
    .cpu_rd_req(cpu_rd), .cpu_wr_req(cpu_wr), .cpu_addr(cpu_addr),
    .axi(axi_L1), .bus_trn(bus_trn_L1), .curr_state(state_L1)
  );

  assign axi_L2 = axi_L1;

  logic [2:0] state_L2;
  logic       bus_trn_L2;
  ace_controller ctrl_L2 (
    .clk(clk), .rst(rst),
    .cpu_rd_req(bus_trn_L1), .cpu_wr_req(bus_trn_L1), .cpu_addr(cpu_addr),
    .axi(axi_L2), .Shared_line(Shared_L2), .bus_trn(bus_trn_L2),
    .curr_state(state_L2)
  );
  ace_datapath #(.ADDR_WIDTH(15)) dp_L2 (
    .clk(clk), .rst(rst), .next_state(state_L2),
    .cpu_rd_req(bus_trn_L1), .cpu_wr_req(bus_trn_L1), .cpu_addr(cpu_addr),
    .axi(axi_L2), .bus_trn(bus_trn_L2), .curr_state(state_L2)
  );
endmodule

module tb_ace_system;
  logic clk, rst;
  logic cpu_rd, cpu_wr;
  logic [31:0] cpu_addr;
  logic Shared_L1, Shared_L2;
  ace_system uut (.clk(clk), .rst(rst), .cpu_rd(cpu_rd), .cpu_wr(cpu_wr),
                  .cpu_addr(cpu_addr), .Shared_L1(Shared_L1), .Shared_L2(Shared_L2));
  initial clk=0; always #5 clk=~clk;
  initial begin
    rst=1; cpu_rd=0; cpu_wr=0; cpu_addr=0;
    Shared_L1=0; Shared_L2=0;
    #10 rst=0;
    #10 cpu_rd=1; cpu_addr=32'h1000;
    #10 cpu_rd=0;
    #20 cpu_wr=1; cpu_addr=32'h1000;
    #10 cpu_wr=0;
    #50 $finish;
  end
endmodule
