module dragon_controller (
    input  wire        PrRd,
    input  wire        PrRdMiss,
    input  wire        PrWr,
    input  wire        PrWrMiss,
    input  wire        Shared_line,
    input  wire        BusRd,
    input  wire        BusUpd,
    input  wire [2:0]  current_state,
    output reg [2:0]   next_state,
    output reg         bus_trn
);
    localparam NP  = 3'b000;
    localparam SC  = 3'b001;
    localparam M   = 3'b010;
    localparam SM  = 3'b011;
    localparam E   = 3'b100;

    always @(*) begin
        next_state = current_state;
        bus_trn    = 1'b0;
        case (current_state)
            M: begin
                if      (PrWr)        next_state = M;
                else if (BusRd)       begin next_state = SM; bus_trn = 1'b1; end
                else if (BusUpd)      begin next_state = M;  bus_trn = 1'b1; end
            end
            SM: begin
                if      (PrWr & !Shared_line) next_state = M;
                else if (PrWr & Shared_line)  begin next_state = SM; bus_trn = 1'b1; end
                else if (BusUpd)              next_state = SC;
                else if (BusRd)               begin next_state = SM; bus_trn = 1'b1; end
            end
            SC: begin
                if      (PrWr & !Shared_line) next_state = M;
                else if (PrWr & Shared_line)  begin next_state = SM; bus_trn = 1'b1; end
                else                             next_state = SC;
            end
            E: begin
                if      (PrWr)        next_state = M;
                else if (BusRd)       begin next_state = SC; bus_trn = 1'b1; end
            end
            NP: begin
                if      (PrRdMiss & !Shared_line) next_state = E;
                else if (PrRdMiss & Shared_line)  next_state = SC;
                else if (PrWrMiss & !Shared_line) next_state = M;
                else if (PrWrMiss & Shared_line)  next_state = SM;
                if (PrRdMiss | PrWrMiss)         bus_trn = 1'b1;
            end
        endcase
    end
endmodule

module dragon_datapath #(
    parameter ADDR_WIDTH = 15
)(
    input  wire                   clk,
    input  wire                   rst,
    input  wire                   PrRd,
    input  wire                   PrRdMiss,
    input  wire                   PrWr,
    input  wire                   PrWrMiss,
    input  wire                   BusRd,
    input  wire                   BusUpd,
    input  wire                   bus_trn,
    input  wire [2:0]             next_state,
    input  wire                   Shared_line,
    output reg [2:0]              current_state,
    output reg [ADDR_WIDTH-1:0]   bus_addr,
    output reg [31:0]             bus_data
);
    localparam NP    = 3'b000;
    localparam WORDS = (1 << (ADDR_WIDTH-2));
    reg [31:0] mem_array [0:WORDS-1];
    wire [ADDR_WIDTH-1:0] idx = bus_addr;

    always @(posedge clk or posedge rst) begin
        if (rst)        current_state <= NP;
        else            current_state <= next_state;
    end

    always @(*) begin
        bus_addr = {ADDR_WIDTH{1'b0}};
        bus_data = 32'b0;
        if (PrRd | PrRdMiss) begin
            bus_data = mem_array[idx[ADDR_WIDTH-1:2]];
        end else if (PrWr | PrWrMiss) begin
            bus_data = 32'hAAAA_AAAA;
        end else if (BusRd | BusUpd) begin
            bus_data = mem_array[idx[ADDR_WIDTH-1:2]];
        end
    end

    always @(posedge clk) begin
        if (PrWr | PrWrMiss)
            mem_array[idx[ADDR_WIDTH-1:2]] <= 32'hAAAA_AAAA;
        else if (BusUpd | (current_state==NP && next_state!=NP && bus_trn))
            mem_array[idx[ADDR_WIDTH-1:2]] <= bus_data;
    end
endmodule

module dragon_system (
    input  wire        clk,
    input  wire        rst,
    input  wire        PrRd_cpu,
    input  wire        PrRdMiss_cpu,
    input  wire        PrWr_cpu,
    input  wire        PrWrMiss_cpu,
    input  wire        Shared_L1,
    input  wire        Shared_L2,
    output wire [2:0]  state_L1,
    output wire [2:0]  state_L2
);
    wire [2:0] next_state_L1;
    wire       bus_trn_L1;

    wire BusRd_L2  = bus_trn_L1;
    wire BusUpd_L2 = bus_trn_L1;

    dragon_controller ctrl_L1 (
        .PrRd(PrRd_cpu), .PrRdMiss(PrRdMiss_cpu),
        .PrWr(PrWr_cpu), .PrWrMiss(PrWrMiss_cpu),
        .Shared_line(Shared_L1),
        .BusRd(BusRd_L2), .BusUpd(BusUpd_L2),
        .current_state(state_L1), .next_state(next_state_L1),
        .bus_trn(bus_trn_L1)
    );
    dragon_datapath #(.ADDR_WIDTH(15)) dp_L1 (
        .clk(clk), .rst(rst),
        .PrRd(PrRd_cpu), .PrRdMiss(PrRdMiss_cpu),
        .PrWr(PrWr_cpu), .PrWrMiss(PrWrMiss_cpu),
        .BusRd(BusRd_L2), .BusUpd(BusUpd_L2),
        .bus_trn(bus_trn_L1), .next_state(next_state_L1),
        .Shared_line(Shared_L1),
        .current_state(state_L1), .bus_addr(), .bus_data()
    );

    wire [2:0] next_state_L2;

    dragon_controller ctrl_L2 (
        .PrRd(BusRd_L2), .PrRdMiss(1'b0),
        .PrWr(BusUpd_L2), .PrWrMiss(1'b0),
        .Shared_line(Shared_L2),
        .BusRd(1'b0), .BusUpd(1'b0),
        .current_state(state_L2), .next_state(next_state_L2),
        .bus_trn()
    );
    dragon_datapath #(.ADDR_WIDTH(15)) dp_L2 (
        .clk(clk), .rst(rst),
        .PrRd(BusRd_L2), .PrRdMiss(1'b0),
        .PrWr(BusUpd_L2), .PrWrMiss(1'b0),
        .BusRd(1'b0), .BusUpd(1'b0),
        .bus_trn(), .next_state(next_state_L2),
        .Shared_line(Shared_L2),
        .current_state(state_L2), .bus_addr(), .bus_data()
    );
endmodule

module tb_dragon_system;
    reg clk, rst;
    reg PrRd_cpu, PrRdMiss_cpu;
    reg PrWr_cpu, PrWrMiss_cpu;
    reg Shared_L1, Shared_L2;
    wire [2:0] state_L1, state_L2;

    dragon_system dut (
        .clk(clk), .rst(rst),
        .PrRd_cpu(PrRd_cpu), .PrRdMiss_cpu(PrRdMiss_cpu),
        .PrWr_cpu(PrWr_cpu), .PrWrMiss_cpu(PrWrMiss_cpu),
        .Shared_L1(Shared_L1), .Shared_L2(Shared_L2),
        .state_L1(state_L1), .state_L2(state_L2)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        rst = 1;
        PrRd_cpu = 0; PrRdMiss_cpu = 0;
        PrWr_cpu = 0; PrWrMiss_cpu = 0;
        Shared_L1 = 0; Shared_L2 = 0;
        #12 rst = 0;

        #10 PrRdMiss_cpu = 1;
        #10 PrRdMiss_cpu = 0;
        #10 $display("T1: L1=%b, L2=%b after PrRdMiss", state_L1, state_L2);

        #10 PrWr_cpu = 1;
        #10 PrWr_cpu = 0;
        #10 $display("T2: L1=%b, L2=%b after PrWr", state_L1, state_L2);

        #10 Shared_L1 = 1; Shared_L2 = 1;
        #10 PrRd_cpu = 1;
        #10 PrRd_cpu = 0;
        #10 $display("T3: L1=%b, L2=%b after shared PrRd", state_L1, state_L2);

        #10 PrWrMiss_cpu = 1;
        #10 PrWrMiss_cpu = 0;
        #10 $display("T4: L1=%b, L2=%b after PrWrMiss shared", state_L1, state_L2);

        #20 $finish;
    end

    initial begin
        $monitor("%0t | PrRd=%b PrRdMiss=%b PrWr=%b PrWrMiss=%b Shared_L1=%b Shared_L2=%b || L1=%b L2=%b",
                 $time, PrRd_cpu, PrRdMiss_cpu, PrWr_cpu, PrWrMiss_cpu,
                 Shared_L1, Shared_L2, state_L1, state_L2);
    end
endmodule
