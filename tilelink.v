module cpu (
    input  logic clk,
    input  logic reset,
    output logic req_valid,
    output logic req_write,
    output logic [31:0] req_addr,
    output logic [31:0] req_wdata,
    input  logic resp_valid,
    input  logic [31:0] resp_rdata
);
    typedef enum logic [1:0] {IDLE, READ, WRITE, DONE} state_t;
    state_t state;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            req_valid <= 0;
            req_write <= 0;
            req_addr <= 32'h4;
            req_wdata <= 32'hface_cafe;
        end else begin
            case (state)
                IDLE: begin
                    req_valid <= 1;
                    req_write <= 0;
                    state <= READ;
                end
                READ: if (resp_valid) begin
                    req_valid <= 1;
                    req_write <= 1;
                    req_addr  <= 32'h8;
                    state <= WRITE;
                end
                WRITE: if (resp_valid) begin
                    req_valid <= 0;
                    state <= DONE;
                end
                DONE: begin end
            endcase
        end
    end
endmodule

module l1_cache_fsm (
    input  logic clk,
    input  logic reset,
    input  logic        cpu_req_valid,
    input  logic        cpu_req_write,
    input  logic [31:0] cpu_req_addr,
    input  logic [31:0] cpu_req_wdata,
    output logic        cpu_resp_valid,
    output logic [31:0] cpu_resp_rdata,

    output logic        tl_a_valid,
    output logic        tl_a_excl,
    output logic [31:0] tl_a_addr,

    input  logic        tl_d_valid,
    input  logic [31:0] tl_d_data,
    input  logic        tl_d_excl
);
    typedef enum logic [2:0] {IDLE, SEND_ACQUIRE, WAIT_GRANT, PROCESS} fsm_t;
    fsm_t state;

    logic [31:0] data, tag;
    logic        cached;

    assign tl_a_addr = cpu_req_addr;
    assign tl_a_excl = cpu_req_write;
    assign tl_a_valid = (state == SEND_ACQUIRE);

    assign cpu_resp_valid = (state == PROCESS) && cpu_req_valid;
    assign cpu_resp_rdata = data;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            data  <= 0;
            tag   <= 0;
        end else begin
            case (state)
                IDLE: if (cpu_req_valid) begin
                    if (tag == cpu_req_addr) begin
                        state <= PROCESS;
                    end else begin
                        state <= SEND_ACQUIRE;
                    end
                end

                SEND_ACQUIRE: begin
                    state <= WAIT_GRANT;
                end

                WAIT_GRANT: if (tl_d_valid) begin
                    tag  <= tl_a_addr;
                    data <= tl_d_data;
                    state <= PROCESS;
                end

                PROCESS: begin
                    if (cpu_req_write)
                        data <= cpu_req_wdata;
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule


module l2_directory_fsm (
    input  logic clk,
    input  logic reset,
    input  logic        tl_a_valid,
    input  logic        tl_a_excl,
    input  logic [31:0] tl_a_addr,
    output logic        tl_d_valid,
    output logic [31:0] tl_d_data,
    output logic        tl_d_excl
);
    typedef enum logic [1:0] {L2_IDLE, PROCESS_ACQUIRE, SEND_GRANT} fsm_t;
    fsm_t state;

    logic [31:0] mem_data;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= L2_IDLE;
            mem_data <= 32'hdead_beef;
            tl_d_data <= 0;
            tl_d_valid <= 0;
            tl_d_excl <= 0;
        end else begin
            case (state)
                L2_IDLE: if (tl_a_valid) begin
                    state <= PROCESS_ACQUIRE;
                end

                PROCESS_ACQUIRE: begin
                    tl_d_data <= (tl_a_addr == 32'h8) ? 32'hc0ffee00 : mem_data;
                    tl_d_excl <= tl_a_excl;
                    state <= SEND_GRANT;
                end

                SEND_GRANT: begin
                    tl_d_valid <= 1;
                    state <= L2_IDLE;
                end
            endcase
        end
    end
endmodule

module tb_tilelink_fsm_demo;
    logic clk, reset;

    // CPU-L1
    logic cpu_req_valid, cpu_req_write;
    logic [31:0] cpu_req_addr, cpu_req_wdata;
    logic cpu_resp_valid;
    logic [31:0] cpu_resp_rdata;

    // TL signals
    logic tl_a_valid, tl_a_excl;
    logic [31:0] tl_a_addr;
    logic tl_d_valid, tl_d_excl;
    logic [31:0] tl_d_data;

    cpu cpu0 (
        .clk(clk), .reset(reset),
        .req_valid(cpu_req_valid),
        .req_write(cpu_req_write),
        .req_addr(cpu_req_addr),
        .req_wdata(cpu_req_wdata),
        .resp_valid(cpu_resp_valid),
        .resp_rdata(cpu_resp_rdata)
    );

    l1_cache_fsm l1 (
        .clk(clk), .reset(reset),
        .cpu_req_valid(cpu_req_valid),
        .cpu_req_write(cpu_req_write),
        .cpu_req_addr(cpu_req_addr),
        .cpu_req_wdata(cpu_req_wdata),
        .cpu_resp_valid(cpu_resp_valid),
        .cpu_resp_rdata(cpu_resp_rdata),
        .tl_a_valid(tl_a_valid),
        .tl_a_excl(tl_a_excl),
        .tl_a_addr(tl_a_addr),
        .tl_d_valid(tl_d_valid),
        .tl_d_data(tl_d_data),
        .tl_d_excl(tl_d_excl)
    );

    l2_directory_fsm l2 (
        .clk(clk), .reset(reset),
        .tl_a_valid(tl_a_valid),
        .tl_a_excl(tl_a_excl),
        .tl_a_addr(tl_a_addr),
        .tl_d_valid(tl_d_valid),
        .tl_d_data(tl_d_data),
        .tl_d_excl(tl_d_excl)
    );

    always #5 clk = ~clk;

    initial begin
        $dumpfile("tilelink_fsm.vcd");
        $dumpvars(0, tb_tilelink_fsm_demo);
        clk = 0;
        reset = 1;
        #10 reset = 0;
        #200 $finish;
    end
endmodule
