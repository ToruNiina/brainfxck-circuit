module top(CLK100MHZ, btn, uart_txd_in, uart_rxd_out, led0, led1);
    input  wire      CLK100MHZ;
    input  wire[3:0] btn;
    input  wire      uart_txd_in;
    output wire      uart_rxd_out;
    output wire[2:0] led0;
    output wire[2:0] led1;

    // -----------------------------------------------------
    // define modules

    wire      uart_send_enable;
    wire[7:0] uart_send_data;
    wire      uart_sending;

    uart_send send(
        .clock(CLK100MHZ),
        .reset(btn[0]),
        .uart_enable(uart_send_enable),
        .uart_data(uart_send_data),
        .uart_busy(uart_sending),
        .uart_tx(uart_rxd_out),
        .led(led0)
        );

    wire[7:0] uart_recv_data;
    wire      uart_receiving;
    wire      uart_recv_okay;

    uart_recv recv(
        .clock(CLK100MHZ),
        .reset(btn[0]),
        .uart_data(uart_recv_data),
        .uart_busy(uart_receiving),
        .uart_okay(uart_recv_okay),
        .uart_rx(uart_txd_in),
        .led(led1)
        );

    wire      sram_writing;
    wire[7:0] sram_addr;
    wire[7:0] sram_w_data;
    wire[7:0] sram_r_data;

    sram ram(
        .clock(CLK100MHZ),
        .reset(btn[0]),
        .write(sram_writing),
        .addr(sram_addr),
        .in (sram_w_data),
        .out(sram_r_data)
        );

    reg [7:0] r_addr = 8'b0;
    reg [7:0] w_addr = 8'b0;

    // -----------------------------------------------------
    // make a state machene

    reg[7:0] data;
    reg data_saved;
    wire has_data = (~uart_receiving) && uart_recv_okay;

    assign sram_w_data    = data;
    assign uart_send_data = data;

    parameter IDLE_STATE = 0;
    parameter SAVE_STATE = 1; // receive data and save it to sram
    parameter SEND_STATE = 2; // start sending the data
    parameter WAIT_STATE = 3; // wait until the whole data is sent
    reg [1:0]   state = IDLE_STATE;
    wire[1:0] w_state;

    assign uart_send_enable = state == SEND_STATE;
    assign sram_writing = (state == SAVE_STATE);
    assign sram_addr = (state == SAVE_STATE) ? w_addr : /* send or wait */r_addr;

    function [1:0] next_state(
        input[1:0] state,
        input      uart_sending,
        input      has_data,
        input      data_saved,
        input[3:0] button,
        input[7:0] raddr,
        input[7:0] waddr
        );
        if (state == IDLE_STATE) begin
            if (has_data && (~data_saved)) begin
                next_state = SAVE_STATE;
            end else if (button[1] && raddr != waddr) begin
                next_state = SEND_STATE;
            end else begin
                next_state = IDLE_STATE;
            end
        end else if (state == SAVE_STATE) begin
            next_state = IDLE_STATE;
        end else if (state == SEND_STATE) begin
            if (raddr == waddr) begin
                next_state = IDLE_STATE;
            end else if (uart_sending) begin
                next_state = WAIT_STATE;
            end else begin
                next_state = SEND_STATE;
            end
        end else /*if (state == WAIT_STATE)*/ begin
            if (uart_sending) begin
                next_state = WAIT_STATE;
            end else begin
                next_state = SEND_STATE;
            end
        end
    endfunction

    assign w_state = next_state(state, uart_sending, has_data, data_saved, btn, r_addr, w_addr);

    always @(posedge CLK100MHZ) begin
        if (btn[0]) begin
            state      <= IDLE_STATE;
            data       <= 8'b0;
            data_saved <= 1'b0;

//             sram_w_data <= 8'b0;
//             sram_r_data <= 8'b0;

            w_addr <= 8'b0;
            r_addr <= 8'b0;

        end else begin

            state <= w_state;

            // update data to write
            if (uart_recv_okay) begin
                data <= uart_recv_data;
            end else begin
                data <= 8'b0;
            end

            // update data save flag
            if (uart_receiving) begin
                data_saved <= 1'b0;
            end else if (state == SAVE_STATE) begin
                data_saved <= 1'b1;
            end

            if(state == IDLE_STATE) begin
                ; // do nothing
            end else if (state == SAVE_STATE) begin
                w_addr <= w_addr + 1;
            end else if (state == SEND_STATE) begin
                data   <= sram_r_data;
                if (~uart_sending) begin
                    r_addr <= r_addr + 1;
                end
            end else if (state == WAIT_STATE) begin
                ; // do nothing
            end
        end
    end
endmodule

module sram(clock, reset, write, addr, in, out);

    input  wire      clock;
    input  wire      reset;
    input  wire      write;
    input  wire[7:0] addr;
    input  wire[7:0] in;
    output reg[7:0]  out;

    reg [7:0] data[0:255];

    integer i;
    always @(posedge clock) begin
        if (reset) begin
            for (i=0; i<256; i=i+1)
                data[i] <= 0;
        end else if (write) begin
            data[addr] <= in;
        end else begin
            out <= data[addr];
        end
    end
endmodule

module uart_send(clock, reset, uart_enable, uart_data, uart_busy, uart_tx, led);
    input  wire       clock;
    input  wire       reset;
    input  wire       uart_enable;
    input  wire [7:0] uart_data;

    output wire       uart_busy;
    output wire       uart_tx;
    output wire [2:0] led;

    parameter CLK_PER_BIT = 868; // 100 MHz / 115200 Hz

    parameter IDLE_STATE = 0;
    parameter SEND_STATE = 1;

    reg  state = IDLE_STATE;
    wire w_state;

    reg [9:0] data = 10'b1111111111;
    wire[9:0] w_data;

    reg [9:0] clock_counter = 10'b0;
    wire[9:0] w_clock_counter;

    reg [3:0] bit_counter = 4'b0;
    wire[3:0] w_bit_counter;

    assign uart_tx   = data[0];
    assign uart_busy = (state == IDLE_STATE) ? 1'b0 : 1'b1;

    assign led[2:0]  = (state == IDLE_STATE) ? 3'b001 : 3'b010;

    // ----------------------------------------------------------------------
    // update state

    function [0:0] next_state(
        input      state,
        input      enable,
        input[9:0] clock_counter,
        input[3:0] bit_counter
        );

        if (state == IDLE_STATE) begin
            if(enable) begin
                next_state = SEND_STATE;
            end else begin
                next_state = IDLE_STATE;
            end
        end else begin // SEND_STATE
            if (clock_counter == CLK_PER_BIT-1) begin
                if (bit_counter == 4'd9) begin
                    next_state = IDLE_STATE;
                end else begin
                    next_state = SEND_STATE;
                end
            end else begin
                next_state = SEND_STATE;
            end
        end
    endfunction

    assign w_state = next_state(state, uart_enable, clock_counter, bit_counter);

    // ----------------------------------------------------------------------
    // update clock counter

    function [9:0] next_clock_counter(
        input      state,
        input[9:0] clock_counter
        );

        if (state == IDLE_STATE) begin
            next_clock_counter = 10'b0;
        end else begin
            if (clock_counter == CLK_PER_BIT-1) begin
                next_clock_counter = 10'b0;
            end else begin
                next_clock_counter = clock_counter + 1;
            end
        end
    endfunction

    assign w_clock_counter = next_clock_counter(state, clock_counter);

    // ----------------------------------------------------------------------
    // update number of bits sent

    function [3:0] next_bit_counter(
        input      state,
        input[9:0] clock_counter,
        input[3:0] bit_counter
        );

        if(state == IDLE_STATE) begin
            next_bit_counter = 4'b0;
        end else begin
            if (clock_counter == CLK_PER_BIT-1) begin
                if (bit_counter == 4'd9) begin
                    next_bit_counter = 4'd0;
                end else begin
                    next_bit_counter = bit_counter + 1;
                end
            end else begin
                next_bit_counter = bit_counter;
            end
        end
    endfunction

    assign w_bit_counter = next_bit_counter(state, clock_counter, bit_counter);

    // ----------------------------------------------------------------------
    // update data to send

    function [9:0] next_data(
        input      state,
        input      enable,
        input[7:0] uart_data,
        input[9:0] data,
        input[9:0] clock_counter
        );

        if (state == IDLE_STATE) begin
            if (enable) begin
                next_data = {1'b1, uart_data, 1'b0};
            end else begin
                next_data = data;
            end
        end else begin
            if (clock_counter == CLK_PER_BIT - 1) begin
                next_data = {1'b1, data[9:1]};
            end else begin
                next_data = data;
            end
        end
    endfunction
    assign w_data = next_data(state, uart_enable, uart_data, data, clock_counter);

    // ----------------------------------------------------------------------
    // update registers

    always @(posedge clock) begin
        if (reset) begin
            state         <= IDLE_STATE;
            clock_counter <= 10'b0;
            bit_counter   <=  4'b0;
            data          <= 10'b1111111111;
        end else begin
            state         <= w_state;
            clock_counter <= w_clock_counter;
            bit_counter   <= w_bit_counter;
            data          <= w_data;
        end
    end
endmodule

module uart_recv(clock, reset, uart_data, uart_busy, uart_okay, uart_rx, led);
    input  wire       clock;
    input  wire       reset;
    output wire [7:0] uart_data;

    output wire       uart_busy;
    output wire       uart_okay;
    input  wire       uart_rx;
    output wire [2:0] led;

    parameter CLK_PER_BIT = 868; // 100 MHz / 115200 Hz

    parameter IDLE_STATE = 0;
    parameter CHCK_STATE = 1; // checking start bit
    parameter RECV_STATE = 2;
    parameter STOP_STATE = 3; // receiving stop bit

    reg [1:0] state = IDLE_STATE;
    wire[1:0] w_state;

    reg [9:0] clock_counter = 10'b0;
    wire[9:0] w_clock_counter;

    reg [3:0] bit_counter = 4'b0;
    wire[3:0] w_bit_counter;

    reg [9:0] data = 10'b1111111111;
    wire[9:0] w_data;

    assign uart_data = data[8:1]; // except start/stop bits
    assign uart_okay = (data[0:0] == 1'b0) && (data[9:9] == 1'b1); // start with 0, end with 1
    assign uart_busy = (state == RECV_STATE) ? 1'b1 : 1'b0;
    assign led[2:0]  = (state == IDLE_STATE) ? 3'b001 :
                       (state == CHCK_STATE) ? 3'b010 :
                       (state == RECV_STATE) ? 3'b100 : 3'b111;

    // ----------------------------------------------------------------------
    // update state

    wire rx_is_zero;
    assign rx_is_zero = (uart_rx == 1'b0);

    function [1:0] next_state(
        input[1:0] state,
        input      uart_rx,
        input      rx_is_zero,
        input[9:0] clock_counter,
        input[3:0] bit_counter
        );

        if (state == IDLE_STATE) begin
            if(rx_is_zero) begin
                next_state = CHCK_STATE;
            end else begin
                next_state = IDLE_STATE;
            end
        end else if (state == CHCK_STATE) begin
            if (clock_counter == (CLK_PER_BIT / 2) - 1) begin
                if (rx_is_zero) begin
                    next_state = RECV_STATE;
                end else begin
                    next_state = IDLE_STATE;
                end
            end else begin
                next_state = CHCK_STATE;
            end
        end else if (state == RECV_STATE) begin
            if (clock_counter == CLK_PER_BIT-1) begin
                if (bit_counter == 4'd8) begin
                    next_state = STOP_STATE;
                end else begin
                    next_state = RECV_STATE;
                end
            end else begin
               next_state = RECV_STATE;
            end
        end else begin // STOP_STATE
            if (clock_counter == (CLK_PER_BIT / 2) - 1) begin
                next_state = IDLE_STATE;
            end else begin
                next_state = STOP_STATE;
            end
        end
    endfunction

    assign w_state = next_state(state, uart_rx, rx_is_zero, clock_counter, bit_counter);

    // ----------------------------------------------------------------------
    // update clock counter

    function [9:0] next_clock_counter(
        input[1:0] state,
        input[9:0] clock_counter
        );

        if (state == IDLE_STATE) begin
            next_clock_counter = 10'b0;
        end else if(state == CHCK_STATE) begin
            if (clock_counter == CLK_PER_BIT / 2 - 1) begin
                next_clock_counter = 10'b0;
            end else begin
                next_clock_counter = clock_counter + 1;
            end
        end else begin
            if (clock_counter == CLK_PER_BIT-1) begin
                next_clock_counter = 10'b0;
            end else begin
                next_clock_counter = clock_counter + 1;
            end
        end
    endfunction

    assign w_clock_counter = next_clock_counter(state, clock_counter);

    // ----------------------------------------------------------------------
    // update number of bits sent

    function [3:0] next_bit_counter(
        input[1:0] state,
        input[9:0] clock_counter,
        input[3:0] bit_counter
        );

        if(state == IDLE_STATE) begin
            next_bit_counter = 4'b0;
        end else if(state == CHCK_STATE) begin
            next_bit_counter = 4'b0;
        end else begin
            if (clock_counter == CLK_PER_BIT-1) begin
                if (bit_counter == 4'd9) begin
                    next_bit_counter = 4'd0;
                end else begin
                    next_bit_counter = bit_counter + 1;
                end
            end else begin
                next_bit_counter = bit_counter;
            end
        end
    endfunction

    assign w_bit_counter = next_bit_counter(state, clock_counter, bit_counter);

    // ----------------------------------------------------------------------
    // update received data

    function [9:0] next_data(
        input[1:0] state,
        input      uart_rx,
        input[9:0] data,
        input[9:0] clock_counter
        );

        if (state == IDLE_STATE) begin
            next_data = data;
        end else if (state == CHCK_STATE) begin
            if (uart_rx == 1'b0) begin
                next_data = 10'b0111111111;
            end else begin
                next_data = data;
            end
        end else if (state == RECV_STATE) begin
            if (clock_counter == CLK_PER_BIT - 1) begin
                next_data = {uart_rx, data[9:1]};
            end else begin
                next_data = data;
            end
        end else begin
            next_data = data;
        end
    endfunction
    assign w_data = next_data(state, uart_rx, data, clock_counter);

    // ----------------------------------------------------------------------
    // update registers

    always @(posedge clock) begin
        if (reset) begin
            state         <= IDLE_STATE;
            clock_counter <= 10'b0;
            bit_counter   <=  4'b0;
            data          <= 10'b1111111111;
        end else begin
            state         <= w_state;
            clock_counter <= w_clock_counter;
            bit_counter   <= w_bit_counter;
            data          <= w_data;
        end
    end
endmodule

