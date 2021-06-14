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
    wire[7:0] sram_r_addr;
    reg [7:0] sram_w_addr = 8'b0;
    wire[7:0] sram_w_data;
    wire[7:0] sram_r_data;

    sram ram(
        .clock(CLK100MHZ),
        .reset(btn[0]),
        .write(sram_writing),
        .r_addr(sram_r_addr),
        .w_addr(sram_w_addr),
        .in (sram_w_data),
        .out(sram_r_data)
        );

    brainfuck bf(
        .clock(CLK100MHZ),
        .reset(btn[0]),
        .run(btn[1]),
        .inst_data(sram_r_data),
        .inst_len(sram_w_addr),
        .inst_addr(sram_r_addr),
        .out_flag(uart_send_enable),
        .out_data(uart_send_data),
        .out_busy(uart_sending)
        );

    // -----------------------------------------------------
    // make a state machene

    reg[7:0] data_recv = 8'b0;
    reg data_saved;
    wire has_data = (~uart_receiving) && uart_recv_okay;

    assign sram_w_data    = uart_recv_data;
//     assign uart_send_data = (~out_busy) && data_recv;

    parameter IDLE_STATE = 0;
    parameter SAVE_STATE = 1; // receive data and save it to sram
    reg    state = IDLE_STATE;
    wire w_state;

//     assign uart_send_enable = (state == SAVE_STATE);
    assign sram_writing     = (state == SAVE_STATE);

    function [0:0] next_state(
        input[0:0] state,
        input      has_data,
        input      data_saved
        );
        if (state == IDLE_STATE) begin
            if (has_data && (~data_saved)) begin
                next_state = SAVE_STATE;
            end else begin
                next_state = IDLE_STATE;
            end
        end else begin // SAVE_STATE
            next_state = IDLE_STATE;
        end
    endfunction

    assign w_state = next_state(state, has_data, data_saved);

    always @(posedge CLK100MHZ) begin
        if (btn[0]) begin

            state       <= IDLE_STATE;
            data_recv   <= 8'b0;
            data_saved  <= 1'b0;
            sram_w_addr <= 8'b0;

        end else begin

            state <= w_state;

            // update data to write
            if (uart_recv_okay) begin
                data_recv <= uart_recv_data;
            end else begin
                data_recv <= 8'b0;
            end

            // update data save flag
            if (uart_receiving) begin
                data_saved <= 1'b0;
            end else if (state == SAVE_STATE) begin
                data_saved <= 1'b1;
            end

            if (state == SAVE_STATE) begin
                sram_w_addr <= sram_w_addr + 1;
            end
        end
    end
endmodule

module brainfuck(clock, reset, run, inst_data, inst_len, inst_addr, out_flag, out_data, out_busy);

    input  wire      clock;
    input  wire      reset;
    input  wire      run;
    input  wire[7:0] inst_data;
    input  wire[7:0] inst_len;
    output wire[7:0] inst_addr;
    output wire      out_flag;
    output wire[7:0] out_data;
    input  wire      out_busy;

    // ----------------------------------------------------------------------

    reg [7:0] inst_ptr = 8'b0;
    assign inst_addr = inst_ptr;

    // ----------------------------------------------------------------------
    // data memory

    reg [7:0] data_ptr = 8'b0;
    wire[7:0] w_data;
    wire[7:0] r_data;
    wire write_data;

    sram data_mem(
        .clock(clock),
        .reset(reset),
        .write(write_data),
        .r_addr(data_ptr),
        .w_addr(data_ptr),
        .in (w_data),
        .out(r_data)
        );

    assign out_data = r_data; // connect pointed byte to uart_out

    // ----------------------------------------------------------------------

    parameter IDLE_STATE = 0; // doing nothing
    parameter EXEC_STATE = 1; // executing bf
    parameter WAIT_STATE = 2; // waiting UART

    reg[1:0] state = IDLE_STATE;
    function [1:0] next_state(
        input state,
        input run,
        input[7:0] inst_len,
        input[7:0] inst_addr
        );

        if (state == IDLE_STATE) begin
            if (run && (inst_len != inst_addr)) begin
                next_state = EXEC_STATE;
            end else begin
                next_state = IDLE_STATE;
            end
        end else if (state == EXEC_STATE) begin
            if (inst_len == inst_addr) begin
                next_state = IDLE_STATE;
            end else if (out_busy) begin // wait until UART sends current byte
                next_state = WAIT_STATE;
            end else begin
                next_state = EXEC_STATE;
            end
        end else if (state == WAIT_STATE) begin
            if (out_busy) begin
                next_state = WAIT_STATE;
            end else begin
                next_state = EXEC_STATE;
            end
        end
    endfunction
    wire[1:0] w_state;
    assign w_state = next_state(state, run, inst_len, inst_ptr);

    // +-----+------+-------------------------------------+
    // |inst |ascii |expr                                 |
    // +-----+------+-------------------------------------+
    // |  >  | 0x3E | increment data ptr                  |
    // |  <  | 0x3C | decrement data ptr                  |
    // |  +  | 0x2B | increment data pointed by data ptr  |
    // |  -  | 0x2D | decrement data pointed by data ptr  |
    // |  .  | 0x2E | output a byte                       |
    // |  ,  | 0x2C | input a byte (is not supported)     |
    // |  [  | 0x5B | jump to `]` if data is zero         |
    // |  ]  | 0x5D | jump to `[` if data is not zero     |
    // +-----+------+-------------------------------------+
    //
    // loop:           [>>+<<-]
    // case skip: dep= 11111110
    // case loop: dep= 0000000F
    // case exit: dep= 00000000

    reg [8:0] nest_depth = 9'b0; // to find the corresponding `[` and `]`
    wire[8:0] w_nest_depth;

    wire executing;
    assign executing = (nest_depth == 0) && (state == EXEC_STATE) && (inst_ptr != inst_len);

    function [8:0] next_nest_depth(
        input       executing, // forward == 1, backward == 0
        input [7:0] inst_data,
        input [7:0] r_data,
        input [8:0] nest_depth
        );

        if (executing) begin
            if (inst_data == 8'h5B) begin // `[`
                if (r_data == 8'b0) begin // jump to the corresponding `]`
                    next_nest_depth = nest_depth + 1;
                end else begin
                    next_nest_depth = nest_depth; // execute the loop body
                end
            end else if (inst_data == 8'h5D) begin // `]`
                if (r_data == 8'h0) begin
                    next_nest_depth = nest_depth; // go through
                end else begin // jump to the corresponding `[`
                    next_nest_depth = nest_depth - 1;
                end
            end else begin
                next_nest_depth = nest_depth;
            end
        end else begin
            if (inst_data == 8'h5B) begin // [
                next_nest_depth = nest_depth + 1;
            end else if (inst_data == 8'h5D) begin
                next_nest_depth = nest_depth - 1;
            end else begin
                next_nest_depth = nest_depth;
            end
        end
    endfunction
    assign w_nest_depth = next_nest_depth(executing, inst_data, r_data, nest_depth);

    wire [7:0] w_inst_ptr;
    assign w_inst_ptr = (inst_ptr == inst_len)    ? inst_ptr :
                       ((w_nest_depth[8] == 1'b1) ? inst_ptr - 1 : inst_ptr + 1);

    function [7:0] next_data_ptr(
        input       executing,
        input [7:0] inst_data,
        input [7:0] data_ptr
        );

        if (executing) begin
            if (inst_data == 8'h3E) begin // `>`
                next_data_ptr = data_ptr + 1;
            end else if (inst_data == 8'h3C) begin // `<`
                next_data_ptr = data_ptr - 1;
            end else begin
                next_data_ptr = data_ptr;
            end
        end else begin
            next_data_ptr = data_ptr;
        end
    endfunction
    wire [7:0] w_data_ptr;
    assign w_data_ptr = next_data_ptr(executing, inst_data, data_ptr);

    function [7:0] next_w_data(
        input       executing,
        input [7:0] inst_data,
        input [7:0] r_data
        );

        if (executing) begin
            if (inst_data == 8'h2B) begin // `+`
                next_w_data = r_data + 1;
            end else if (inst_data == 8'h2D) begin // `-`
                next_w_data = r_data - 1;
            end else begin
                next_w_data = r_data;
            end
        end else begin
            next_w_data = r_data;
        end
    endfunction
    assign w_data = next_w_data(executing, inst_data, r_data);

    // write data memory when inst_data == `+` or `-`
    assign write_data = executing && ((inst_data == 8'h2B) || (inst_data == 8'h2D));

    // uart output when inst_data == `.`
    assign out_flag = executing && (inst_data == 8'h2E);

    always @(posedge clock) begin
        if (reset) begin
            state      <= IDLE_STATE;
            nest_depth <= 0;
            inst_ptr   <= 0;
            data_ptr   <= 0;
        end else begin
            if (state == EXEC_STATE) begin
                state      <= w_state;
                nest_depth <= w_nest_depth;
                inst_ptr   <= w_inst_ptr;
                data_ptr   <= w_data_ptr;
            end else begin // IDLE / WAIT
                state      <= w_state   ;
                nest_depth <= nest_depth; // these program
                inst_ptr   <= inst_ptr  ; // states are
                data_ptr   <= data_ptr  ; // kept intact
            end
        end
    end
endmodule

module sram(clock, reset, write, r_addr, w_addr, in, out);

    input  wire      clock;
    input  wire      reset;
    input  wire      write;
    input  wire[7:0] r_addr;
    input  wire[7:0] w_addr;
    input  wire[7:0] in;
    output wire[7:0] out;

    reg [7:0] mem[0:255];

    assign out = mem[r_addr];

    integer i;
    always @(posedge clock) begin
        if (reset) begin
            for (i=0; i<256; i=i+1)
                mem[i] <= 0;
        end else begin
            if (write) begin
                mem[w_addr] <= in;
            end
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

