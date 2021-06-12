`timescale 1ps / 1ps

// module sim();
//     reg  CLK, RES;
//     reg  write;
//     reg [9:0] addr;
//     reg [7:0] in;
//     wire[7:0] out;
//
//     sram ram1(CLK, RES, write, addr, in, out);
//
//     always begin
//         CLK <= 0; #10;
//         CLK <= 1; #10;
//     end
//
//     initial begin
//         RES <= 1'b0; #30
//         RES <= 1'b1; #30
//         RES <= 1'b0;
//     end
//
//     initial begin
//         write <= 1'b0; # 100
//         write <= 1'b1; #  20
//         write <= 1'b0;
//     end
//
//     initial begin
//         in   <=  8'b01000001;
//         addr <= 10'b00001111;
//     end
//
//     //initial $monitor ($stime, "clock=%b, out=%b, data = %h", CLK, UART_OUT, UART_DATA);
//
// endmodule

module sim();
    reg  CLK;
    reg[3:0] BTN;
    reg  UART_IN;
    wire UART_OUT;
    wire[2:0] LED0;
    wire[2:0] LED1;

//     usb_uart  uart1(CLK, RES, UART_ENABLE, UART_DATA, BUSY, UART_OUT, LED);

    top top0(CLK, BTN, UART_IN, UART_OUT, LED0, LED1);

//     wire [7:0] uart_recv_data;
//     wire       uart_receiving;
//     wire       uart_recv_okay;
//     uart_recv recv(
//         .clock(CLK),
//         .reset(RES),
//         .uart_data(uart_recv_data),
//         .uart_busy(uart_receiving),
//         .uart_okay(uart_recv_okay),
//         .uart_rx(UART_IN),
//         .led(led1)
//         );

    always begin
        CLK = 0; #10;
        CLK = 1; #10;
    end

    initial begin
        UART_IN = 1'b1; # 20000
        UART_IN = 1'b1; # 17360
        // --------------------
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b1; # 17360
        // --------------------
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b1;
    end

    initial begin
        BTN[0] = 1'b0; #30
        BTN[0] = 1'b1; #30
        BTN[0] = 1'b0;
    end

    initial begin
//         BTN[1] = 1'b0; #30
//         BTN[1] = 1'b1; #30
        BTN[1] = 1'b0; # 500000
        BTN[1] = 1'b1;
//         BTN[1] = 1'b1; # 20
//         BTN[1] = 1'b0;
    end

    initial begin
        BTN[2] = 1'b0;
    end
    initial begin
        BTN[3] = 1'b0;
    end

//     initial begin
//         UART_ENABLE = 1'b0; # 90
//         UART_ENABLE = 1'b1; # 20
//         UART_ENABLE = 1'b0;
//     end

    //initial $monitor ($stime, "clock=%b, out=%b, data = %h", CLK, UART_OUT, UART_DATA);

endmodule





