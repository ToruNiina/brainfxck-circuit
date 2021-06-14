`timescale 1ps / 1ps

module sim();
    reg  CLK;
    reg[3:0] BTN;
    reg  UART_IN;
    wire UART_OUT;
    wire[2:0] LED0;
    wire[2:0] LED1;

    top top0(CLK, BTN, UART_IN, UART_OUT, LED0, LED1);

    always begin
        CLK = 0; #10;
        CLK = 1; #10;
    end

//  + 0x2B 0010'1011
//  - 0x2D 0010'1101
//  > 0x3E 0011'1110
//  < 0x3C 0011'1100
//  [ 0x5B 0101'1011
//  ] 0x5D 0101'1101
//  . 0x2E 0010'1110

    initial begin
        BTN[1] = 1'b0;
        UART_IN = 1'b1; # 20000
        // ---------------------
        UART_IN = 1'b0; # 17360

        UART_IN = 1'b1; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b1; # 17360 // +
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b0; # 17360

        UART_IN = 1'b1; # 17360
        // ---------------------
        UART_IN = 1'b0; # 17360

        UART_IN = 1'b1; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360 // +
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b0; # 17360

        UART_IN = 1'b1; # 17360
        // ---------------------
        UART_IN = 1'b0; # 17360

        UART_IN = 1'b0; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360 // .
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b0; # 17360

        UART_IN = 1'b1; # 17360
        // ---------------------
        UART_IN = 1'b0; # 17360

        UART_IN = 1'b1; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b1; # 17360 // [
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360

        UART_IN = 1'b1; # 17360
        // ---------------------
        UART_IN = 1'b0; # 17360

        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b1; # 17360 // -
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b0; # 17360

        UART_IN = 1'b1; # 17360
        // ---------------------
        UART_IN = 1'b0; # 17360

        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b1; # 17360 // ]
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360
        UART_IN = 1'b1; # 17360
        UART_IN = 1'b0; # 17360

        UART_IN = 1'b1; # 17360
        //---------------------
        UART_IN = 1'b1; # 17360
        BTN[1] = 1'b1;
    end

    initial begin
        BTN[0] = 1'b0; #30
        BTN[0] = 1'b1; #30
        BTN[0] = 1'b0;
    end

//     initial begin
//         BTN[1] = 1'b0; #30
//         BTN[1] = 1'b1; #30
//         BTN[1] = 1'b0; # 500000
//         BTN[1] = 1'b1;
//         BTN[1] = 1'b1; # 20
//         BTN[1] = 1'b0;
//     end

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





