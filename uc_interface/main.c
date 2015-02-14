/********************************************************
 *   File Name: main.c
 *
 *   Description:
 *              main file,
 *
 *
 *********************************************************/

//Clock settings

// PIC32MX340F512H Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

/* Oscillator Settings
*/
#pragma config FNOSC = PRIPLL // Oscillator selection
#pragma config POSCMOD = EC // Primary oscillator mode
#pragma config FPLLIDIV = DIV_2 // PLL input divider
#pragma config FPLLMUL = MUL_20 // PLL multiplier
#pragma config FPLLODIV = DIV_1 // PLL output divider
#pragma config FPBDIV = DIV_2 // Peripheral bus clock divider
#pragma config FSOSCEN = OFF // Secondary oscillator enable


//includes
#include "system.h"

/*************************************************************************
 Constants
 ************************************************************************/

/*************************************************************************
 Structure Definitions
 ************************************************************************/

/*************************************************************************
 Enums
 ************************************************************************/

/*************************************************************************
 Global Variables
 ************************************************************************/
uint8 tx1buff[50] = {0};
uint8 tx2buff[50] = {0};
uint8 rx1buff[50] = {0};
uint8 rx2buff[50] = {0};

/*************************************************************************
 Function Declarations
 ************************************************************************/
void setupUART(void);

/*************************************************************************
 Main Code
 ************************************************************************/

/*
    uint8 command = 0xE0;
    if ((data & 0b1) == 0b1)
    {
       // send_UART (UART2, 1, command);
        U2TXREG = command;
    }
    else if ((data & 0b10) == 0b10)
    {
        //send_UART (UART2, 1, command);
        U2TXREG = command;
    }
 */
int main(void) {
    uint8 data;
    uint8 prev_data;

    setupUART();

    INTEnableSystemMultiVectoredInt();
    INTEnableInterrupts();

    while (1) {
        data = (PORTB & 0b111100) | ((PORTG & 0b110000000) >> 7);
        
        if(data ^ prev_data){
            send_UART(UART1, 1, &data);
        }
        prev_data = data;
    }

    return 0;
}

void setupUART(void)
{
    //Initialize the UART signals
    initialize_UART(115200, 10000000, UART1, rx1buff, 50, tx1buff, 50, TRUE, TRUE, NULL, NULL);
    initialize_UART(1500000, 10000000, UART2, rx2buff, 50, tx2buff, 50, TRUE, TRUE, NULL, NULL);
}

/*
void UartStuff(uint speed, uint pb_clk, boolean tx_en, boolean rx_en) {
    U1BRG = pb_clk / (16 * speed) - 1; //calculate the proper baud rate

    U1MODEbits.PDSEL = 0; //parity and data size selection bits (no parity, 8bit)


    IEC0bits.U1TXIE = (tx_en & 0b1); //enable or disable the rx/tx interrupts
    IEC0bits.U1RXIE = (rx_en & 0b1);
    IPC6bits.U1IP = 7; //set interrupt priority to 7

    U1STAbits.UTXISEL = 2; //set tx interrupt to fire when the tx buffer is empty
    U1STAbits.URXISEL = 0; //set rx interrupt to fire whenever a new byte is received

    U1STAbits.UTXEN = (tx_en & 0b1); //enable or disable the rx/tx modules
    U1STAbits.URXEN = (rx_en & 0b1); //enable or disable the rx/tx modules

    U1MODEbits.ON = 1; //enable the UART
}
*/