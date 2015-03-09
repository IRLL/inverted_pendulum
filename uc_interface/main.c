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
#pragma config POSCMOD = HS // Primary oscillator mode
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
uint8 rx1buff[50] = {0};

/*************************************************************************
 Function Declarations
 ************************************************************************/
void setupUART(void);
void setupPorts(void);
void updateTicks(uint8 data, uint8 *prev_data, int *armData, int *motorData);

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


    AD1PCFGSET = 0xFF;
    setupUART();
    setupPorts();

    //Timer setup
//    TRISDbits.TRISD0 = 0;
//    T1CONbits.TCKPS = 0;
//    PR1 = 1;
//    T1CONbits.ON = 1;
//    IEC0bits.T1IE =1;
//    IPC1bits.T1IP = 7;

    INTEnableSystemMultiVectoredInt();
    INTEnableInterrupts();

    while (1) {
        data = (PORTB & 0b111100) | ((PORTG & 0b110000000) >> 7);
        
        if(data ^ prev_data){
            U1TXREG = data;
            //send_UART(UART1, 1, &data);
        }
        prev_data = data;
    }

    return 0;
}

void setupPorts()
{
    TRISB = TRISB | 0b111100;
    TRISG = TRISG | 0b110000000;
}

//Function that computes ticks
void updateTicks(uint8 data, uint8 *prev_data, int *armData, int *motorData)
{
    //Arm Encoder Data
    uint8 LastState = *(prev_data) >> 4;
        
    switch (data >> 4)
    {
            case 0: //current state = 00
                    //if LastState=01, increment Count (CW)
                    //else LastState=10, decrement Count (CCW)
                    LastState==1 ? *armData++ : *armData--;
                    break;
            case 1: //current state = 01
                    //if LastState=11, increment Count (CW)
                    //else LastState=00, decrement Count (CCW)
                    LastState==3 ? *armData++ : *armData--;
                    break;
            case 2: //current state = 10
                    //if LastState=00, increment Count (CW)
                    //else LastState=11, decrement Count (CCW)
                    LastState==0 ? *armData++ : *armData--;
                    break;
            case 3: //current state = 11
                    //if LastState=10, increment Count (CW)
                    //else LastState=01, decrement Count (CCW)
                    LastState==2 ? *armData++ : *armData--;
                    break;
    }

    //Motor Encoder Data
    uint8 LastState = *(prev_data) & 0b1100 >> 2;
    
    switch (data & 0b1100 >> 2)
    {
            case 0: //current state = 00
                    //if LastState=01, increment Count (CW)
                    //else LastState=10, decrement Count (CCW)
                    LastState==1 ? *motorData++ : *motorData--;
                    break;
            case 1: //current state = 01
                    //if LastState=11, increment Count (CW)
                    //else LastState=00, decrement Count (CCW)
                    LastState==3 ? *motorData++ : *motorData--;
                    break;
            case 2: //current state = 10
                    //if LastState=00, increment Count (CW)
                    //else LastState=11, decrement Count (CCW)
                    LastState==0 ? *motorData++ : *motorData--;
                    break;
            case 3: //current state = 11
                    //if LastState=10, increment Count (CW)
                    //else LastState=01, decrement Count (CCW)
                    LastState==2 ? *motorData++ : *motorData--;
                    break;
    }
    
    *prev_data = data;
}

void setupUART(void)
{
    //Initialize the UART signals
    initialize_UART(2000000, 40000000, UART1, rx1buff, 50, tx1buff, 50, TRUE, TRUE, NULL, NULL);
}

void __ISR(_TIMER_1_VECTOR, IPL7AUTO) Timer_Handler_1(void) {
    asm volatile ("di"); //disable interrupt

    LATDbits.LATD0 = ~LATDbits.LATD0;

    IFS0bits.T1IF = 0; //clear the interrupt flag
    asm volatile ("ei"); //reenable interrupts
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