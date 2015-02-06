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

// DEVCFG3
// USERID = No Setting

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8

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
void setupChangeNotification(void);
void setupUART(void);

/*************************************************************************
 Main Code
 ************************************************************************/

/*
12.3.3.1CN CONFIGURATION AND OPERATION
The CN pins are configured as follows:
1.Disable CPU interrupts.
2.Set desired CN I/O pin as input by setting corresponding TRISx register bits = 1.
3.Enable the CN Module ON bit (CNCON<15>) = 1.
4.Enable individual CN input pin(s), enable optional pull up(s) or pull down(s).
5.Read corresponding PORTx registers to clear mismatch condition on CN input pins.
6.Configure the CN interrupt priority bits, CNIP<2:0> (IPC6<20:18>), and subpriority bits CNIS<1:0> (IPC6<17:16>).
7.Clear the CN interrupt flag bit, CNIF(IFS1<0>) = 0.
8.Enable the CN interrupt enable bit, CNIE (IEC1<0>) = 1.
9.Enable CPU interrupts.
When a CN interrupt occurs, the user should read the PORTx register associated
 * with the CN pin(s). This will clear the mismatch condition and set up the CN
 * logic to detect the next pin change. The current PORTx value can be compared
 * to the PORTx read value obtained at the last CN interrupt or during initialization,
 * and used to determine which pin changed.The CN pins have a minimum input pulse-width
 * specification. Refer to the ?Electrical Characteristics? chapter of the specific
 * device data sheet to learn more
 */
int main(void) {

    setupChangeNotification();
    setupUART();

    while (1) {

    }

    return 0;
}

//Need interrupt/callback for a received message from the computer/motor controller


//Setup the callback for a Change notification interrupt
void __ISR(_CHANGE_NOTICE_VECTOR, IPL1AUTO) cnHandler(void) {
    asm volatile ("di"); //disable interrupts while processing this function
    uint8 command[1] = {0xE0};
    int uartstatus = 0;
    uint8 data[1] = {0};

    //PORTB bits 2, 3 = Motor encoder
    //      bits 4, 5 = Arm encoder
    //PORTG bits 7, 8 = limit switches

    //parse data into a uint8 array
    data[0] = (PORTB & 0b111100) | ((PORTG & 0b110000000) >> 7);

    //data bits 0, 1 = limit switches
    //     bits 2, 3 = Motor encoder
    //     bits 4, 5 = Arm encoder

    //If a limit is triggered then stop the motor before sending UART signal
    if (data[0] & 0b1)
    {
        send_UART (UART2, 1, command);
    }
    else if (data[0] & 0b10)
    {
        send_UART (UART2, 1, command);
    }
    //Send Bits to computer for decoding
    uartstatus = send_UART(UART2, 1, data); //Send the bits to the computer

    IFS1bits.CNIF = 0; //Reset the Interrupt Flag
    asm volatile ("ei"); //reenable interrupts
}

void setupChangeNotification(void)
{
    int a;

    CNCONbits.ON = 1;
    CNENbits.CNEN4 = 1;
    CNENbits.CNEN5 = 1;
    CNENbits.CNEN6 = 1;
    CNENbits.CNEN7 = 1;
    CNENbits.CNEN8 = 1;
    CNENbits.CNEN10 = 1;
    a = PORTB;
    a = PORTG;
    IPC6bits.CNIP = 1;
    IFS1bits.CNIF = 0;
    IEC1bits.CNIE = 1;

    INTEnableSystemMultiVectoredInt();
    INTEnableInterrupts();
}

//Not sure if it is correct
void setupUART(void)
{
    //Initialize the UART signals
    initialize_UART(115200, 10000000, UART1, rx1buff, 50, tx1buff, 50, TRUE, TRUE, NULL, NULL);
    initialize_UART(115200, 10000000, UART2, rx2buff, 50, tx2buff, 50, TRUE, TRUE, NULL, NULL);
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