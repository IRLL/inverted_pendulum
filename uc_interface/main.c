/********************************************************
 *   File Name: main.c
 *
 *   Description:
 *              main file,
 *
 *
 *********************************************************/

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

/*************************************************************************
 Function Declarations
 ************************************************************************/
void setupChangeNotification(void);

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

    while (1) {

    }

    return 0;
}


//Setup the callback for a Change notification interrupt

void __ISR(_CHANGE_NOTICE_VECTOR, IPL1AUTO) cnHandle(void) {
    asm volatile ("di"); //disable interrupts while processing this function



    IFS1bits.CNIF = 0;
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
 * */