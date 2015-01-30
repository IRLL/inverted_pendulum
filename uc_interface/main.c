/********************************************************
 *   File Name: main.c
 *
 *   Description:
 *              main file,
 *
 *
 *********************************************************/

/*************************************************************************
 System Includes
 ************************************************************************/
/*This define suppresses warning that new versions of compiler will not support
 * periopheral/ports.h. New teams might need to download legacy compiler in the future
 */
#define _SUPPRESS_PLIB_WARNING
#include <peripheral/ports.h>
/*************************************************************************
 Typedefs
 ************************************************************************/
typedef unsigned int uint;
typedef unsigned short int uint16;
typedef unsigned char uint8;
typedef signed char sint8;

typedef enum {
        FALSE,
        TRUE
    } boolean;
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
 Function Function Declarations
 ************************************************************************/

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

    while (1) {

    }

    return 0;
}


//Setup the callback for a Change notification interrupt

void __ISR(_CHANGE_NOTICE_VECTOR, IPL1AUTO) cnHandle(void) {
    asm volatile ("di"); //disable interrupts will processing this function



    IFS1bits.CNIF = 0;
    asm volatile ("ei"); //reenable interrupts
}

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
/*
Uart_Data* initialize_UART(uint speed, uint pb_clk, Uart which_uart, uint8 *rx_buffer_ptr, uint8 rx_buffer_size,
                           uint8 *tx_buffer_ptr, uint8 tx_buffer_size, boolean tx_en, boolean rx_en,
                           void* rx_callback, void* tx_callback) {

    switch (which_uart) {
        case UART1:
            U1BRG = pb_clk / (16 * speed) - 1; //calculate the proper baud rate

            U1MODEbits.PDSEL = 0; //parity and data size selection bits (no parity, 8bit)

            //setup the rx and tx buffers
            u1.Rx_queue = create_queue(rx_buffer_ptr, rx_buffer_size);
            u1.Tx_queue = create_queue(tx_buffer_ptr, tx_buffer_size);

            IEC1bits.U1TXIE = (tx_en & 0b1); //enable or disable the rx/tx interrupts
            IEC1bits.U1RXIE = (rx_en & 0b1);
            IPC8bits.U1IP = 7; //set interrupt priority to 7

            U1STAbits.UTXISEL = 2; //set tx interrupt to fire when the tx buffer is empty
            U1STAbits.URXISEL = 0; //set rx interrupt to fire whenever a new byte is received

            U1STAbits.UTXEN = (tx_en & 0b1); //enable or disable the rx/tx modules
            U1STAbits.URXEN = (rx_en & 0b1); //enable or disable the rx/tx modules

            U1MODEbits.ON = 1; //enable the UART

            uart_1_tx_callback = tx_callback; //link the callback functions
            uart_1_rx_callback = rx_callback;

            u1.Tx_is_idle = TRUE;

            return &u1;
            break;

        case UART2:
            U2BRG = pb_clk / (16 * speed) - 1; //calculate the proper baud rate

            U2MODEbits.PDSEL = 0; //parity and data size selection bits (no parity, 8bit)

            //setup the rx and tx buffers
            u2.Rx_queue = create_queue(rx_buffer_ptr, rx_buffer_size);
            u2.Tx_queue = create_queue(tx_buffer_ptr, tx_buffer_size);

            IEC1bits.U2TXIE = (tx_en & 0b1); //enable or disable the rx/tx interrupts
            IEC1bits.U2RXIE = (rx_en & 0b1);
            IPC9bits.U2IP = 7; //set interrupt priority to 7

            U2STAbits.UTXISEL = 2; //set tx interrupt to fire when the tx buffer is empty
            U2STAbits.URXISEL = 0; //set rx interrupt to fire whenever a new byte is received

            U2STAbits.UTXEN = (tx_en & 0b1); //enable or disable the rx/tx modules
            U2STAbits.URXEN = (rx_en & 0b1); //enable or disable the rx/tx modules

            U2MODEbits.ON = 1; //enable the UART

            uart_2_tx_callback = tx_callback; //link the callback functions
            uart_2_rx_callback = rx_callback;

            u2.Tx_is_idle = TRUE;
            return &u2;
            break;

        default:
            //some sort of error handling?
            break;
    }

    return NULL;
}
 * */