#include "system.h"
#include "UART.h"

//callback functions
void (*uart_1_tx_callback) (void);
void (*uart_1_rx_callback) (void);

void (*uart_2_tx_callback) (void);
void (*uart_2_rx_callback) (void);

Uart_Data u1;
Uart_Data u2;

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

            IEC0bits.U1TXIE = (tx_en & 0b1); //enable or disable the rx/tx interrupts
            IEC0bits.U1RXIE = (rx_en & 0b1);
            IPC6bits.U1IP = 7; //set interrupt priority to 7

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
            IPC8bits.U2IP = 7; //set interrupt priority to 7

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