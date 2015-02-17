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
            U1BRG = pb_clk / (4 * speed) - 1; //calculate the proper baud rate

            U1MODEbits.PDSEL = 0; //parity and data size selection bits (no parity, 8bit)

            //setup the rx and tx buffers
            u1.Rx_queue = create_queue(rx_buffer_ptr, rx_buffer_size);
            u1.Tx_queue = create_queue(tx_buffer_ptr, tx_buffer_size);

            IEC0bits.U1TXIE = (tx_en & 0b1); //enable or disable the rx/tx interrupts
            IEC0bits.U1RXIE = (rx_en & 0b1);
            IPC6bits.U1IP = 7; //set interrupt priority to 7

            U1MODEbits.BRGH = 1; //set for high-speed mode

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

int send_UART(Uart channel, uint8 data_size, uint8 *data_ptr) {
    int status;
    //we need to place the provided data onto the Tx queue
    switch (channel) {
        case UART1:
            status = enqueue(&(u1.Tx_queue), data_ptr, data_size);
            if (u1.Tx_is_idle) { //if the tx is idle, force-start it
                IEC0bits.U1TXIE = 1;
                IFS0bits.U1TXIF = 1;
            }
            break;
        case UART2:
            status = enqueue(&(u2.Tx_queue), data_ptr, data_size);
            if (u2.Tx_is_idle) { ////if the tx is idle, force-start it
                IEC1bits.U2TXIE = 1;
                IFS1bits.U2TXIF = 1;
            }
            break;
        default:
            status = 1; //return failure
            break;

    }
    return status;
}

int receive_UART(Uart channel, uint8 data_size, uint8 *data_ptr) {
    int status;
    //we need to read the specified data from the rx queue
    switch (channel) {
        case UART1:
            status = dequeue(&(u1.Rx_queue), data_ptr, data_size);
            break;
        case UART2:
            status = dequeue(&(u2.Rx_queue), data_ptr, data_size);
            break;
        default:
            status = 1; //return failure
            break;
    }
    return status; //return success
}

void __ISR(_UART_1_VECTOR, IPL7AUTO) Uart_1_Handler(void) {
    uint8 received, transmit;
    asm volatile ("di"); //disable interrupt

    if (IFS0bits.U1RXIF) { //if the interrupt flag of RX is set


        //we have received information - pop that information off the channel
        //push that data onto our received queue
        received = U1RXREG;
        enqueue(&(u1.Rx_queue), &received, 1);

        if (uart_1_rx_callback != NULL) {
            uart_1_rx_callback(); //call additional ISR functionality
        }

        //now clear the interrupt flag
        IFS0bits.U1RXIF = 0;
    }
    if (IFS0bits.U1TXIF) { //if the interrupt flag of TX is set
        u1.Tx_is_idle = 0; //tx is not idle
        //if the transmit queue is empty
        if (u1.Tx_queue.numStored == 0) {
            //our buffer is empty
            //we need to disable the interrupts
            IEC0bits.U1TXIE = 0;
            //and set the queue to idle
            u1.Tx_is_idle = 1;

        } else {
            //we have data to transmit - pop that data off the queue
            //store popped data into the transmit registry
            while (!dequeue(&(u1.Tx_queue), &transmit, 1) && !U1STAbits.UTXBF) { //while we are dequeuing data AND the transmit buffer is not full
                //write the data to the buffer
                U1TXREG = transmit;
            } //write data until the queue is empty or the registry is full

            if (uart_1_tx_callback != NULL) {
                uart_1_tx_callback(); //call additional ISR functionality
            }
            //now clear the interrupt flag
            IFS0bits.U1TXIF = 0;
        }
    }

    asm volatile ("ei"); //reenable interrupts
}

void __ISR(_UART_2_VECTOR, IPL7AUTO) Uart_2_Handler(void) {
    uint8 received, transmit;
    asm volatile ("di"); //disable interrupt

    if (IFS1bits.U2RXIF) { //if the interrupt flag of RX is set


        //we have received information - pop that information off the channel
        //push that data onto our received queue
        received = U2RXREG;
        enqueue(&(u2.Rx_queue), &received, 1);

        if (uart_2_rx_callback != NULL) {
            uart_2_rx_callback(); //call additional ISR functionality
        }

        //now clear the interrupt flag
        IFS1bits.U2RXIF = 0;
    }
    if (IFS1bits.U2TXIF) { //if the interrupt flag of TX is set
        u2.Tx_is_idle = 0; //tx is not idle

        //if the transmit queue is empty
        if (u2.Tx_queue.numStored == 0) {
            //our buffer is empty
            //we need to disable the interrupts
            IEC1bits.U2TXIE = 0;
            //and set the queue to idle
            u2.Tx_is_idle = 1;

        } else {
            //we have data to transmit - pop that data off the queue
            //store popped data into the transmit registry
            while (!dequeue(&(u2.Tx_queue), &transmit, 1) && !U2STAbits.UTXBF) { //while we are dequeuing data AND the transmit buffer is not full
                //write the data to the buffer
                U2TXREG = transmit;
            } //write data until the queue is empty or the registry is full

            if (uart_2_tx_callback != NULL) {
                uart_2_tx_callback(); //call additional ISR functionality
            }
            //now clear the interrupt flag
            IFS1bits.U2TXIF = 0;
        }
    }

    asm volatile ("ei"); //reenable interrupts
}
