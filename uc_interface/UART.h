/* 
 * File:   UART.h
 * Author: Ryan Summers
 *
 * Created on December 17, 2014, 4:22 PM
 */

#ifndef UART_H
#define	UART_H

#ifdef	__cplusplus
extern "C" {
#endif

    /*Includes*/
#define _SUPPRESS_PLIB_WARNING
#include <peripheral/ports.h>
#include "Queue.h"

    /*Enum Defintions*/
#define UART_BUFF_SIZE 4

    /*Object Defintions*/

    typedef struct UART_DATA {
        //Rx queue
        Queue Rx_queue;
        //Tx queue
        Queue Tx_queue;
        //idle information
        boolean Tx_is_idle;
    } Uart_Data;

    typedef enum {
        UART1,
        UART2
    } Uart;

    /*Function Prototypes*/
    Uart_Data* initialize_UART(uint speed, uint pb_clk, Uart which_uart, uint8 *rx_buffer_ptr, uint8 rx_buffer_size,
            uint8 *tx_buffer_ptr, uint8 tx_buffer_size, boolean tx_en, boolean rx_en,
            void* rx_callback, void* tx_callback);
    int send_UART(Uart channel, uint8 data_size, uint8 *data_ptr);
    int receive_UART(Uart channel, uint8 data_size, uint8 *data_ptr);
#ifdef	__cplusplus
}
#endif

#endif	/* UART_H */

