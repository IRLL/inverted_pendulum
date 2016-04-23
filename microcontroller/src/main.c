/* 
 * File:   main.c
 * Author: james
 *
 * Created on February 19, 2016, 10:59 PM
 */

#include "sublibinal.h"
#include "sublibinal_config.h"
#pragma config ICESEL = ICS_PGx2

void timer_callback();
void adc_callback(ADC_Node node);
void i2c_callback(I2C_Node node);

/*************************************************************************
 Main Function
 ************************************************************************/
int main(void) {

    //structures for configuring peripherals
    Timer_Config timer_config = {0};
    UART_Config uart_config = {0};
    ADC_Config adc_config = {0};
    Packetizer_Config packetizer_config = {0};
    I2C_Config i2c_config = {0};
    uint8 uart_tx_buf[128], uart_rx_buf[128];
    uint8 adc_work_buf[3*sizeof(ADC_Node)], adc_result_buf[3*sizeof(ADC_Node)];
    uint8 i2c_work_buf[3*sizeof(I2C_Node)], i2c_result_buf[3*sizeof(I2C_Node)], i2c_data_buf[32];
    
    //Configure our output pin, RB5
    TRISBbits.TRISB5 = 0; //output

    //setup peripherals
    timer_config.frequency = 50; //Have the timer trigger at a rate of 1Hz
    timer_config.pbclk = 15000000; //The peripheral bus clock is configured to operate at 15MHz
    timer_config.which_timer = Timer_1; //Use Timer 1
    timer_config.callback = &timer_callback; //Hand a callback function for the ISR
    timer_config.enabled = 1; //Enable the Timer
    initialize_Timer(timer_config); //Initialize the timer module
    
    uart_config.speed = 115200;
    uart_config.pb_clk = PB_CLK;
    uart_config.which_uart = UART_CH_1;
    uart_config.tx_pin = Pin_RPB4;
    uart_config.tx_en = 1;
    uart_config.tx_buffer_ptr = uart_tx_buf;
    uart_config.tx_buffer_size = sizeof(uart_tx_buf);
    uart_config.rx_pin = Pin_RPA4;
    uart_config.rx_en = 1;
    uart_config.rx_buffer_ptr = uart_rx_buf;
    uart_config.rx_buffer_size = sizeof(uart_rx_buf);
    
    packetizer_config.control_byte = 0x0A;
    packetizer_config.uart_config = uart_config;
    packetizer_config.which_channel = PACKET_UART_CH_1;
    initialize_packetizer(packetizer_config);
    
    adc_config.result_buffer_ptr = adc_result_buf;
    adc_config.result_buffer_size = sizeof(adc_result_buf);
    adc_config.result_buffer_ptr = adc_work_buf;
    adc_config.result_buffer_size = sizeof(adc_work_buf);
    //adc_config.channels //TODO
    initialize_ADC(adc_config);
    
    i2c_config.channel = I2C_CH_1;
    i2c_config.pb_clk = PB_CLK;
    i2c_config.data_buffer_ptr = i2c_data_buf;
    i2c_config.data_buffer_size = sizeof(i2c_data_buf);
    i2c_config.work_buffer_ptr = i2c_work_buf;
    i2c_config.work_buffer_size = sizeof(i2c_work_buf);
    i2c_config.result_buffer_ptr = i2c_result_buf;
    i2c_config.result_buffer_size = sizeof(i2c_result_buf);
    initialize_I2C(i2c_config);
    
 
    //Global interrupt enable. Do this last!
	enable_Interrupts();

    while (1) 
    {
        
    }

    return 0;
}

void timer_callback(void)
{
    ADC_Node adc_node;
    I2C_Node i2c_node;
    
    adc_node.callback = &adc_callback;
    adc_node.device_id = 0;
    //adc_node.channel = ; //TODO
    read_ADC(adc_node);
    
    i2c_node.channel = I2C_CH_1;
    i2c_node.callback = &i2c_callback;
    i2c_node.data_size = 1;
    i2c_node.device_address = 0x36;
    i2c_node.device_id = 0;
    i2c_node.mode = READ;
    i2c_node.sub_address = 0x0B;
    
    send_I2C(I2C_CH_1, i2c_node);
    
    i2c_node.data_size = 2;
    i2c_node.sub_address = 0x0E;
    send_I2C(I2C_CH_1, i2c_node);    
}

void adc_callback(ADC_Node node)
{
    uint8 data[3];
    
    data[0] = 0x00;
    data[1] = (node.data >> 8);
    data[2] = (node.data & 0xFF);
    
    send_packet(PACKET_UART_CH_1, data, 3);
    
}

void i2c_callback(I2C_Node node)
{
    uint8 data[3];
    
    get_data_I2C(&node, &(data[1]));
    
    switch(node.sub_address)
    {
        case 0x0B: //read status register
            data[0] = 1;
            break;
        case 0x0E: //read angle registers (2))
            data[0] = 2;
            break;
    }
    
    send_packet(PACKET_UART_CH_1, data, 3);
}
