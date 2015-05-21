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
uint8 tx1buff[10] = {0};
uint8 packet[6] = {0};
sint16 ArmCount = 0;
sint16 MotorCount = 60000;
uint8 data = 0;
uint8 prev_data = 0;

/*************************************************************************
 Function Declarations
 ************************************************************************/
void setupUART(void);
void setupPorts(void);
void updateTicks(void);
void packetize(void);
void setupTimer(void);

/*************************************************************************
 Main Code
 ************************************************************************/
int main(void) {
    uint8 LastState;
    uint8 CurrentState;

    setupUART();
    setupPorts();

    //Timer setup
    //Need the timer for periodic transmission of data ot the computer
    setupTimer();


    INTEnableSystemMultiVectoredInt();
    INTEnableInterrupts();

    while (1) {
        data = (PORTB & 0b111100);

        if (data ^ prev_data) {
            /*
             * Arm Encoder Data
             */

            //save the previous state of the arm
            LastState = (prev_data >> 4) & 0b11;
            CurrentState = (data >> 4) & 0b11;

            if (CurrentState != LastState) {
                switch (CurrentState) {
                    case 0: //current state = 00
                        //if LastState=01, increment Count (CW)
                        //else LastState=10, decrement Count (CCW)
                        LastState == 1 ? ArmCount++ : ArmCount--;
                        break;
                    case 1: //current state = 01
                        //if LastState=11, increment Count (CW)
                        //else LastState=00, decrement Count (CCW)
                        LastState == 3 ? ArmCount++ : ArmCount--;
                        break;
                    case 2: //current state = 10
                        //if LastState=00, increment Count (CW)
                        //else LastState=11, decrement Count (CCW)
                        LastState == 0 ? ArmCount++ : ArmCount--;
                        break;
                    case 3: //current state = 11
                        //if LastState=10, increment Count (CW)
                        //else LastState=01, decrement Count (CCW)
                        LastState == 2 ? ArmCount++ : ArmCount--;
                        break;
                }


                if (ArmCount < 0) {
                    ArmCount = 2000;
                } else if (ArmCount > 2000) {
                    ArmCount = 0;
                }
            }

            /*
             * Motor Encoder Data Processing
             */

            //save the previous state of the motor
            LastState = (prev_data & 0b01100) >> 2;
            CurrentState = (data & 0b01100) >> 2;

            if (CurrentState != LastState) {
                switch (CurrentState) {
                    case 0: //current state = 00
                        //if LastState=01, increment Count (CW)
                        //else LastState=10, decrement Count (CCW)
                        LastState == 1 ? MotorCount++ : MotorCount--;
                        break;
                    case 1: //current state = 01
                        //if LastState=11, increment Count (CW)
                        //else LastState=00, decrement Count (CCW)
                        LastState == 3 ? MotorCount++ : MotorCount--;
                        break;
                    case 2: //current state = 10
                        //if LastState=00, increment Count (CW)
                        //else LastState=11, decrement Count (CCW)
                        LastState == 0 ? MotorCount++ : MotorCount--;
                        break;
                    case 3: //current state = 11
                        //if LastState=10, increment Count (CW)
                        //else LastState=01, decrement Count (CCW)
                        LastState == 2 ? MotorCount++ : MotorCount--;
                        break;
                }
            }
        }
        prev_data = data;
    }

    return 0;
}

/*************************************************************************
 Function Definitions
 ************************************************************************/

//setup the required ports for digital inputs

void setupPorts(void) {
    //Ensure the Analog Pins are set to digital inputs
    AD1PCFGSET = 0xFF;

    TRISBSET = 0b111100;
    TRISGSET = 0b110000000;
}

void setupTimer(void) {
    TRISDbits.TRISD0 = 0;
    T1CONbits.TCKPS = 0b11; //0b11 is 1:256 clock prescale
    PR1 = 2000;
    T1CONbits.ON = 1;
    IEC0bits.T1IE = 1;
    IPC1bits.T1IP = 7;

}


//Setup the UART communication

void setupUART(void) {
    //Initialize the UART signals
    initialize_UART(115200, 40000000, UART1, 0, 0, tx1buff, sizeof (tx1buff), TRUE, TRUE, NULL, NULL);
}

void packetize(void) {
    //Packetize the data to send
    packet[0] = 0x0A;
    packet[1] = ArmCount >> 8; //High data of ArmCount
    packet[2] = ArmCount & 0xFF; //Low data of ArmCount
    packet[3] = MotorCount >> 8;
    packet[4] = MotorCount & 0xFF;
    packet[5] = ((PORTG & 0b110000000) >> 7) & 3;
}

void __ISR(_TIMER_1_VECTOR, IPL7AUTO) Timer_Handler_1(void) {
    int i = 0;
    asm volatile ("di"); //disable interrupt

    //LATDbits.LATD0 = ~LATDbits.LATD0;
    packetize();
    
    //Load in the first 4 bytes
    for (i = 0; i < 4; ++i)
    {
        U1TXREG = packet[i];
    }
    
    //Enable the U1 TX interrupt
    IEC0bits.U1TXIE = 1;
    
    //Send the latest values to the computer
    //send_UART(UART1, sizeof (packet), packet);

    IFS0bits.T1IF = 0; //clear the interrupt flag
    asm volatile ("ei"); //re-enable interrupts
}
