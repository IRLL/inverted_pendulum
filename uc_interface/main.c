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
int main (void)
{

    
    
}





#endif	/* SYSTEM_H */

