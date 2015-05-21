/* 
 * File:   system.h
 * Author: Brandon
 *
 * Created on January 29, 2015, 8:51 PM
 */

#ifndef SYSTEM_H
#define	SYSTEM_H

#ifdef	__cplusplus
extern "C" {
#endif

/*************************************************************************
 System Includes
 ************************************************************************/
/*This define suppresses warning that new versions of compiler will not support
 * peripheral/ports.h. New teams might need to download legacy compiler in the future
 */
#define _SUPPRESS_PLIB_WARNING
#include <p32xxxx.h>
#include <peripheral/ports.h>
#include "UART.h"
/*************************************************************************
 Typedefs
 ************************************************************************/
 #define RESET_BYTE 0x0A

//Universal Variables
uint8 packet[6];

#ifdef	__cplusplus
}
#endif

#endif	/* SYSTEM_H */

