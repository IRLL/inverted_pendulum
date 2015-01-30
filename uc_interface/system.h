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


#ifdef	__cplusplus
}
#endif

#endif	/* SYSTEM_H */

