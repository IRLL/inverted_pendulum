/* 
 * File:   Queue.h
 * Author: Ryan Summers
 *
 * Created on December 19, 2014, 1:51 PM
 */

#ifndef QUEUE_H
#define	QUEUE_H

#ifdef	__cplusplus
extern "C" {
#endif

    /*Structure Defintions*/
    typedef unsigned int uint;
    typedef unsigned short int uint16;
    typedef signed short int sint16;
    typedef unsigned char uint8;
    typedef signed char sint8;

    typedef enum {
        FALSE,
        TRUE
    } boolean;

    typedef struct QUEUE{
        uint8 *buffer; //pointer to the queue memory
        uint buffer_size; //size of the supplied buffer
        uint QueueStart; //location of first data point (start of queue)
        uint QueueEnd; //location of the last data point (end of queue)
        uint numStored; //amount of data within the queue
    } Queue;


    /*Queue Functions*/
    Queue create_queue(uint8* buffer, uint buffer_size);
    int enqueue(Queue* queue, uint8* data, uint data_size);
    int dequeue(Queue* queue, uint8* output_data, uint data_size);



#ifdef	__cplusplus
}
#endif

#endif	/* QUEUE_H */

