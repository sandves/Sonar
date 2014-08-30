#ifndef _QUEUE_H_
#define _QUEUE_H_

#include <stdio.h>
#include <stm32f10x.h>

#define QUEUE_SIZE	128

typedef struct {
	uint16_t pRD, pWR;
	uint8_t q[QUEUE_SIZE];
} Queue;

int QueueFull(Queue *q);
int QueueEmpty(Queue *q);
int Enqueue(Queue *q, uint8_t data);
int Dequeue(Queue *q, uint8_t *data);

#endif
