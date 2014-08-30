#include "queue.h"

/**
 * Circular buffer
 */

int QueueFull(Queue *q)
{
	return (((q->pWR + 1) % QUEUE_SIZE) == q->pRD);
}

int QueueEmpty(Queue *q)
{
	return (q->pWR == q->pRD);
}

int Enqueue(Queue *q, uint8_t data)
{
	if (QueueFull(q))
		return 0;
	else {
		q->q[q->pWR] = data;
		q->pWR = ((q->pWR + 1) ==
				QUEUE_SIZE) ? 0 : q->pWR + 1;
	}
	return 1;
}

int Dequeue(Queue *q, uint8_t *data)
{
	if (QueueEmpty(q))
		return 0;
	else
	{
		*data = q->q[q->pRD];
		q->pRD = ((q->pRD + 1) ==
				QUEUE_SIZE) ? 0 : q->pRD + 1;
	}
	return 1;
}
