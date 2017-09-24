#ifndef _QUEUE_BUFFER_H_
#define _QUEUE_BUFFER_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

typedef enum algorithm{
    ALG_MEAN_VALUE,
    ALG_MEDIAN_VALUE,
} algorithm_e;

typedef struct {
	int32_t head;
	bool full;
    int32_t size;
    int32_t *pData;
} queue_buffer_t;

bool queue_buffer_init(queue_buffer_t* pqueue, int32_t* pBuf, int32_t size);
void queue_buffer_push(queue_buffer_t* pqueue, int32_t data);
int32_t queue_last(queue_buffer_t* f);
int32_t queue_get_value(queue_buffer_t* pqueue, algorithm_e algorithm);
void queue_dump(queue_buffer_t* pqueue);
void queue_test();

#endif  /*_QUEUE_BUFFER_H_*/
