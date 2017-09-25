#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "queue_buffer.h"

#define CHECK_NULL(x)	if ((x) == NULL) return false;

bool queue_buffer_init(queue_buffer_t* f, int32_t* pBuf, int32_t size)
{
  CHECK_NULL(f)
  f->head = 0;
  f->full = false;
  f->size = size;
  f->pData = pBuf;
  return true;
}

void queue_buffer_push(queue_buffer_t* f, int32_t data)
{
  CHECK_NULL(f)

  f->pData[f->head] = data;
  f->head++;
  if (f->head >= f->size) {
    f->head=0;
    f->full=true;
  }
  // printf("push:");
  // queue_dump(f);
}

int32_t queue_last(queue_buffer_t* f)
{
  CHECK_NULL(f)

  if (f->head == 0) {
    if (f->full) {
      return f->pData[f->size-1];
    }else
      return 0;
  }else{
    return f->pData[f->head-1];
  }
}

static int32_t queue_average(queue_buffer_t* f)
{
  CHECK_NULL(f)

  // printf("avg:");
  // queue_dump(f);

  int32_t end = f->head;
  if (f->full) {
    end = f->size;
  }
  if (end == 0) return 0;

  int32_t sum=0;
  for ( int i=0; i<end; i++ ) {
    sum+=f->pData[i];
  }

  //rounding
  return (sum*10/end+5)/10;
}

static int32_t queue_median(queue_buffer_t* f)
{
  CHECK_NULL(f)

  // median value must has more than 3 value
  if (f->size < 3) return 0;

  int32_t end = f->head;
  if (f->full) {
    end = f->size;
  }
  if (end == 0) return 0;
  if (end < 3) return 0;

  int32_t sum=0;
  int32_t max=-10000000;
  int32_t min=10000000;
  for ( int i=0; i<end; i++ ) {
    if (f->pData[i] > max) { max = f->pData[i]; }
    if (f->pData[i] < min) { min = f->pData[i]; }
    sum += f->pData[i];
  }

  sum -= (max + min) ;

  //rounding
  return sum/(end-2);
}

int32_t queue_get_value(queue_buffer_t* pqueue, algorithm_e algorithm)
{
    if (algorithm == ALG_MEAN_VALUE) {
        return queue_average(pqueue);
    } else if (algorithm == ALG_MEDIAN_VALUE) {
        return queue_median(pqueue);
    }
    return 0;
}

void queue_dump(queue_buffer_t* f)
{
  CHECK_NULL(f)

  int32_t end = f->head;
  if (f->full) {
    end = f->size;
  }

  if (end == 0) {
    printf("empty !\n");
    return;
  }

  printf("queue(%d):", end);

  for ( int i=0; i<end; i++ ) {
    printf("    %d", f->pData[i]);
  }
  printf("\n");
}

void queue_test()
{
    queue_buffer_t qbuffer;
    int32_t dataBuffer[4];

    // Queue Buffer init
    memset(dataBuffer,0,sizeof(dataBuffer));
    queue_buffer_init(&qbuffer, dataBuffer, 4);

    queue_buffer_push(&qbuffer, 803);
    queue_buffer_push(&qbuffer, 804);
    queue_buffer_push(&qbuffer, 804);
    queue_buffer_push(&qbuffer, 804);

    queue_dump(&qbuffer);
    printf("median value: %d !!!\n", queue_median(&qbuffer));
    while(1) {}
}
