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
}

int32_t queue_average(queue_buffer_t* f)
{
  CHECK_NULL(f)

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

void queue_dump(queue_buffer_t* f)
{
  CHECK_NULL(f)

  uint32_t end = f->head;
  if (f->full) {
    end = f->size;
  }

  if (end == 0) {
    printf("empty !\n");
    return;
  }

  printf("queue size: %d !\n", end);

  for ( int i=0; i<end; i++ ) {
    printf("%d: %d!\n", i, f->pData[i]);
  }
}

void queue_test()
{
    queue_buffer_t qbuffer;
    int32_t dataBuffer[3];

    // Queue Buffer init
    memset(dataBuffer,0,sizeof(dataBuffer));
    queue_buffer_init(&qbuffer, dataBuffer, 3);

    queue_buffer_push(&qbuffer, 2);
    queue_buffer_push(&qbuffer, 1);
    queue_buffer_push(&qbuffer, 1);

    printf("QUEUE BUFFER: %d !!!\n", queue_average(&qbuffer));
}
