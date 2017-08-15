#include <stdio.h>
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

  // printf("queue push to: %d !\n", f->head);

  f->pData[f->head] = data;
  f->head++;
  if (f->head >= f->size) {
    f->head=0;
    f->full=true;
  }
  // printf("next to: %d !\n", f->head);
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
  int32_t rtn = sum/end;
  return rtn;
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

