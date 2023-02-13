#include "msg.h"




void msgq_enqueue(MessageQueue_t *queue, Message_t message) {
  if (queue->count >= QUEUE_SIZE) {
    // Handle error: queue is full
  } else {
    queue->messages[queue->rear] = message;
    queue->rear = (queue->rear + 1) % QUEUE_SIZE;
    queue->count++;
  }
}

Message_t msgq_dequeue(MessageQueue_t *queue) {
  if (queue->count <= 0) {
    // Handle error: queue is empty
	  Message_t t;
	  t.event = 255;
	  return t;
  } else {
    struct Message message = queue->messages[queue->front];
    queue->front = (queue->front + 1) % QUEUE_SIZE;
    queue->count--;
    return message;
  }
}

uint8_t msgq_is_empty(MessageQueue_t *queue) {
  return queue->count == 0;
}

uint8_t msgq_is_full( MessageQueue_t *queue) {
  return queue->count == QUEUE_SIZE;
}

