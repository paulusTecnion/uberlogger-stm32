#ifndef _MSG_H
#define _MSG_H

#include "stdint.h"
#include "string.h"

#define QUEUE_SIZE 2

typedef struct Message {
  uint8_t event;
  uint8_t message_len;
  uint8_t message_payload[4];
} Message_t;

typedef struct MessageQueue {
  int front;
  int rear;
  int count;
  struct Message messages[QUEUE_SIZE];
} MessageQueue_t;

void msgq_enqueue(MessageQueue_t *queue, Message_t message);
Message_t msgq_dequeue(MessageQueue_t *queue);
uint8_t msgq_is_empty(MessageQueue_t *queue);
uint8_t msgq_is_full( MessageQueue_t *queue);

#endif
