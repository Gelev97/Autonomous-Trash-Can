#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H
#include "sys.h"


typedef struct QNode{
	u8 data;
	struct QNode* next;
} QNode;

//队列的结构，嵌套
typedef struct {
	QNode *Hand;
	QNode *Tail;
} LinkQueue;


void USART2_Init(u32 bound);
void USART2_Send_Data(u8 Dat);
u8 InspectQueue(void);
u8 OutQueue(void);

#endif
