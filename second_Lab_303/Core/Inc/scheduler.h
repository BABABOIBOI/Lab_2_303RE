/*
 * scheduler.h
 *
 *  Created on: 14 ����� 2017 �.
 *      Author: Maxim
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

/* Includes -----------------------------------------------------------*/
#include <stdint.h>
#include "stm32f3xx.h"

/* Defines ------------------------------------------------------------*/
#define TaskQueueSize   20
#define MainTimerQueueSize  15

/* Type Defines -------------------------------------------------------*/
typedef void (*tptr_t)(void);
typedef struct
{
	tptr_t GoToTask;    // Task function pointer
	uint32_t Time;      // Time delay before task start (in system timer ticks, where one tick - 1 ms)
} ptimer_t;

/* Function prototypes  -----------------------------------------------*/
void InitScheduler(void);
void Idle(void);
void SetTask(tptr_t TS);
void SetTimerTask(tptr_t TS, uint32_t NewTime);
void TaskManager(void);
void TimerService(void);

#endif /* SCHEDULER_H_ */
