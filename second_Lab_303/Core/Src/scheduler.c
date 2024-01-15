/*
 * scheduler.c
 *
 *  Created on: 14 марта 2017 г.
 *      Author: Maxim
 */
#include "scheduler.h"

__IO static tptr_t  TaskQueue[TaskQueueSize+1];       // Task queue
__IO static ptimer_t MainTimer[MainTimerQueueSize+1]; // Timer queue

// Scheduler initialization
inline void InitScheduler(void)
  {
    uint8_t index;

    for (index = 0; index != TaskQueueSize + 1; index++)      // Fill task queue with  Idle task
      {
        TaskQueue[index] = Idle;
      }

    for (index = 0; index != MainTimerQueueSize + 1; index++) // Fill timer queue with zeros
      {
        MainTimer[index].GoToTask = Idle;
        MainTimer[index].Time = 0;
      }
  }

// Idle task
inline void  Idle(void)
{

}


// Функция установки задачи в очередь. Передаваемый параметр - указатель на функцию
// Function sets the task to queue. The argument is task pointer
void SetTask(tptr_t TS)
  {
    uint8_t index = 0;
    uint8_t interrupts_enable = ~__get_PRIMASK(); // Get the state of interrupts
    // If interrupts are enabled, disable them (Если прерывания разрешены, то запрещаем их.)
    if (interrupts_enable)
      {
        __disable_irq();
      }
    // Go thru task queue. Looking for the empty cell (Idle task)
    // (Прочесываем очередь задач на предмет свободной ячейки)
    while (TaskQueue[index] != Idle)
      {
        index++;
        // If the queue is full then return without result.
        // (Если очередь переполнена то выходим не солоно хлебавши)
        if (index == TaskQueueSize + 1)
          {
            if (interrupts_enable) __enable_irq();    // If interrupts were enabled, enable them back
            return;
          }
      }
    // If there is free cell in task queue set the task in this cell
    TaskQueue[index] = TS;
    if (interrupts_enable) __enable_irq();            // If interrupts were enabled, enable them back
  }

//Функция установки задачи по таймеру. Передаваемые параметры - указатель на функцию, и время задержки
// Function sets the task to timer queue. The arguments are task pointer and time delay (in ms)
void SetTimerTask(tptr_t TS, uint32_t NewTime)
  {
    uint8_t index = 0;
    uint8_t interrupts_enable = ~__get_PRIMASK();     // Get the state of interrupts
    // If interrupts are enabled, disable them (Если прерывания разрешены, то запрещаем их)
    if (interrupts_enable)
      {
        __disable_irq();
      }
    // Go thru timer queue (Прочесываем очередь таймеров)
    for (index = 0; index != MainTimerQueueSize + 1; ++index)
      {
    		// If there is the same task in the queue (Если уже есть запись с таким адресом)
        if (MainTimer[index].GoToTask == TS)
          {
            MainTimer[index].Time = NewTime;          // Rewrite the time delay (Перезаписываем ей выдержку)
            if (interrupts_enable) __enable_irq();    // If interrupts were enabled, enable them back
            return;                                   // Return from function!
          }
      }
    // If there isn't same task in the queue then look for empty cell
    // (Если не находим похожий таймер, то ищем любой пустой)
    for (index = 0; index != MainTimerQueueSize + 1; ++index)
      {
        if (MainTimer[index].GoToTask == Idle)
          {
            MainTimer[index].GoToTask = TS;             // Set the task pointer (Заполняем поле перехода задачи)
            MainTimer[index].Time = NewTime;            // Set the time delay (И поле выдержки времени)
            if (interrupts_enable) __enable_irq();      // If interrupts were enabled, enable them back
            return;                                     // Return from function!
          }
      }
  }


// Scheduler (Task manager)
inline void TaskManager(void)
  {
    uint8_t index = 0;
    tptr_t GoToTask = Idle;   // Initialize the pointer to Idle (Инициализируем переменные)

    __disable_irq();          // Disable interrupts (Запрещаем прерывания!!!)
    GoToTask = TaskQueue[0];  // Get the first item from the queue (Хватаем первое значение из очереди)

    if (GoToTask == Idle)     // If there is empty cell (Если там пусто)
      {
        __enable_irq();       // Enable interrupts (Разрешаем прерывания)
        (Idle)();             // Go to default task Idle (Переходим на обработку пустого цикла)
      }
    else
      {
    		// In other case shift the queue (В противном случае сдвигаем всю очередь)
        for (index = 0; index != TaskQueueSize; index++)
          {
            TaskQueue[index] = TaskQueue[index + 1];
          }
        // Set the Idle task in the last cell of the queue (В последнюю запись пихаем затычку)
        TaskQueue[TaskQueueSize] = Idle;

        __enable_irq();       // Enable interrupts (Разрешаем прерывания)
        (GoToTask)();         // Go to the task (Переходим к задаче)
      }
  }

// Служба таймеров ядра. Должна вызываться из прерывания раз в 1мс.
// Хотя время можно варьировать в зависимости от задачи
// Timer service
inline void TimerService(void)
  {
    uint8_t index;
    // Go thru timer queue (Прочесываем очередь таймеров)
    for (index = 0; index != MainTimerQueueSize + 1; index++)
      {
    		// If the cell is empty go to the next iteration (Если нашли пустышку - щелкаем следующую итерацию)
        if (MainTimer[index].GoToTask == Idle) continue;
        // If timer delay is not empty then decrement the timer (Если таймер не выщелкал, то щелкаем еще раз)
        if (MainTimer[index].Time > 1)
          {
            MainTimer[index].Time--;
          }
        else
          {
        		// If the time is zero, set the task to the task queue (Дощелкали до нуля? Пихаем в очередь задачу)
            SetTask(MainTimer[index].GoToTask);
            MainTimer[index].GoToTask = Idle;   // Mark cell as empty (А в ячейку пишем затычку)
          }
      }
  }

