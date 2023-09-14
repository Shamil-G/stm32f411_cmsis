/*
 * SysTick.h
 *
 *  Created on: Aug 30, 2023
 *      Author: sguss
 */

#ifndef INC_SYSTICK_H_
#define INC_SYSTICK_H_

void Delay(uint32_t sleep);

int8_t getTimer(uint32_t sleep);
uint32_t remain_time(int8_t num_timer);
void clearTimer(int8_t num_timer);


#endif /* INC_SYSTICK_H_ */
