
//gpio H File
//written by Yifan Wang
//9/27/2025


#ifndef INC_gpio_H_
#define INC_gpio_H_

#include "main.h"
#include "stm32wbxx_hal.h"

void Haptic_Buzz_Check2STOP(uint32_t* haptic_deadline, uint8_t* haptic_active);
void Haptic_Buzz_Start(uint32_t* haptic_deadline, uint8_t* haptic_active);





#endif /* INC_gpio_H_ */
