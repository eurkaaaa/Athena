/*
 * led.h
 *
 *  Created on: May 27, 2024
 *      Author: Lenovo
 */

#ifndef BSP_LED_LED_H_
#define BSP_LED_LED_H_
#include "main.h"

#define LED(x)				x? HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET) : \
		                       HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET)
#define LED_TOGGLE()        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin)
#endif /* BSP_LED_LED_H_ */
