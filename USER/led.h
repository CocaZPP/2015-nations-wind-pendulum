#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"

#define LED1ON  	GPIOC->BRR  = GPIO_Pin_6
#define LED1OFF 	GPIOC->BSRR = GPIO_Pin_6
#define LED1Toggle	GPIOC->ODR ^= GPIO_Pin_6

#define LED2ON  	GPIOC->BRR  = GPIO_Pin_7
#define LED2OFF 	GPIOC->BSRR = GPIO_Pin_7
#define LED2Toggle	GPIOC->ODR ^= GPIO_Pin_7

//#define BEEPON		GPIOB->BRR  = GPIO_Pin_8
//#define BEEPOFF		GPIOB->BSRR = GPIO_Pin_8
#define LED PBout(5)// PB5
#define LED1 PBout(6)// PB6
void LED_GPIO_Config(void);

#endif

