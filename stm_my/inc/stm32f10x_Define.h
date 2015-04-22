//Стандартные константы для процессора STM32 в проект

#ifndef __STM32F10X_DEFINE_H
#define __STM32F10X_DEFINE_H

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_iwdg.h"

#define code

#define STM32_UNIT

typedef uint16_t uint;
typedef uint8_t uchar;
typedef uint8_t bit;


#define ClrDog  IWDG_ReloadCounter();
#define SETEA	__enable_irq()
#define CLREA	__disable_irq()

#pragma pack(1)




#define NOP asm("nop")

#define Sound   GPIOA->ODR^=GPIO_Pin_4;



#endif
