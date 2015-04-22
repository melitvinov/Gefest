#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

//---------------------- Подпрограммы для функционирования STM32 -------------------------------


#define cInit0		1
#define cInit1		3
#define cInit2		3
#define cInit3		3
#define cInit4		140


#define MAX_MODUL_REG	8
#define mtRS485			1

#define NumCtr          1

#define PIN_REL1_OUT	0xFF
#define PIN_REL2_OUT	0xFF
#define PIN_REL3_OUT	0xFF

#define SM_REL1_OUT		0
#define SM_REL2_OUT		0
#define SM_REL3_OUT		0

#define PORT_REL1_OUT	GPIOA
#define PORT_REL2_OUT	GPIOB
#define PORT_REL3_OUT	GPIOC

#define PORT_ADDR		GPIOB
#define SM_ADDR			12
#define PIN_ADDR		0xf000


void Init_IWDG(uint16_t* fIWDG_reset);
void Check_IWDG(void);
void Init_STM32(void);
void InitMainTimer(void);

