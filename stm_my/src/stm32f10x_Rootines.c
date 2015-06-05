#include "stm32f10x_Define.h"
#include "stm32f10x_Rootines.h"
#include "misc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_iwdg.h"



uint16_t* IWDG_Reset;
uint16_t	SetSpeed;

/*void CheckWithoutPC(void)
{
	if (NMinPCOut>3)
	{
		NMinPCOut=0;
		USART_PC_Configuration(&GD.Control.NFCtr,AdrGD,&GD.SostRS,&NumBlock,9600);
		GD.TControl.Tepl[0].WithoutPC++;
	}
	NMinPCOut++;
}*/



void Control_Init(void) {
/* адреса передачи данных */

        AdrGD[0/*cblHot*/]=&GD.Hot;
        AdrGD[1/*cblControl*/]=&GD.Control;
        AdrGD[2/*cblCal*/]=&GD.Calibr;
        AdrGD[3/*cblTimer*/]=&GD.Hot;
        AdrGD[4/*cblStrategy*/]=&GD.Hot;
        AdrGD[5/*cblMechanic*/]=&GD.Hot;
        ClrDog;
        AdrGD[6/*cblMechConfig*/]=&GD.Hot;
        AdrGD[7/*cblTuneClimate*/]=&GD.Hot;
        AdrGD[8/*cblLevel*/]=&GD.RegBlock[0];
        AdrGD[9/*cblHot*/]=&GD.FanBlock;
        AdrGD[10/*cblHot*/]=&GD.Hot;
/* параметры контроллера */

        GD.Control.rInit[0]=cInit0;
        GD.Control.rInit[1]=cInit1;
        GD.Control.rInit[2]=cInit2;
        GD.Control.rInit[3]=cInit3;//cNowSTepl;
        GD.Control.rInit[4]=cInit4;
}

void Port_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = PIN_REL1_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(PORT_REL1_OUT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = PIN_REL2_OUT;
    GPIO_Init(PORT_REL2_OUT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = PIN_REL3_OUT;
    GPIO_Init(PORT_REL3_OUT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}


void Addr_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = PIN_ADDR;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(PORT_ADDR, &GPIO_InitStructure);
}




void Init_STM32(void) {

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB| RCC_APB2Periph_GPIOD| RCC_APB2Periph_GPIOC, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

//		GD.Hot.OutR=0xf0f0f0;
/*		GD.Hot.Mode=0x0000f8;
		GD.Hot.Pulse[3][0]=10;
		GD.Hot.Pulse[3][1]=15;
		GD.Hot.Pulse[4][0]=15;
		GD.Hot.Pulse[4][1]=20;
		GD.Hot.Pulse[5][0]=20;
		GD.Hot.Pulse[5][1]=25;
		GD.Hot.Pulse[6][0]=30;
		GD.Hot.Pulse[6][1]=40;
		GD.Hot.Pulse[7][0]=40;
		GD.Hot.Pulse[7][1]=50;*/
		USART_OUT_Configuration(19200);
		USART_PC_Configuration(&GD.Control.NFCtr,AdrGD,&GD.SostRS,&NumBlock,9600);
		Control_Init();
		Port_Init();
		Addr_Init();
		InitMainTimer();

		Init_IWDG(&nReset);
		//Init_IWDG(625);

		Check_IWDG();


}


void SetRegOut(uint32_t Condition)
{
	ClrDog;

	PORT_REL1_OUT->ODR&=~PIN_REL1_OUT;
	PORT_REL1_OUT->ODR|=((Condition&0xFF)<<(SM_REL1_OUT));
	//PORT_REL1_OUT->ODR=0xff;
	PORT_REL2_OUT->ODR&=~PIN_REL2_OUT;
	PORT_REL2_OUT->ODR|=(((Condition&0xFF00)>>8)<<(SM_REL2_OUT));
	PORT_REL3_OUT->ODR&=~PIN_REL3_OUT;
	PORT_REL3_OUT->ODR|=(((Condition&0xFF0000)>>16)<<(SM_REL3_OUT));

}

uint32_t SetOutBit(uint32_t maskBit, char nBit)
{
	ClrDog;

	if (GD.Hot.Mode&maskBit)
	{
		if (GD.Hot.OutR&maskBit)
		{
			Work[nBit]=GD.Hot.Pulse[nBit][0];
			GD.Hot.OutR&=~(maskBit);
		}
		if (Work[nBit])
		{
			Work[nBit]--;
			Pause[nBit]=GD.Hot.Pulse[nBit][1];
			return maskBit;
		}
		if (Pause[nBit]==0xFF)
		{
			Pause[nBit]=GD.Hot.Pulse[nBit][1];

		}
		else
		{
			if (Pause[nBit])
				Pause[nBit]--;
			else
				Work[nBit]=GD.Hot.Pulse[nBit][0];
		}
		return 0;
	}
	else if (GD.Hot.OutR&maskBit) return maskBit;;
	return 0;
}


uint32_t DoIt(void)
{
	uint32_t OutReg;
	int16_t i;
	OutReg=0;

	ClrDog;

	for (i=0;i<24;i++)
	{
		OutReg|=SetOutBit(0x01<<i,i);
	}
	return OutReg;
}


void InitMainTimer(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);

    RCC->APB1ENR |= RCC_APB1Periph_TIM2;

//    TIM2->CR1 = /* TIM_CR1_DIR |*/ TIM_CR1_URS;
//    TIM2->CR2 = 0;
//    TIM2->CNT = 0;
    TIM2->PSC = 8000-1; // Clock prescaler;

    TIM2->ARR = 1000; // Auto reload value
    TIM2->SR = 0; // Clean interrups & events flag

    TIM2->DIER = TIM_DIER_UIE; // Enable update interrupts

    /* NVIC_SetPriority & NVIC_EnableIRQ defined in core_cm3.h */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN; // Enable timer
}


void TIM2_IRQHandler(void)
{
	uint32_t inp,i;
	inp=(~GPIOB->IDR)&PIN_ADDR;
	//GD.Hot.OutR=(inp>>SM_ADDR)<<16;
    GD.Control.NFCtr=120+((uint8_t)(inp>>SM_ADDR));
//	GPIO_WriteBit(GPIOC,GPIO_Pin_11,Bit_SET);
    //GD.RegBlock[0].Value=1;
    for (i=0;i<MAX_MODUL_REG;i++)
    {
    	if (GD.RegBlock[i].Type==mtRS485)
    	{
    			GD.FanBlock.Speed=GD.RegBlock[i].Value;
    			break;
    	}
    }

    ClrDog;  //XXX

    //GD.FanBlock.Speed=GD.RegBlock[0].Value;
    SendFans(&GD.FanBlock);
    //GD.Hot.OutR=22;//GD.RegBlock[0].Value;
	SetRegOut(DoIt());
//	GPIO_WriteBit(GPIOA,0xffff,Bit_SET);
//	GPIOA->ODR=0xffff;
	TIM2->SR=0;

}


void Init_IWDG(uint16_t* fIWDG_Reset)
{
	IWDG_Reset=fIWDG_Reset;
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32);
	IWDG_SetReload(80000);

	IWDG_ReloadCounter();
	#ifndef DEBUG
	IWDG_Enable();
	#endif
}

void Check_IWDG(void)
{
		(*IWDG_Reset)++;

}



