//Работа порта RS-485
//#include "stm_Debugs.h"


#ifndef __STM32F10X_RS485MASTER_H
#define __STM32F10X_RS485MASTER_H

#define ERR_MASTER_NOANSWER		1
#define ERR_MASTER_WRONGANSWER	2
#define ERR_MASTER_TXHEAD		3
#define ERR_MASTER_HEADNOSUM	4
#define ERR_MASTER_HEADWRSUM	5
#define ERR_MASTER_TXDATA		6
#define ERR_MASTER_DATANOSUM	7
#define ERR_MASTER_DATAWRSUM	8
#define ERR_MASTER_RXDATA		9
#define ERR_MASTER_RXDATA		9

#define ERR_MODULE_NOSENSOR        	0x01
#define ERR_MODULE_LINK				0x02
#define ERR_MODULE_CHKSUM			0x04
#define ERR_MODULE_INNUM			0x08
//Все остальные аварии не стираются

#define ERR_MASK_CLEARED			0xf0
#define ERR_MODULE_RESET			0x40
#define NEED_MODULE_RESET			0x80

#define iMODULE_MAX_ERR				100
#define iMODULE_MAX_FAILURES		125


#define		cmt0_5V		0
#define 	cmt4_20mA	1
#define		cmtAD592	2
#define		cmt1Wire	3
#define 	cmtEC		4
#define		cmtT_EC		5
#define		cmtPH1		6
#define		cmtPH2		7
#define		cmt0_3V		8
#define		cmtWater	9
#define		cmtWeigth	10
#define		cmtSun		11



#define USART_OUT				USART3
//Ножка передатчика USART
#define USART_OUT_TX_PIN			GPIO_Pin_10
//Ножка приемника USART
#define USART_OUT_RX_PIN			GPIO_Pin_11
//Ножка направления данных USART
#define USART_OUT_DIR_PIN			GPIO_Pin_9
//Имя порта, на котором находится USART
#define USART_OUT_TX_PORT			GPIOB
#define USART_OUT_RX_PORT			GPIOB
#define USART_OUT_DIR_PORT			GPIOC
#define USART_OUT_INT_VECT			void USART3_IRQHandler(void)
//Вектор прерывания USART
#define USART_OUT_IRQ				USART3_IRQn


//Инициализационная строка USART
#define USART_OUT_STARTUP    	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE)



#define OUT_MODUL_SUM 	30



typedef struct sIModulConf {
	uint8_t		Type;
	uint8_t		Input;
	uint8_t		Output;
	uint8_t		Corr;
	uint16_t		U1;
	uint16_t		V1;
	uint16_t		U2;
	uint16_t		V2;

}TIModulConf;

typedef struct eeFanData
		{
			uint16_t		Actual;
			uint16_t 		Cond;
			uint16_t   		Temp;
			uint16_t     	ActualSpeed;
			uint16_t     	ActualPower;
			uint16_t     	ActualPWM;
			uint16_t     	Flow;

			} eFanData;

typedef struct eeFanBlock
		{
			uint16_t		Speed;
			uint8_t   		NFans;
			eFanData 		FanData[128];

		} eFanBlock;


/*------------ Признаки обмена данных --------*/
#define OUT_UNIT        0x50
#define IN_UNIT         0xa0
#define WORK_UNIT       0x70




typedef void(*CallBackRS)(void);

void USART_OUT_Configuration(uint16_t fbrate);
/*
 * fNCtr - указатель на номер контроллера &GD.Control.NFCtr
 * fADRGD - указатель на структуру адресов блоков памяти ArdGD
 * fSostRS - указатель на переменну состояние передатчика &GD.Hot.SostRS
 * fNumBlock - номер блока, который был принят или передан &NumBlock
 * fbrate - скорость обмена 9600

 */

uint8_t RS485_Master_ExchangeDataIRQ(uint8_t fNCtr, uint8_t fCmd, uint16_t fnReg, uint16_t fData, uint8_t *fCond, CallBackRS pF);
int16_t RS485_Master_ReadType(uint8_t fNCtr, uint8_t*  fIdent);
int16_t RS485_Master_ReadData(uint8_t fNCtr, uint16_t fAdrSend, uint16_t fNBytes, void* fData, uint8_t fNBlock);
uint16_t GetIPCComMod(uint16_t nAddress);
uint16_t GetIPCNum(uint16_t nAddress);
char GetOutIPCDigit(uint16_t nAddress, char* nErr);
void SetOutIPCDigit(char How, uint16_t nAddress,char* nErr);
void ClrAllOutIPCDigit(void);
void ResumeOutIPCDigit(void);
uint16_t GetInIPC(uint16_t nAddress,char* nErr);
uint16_t GetDiskrIPC(uint16_t nAddress,char* nErr);
uint16_t UpdateInIPC(uint16_t nAddress,TIModulConf* ModulConf);
void ModStatus(uint8_t nMod,uint16_t* fCpM,uint8_t *fErr,uint8_t *fFail, uint8_t *fCond,uint8_t *fMaxIn,uint16_t **fInputs);

int16_t IMOD_WriteOutput(char COMPort,int nModule, uint32_t Values);





void SendFans(void* fFanBlock);




#endif

