#pragma pack(1)
#include "stm32f10x_Define.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_RS485Master.h"
#include "misc.h"

#define USART_MASTER_RX     			fTimeout=10000; while((USART_GetFlagStatus(USART_OUT,USART_FLAG_RXNE)==RESET)&&(fTimeout)) fTimeout--;
#define USART_MASTER_TX     			fTimeout=10000; while(!(USART_GetFlagStatus(USART_OUT,USART_FLAG_TC))&&(fTimeout)) fTimeout--;
#define USART_MASTER_TXE     			fTimeout=10000; while(!(USART_GetFlagStatus(USART_OUT,USART_FLAG_TXE))&&(fTimeout)) fTimeout--;

#define USART_MASTER_STOPSEND			GPIO_WriteBit(USART_OUT_DIR_PORT,USART_OUT_DIR_PIN,Bit_RESET);
#define USART_MASTER_STARTSEND			for(i=0;i<500;i++); GPIO_WriteBit(USART_OUT_DIR_PORT,USART_OUT_DIR_PIN,Bit_SET);

#define HEAD_SIZE				5
#define IDENT_SIZE				7

#define MODULE_IS_BUSY			1
#define MODULE_IS_OK			0


#define MAX_SUM_TRY				4

#define MAX_IN_SENS		32
#define MAX_OUT_REGS	32

typedef struct sOModulConf {
	uint32_t	Type;
	uint8_t		Pulse[MAX_OUT_REGS][2];
}TOModulConf;

typedef struct sModulData {
	uint8_t		Cond;
	uint16_t 	CpM;
	uint8_t		MaxIn; //ћаксимальный номер входа, используемый в модуле
	uint8_t		MaxOut; //ћаксимальный номер импульсного регул€тора,используемого в модуле
	uint32_t 	OutValues;
	TOModulConf	OutConfig;
	uint16_t 	InValues[MAX_IN_SENS];
	TIModulConf	InConfig[MAX_IN_SENS];
	char Err;
	char Failures;
	} TModulData;

TModulData ModulData[OUT_MODUL_SUM];


unsigned char	HeadOUT[10];
uint8_t	ReadBuf[1000];
char PHASE_RS_OUT;
int	 RSOutTime;
int	ptrUARTOUT;
uint8_t chSumUARTOUT;
//uint8_t	pSostRS485OUT;
uint8_t *pDataRSOUT;
uint16_t StatusByte;
uint8_t	cOperInModule;
uint8_t cCycle;
uint8_t	cModule;
CallBackRS GLF;
uint8_t bOutIPCBlock;






#define MAX_IN_SENS		32
#define MAX_OUT_REGS	32


#define RSOUT_INIT					0
#define RSOUT_START					1
#define RSOUT_HEAD					2
#define RSOUT_CHK					3
#define RSOUT_RECV					4
#define RSOUT_SEND					5
#define RSOUT_SENDCHK				6


//#define IS_OK					5
//#define IS_FAILURE				6
//#define IS_WORK					7

#define MAX_RSOUT_TIME					5 //10*33mil sec=330mil sec 9600bod is 1.2kbyte/sec  per 100 byte











//uint16_t	GLSize;
uint16_t	fanCycle;
uint8_t		fanCmd;
eFanData	*GLFanData;
eFanData	FanBroadcast;
eFanBlock	*FanBlock;

uint16_t CRCCalc(char *buf, int n)
{
unsigned char i,carry;
uint16_t crc16=0xFFFF;
while(n)
{
crc16^=*buf;
for  (i=0; i<8; i++)
{
carry=crc16&1;
crc16>>=1;
if (carry)crc16^=0xA001;
}
n--;
buf++;
}
return crc16;
}



void CpyBuf(uint8_t *pp1,uint8_t *pp2, uint16_t size)
{
	int16_t i;
	for (i=0;i<size;i++)
		*(pp1++)=*(pp2++);

}

uint16_t NoSameBuf(uint8_t *pp1,uint8_t *pp2, uint16_t size)
{
	int16_t i;
	for (i=0;i<size;i++)
		if ((*(pp1++))!=(*(pp2++)))
				return i;
	return 0;

}


void USART_OUT_Configuration(uint16_t fbrate)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  USART_OUT_STARTUP;

  NVIC_InitStructure.NVIC_IRQChannel = USART_OUT_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


  GPIO_InitStructure.GPIO_Pin = USART_OUT_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USART_OUT_TX_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = USART_OUT_DIR_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(USART_OUT_DIR_PORT, &GPIO_InitStructure);


   GPIO_InitStructure.GPIO_Pin = USART_OUT_RX_PIN;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(USART_OUT_RX_PORT, &GPIO_InitStructure);

/* USART1 configuration ------------------------------------------------------*/
  /* USART1 configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
        - USART Clock disabled
        - USART CPOL: Clock is active low
        - USART CPHA: Data is captured on the middle
        - USART LastBit: The clock pulse of the last data bit is not output to
                         the SCLK pin
  */

  USART_InitStructure.USART_BaudRate = fbrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_9b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_Even;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART_OUT, &USART_InitStructure);

  /* Enable USA/RT1 */
  USART_Cmd(USART_OUT, ENABLE);

  USART_ITConfig(USART_OUT, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART_OUT, USART_IT_TC, ENABLE);

  USART_MASTER_STOPSEND;

  PHASE_RS_OUT=RSOUT_INIT;

}

#define MODBUS_OFFSET_CTR	0
#define MODBUS_OFFSET_CMD	1
#define MODBUS_OFFSET_DATA	2
#define MODBUS_DATA_SIZE	4
#define MODBUS_SIZE_SNOP	6
#define MODBUS_END_FRAME	7

#define FAN_WRONG_DEMAND	0x04
#define FAN_WRONG_CHECKSUM	0x08
#define FAN_NO_ANSWER		0x10

#define MAX_FAN_CMD			6
#define MAX_FAN_SPEED		60

#define FUN_TYPE1
//#define FUN_TYPE2

#if defined (FUN_TYPE1)
#define FAN_CMD_0			0x01
#define FAN_CMD_1			0x10
#define FAN_CMD_2			0x11
#define FAN_CMD_3			0x16
#define FAN_CMD_4			0x1A
#define FAN_CMD_5			0x21
#endif

#if defined (FUN_TYPE2)
#define FAN_CMD_0			0x00
#define FAN_CMD_1			0x10
#define FAN_CMD_2			0x05
#define FAN_CMD_3			0x08
#define FAN_CMD_4			0x1A
#define FAN_CMD_5			0x21
#define FAN_CMD_6			0x02
#define FAN_CMD_7			0x04
#define FAN_CMD_8			0x22
#endif


USART_OUT_INT_VECT
{
	uint16_t retByte,i;
	//uint16_t i;

	if ((USART_OUT->SR & USART_FLAG_RXNE)!=0)
	{
//			USART_PC->SR=0;
			retByte = USART_ReceiveData(USART_OUT);
			retByte&=0xff;
//			if (retByte&0x100) PHASE_RS_OUT=RSOUT_START;
			switch(PHASE_RS_OUT)
			{
				case RSOUT_RECV:
				{
					if ((ptrUARTOUT==MODBUS_OFFSET_DATA)&&(ReadBuf[MODBUS_OFFSET_CMD]==0x04))
						ptrUARTOUT++;
					if ((!(retByte&0xD0))&&(ptrUARTOUT==MODBUS_OFFSET_DATA))
					{
						GLFanData->Actual|=FAN_WRONG_DEMAND;
						ptrUARTOUT+=(MODBUS_DATA_SIZE-1);
					}
					if (ptrUARTOUT==MODBUS_END_FRAME)
					{
						*pDataRSOUT=retByte;
						retByte=CRCCalc(ReadBuf,MODBUS_SIZE_SNOP);
						if ((retByte/256==ReadBuf[MODBUS_END_FRAME])&&(retByte%256==ReadBuf[MODBUS_END_FRAME-1]))
						{
							if (GLF)
								GLF();
						}
						else
							GLFanData->Actual|=FAN_WRONG_CHECKSUM;

						PHASE_RS_OUT=RSOUT_INIT;
						return;
					}
					*pDataRSOUT=retByte;
					pDataRSOUT++;
					ptrUARTOUT++;
					return;
				}
				default:
				{

					//PHASE_RS_OUT=RSOUT_START;

					//*pSostRS485OUT=OUT_UNIT;
				}

			}
			return;
	}
	if ((USART_OUT->SR & USART_FLAG_TXE)!=0)
	{
		if (ptrUARTOUT>0)
		{
			//for (i=0;i<10;i++);
			USART_SendData(USART_OUT,*pDataRSOUT);
			chSumUARTOUT+=*pDataRSOUT;
			pDataRSOUT++;
			ptrUARTOUT--;
			return;
		}
		USART_ITConfig(USART_OUT, USART_IT_TXE, DISABLE);
	}
	if ((USART_OUT->SR & USART_FLAG_TC)!=0)
	{
			//USART_ITConfig(USART_OUT, USART_IT_TXE, DISABLE);
			USART_ClearITPendingBit(USART_OUT,USART_IT_TC);
			//if (PHASE_RS_OUT==RSOUT_INIT) {USART_MASTER_STOPSEND; return;}
			if ((PHASE_RS_OUT==RSOUT_START)||(PHASE_RS_OUT==RSOUT_HEAD))
			{
				//for (i=0;i<10;i++);
				USART_MASTER_STOPSEND;
				PHASE_RS_OUT=RSOUT_RECV;
				ptrUARTOUT=0;
				pDataRSOUT=ReadBuf;
//				return;
			}

			return;

	}
	//USART_ITConfig(USART_PC, USART_IT_TXE, DISABLE);
	//USART_PC->SR=0;

//	USART_PC->SR=0;
}


// Send to FUN
uint8_t RS485_Master_SendData(uint8_t fNCtr, uint8_t fCmd, uint16_t fnReg, uint16_t fData, eFanData *fFanData, CallBackRS pF)
{
	uint16_t	retByte,i;
	char* fPtr;
//	retByte=fNCtr|0x100;
	if (PHASE_RS_OUT!=RSOUT_INIT)
	{
		GLFanData->Actual|=FAN_NO_ANSWER;
		PHASE_RS_OUT=RSOUT_INIT;
	}
	GLF=pF;
	GLFanData=fFanData;
	RSOutTime=((MAX_RSOUT_TIME*5)/100)+2;
	USART_MASTER_STARTSEND;
	//(*GLCond)&=0x80;
	ReadBuf[0]=fNCtr;
	ReadBuf[1]=fCmd;
	ReadBuf[2]=fnReg/256;
	ReadBuf[3]=fnReg%256;
	ReadBuf[4]=fData/256;
	ReadBuf[5]=fData%256;
/******************************************************************************/
	//OutSignal=0;
/******************************************************************************/

	retByte=CRCCalc(ReadBuf,6);
	ReadBuf[6]=retByte%256;
	ReadBuf[7]=retByte/256;
	pDataRSOUT=ReadBuf;
	ptrUARTOUT=7;
	PHASE_RS_OUT=RSOUT_START;
	for (i=0;i<100;i++);
	USART_ITConfig(USART_OUT, USART_IT_TXE, ENABLE);
	USART_SendData(USART_OUT,*pDataRSOUT);
	pDataRSOUT++;
	return MODULE_IS_OK;
}






void SetActualInfo()
{
	uint16_t	tvar;
	switch (ReadBuf[3])
	{
	case 0x10:
		GLFanData->Actual=0;
		GLFanData->ActualSpeed=(ReadBuf[4]*256+ReadBuf[5])*1500/64000;
		break;
	case 0x11:
		GLFanData->Actual=0;
		GLFanData->Cond=(ReadBuf[4]*256+ReadBuf[5]);
		break;
	case 0x16:
		GLFanData->Actual=0;
		GLFanData->Temp=(ReadBuf[5]);
		break;
	case 0x1A:
		GLFanData->Actual=0;
		GLFanData->ActualPWM=(ReadBuf[4]*256+ReadBuf[5])*100/65536;
		break;
	case 0x21:
		GLFanData->Actual=0;
		GLFanData->ActualPower=(ReadBuf[4]*256+ReadBuf[5])*100/65536;
		break;
	}
}

#if defined FUN_TYPE1
void SendFans(void* fFanBlock)
{
	uint16_t fSpeed;
	uint8_t Cond;
	uint32_t tSpeed;
	FanBlock=fFanBlock;
//*****************************MOVE

	FanBlock->NFans=3;

//***********************************
	if (FanBlock->NFans<=0)
		return;
	fanCmd++;
	fanCmd%=MAX_FAN_CMD;
	if (!fanCmd)
	{
		fanCycle++;
		if (FanBlock->NFans>=128)
			FanBlock->NFans=127;
		fanCycle%=FanBlock->NFans;
	}

	ClrDog;

	switch(fanCmd)
	{
	case 0:
		fSpeed=FanBlock->Speed;
		if (fSpeed>MAX_FAN_SPEED)
			fSpeed=MAX_FAN_SPEED;
//		fSpeed=5;
		fSpeed=((int)fSpeed*65536)/100;
		RS485_Master_SendData(0,0x06,0xD000+FAN_CMD_0,fSpeed,&FanBroadcast,0);
		break;
	case 1:
		RS485_Master_SendData(fanCycle+1,0x04,0xD000+FAN_CMD_1,256+1,&FanBlock->FanData[fanCycle],0);
		break;
	case 2:
		RS485_Master_SendData(fanCycle+1,0x04,0xD000+FAN_CMD_2,256+1,&FanBlock->FanData[fanCycle],0);
		break;
	case 3:
		RS485_Master_SendData(fanCycle+1,0x04,0xD000+FAN_CMD_3,256+1,&FanBlock->FanData[fanCycle],0);
		break;
	case 4:
		RS485_Master_SendData(fanCycle+1,0x04,0xD000+FAN_CMD_4,256+1,&FanBlock->FanData[fanCycle],0);
		break;
	case 5:
		RS485_Master_SendData(fanCycle+1,0x04,0xD000+FAN_CMD_5,256+1,&FanBlock->FanData[fanCycle],0);
		break;
	}
//	fSpeed=(10)*656;
//	RS485_Master_SendData(0,0x06,0xD001,fSpeed,&Cond,0);
}
#endif

#if defined FUN_TYPE2
void SendFans(void* fFanBlock)
{
	uint16_t fSpeed;
	uint8_t Cond;
	uint32_t tSpeed;
	FanBlock=fFanBlock;
//*****************************MOVE
	FanBlock->NFans=3;
//***********************************
	if (FanBlock->NFans<=0)
		return;
	fanCmd++;
	fanCmd%=MAX_FAN_CMD;
	if (!fanCmd)
	{
		fanCycle++;
		if (FanBlock->NFans>=128)
			FanBlock->NFans=127;
		fanCycle%=FanBlock->NFans;
	}
	switch(fanCmd)
	{
	case 0:
	{
		fSpeed=FanBlock->Speed;
		//if (fSpeed>1015)
		//	fSpeed=1015;

		//fSpeed=((int)fSpeed*1000)/100;

		//RS485_Master_SendData(0,0x06,0x00+FAN_CMD_2,fSpeed,&FanBroadcast,0);
		RS485_Master_SendData(0,0x06,0x00+FAN_CMD_6,fSpeed,&FanBroadcast,0);
	}
	break;
	//case 1:
	//	RS485_Master_SendData(0,0x06,0x00+FAN_CMD_7,0x03,0,0);
		//RS485_Master_SendData(0,0x06,0x00+FAN_CMD_3,0x03E8,0,0); // установка максимальной скорости если стоит работа по предустановкам
	//	break;
	}
}
#endif





