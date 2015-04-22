//������ ����� RS-485

#ifndef __STM32F10X_RS485_H
#define __STM32F10X_RS485_H

//����� ������������� USART
#define USART_PC1				USART1
//#define USART_PC2				USART2
//#define USART_PC3				USART3
//#define USART_PC5				USART5





/*------------ �������� ������ ������ --------*/
#define OUT_UNIT        0x50
#define IN_UNIT         0xa0
#define WORK_UNIT       0x70


void USART_PC_Configuration(uint8_t *fNFCtr,uint8_t** fADRGD,uint8_t* fSostRS,uint8_t* fNumBlock,uint16_t fbrate);
/*
 * fNCtr - ��������� �� ����� ����������� &GD.Control.NFCtr
 * fADRGD - ��������� �� ��������� ������� ������ ������ ArdGD
 * fSostRS - ��������� �� ��������� ��������� ����������� &GD.Hot.SostRS
 * fNumBlock - ����� �����, ������� ��� ������ ��� ������� &NumBlock
 * fbrate - �������� ������ 9600

 */
void CheckRSTime();
/*�� ��������� �����������, ���� �����*/


#endif

