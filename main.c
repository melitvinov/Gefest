/*
 *  defined FUN_TYPE1
 *  Version:
 *  Gefest - Firmware_0-0-2-01
 */

/*
 *  defined FUN_TYPE2
 *  Version:
 *  Gefest - Firmware_0-0-1-03
 */

//#define DEBUG
//#define NOTESTMEM


#include "stm32f10x_Define.h"
#include "stm32f10x_RS485Master.h"
#include "stm32f10x_RS485.h"
#include "stm32f10x_Rootines.h"
#include "65_GD.c"
#include "src/stm32f10x_Rootines.c"

main()
{
	Init_STM32();
	SETEA;
	while(1)
	{
		//Check_IWDG();  //XXX
	}
}

