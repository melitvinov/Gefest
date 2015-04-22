#pragma pack(1)

typedef struct eeControl
	{
        uchar 		  	rInit[5];
		uint8_t   		NFCtr;
	//	uint8_t			Saverez;
        int16_t     	Rez[5];
	
	} eControl;


typedef struct eeRegBlock
	{
		uint8_t   		Type;
	//	uint8_t			Saverez;
	    uint16_t     	Value;

	} eRegBlock;



typedef struct eeHot
	{
	        uint32_t 		OutR;
	        uint32_t 		Mode;
//	        uint32_t  		ImpMode;
	        uint8_t   		Pulse[24][2];
	        int16_t     	Rez[100];
		
	} eHot;



char* AdrGD[16];

uint8_t NumBlock;

uint16_t nReset;

/* ======== Глобальный блок данных=============*/
struct  eGData{
        uchar           SostRS;   /* не передвигать использовано в .asm*//*Не буду*/
        eHot            Hot;           /* не передвигать использовано News в .asm*//*Не буду*/
        int	        	Calibr[1000];
        eControl        Control;
        eFanBlock       FanBlock;
        eRegBlock		RegBlock[20];

        } GD;

char Pause[24];
char Work[24];

