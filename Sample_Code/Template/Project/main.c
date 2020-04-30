/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2019 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

/***********************************************************************************************************/
/* Website: http://www.nuvoton.com                                                                         */
/*  E-Mail : MicroC-8bit@nuvoton.com                                                                       */
/*  Date   : Jan/21/2019                                                                                   */
/***********************************************************************************************************/

/************************************************************************************************************/
/*  File Function: MS51 DEMO project                                                                        */
/************************************************************************************************************/

#include "MS51_16K.h"

enum{
	TARGET_CH0 = 0 ,
	TARGET_CH1 ,
	TARGET_CH2 ,
	TARGET_CH3 ,	
	
	TARGET_CH4 ,
	TARGET_CH5 ,
	TARGET_CH6 ,
	TARGET_CH7 ,

	TARGET_CH_DEFAULT	
}Channel_TypeDef;

//#define ENABLE_16MHz
#define ENABLE_24MHz

#if defined (ENABLE_16MHz)
#define SYS_CLOCK 								(16000000ul)
#elif defined (ENABLE_24MHz)
#define SYS_CLOCK 								(24000000ul)
#endif

uint8_t 	u8TH0_Tmp = 0;
uint8_t 	u8TL0_Tmp = 0;

//UART 0
bit BIT_TMP;
bit BIT_UART;

typedef enum{
	flag_reverse = 0 ,
	flag_uart0_receive , 	

	
	flag_DEFAULT	
}Flag_Index;

uint8_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint8_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

#define PWM_PSC 								(32)	
#define PWM_FREQ 								(375)	
#define PWM_DUTY                               (0)

//16 bit
#define PWM_CNR 								((SYS_CLOCK/PWM_FREQ)/PWM_PSC - 1)
#define PWM_CMR 								(PWM_DUTY * (PWM_CNR + 1)/1000)

#define CalNewDutyCMR(u32DutyCycle, u32CycleResolution)	(u32DutyCycle * (PWM_CNR + 1) / u32CycleResolution)

void send_UARTString(uint8_t* Data)
{
	#if 1
	uint16_t i = 0;

	while (Data[i] != '\0')
	{
		#if 1
		SBUF = Data[i++];
		#else
		UART_Send_Data(UART0,Data[i++]);		
		#endif
	}

	#endif

	#if 0
	uint16_t i = 0;
	
	for(i = 0;i< (strlen(Data)) ;i++ )
	{
		UART_Send_Data(UART0,Data[i]);
	}
	#endif

	#if 0
    while(*Data)  
    {  
        UART_Send_Data(UART0, (unsigned char) *Data++);  
    } 
	#endif
}

void send_UARTASCII(uint16_t Temp)
{
    uint8_t print_buf[16];
    uint16_t i = 15, j;

    *(print_buf + i) = '\0';
    j = (uint16_t)Temp >> 31;
    if(j)
        (uint16_t) Temp = ~(uint16_t)Temp + 1;
    do
    {
        i--;
        *(print_buf + i) = '0' + (uint16_t)Temp % 10;
        (uint16_t)Temp = (uint16_t)Temp / 10;
    }
    while((uint16_t)Temp != 0);
    if(j)
    {
        i--;
        *(print_buf + i) = '-';
    }
    send_UARTString(print_buf + i);
}


void GPIO_Toggle_P12(void)
{
	P12 = ~P12;	
}

void GPIO_Init(void)	//for test
{
    P12_PUSHPULL_MODE;
}

void PWM0_CH3_SetDuty(uint16_t duty)	// 1 ~ 1000 , 0.1 % to 100%
{
	uint16_t res = 0 ;

	res = CalNewDutyCMR(duty , 1000);

    PWM3H = HIBYTE(res);
    PWM3L = LOBYTE(res);

    set_PWMCON0_LOAD;
    set_PWMCON0_PWMRUN;	
}

void PWM0_CHx_Init(void)
{
	/*
		PWM : PWM0_CH3 , P0.0
	*/

	uint32_t res = 0;


	P00_PUSHPULL_MODE;	//Add this to enhance MOS output capability
    PWM3_P00_OUTPUT_ENABLE;
  
    PWM_IMDEPENDENT_MODE;
    PWM_CLOCK_DIV_32;

/*
	PWM frequency   = Fpwm/((PWMPH,PWMPL)+1) = (24MHz/2)/(PWMPH,PWMPL)+1) = 20KHz
*/	
	res = PWM_CNR;
	
    PWMPH = HIBYTE(res);
    PWMPL = LOBYTE(res);

//	send_UARTString("\r\nPWM:");	
//	send_UARTASCII(PWMPH);
//	send_UARTString(",");	
//	send_UARTASCII(PWMPL);
//	send_UARTString("\r\n\r\n");	

	PWM0_CH3_SetDuty(0);

	set_flag(flag_reverse , Enable);

}

void Timer0_IRQHandler(void)
{
	static uint16_t CNT_PWM = 1;
	static uint16_t duty = 500;	// 1 ~ 1000 , 0.1 % to 100%

	if (CNT_PWM++ == 100)
	{
		CNT_PWM = 1;
		
		PWM0_CH3_SetDuty(duty);

		if (is_flag_set(flag_reverse))
		{
			duty++;	
		}
		else
		{
			duty--;
		}

		if (duty == 1000)
		{
			set_flag(flag_reverse , Disable);				
		}
		else if (duty == 0)
		{
			set_flag(flag_reverse , Enable);
		}
		
		GPIO_Toggle_P12();	//for test
	}	
	
}

void Timer0_ISR(void) interrupt 1        // Vector @  0x0B
{
    TH0 = u8TH0_Tmp;
    TL0 = u8TL0_Tmp;
    clr_TCON_TF0;
	
	Timer0_IRQHandler();
}

void TIMER0_Init(void)
{
	uint16_t res = 0;

	ENABLE_TIMER0_MODE1;
	
	u8TH0_Tmp = HIBYTE(TIMER_DIV12_VALUE_1ms_FOSC_240000);
	u8TL0_Tmp = LOBYTE(TIMER_DIV12_VALUE_1ms_FOSC_240000); 

    TH0 = u8TH0_Tmp;
    TL0 = u8TL0_Tmp;

    ENABLE_TIMER0_INTERRUPT;                       //enable Timer0 interrupt
    ENABLE_GLOBAL_INTERRUPT;                       //enable interrupts
  
    set_TCON_TR0;                                  //Timer0 run
}

void UART0_Process(uint8_t res)
{
	if (res > 0x7F)
	{
		return;
	}

	if (is_flag_set(flag_uart0_receive))
	{
		set_flag(flag_uart0_receive , Disable);
		switch(res)
		{
			case '1':
				
				break;
		}		
	}
}

void Serial_ISR (void) interrupt 4 
{
    if (RI)
    {   
	  set_flag(flag_uart0_receive , Enable);
	  UART0_Process(SBUF);

      clr_SCON_RI;                                         // Clear RI (Receive Interrupt).
    }
    if  (TI)
    {
      if(!BIT_UART)
      {
          TI = 0;
      }
    }
}

void UART0_Init(void)
{
	#if 1
	unsigned long u32Baudrate = 115200;
	P06_QUASI_MODE;    //Setting UART pin as Quasi mode for transmit
	SCON = 0x50;          //UART0 Mode1,REN=1,TI=1
	set_PCON_SMOD;        //UART0 Double Rate Enable
	T3CON &= 0xF8;        //T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1)
	set_T3CON_BRCK;        //UART0 baud rate clock source = Timer3

	#if defined (ENABLE_16MHz)
	RH3    = HIBYTE(65536 - (1000000/u32Baudrate)-1);  
	RL3    = LOBYTE(65536 - (1000000/u32Baudrate)-1);  
	#elif defined (ENABLE_24MHz)
	RH3    = HIBYTE(65536 - (SYS_CLOCK/16/u32Baudrate));  
	RL3    = LOBYTE(65536 - (SYS_CLOCK/16/u32Baudrate));  
	#endif
	
	set_T3CON_TR3;         //Trigger Timer3
	set_IE_ES;

	ENABLE_GLOBAL_INTERRUPT;

	set_SCON_TI;
	BIT_UART=1;
	#else	
    UART_Open(SYS_CLOCK,UART0_Timer3,115200);
    ENABLE_UART0_PRINTF; 
	#endif
}


#if defined (ENABLE_16MHz)
void MODIFY_HIRC_16(void)
{
    unsigned char data hircmap0,hircmap1;
    set_CHPCON_IAPEN;
    IAPAL = 0x30;
    IAPAH = 0x00;
    IAPCN = READ_UID;
    set_IAPTRG_IAPGO;
    hircmap0 = IAPFD;
    IAPAL = 0x31;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
    hircmap1 = IAPFD;
    clr_CHPCON_IAPEN;
    TA=0XAA;
    TA=0X55;
    RCTRIM0 = hircmap0;
    TA=0XAA;
    TA=0X55;
    RCTRIM1 = hircmap1;
}

#elif defined (ENABLE_24MHz)
void MODIFY_HIRC_24(void)
{
    unsigned char data hircmap0,hircmap1;
/* Check if power on reset, modify HIRC */
    if (PCON&SET_BIT4)
    {
        set_CHPCON_IAPEN;
        IAPAL = 0x38;
        IAPAH = 0x00;
        IAPCN = READ_UID;
        set_IAPTRG_IAPGO;
        hircmap0 = IAPFD;
        IAPAL = 0x39;
        IAPAH = 0x00;
        set_IAPTRG_IAPGO;
        hircmap1 = IAPFD;
        clr_CHPCON_IAPEN;
        TA=0XAA;
        TA=0X55;
        RCTRIM0 = hircmap0;
        TA=0XAA;
        TA=0X55;
        RCTRIM1 = hircmap1;
        clr_CHPCON_IAPEN;
    }
}

#endif

void SYS_Init(void)
{
    MODIFY_HIRC_24();

    ALL_GPIO_QUASI_MODE;
    ENABLE_GLOBAL_INTERRUPT;                // global enable bit	
}

void main (void) 
{
    SYS_Init();

    UART0_Init();

	// PWM : PWM0_CH3 , P0.0
	PWM0_CHx_Init();
								
	//P1.2 , GPIO , for test
	GPIO_Init();					
			
	TIMER0_Init();	
	
    while(1)
    {

    }


}



