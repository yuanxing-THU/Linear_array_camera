/*
BY ZYX 
%小动车  测速程序
Systick 1ms 进中断 测时间
exti 下降沿触发 测车轮到达的位置
两次到达之后计算速度给脉冲
最后一次到达 计算延时时间 

板子上 引脚分布
PA5 PA7
GND PA6

PA5为外部触发
PA6为tim PWM 输出

*/

#include "stm32f10x.h"
#include <stdio.h>

GPIO_InitTypeDef GPIO_InitStructure;  //定义GPIO初始化结构体
USART_InitTypeDef USART_InitStructure;//定义串口初始化结构体
EXTI_InitTypeDef   EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
/* Private function prototypes -----------------------------------------------*/

struct rail
{
	unsigned int bogie_leading[5];
	unsigned int vehicle[5];
	unsigned int bogie_trailing[5];
	unsigned int vehicle_dist[4];
}Myrail;
// mm
#define bogie_length  29
#define vehicle_length 171
#define vehicle_dist_length 60


#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
	
#define LED_ON GPIO_ResetBits(GPIOB, GPIO_Pin_3);  //由电路图D4
#define LED_OFF GPIO_SetBits(GPIOB, GPIO_Pin_3);

void Delay(u32 nCount);				  //延时函数
void RCC_Config(void);
void GPIO_Config(void);
void USART_Config(void);
void EXIT_Config(void);
void NVIC_Config(void);
void falling_exti(void);
void rising_exti(void);
void TIM_Config(void);
void timer_updata(void);
void Systick_Config(void);
void Display_rail(void);
void TIM3_fq(int) ;
int count=0;
int time_flying=0; 
int rail_cpl_flag=0;//标志一次记录完成
int main()
{
  RCC_Config();
	GPIO_Config();
	USART_Config();
	NVIC_Config();
	EXIT_Config();
	TIM_Config();
	
	Systick_Config();
	printf("System_begining\r\n");
	printf("System_begining\r\n");
	
	TIM_Cmd(TIM2, ENABLE);
	//TIM_Cmd(TIM3, ENABLE);
	
	time_flying = 1 ;
	while(1)
	{
		if(rail_cpl_flag==1)
		{
			rail_cpl_flag=0;
			Display_rail();
		}
	}
	
}

void Delay(u32 nCount)
{
   for(; nCount != 0; nCount--);
}
void RCC_Config(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		 //开启GPIO B的时钟，D4LED输出引脚
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);   //开启USART2，引脚时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		 //开启GPIO B的时钟，D4LED输出引脚
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); 
}
void GPIO_Config(void)				  
{
	//初始化LED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				     //LED管脚 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			 //将PB3 配置为通用推挽输出  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //IO口翻转速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //将上述设置赋予GPIOB
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);   //关键一步！！
	
	//初始化USART2  PA2 TX   PA3 RX
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//初始化PA5 EXIT
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	//初始化 PA6 TIM3_CH1
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void EXIT_Config(void)
{
	EXTI_DeInit();
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5);
	/* Configure EXTI8 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line5;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //上升沿下降沿均触发_Falling
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void USART_Config(void)
{
	USART_InitStructure.USART_BaudRate=115200;
	USART_InitStructure.USART_HardwareFlowControl=USART_StopBits_1;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART2,&USART_InitStructure);
	USART_Cmd(USART2,ENABLE);
}
void NVIC_Config(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	//EXTI9_5_IRQn
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	//TIM2
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;//
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void falling_exti(void)
{
/*#define bogie_length  29
#define vehicle_length 171
#define vehicle_dist_length 60
	*/
	
	int cable_num=(count-2)/4;
	int position=count-4*cable_num-1;
	int arr = 0;
	if(cable_num>=0&&cable_num<5&&position>0&&position<5)      //cable_num  01234    position  1234
	{
		if(position==1) //bogie_leading
		{
			Myrail.bogie_leading[cable_num]=time_flying;
			arr = 3348*time_flying/bogie_length-1 ;
			TIM3_fq(arr) ; 
		}else if(position==2)
		{
			Myrail.vehicle[cable_num]=time_flying;
			arr = 3348*time_flying/vehicle_length-1 ;
			TIM3_fq(arr) ; 
		}else if(position==3)
		{
			Myrail.bogie_trailing[cable_num]=time_flying;
			arr = 3348*time_flying/bogie_length-1 ;
			TIM3_fq(arr) ; 
		}else if(position==4)
		{
			if(cable_num<4)
				Myrail.vehicle_dist[cable_num]=time_flying;
			arr = 3348*time_flying/vehicle_dist_length-1 ;
			TIM3_fq(arr) ; 
		}
	}
	printf("%d falling %d ,arr = %d\r\n",count,time_flying,arr);
	if (count == 2)  //第二个脉触发到来时开始给相机脉冲
	{
		TIM_Cmd(TIM3, ENABLE);
	}
	if(cable_num==4&&position==3)   //最后一个位置
	{
		count=0;
		rail_cpl_flag=1;
		TIM2->ARR =(time_flying)*90+5000 ;
	}

	TIM_Cmd(TIM2, DISABLE);
	TIM2->CNT = 0 ;
	TIM_Cmd(TIM2, ENABLE); 
}
void Display_rail(void)
{
	printf("Num.1: bogie_leading:%d ;vehicle : %d ;bogie_trailing %d ; vehicle_dist: %d\r\n",Myrail.bogie_leading[0],Myrail.vehicle[0],Myrail.bogie_trailing[0],Myrail.vehicle_dist[0]);
	printf("Num.2: bogie_leading:%d ;vehicle : %d ;bogie_trailing %d ; vehicle_dist: %d\r\n",Myrail.bogie_leading[1],Myrail.vehicle[1],Myrail.bogie_trailing[1],Myrail.vehicle_dist[1]);
	printf("Num.3: bogie_leading:%d ;vehicle : %d ;bogie_trailing %d ; vehicle_dist: %d\r\n",Myrail.bogie_leading[2],Myrail.vehicle[2],Myrail.bogie_trailing[2],Myrail.vehicle_dist[2]);
	printf("Num.4: bogie_leading:%d ;vehicle : %d ;bogie_trailing %d ; vehicle_dist: %d\r\n",Myrail.bogie_leading[3],Myrail.vehicle[3],Myrail.bogie_trailing[3],Myrail.vehicle_dist[3]);
	printf("Num.5: bogie_leading:%d ;vehicle : %d ;bogie_trailing %d \r\n",Myrail.bogie_leading[4],Myrail.vehicle[4],Myrail.bogie_trailing[4]);
}
void rising_exti(void)
{
	printf("rising_exti\r\n");
}
void TIM_Config(void)
{
	TIM_DeInit(TIM2);
	TIM_TimeBaseStructure.TIM_Period = 17999;   // 9000 -1  为1s
	TIM_TimeBaseStructure.TIM_Prescaler = 7999;           
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;      
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;    
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
	TIM_ARRPreloadConfig(TIM2, ENABLE); 
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);	
	TIM_ARRPreloadConfig(TIM2, DISABLE);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
	
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Prescaler = 1 ;   //36M
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;      
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;  
	TIM_TimeBaseStructure.TIM_Period =4499  ;   //ARR       8k  4499
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure) ;
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable ;
	TIM_OCInitStructure.TIM_Pulse = 3000 ;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High ;
	
	TIM_OC1Init(TIM3,&TIM_OCInitStructure) ;
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); 
	
}
void TIM3_fq(int f)
{
	TIM3->ARR = f ;
}
void timer_updata(void)
{
	printf("overtime!! %d\r\n",time_flying);
	TIM_Cmd(TIM2, DISABLE);
	count=0 ;
	TIM_Cmd(TIM3, DISABLE);
	TIM2->ARR = 17999 ;
	//EXTI->IMR=0X00000000; //关闭EXTI中断
	//EXTI->IMR=0X00000020;//打开EXTI中断line5
}
void Systick_Config(void)
{
	SysTick_Config(SystemCoreClock / 1000);
}
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART2, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
  {}

  return ch;
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
