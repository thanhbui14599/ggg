/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4x7_eth.h"
#include "netconf.h"
#include "main.h"
#include "udp_echoserver.h"
#include "serial_debug.h"
#include "stdio.h"
#include "stm32_ub_udp_server.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTEMTICK_PERIOD_MS  10

float servo_pos_Begin[6]={0,0,0,0,0,0};
float servo_pos_Race[6]= {0,0,0,0,0,0};
float servo_pos_end[6]=  {0,0,0,0,0,0};

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */
uint32_t timingdelay;
int32_t  Pulse_Feed_Motor1[1],Pulse_Feed_Motor2[1],Pulse_Feed_Motor3[1],Pulse_Feed_Motor4[1],Pulse_Feed_Motor5[1],Pulse_Feed_Motor6[1];

/*
Description: Convert data get from Server to 6 input prameter movement of Motion Base
Parameter  : None
Return     : None
*/
void LCD_LED_Init(void)
{  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable GPIOs clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  

  /* Configure MCO (PD12, PD13, PD14, PD15) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;  
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void Turn_on_led(uint16_t GPIO_Pin)
{
 GPIO_SetBits(GPIOD,GPIO_Pin);
}
void Turn_off_led(uint16_t GPIO_Pin)
{
 GPIO_ResetBits(GPIOD,GPIO_Pin);
}
void ToggleBit_led(uint16_t GPIO_Pin)
{
GPIO_ToggleBits(GPIOD,GPIO_Pin);
}
/*
Description: Convert data get from Server to 6 input prameter movement of Motion Base
Parameter  : Array contain value get from Host Server
Return     : None
*/
float Yaw_angle=0, Yaw_angle0=0, Sub_Angle;
static float Angle=0, Angle0=0;
static float Pre_Angle=0, Pre_Angle0=0;
float alpha=0.795;
void Caculate_data(char *Buffer)
{
int32_t Value[6];
	
	    Value[0]= (Buffer[1]-48)*10000 +  (Buffer[2]-48)*1000  + (Buffer[3]-48)*100 + (Buffer[4]-48)*10 +(Buffer[5]-48);
			Value[1]= (Buffer[6]-48)*10000 +  (Buffer[7]-48)*1000  + (Buffer[8]-48)*100 + (Buffer[9]-48)*10 +(Buffer[10]-48);
			Value[2]= (Buffer[11]-48)*10000+  (Buffer[12]-48)*1000 + (Buffer[13]-48)*100+ (Buffer[14]-48)*10+(Buffer[15]-48);
			Value[3]= (Buffer[16]-48)*10000+  (Buffer[17]-48)*1000 + (Buffer[18]-48)*100+ (Buffer[19]-48)*10+(Buffer[20]-48);
			Value[4]= (Buffer[21]-48)*10000+  (Buffer[22]-48)*1000 + (Buffer[23]-48)*100+ (Buffer[24]-48)*10+(Buffer[25]-48);
			Value[5]= (Buffer[26]-48)*10000+  (Buffer[27]-48)*1000 + (Buffer[28]-48)*100+ (Buffer[29]-48)*10+(Buffer[30]-48);
	
	
//      printf("\n %d   %d   %d   %d   %d   %d ",Value[0],Value[1],Value[2],Value[3],Value[4],Value[5]);	
	
	      servo_pos_Race[3]= -(1-(float)Value[0]/32767)*100*1.8*(float)deg2rad;
				servo_pos_Race[4]= -(1-(float)Value[1]/32767)*100*1.8*(float)deg2rad;
//				servo_pos_Race[5]= -(1-(float)Value[2]/32767)*360;
				servo_pos_Race[0]= 0;//-(1-(float)Value[3]/32767)*20/100;
				servo_pos_Race[1]= 0;//-(1-(float)Value[4]/32767)*20/100;
				servo_pos_Race[2]= 0;//-(1-(float)Value[5]/32767)*16/50;
			
//		printf("\n %f   %f   %f   %f   %f   %f ",servo_pos_Race[0],servo_pos_Race[1],servo_pos_Race[2],servo_pos_Race[3],servo_pos_Race[4],servo_pos_Race[5]);	
			
    	Yaw_angle= -(1-(float)Value[2]/32767)*360;
	    Sub_Angle= Yaw_angle - Yaw_angle0;
	    if(Sub_Angle>180) Angle= Angle+ Sub_Angle-360;
	    else if(Sub_Angle<-180) Angle= Angle +360+Sub_Angle;
			else Angle= Angle + Sub_Angle;
			Pre_Angle = (Angle- Angle0);
			
			servo_pos_Race[5]= (Pre_Angle0*alpha +Pre_Angle*(1-alpha));//*deg2rad;//(1-(float)Value[5]/32767)*360;//*deg2rad/70;
      Pre_Angle0= servo_pos_Race[5];     
		  servo_pos_Race[5] = servo_pos_Race[5]*(float)0.0175*3;
		  Yaw_angle0= Yaw_angle;
			Angle0= Angle;
			printf("\n -----\n ");
		  //printf("\n %f   %f   %f   %f   %f   %f ",servo_pos_Race[0],servo_pos_Race[1],servo_pos_Race[2],servo_pos_Race[3],servo_pos_Race[4],servo_pos_Race[5]);	
}
float roll, pitch, yall;
uint8_t status_data_pitch=0, status_data_roll=0,status_data_yall=0;
void Input_data(void)
{
	  
		servo_pos_Race[3] = -(float)roll*(float)deg2rad;
		servo_pos_Race[4] = -(float)pitch*(float)deg2rad;
		servo_pos_Race[5] = 0;//-(float)yall*(float)deg2rad;	
	
		servo_pos_Race[0] = 0;
		servo_pos_Race[1] = 0;
		servo_pos_Race[2] = 0;

	  if(status_data_pitch==0) {pitch+=0.5;}
		else if(status_data_pitch==1) {pitch-=0.3;}
		
		if(pitch>=5){status_data_pitch =1;}
		else if(pitch<=-2) status_data_pitch =0;
		
		if(status_data_roll==0) {roll+=0.08;}
		else if(status_data_roll==1) {roll-=0.08;}
		
		if(roll>=2) status_data_roll =1;
		else if(roll<=-2) status_data_roll =0;
		
		if(status_data_yall==0) {yall+=0.05;}
		else if(status_data_yall==1) {yall-=0.05;}
		
		if(yall>=3) status_data_yall =1;
		else if(yall<=-3) status_data_yall =0;
		printf("\n\r ------  \n");
}
/*
Description: Init PWM of 6 Timer to control 6 Electric cylinder
Parameter  : None
Return     : None
*/
void Init_PWM(void)
{
	STM32F4_Pwm_Init(TIM2);
	STM32F4_Pwm_Init(TIM3);
	STM32F4_Pwm_Init(TIM4);
	STM32F4_Pwm_Init(TIM5);
	STM32F4_Pwm_Init(TIM12);
	STM32F4_Pwm_Init(TIM13);
}
/*
Description: Turn off all led indicator when restart system
Parameter  : None
Return     : None
*/
void Turn_off_led_notify(void)
{
   GPIO_ResetBits(GPIOE ,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6);
}
/*
Description: Function Master clear pulse from Slave1, Slave2 when restart system
Parameter  : None
Return     : None
*/
void Delete_encoder_slave(void)
{
   STM32F4_CanWriteData(120,'&');
	 STM32F4_CanWriteData(110,'&');
}
/*
Description: Function Master clear pulse from Slave1, Slave2 when restart system and turn off all led indicator
Parameter  : None
Return     : None
*/
void STM32F4_Clear_Pulse(void)
{
  Delete_encoder_slave();
	Turn_off_led_notify();
}
/*
Description: Configuration for all GPIO using in system
Parameter  : None
Return     : None
*/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
//	EXTI_InitTypeDef EXTI_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure PB0 PB1 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = STEWART_DIR_MOTOR1| STEWART_DIR_MOTOR2| STEWART_DIR_MOTOR3|STEWART_DIR_MOTOR4|STEWART_DIR_MOTOR5|STEWART_DIR_MOTOR6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(STEWART_PORT_DIR, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5| GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
}
/*
Description: Configuration Interrupt button using in system
Parameter  : None
Return     : None
*/
void Interrupt_Configure(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure PB0 PB1 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
		/* Tell system that you will use PD0 for EXTI_Line0 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4);
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource15);

    /* PD0 is connected to EXTI_Line0 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line4|EXTI_Line13|EXTI_Line15;
    /* Enable interrupt */
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStructure);
 
		NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    /* Set priority */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    /* Set sub priority */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    /* Enable interrupt */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStructure);
		
	
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    /* Set priority */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    /* Set sub priority */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
    /* Enable interrupt */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStructure);
	
}
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
uint8_t count1=0;
extern uint8_t State_Control;
int main(void)
{
	UDP_INIT_STATUS_t init_check;
  UDP_RECEIVE_t rec_check;
  char rx_buf[UDP_RX_BUFFER_SIZE];
  
#ifdef SERIAL_DEBUG
	init_USART1(230400);               // Init USART
	printf("\n init USART OK");
#endif
  Delay_Init();                     // Init Delay
	printf("\n init Delay OK");
	TM1_OVF_Configuration();          // Init Tim1 to make Looptime
	printf("\n init Time1 OK"); 
  LCD_LED_Init();                   // Init Led indicator
	printf("\n init led OK");
  STM32F4_CAN_Configuration();      // Init Can communication
	printf("\n init CAN OK");
	
	GPIO_Configuration();             // Init GPIO for Button Control and GPIO permit encoder counter from Master to Slave 
	printf("\n init GPIO OK");
  Interrupt_Configure();	          // Init GPIO Interrupt for Button Control
	printf("\n init Interrup OK");
  Init_PWM();                       // Init 6 Timer control 6 Electric Cylinder
	printf("\n init PWM OK");
	STM32F4_Clear_Pulse();            // Clear pulse before start while loop to clear Pulse from Slave
	printf("\n Clear pulse OK");	
	
	init_check=UB_UDP_Server_Init();  // Init UDP protocol
  if(init_check==UDP_INIT_OK)       // Check state of UDP connect
	{
  if(UB_UDP_Server_Connect()==UDP_CONNECT_OK) {printf("\n UDP Server OK \n");}
  }
  else {
	printf("\n\r Init UDP fail!");
	}

	delay(1000);
	printf("\n Start");
	State_Control=10;
  while (1)
  {  
    rec_check=UB_UDP_Server_Do(rx_buf);              // Check when UDP get new data
		
		if((count1>=10)&&(rec_check==UDP_RECEIVE_READY)) // Count1 will increase after end of loop, it spend about 5 ms -> 25 ms we will get 1 frame from server
			{
			count1=0;
			Caculate_data(rx_buf);                         // Handle data receive from Server
			GPIO_ToggleBits(GPIOE,GPIO_Pin_2);	           // Toggle Led to indicator
			}
	  if(State_Control==1)                             // Button 1, Signal to make Motion Base move from lowest position to balance position
			{				
			GPIO_SetBits(GPIOE ,GPIO_Pin_6);
			GPIO_ResetBits(GPIOE ,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
			STEWART_SetPos0(servo_pos_Begin);				
			}
		else if (State_Control==2)                      // Signal to allow Motion base move following data come from Host Server
			{
			GPIO_SetBits(GPIOE ,GPIO_Pin_5);
			GPIO_ResetBits(GPIOE ,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_6);
			STEWART_SetPos0(servo_pos_Begin);	
      }
		else if(State_Control==3)                        // Button 2: Signal to make Motion Base move from Balance positon to lowest position
			{  
			GPIO_SetBits(GPIOE ,GPIO_Pin_4);					
			GPIO_ResetBits(GPIOE ,GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_6);
      STEWART_SetPosEnd(servo_pos_end);
      }
		else if(State_Control==4)                        //Button 3 : Signal to make Motion Base move dead position
			{
			if(count1>=15)
				{
				count1=0;
		    Input_data();				
				}
			GPIO_SetBits(GPIOE ,GPIO_Pin_3);
			GPIO_SetBits(GPIOE ,GPIO_Pin_5);
			GPIO_ResetBits(GPIOE ,GPIO_Pin_4|GPIO_Pin_6);
      STEWART_SetPos(servo_pos_Race);
      }
		count1++;	
		fix_loop_time();	                               // Loop time function to make time loop in while(1) allway is 5 ms.
		}  
}

/**
  * @brief  Inserts a delay time.
  * @param  nCount: number of 10ms periods to wait for.
  * @retval None
  */
void Delay(uint32_t nCount)
{
  /* Capture the current local time */
  timingdelay = LocalTime + nCount;  

  /* wait until the desired delay finish */  
  while(timingdelay > LocalTime)
  {     
  }
}

/**
  * @brief  Updates the system local time
  * @param  None
  * @retval None
  */
void Time_Update(void)
{
  LocalTime += SYSTEMTICK_PERIOD_MS;
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
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
  {}
}
#endif


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
