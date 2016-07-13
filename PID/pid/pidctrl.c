

 
/**




**/

#include "stm32f10x.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "pidctrl.h"
#include "ds18b20.h" 
#include "main.h"
#ifndef ABS
#define ABS(a)	   (((a) < 0) ? -(a) : (a))
#endif

#define WINDUP_ON(_pid)         (_pid->features & PID_ENABLE_WINDUP)
#define DEBUG_ON(_pid)          (_pid->features & PID_DEBUG)
#define SAT_MIN_ON(_pid)        (_pid->features & PID_OUTPUT_SAT_MIN)
#define SAT_MAX_ON(_pid)        (_pid->features & PID_OUTPUT_SAT_MAX)
#define HIST_ON(_pid)           (_pid->features & PID_INPUT_HIST)


#define PID_CREATTRM_INTERVAL  	(250*4)  /*250*2 ms ---  pid once that again and again*/

#ifdef USING_PID





/*****external SSR ctrl module*********/
 
 uint8_t Interrupt_Extern=0;
 uint16_t Adj_Power_Time=0;
 uint16_t Power_Adj=0;


 void initBta16TMER()
 {
 //使用了一个100US的定时器，
 //初始化
 uint16_t 	usPrescalerValue;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//====================================时钟初始化===========================
	//使能定时器2时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//====================================定时器初始化===========================
	//定时器时间基配置说明
	//HCLK为72MHz，APB1经过2分频为36MHz
	//TIM2的时钟倍频后为72MHz（硬件自动倍频,达到最大）
	//TIM2的分频系数为3599，时间基频率为72 / (1 + Prescaler) = 100KHz,基准为10us
	//TIM最大计数值为usTim1Timerout50u	
	usPrescalerValue = (uint16_t) (72000000 / 25000) - 1;
	//预装载使能
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	//====================================中断初始化===========================
	//设置NVIC优先级分组为Group2：0-3抢占式优先级，0-2的响应式优先级
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//清除溢出中断标志位
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	//定时器2溢出中断关闭
	TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
	//定时器3禁能
	TIM_Cmd(TIM4, DISABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = usPrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = (uint16_t)( 1);//100us
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM4, 0);
	TIM_Cmd(TIM4, DISABLE);
 
 }
 void TIM4_IRQHandler(void)
{
	
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);	     //清中断标记
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);	 //清除定时器TIM3溢出中断标志位

	    Adj_Power_Time++;//increase time 
  	   AutoRunPowerAdjTask();
	}
	
}	
void EXTI4_IRQHandler(void)
{
     
	 	Interrupt_Extern=1;
		Adj_Power_Time=0;
    /* Clear the Key Button EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line4);

   
}
void InitzeroGPIO()
 {
 
//DrvGPIO_SetIntCallback(pfP0P1Callback, pfP2P3P4Callback);
//DrvGPIO_EnableInt(E_PORT2, E_PIN7, E_IO_RISING, E_MODE_EDGE);
	   //配置引脚为外部中断下降沿有效

	 NVIC_InitTypeDef NVIC_InitStructure;
	 GPIO_InitTypeDef GPIO_InitStructure;
     EXTI_InitTypeDef EXTI_InitStructure;

	  /* Enable GPIOC and GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
  
  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

 	/* Enable the EXTI3 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	/* configure PB4 as SSR ctrl pin */

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	  GPIO_SetBits(GPIOE,GPIO_Pin_2);
	  GPIO_ResetBits(GPIOE,GPIO_Pin_2);
	  	/* configure PB3 as external interrupt */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

     GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
	 
	    /* Configure  EXTI Line to generate an interrupt on falling edge */
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

	/* Clear the Key Button EXTI line pending bit */
	EXTI_ClearITPendingBit(EXTI_Line4); 
	  
 }


/*
设置功率百分比

 Power 范围是0-100
*/
void Set_Power(char Power)
{



	 Power_Adj=100-Power;



}

void Trdelay()
{

	  uint32_t i;
	  for(i=500;i;i--);


}
 void Trigger_SSR_Task()
 {

	 

 	// Delay_init(72);
	 GPIO_ResetBits(GPIOE,GPIO_Pin_2);	//
//	 GPIO_SetBits(GPIOE,GPIO_Pin_2);
	// Delay_us(30);
	  Trdelay();
  //  GPIO_ResetBits(GPIOE,GPIO_Pin_2);
//	 Delay_us(30);
	 GPIO_SetBits(GPIOE,GPIO_Pin_2);
	 // DrvGPIO_ClrBit(E_PORT4, E_PIN3); 
	 // DrvSYS_Delay(20);// 20us
	  //DrvGPIO_SetBit(E_PORT4, E_PIN3); 
	 
				   
 }

 /*----------Power Task Auto  Run---------------------------------------------------------------------*/
 void AutoRunPowerAdjTask()
 {
		   if(Interrupt_Extern==1)
		 {	
				 if( Adj_Power_Time>=Power_Adj)
				   {
				   
				   	Trigger_SSR_Task();		 
		 		   				 
				   	 
					 Interrupt_Extern=0;

				   }
		 		  	
			
		 }	


 }


 void InitSSR(void)
 {
 	InitzeroGPIO();
 
   initBta16TMER();
 }

 /*start of PID*/

 extern float Temp_pid[1];
 uint32_t PidCreatTrm=0;

 	PID_t pid;
	float result,kj;
	



extern void DS1820main(void); 		
//PID

void init_PID(void)
{
	/*初始化PID结构*/
		/*
	  	 比例因子=1；
		 积分因子=0；
		 微分因子=0；

	   */	
   	 pid_init(&pid, 15, 0, 0);
	  /*使能或者禁能PID计算的功能*/	
   //pid_enable_feature(&pid, , 0)		
	/*目标值设定*/		 
     pid_set(&pid,40);

}
void PidThread(uint32_t Time)
{  
     float temppid;
     			 
if (Time - PidCreatTrm >= PID_CREATTRM_INTERVAL)
  {
        PidCreatTrm =  Time;
	 
	   DS1820main();//采集当前温度  
	    
	temppid =  Temp_pid[0]/100;

	result  =  pid_calculate(&pid,temppid, 1) ;
		
		   if((result<100)||(result>0))/*0-100区间*/
		   {
		   	  Set_Power(result);
		   }

		  /*if(Power_Adj==100)
			   Power_Adj=0;
			   else
			   Power_Adj++;
		  	  */
		TIM_Cmd(TIM4, ENABLE);

  }
	
	
}


void pid_enable_feature(PID_t *pid, unsigned int feature, float value)
{
    pid->features |= feature;

    switch (feature) {
        case PID_ENABLE_WINDUP:
            /* integral windup is in absolute output units, so scale to input units */
            pid->intmax = ABS(value / pid->ki);
            break;
        case PID_DEBUG:
            break;
        case PID_OUTPUT_SAT_MIN:
            pid->sat_min = value;
            break;
        case PID_OUTPUT_SAT_MAX:
            pid->sat_max = value;
            break;
        case PID_INPUT_HIST:
            break;
    }
}

/**
 *
 * @param pid
 * @param kp
 * @param ki
 * @param kd
 */
void pid_init(PID_t *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->sp = 0;
	pid->error_previous = 0;
	pid->integral = 0;

    pid->features = 0;

    if (DEBUG_ON(pid))
        printf("setpoint,value,P,I,D,error,i_total,int_windup\n");
}

void pid_set(PID_t *pid, float sp)
{
	pid->sp = sp;
	pid->error_previous = 0;
	pid->integral = 0;
}

/**
 *
 * @param pid
 * @param val  NOW VALUE
 * @param dt   
 * @return
 */
float pid_calculate(PID_t *pid, float val, float dt)
{
	float i,d, error, total;

	error = pid->sp - val;
	i = pid->integral + (error * dt);
	d = (error - pid->error_previous) / dt;

    total = (error * pid->kp) + (i * pid->ki) + (d * pid->kd);

    if (DEBUG_ON(pid))
        printf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d\n", 
                pid->sp,val,
                (error * pid->kp), (i * pid->ki), (d * pid->kd),
                error, pid->integral, ABS(i) == pid->intmax);

    if ( WINDUP_ON(pid) ) {
        if ( i < 0 )
            i = ( i < -pid->intmax ? -pid->intmax : i );
        else
   		    i = ( i < pid->intmax ? i : pid->intmax );
    }
    pid->integral = i;

    if ( SAT_MIN_ON(pid) && (total < pid->sat_min) )
        return pid->sat_min;
    if ( SAT_MAX_ON(pid) && (total > pid->sat_max) )
        return pid->sat_max;

	pid->error_previous = error;
	return total;
}


 /*end of PID*/



 #endif

