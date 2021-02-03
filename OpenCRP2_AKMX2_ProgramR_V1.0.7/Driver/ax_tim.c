/**			                                                    
		   ____                    _____ _____  _____        XTARK@塔克创新
		  / __ \                  / ____|  __ \|  __ \  
		 | |  | |_ __   ___ _ __ | |    | |__) | |__) |
		 | |  | | '_ \ / _ \ '_ \| |    |  _  /|  ___/ 
		 | |__| | |_) |  __/ | | | |____| | \ \| |     
		  \____/| .__/ \___|_| |_|\_____|_|  \_\_|     
		    		| |                                    
		    		|_|  OpenCRP 树莓派 专用ROS机器人控制器                                   
									 
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 官网网站： www.xtark.cn
  * 淘宝店铺： https://shop246676508.taobao.com  
  * 塔克媒体： www.cnblogs.com/xtark（博客）
	* 塔克微信： 微信公众号：塔克创新（获取最新资讯）
  *                              
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @版  本  V1.0
  * @日  期  2019-2-8
  * @内  容  定时器定时
  *
  ******************************************************************************
  * @说  明
  *
  * 1.TIM6定时器终于产生定时信号，通过标志位判断的函数传递
  *
  ******************************************************************************
  */

#include "ax_tim.h" 

//中断循环状态控制标志位
uint8_t ax_flag_tim6 = 0;

/**
  * @简  述  TIM6 定时器初始化（溢出中断）
  * @参  数  cnt_us 设置溢出计数值，单位1us，范围0-65535 
  * @返回值  无
  */
void AX_TIM6_Init(uint16_t cnt_us)
{ 
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE); //使能时钟
	
	 // 累计 TIM_Period个后产生一个更新或者中断
	TIM_TimeBaseInitStructure.TIM_Period = cnt_us-1;  //自动重装载值，最大65535
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=1000000Hz
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72-1; //定时器分频,计数周期（period x 1us）
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStructure); //初始化定时器
	
	//清除定时器更新中断标志位
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
	
	//开启定时器更新中断
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(TIM6,ENABLE); //使能定时器 
	
	// 设置中断来源
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn; 	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	 	// 设置抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		// 设置子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

/**
  * @简  述  TIM6 定时器开启关闭
  * @参  数  NewState  This parameter can be: ENABLE or DISABLE.
  * @返回值  无
  */
void AX_TIM6_Cmd(FunctionalState NewState)
{
	TIM_SetCounter(TIM6, 0);
	TIM_Cmd(TIM6, NewState);
}

/**
  * @简  述  TIM6 中断处理函数
  * @参  数  无
  * @返回值  无
  */
void  TIM6_IRQHandler(void)
{
	if ( TIM_GetITStatus( TIM6, TIM_IT_Update) != RESET ) 
	{	
		//中断处理内容
		ax_flag_tim6 = 1;  //置位10ms标志位
		
		TIM_ClearITPendingBit(TIM6 , TIM_IT_Update);  		 
	}		 	
}

/**
  * @brief  检测是否产生中断
  * @param  None
  * @retval None
  */
uint8_t AX_TIM6_CheckIrqStatus(void)
{
	//确认中断,进入控制周期
	if(ax_flag_tim6 != 0) 
	{
		ax_flag_tim6 = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}
/******************* (C) 版权 2018 XTARK **************************************/
