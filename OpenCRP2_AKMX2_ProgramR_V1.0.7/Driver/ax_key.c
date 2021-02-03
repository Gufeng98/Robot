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
  * @日  期  2018-10-26
  * @内  容  按键检测
  *
  ******************************************************************************
  * @说  明
  *
  * 1.获取键值，有软件消抖，按键抬起后输出键值。
  * 
  ******************************************************************************
  */

#include "ax_key.h"
#include "ax_delay.h"
#include "ax_sys.h"

/**
  * @简  述  KEY 按键初始化
  * @参  数  无
  * @返回值  无
  */
void AX_KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//使配置GPIO
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
  * @简  述  KEY 获取按键值
  * @参  数  无
  * @返回值  按键值，按键按下为1，抬起为0（抬起后输出键值）
  */
uint8_t AX_KEY_Scan(void)
{
	if(PCin(9) == 0)		
	{
		AX_Delayms(10);
		
		if(PCin(9) == 0)
		{
			while(PCin(9) == 0);
			
			return 1;
		}
	}
	
	return 0;
}



/******************* (C) 版权 2018 XTARK **************************************/
