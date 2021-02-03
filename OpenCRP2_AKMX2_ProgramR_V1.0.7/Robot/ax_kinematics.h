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
  * @日  期  2019-7-26
  * @内  容  机器人运动学解析
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_KINEMATICS_H
#define __AX_KINEMATICS_H


/* Includes ------------------------------------------------------------------*/	

#include "stm32f10x.h"

#define ENCODER_MAX 32767
#define ENCODER_MIN -32768 
#define SERVO_MAX   35           //rad 舵机极限转角 50度
#define SPEED_MAX   2            //m/s 线速度最大   2m/s
#define ACKMAN_WHEEL_DISTANCE   0.342   //阿克曼前后轮距
#define ACKMAN_WHEEL_TRACK      0.165   //阿克曼左右轮距
#define WHEEL_DIAMETER          0.0834   //阿克曼轮径
#define ENCODER_RESOLUTION      870     //编码器分辨率
#define PID_RATE                50      //PID频率
#define ROBOT_LINEAR_SPEED_LIMIT 5000   //机器人线速度限值 m/s*1000
#define ROBOT_ANGULAR_SPEED_LIMIT 5000  //机器人角速度限值 rad/s*1000

#define ENCODER_LOW_WRAP  ((ENCODER_MAX - ENCODER_MIN)*0.3+ENCODER_MIN)
#define ENCODER_HIGH_WRAP ((ENCODER_MAX - ENCODER_MIN)*0.7+ENCODER_MIN)
#define PI 3.1415926

#define ANGLE_CAL(X) (PI-((X)-1500)/2000*PI)

//ROBOT功能函数
void AX_Kinematics_Init(int16_t* robot_params);  //参数初始化
void AX_Kinematics_Forward(int16_t* input, int16_t* output); //正解(ForwardKinematics)：轮子编码值->底盘三轴里程计坐标
void AX_Kinematics_Inverse(int16_t* input, int16_t* output); //逆解(InverseKinematics)：底盘三轴速度->轮子速度

#endif
