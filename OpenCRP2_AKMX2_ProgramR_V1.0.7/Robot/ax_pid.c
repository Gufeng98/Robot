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
  * @日  期  2019-8-8
  * @内  容  电机PID控制
  *
  ******************************************************************************
  */
#include "ax_pid.h"
#include "ax_delay.h"

#define PID_SCALE  0.01f  //PID缩放系数
#define PID_INTEGRAL_UP 1000  //积分上限

int16_t ax_motor_kp=600;  //电机转速PID-P
int16_t ax_motor_ki=0;    //电机转速PID-I
int16_t ax_motor_kd=400;  //电机转速PID-D

/**
  * @简  述  电机A PID控制函数
  * @参  数  spd_target:编码器速度目标值 ,范围（±250）
  *          spd_current: 编码器速度当前值
  * @返回值  电机PWM速度
  */
int16_t AX_PID_MotorVelocityCtlA(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral = 0;

	//获得偏差值
	bias = spd_target - spd_current;
	
	//计算偏差累加值
	bias_integral += bias;
	
	//抗积分饱和
	if(bias_integral>PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;
	
	//PID计算电机输出PWM值
	motor_pwm_out += ax_motor_kp*bias*PID_SCALE + ax_motor_kd*(bias-bias_last)*PID_SCALE + ax_motor_ki*bias_integral*PID_SCALE;
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;
  
	//返回PWM控制值
	return motor_pwm_out;
}	

/**
  * @简  述  电机B PID控制函数
  * @参  数  spd_target:编码器速度目标值 
  *          spd_target: 编码器速度当前值
  * @返回值  电机PWM速度
  */
int16_t AX_PID_MotorVelocityCtlB(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral = 0;
	
	//获得偏差值
	bias = spd_target - spd_current;
	
	//计算偏差累加值
	bias_integral += bias;
	
  //抗积分饱和
	if(bias_integral>PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;
	
	//PID计算电机输出PWM值
	motor_pwm_out += ax_motor_kp*bias*PID_SCALE + ax_motor_kd*(bias-bias_last)*PID_SCALE + ax_motor_ki*bias_integral*PID_SCALE;
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;

	//返回PWM控制值
	return motor_pwm_out;
}



/******************* (C) 版权 2019 XTARK **************************************/
