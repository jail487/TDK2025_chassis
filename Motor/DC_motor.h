/*
 * DC_motor.h
 *
 *  Created on: Jan 18, 2025
 *      Author: 88698
 */

#ifndef LIBRARY_INC_DC_MOTOR_H_
#define LIBRARY_INC_DC_MOTOR_H_

#include "stm32h7xx_hal.h"




class DC_motor{
public:
	void setup();
	void PI_run();
	void updateSpeed(int sign);
	float get_speed() const { return speed; } 
	void setspeed(float target_speed);
	void set_motor_parameter(float reduction_ratio,int resolution) ;
	void setPulse(float dutyCycle) ;
	void setDirection(bool direction) ;


	DC_motor(TIM_HandleTypeDef *_enc_htim, GPIO_TypeDef *_dirPort, uint16_t _dirPin, TIM_HandleTypeDef *_PWM_htim,
			uint32_t _PWM_TIM_CHANNEL,float _kp,float _ki,bool dirpin) {
		enc_htim = _enc_htim;
		dirPort = _dirPort;
		dirPin = _dirPin;
		PWM_htim = _PWM_htim;
		PWM_TIM_CHANNEL = _PWM_TIM_CHANNEL;
		kp = _kp;
		ki = _ki;
		dir_pin = dirpin;
	};
private:
	//PID parameter
	float kp = 0.f ,ki = 0.f,kd = 0.f;
	//error
	float error = 0.f,pre_error = 0.f,integral = 0.f,differential = 0.f;
	//sp of velocity
	float speed = 0.f;	
	float sp = 0.f;
	//motor and encoder information
	float span = 0.001;
	int resolution = 512;
	float reduction_ratio = 64.f;
	int dir = 0;
	int arr = 799;
	int arr_max = 800;
	bool dir_pin = 0;
	int pulse = 0;
	float u = 0.f;
	TIM_HandleTypeDef *enc_htim;
	//motor driver
	GPIO_TypeDef *dirPort;
	uint16_t dirPin;
	TIM_HandleTypeDef *PWM_htim;
	uint32_t PWM_TIM_CHANNEL;
	
};








#endif /* LIBRARY_INC_DC_MOTOR_H_ */
