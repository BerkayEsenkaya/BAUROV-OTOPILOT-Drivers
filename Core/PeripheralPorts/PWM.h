#ifndef PERIPHERALPORTS_PWM_H_
#define PERIPHERALPORTS_PWM_H_

typedef struct{
	void *handle;
	uint8_t TimerNO;
	uint32_t TimerChannelNo;
	volatile uint16_t PWM;
	volatile float PWM_Raw;
	uint16_t PWM_Max;
	uint16_t PWM_Min;
	uint16_t PWM_Notr;
	uint32_t dataNumb;
}PWM_HandleTypeDef_T;

extern PWM_HandleTypeDef_T THRUSTER_Vert_R, THRUSTER_Vert_L, THRUSTER_Horz_R, THRUSTER_Horz_L;

void PWM_Init(PWM_HandleTypeDef_T *handle, void* timerHandle, uint32_t timerChannel, uint16_t PWM_Min, uint16_t PWM_Max, uint16_t PWM_Notr);
void PWM_SetPulse(PWM_HandleTypeDef_T *handle);

#endif /* PERIPHERALPORTS_PWM_H_ */
