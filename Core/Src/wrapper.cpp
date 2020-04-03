/*
 * wrapper.cpp
 *
 *  Updated on: 2020/03/02
 *      Author: takuj
 */

//lsm9ds1

#include "wrapper.hpp"

#define ADDRESS_ACCELGYRO    0x6B //0b1101010 //for i2c (DataSheet 30/72)
#define REGISTER_CTRL_REG1_G 0x10 //setting (DataSheet 38&45/72)
#define REGISTER_CTRL_REG4   0x1E //setting (DataSheet 38&50/72)
#define REGISTER_OUT_X_L_G   0x18 //output (DataSheet 38&50/72)
#define REGISTER_CTRL_REG10  0x24
/* Variable Begin */
int16_t angularVel[3];
int16_t offset[3];
float buf[3];
float deg[3];
uint8_t error;
int8_t data[6];
float timerCount;
/* Variable End */

/* Class Constructor Begin */
/* Class Constructor End */

/* Function Prototype Begin */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void updateValue();
/* Function Prototype End */

void init(void){
	uint8_t gyroConfig[]={0b11001010,0b00000011,0b01001001}; //CTRL_REG1_G,CTRL_REG2_G,CTRL_REG3_G (DataSheet 45/72) 範囲の設定等
	while(HAL_I2C_Mem_Write(&hi2c1,ADDRESS_ACCELGYRO<<1,REGISTER_CTRL_REG1_G,1,gyroConfig,sizeof(gyroConfig),100) != HAL_OK);

	uint8_t on = 0b00111000; //CTRL_REG4(DataSheet50/72) output enabled出力を有効にする
	while(HAL_I2C_Mem_Write(&hi2c1,ADDRESS_ACCELGYRO<<1,REGISTER_CTRL_REG4,1,&on,1,100) != HAL_OK);

#ifdef OFFSET
	uint32_t tmp = HAL_GetTick();
	uint16_t counter = 0;
	while(HAL_GetTick() - tmp < 10){ //loop for 10ms
		updateValue();
		for(uint8_t i; i<3; i++){
			buf[i] += angularVel[i];
		}
		counter++;
	}
	for(uint8_t i; i<3; i++){
		offset[i] = buf[i] / counter;
	}
#endif //OFFSET

	HAL_TIM_Base_Start_IT(&htim7);
}

void loop(void){

}

/* Function Body Begin */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	static int16_t filter[3][100]={};
	static uint8_t gyroCounter=0;
	if(htim == &htim7){
		uint8_t gyroConfig[]={0b11001010,0b00000011,0b01001001}; //CTRL_REG1_G,CTRL_REG2_G,CTRL_REG3_G (DataSheet 45/72) 範囲の設定等
		while(HAL_I2C_Mem_Write(&hi2c1,ADDRESS_ACCELGYRO<<1,REGISTER_CTRL_REG1_G,1,gyroConfig,sizeof(gyroConfig),100) != HAL_OK);

		uint8_t on = 0b00111000; //CTRL_REG4(DataSheet50/72) output enabled出力を有効にする
		while(HAL_I2C_Mem_Write(&hi2c1,ADDRESS_ACCELGYRO<<1,REGISTER_CTRL_REG4,1,&on,1,100) != HAL_OK);

		updateValue();
		/*
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		for(uint8_t n=0; n<3; n++){
			if(offset[n] - 10 < angularVel[n] && angularVel[n] < offset[n] + 10){
				angularVel[n] = offset[n];
			}
			filter[n][gyroCounter] = angularVel[n] - offset[n];
			filter[n][gyroCounter] = angularVel[n];
			for(uint8_t i=0; i<100; i++){
				//deg[n] += (double)(filter[n][i]) / 2000000.0;
			}
			deg[n] += (double)(angularVel[n] - offset[n])/ 14333.0;
			//while(deg[n]<-180.0) deg[n] += 360;
			//while(deg[n]> 180.0) deg[n] -= 360;
			if(gyroCounter++ >= 100 ){
				gyroCounter = 0;
			}
		}
		*/
	}
}
void updateValue(){
	HAL_I2C_Mem_Read(&hi2c1,ADDRESS_ACCELGYRO<<1,REGISTER_OUT_X_L_G,1,(uint8_t*)data,sizeof(data),100);
	for(uint8_t i; i<3; i++){
		angularVel[i] = (int16_t)data[2*i+1] << 8 | data[2*i];
	}

	timerCount += 0.1;
	if(timerCount>300){
		timerCount = 0;
	}
}
/* Function Body End */
