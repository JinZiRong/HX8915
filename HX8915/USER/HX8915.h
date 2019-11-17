#ifndef __HX8915_H
#define __HX8915_H
#include "gpio.h"
#include "stm32f1xx_hal.h"

#define SCL_1 HAL_GPIO_WritePin(SCL_GPIO_Port,SCL_Pin,GPIO_PIN_SET);
#define SCL_0 HAL_GPIO_WritePin(SCL_GPIO_Port,SCL_Pin,GPIO_PIN_RESET);
#define SDA_1 HAL_GPIO_WritePin(SDA_GPIO_Port,SDA_Pin,GPIO_PIN_SET);
#define SDA_0 HAL_GPIO_WritePin(SDA_GPIO_Port,SDA_Pin,GPIO_PIN_RESET);

#define HX8915_SlaveAddr 0x74

extern const uint8_t InitData[3];
extern uint8_t REG1[3];
extern uint8_t REG2[3];
extern uint8_t Data[12];
extern uint8_t sta;
extern uint16_t SCAL1;
extern uint16_t SCAL2;
extern uint16_t Puse;
extern uint16_t V1;
extern uint16_t V2;

void USER_PWM_SET(uint16_t scal1,uint16_t scal2,uint16_t puse);//输出方财德噬柚猫
void IIC_Delay(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(unsigned char IIC_Byte);
void IIC_Transmit(uint8_t slaveAddr,uint8_t regAddr,uint8_t *data,uint16_t dataLen);//模拟I2C
void HX8915_Init(void);    //HX8915初始化
void HX8915_SET(uint16_t v1,uint16_t v2);//V1\V2设置
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);//串口中断回调函数
uint8_t Data_R(uint8_t data[]);//数据接收处理

#endif
