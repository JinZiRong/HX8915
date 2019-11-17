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

void USER_PWM_SET(uint16_t scal1,uint16_t scal2,uint16_t puse);//������Ƶ�����è
void IIC_Delay(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(unsigned char IIC_Byte);
void IIC_Transmit(uint8_t slaveAddr,uint8_t regAddr,uint8_t *data,uint16_t dataLen);//ģ��I2C
void HX8915_Init(void);    //HX8915��ʼ��
void HX8915_SET(uint16_t v1,uint16_t v2);//V1\V2����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);//�����жϻص�����
uint8_t Data_R(uint8_t data[]);//���ݽ��մ���

#endif
