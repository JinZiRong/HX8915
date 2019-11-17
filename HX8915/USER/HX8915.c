#include "HX8915.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

const uint8_t InitData[3]={0x00,0x00,0x00};  //�Ĵ�����ʼ��д��ֵ
uint8_t REG1[3]={0x00,0x00,0x00};            //B1���Ĵ���ֵ
uint8_t REG2[3]={0x00,0x00,0x00};            //B2���Ĵ���ֵ
uint8_t Data[12];                            //���ڽ������ݻ���
uint8_t sta=0;                               //�ж�λ
uint16_t SCAL1=0;                            //Ԥ��Ƶ
uint16_t SCAL2=0;                            //��Ƶ
uint16_t Puse=0;                             //ռ�ձ�
uint16_t V1=0;                               //V1��ѹд��ֵ
uint16_t V2=0;                               //V2��ѹд��ֵ

void USER_PWM_SET(uint16_t scal1,uint16_t scal2,uint16_t puse)
{
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_4);
	
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = scal1-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = scal2-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	
  HAL_TIM_PWM_Init(&htim2);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = puse-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
}

void IIC_Delay(void)
{
	uint16_t i;
	for(i=0;i<100;i++);
}

void IIC_Start(void)
{
	SDA_1;
	SCL_1;
	IIC_Delay();
	IIC_Delay();
	SDA_0;
	IIC_Delay();
  SCL_0;
}

void IIC_Stop(void)
{
   SCL_0;
	 SDA_0;
	 IIC_Delay();
	 SCL_1;
	 SDA_1;
	 IIC_Delay();
}

void IIC_Send_Byte(unsigned char IIC_Byte)
{
	unsigned char i;
	SCL_0;
	for(i=0;i<8;i++)
	{
		if(((IIC_Byte&0x80)>>7)==1)
		{
			SDA_1;
		}
		else
		{
			SDA_0;
		}
    IIC_Delay();
		SCL_1;
    IIC_Delay();
		SCL_0;
    IIC_Delay();
		IIC_Byte<<=1;
	}
	IIC_Delay();
	SDA_1;
	IIC_Delay();
	SCL_1;
  IIC_Delay();
	SCL_0;
  IIC_Delay();
}

void IIC_Transmit(uint8_t slaveAddr,uint8_t regAddr,uint8_t *data,uint16_t dataLen)
{
	IIC_Start();
	IIC_Send_Byte(slaveAddr<<1);
	IIC_Send_Byte(regAddr);
	for (uint16_t cnt=0;cnt<dataLen;cnt++)
	{
		IIC_Send_Byte(data[cnt]);
	}
	IIC_Stop();
}

void HX8915_Init(void)
{
	IIC_Transmit(HX8915_SlaveAddr,0x02,(uint8_t *)&InitData,3);
	IIC_Transmit(HX8915_SlaveAddr,0x25,(uint8_t *)&InitData,3);
}

void HX8915_SET(uint16_t v1,uint16_t v2)
{
	if((v1>>15)==(v2>>15))            //��V1,V2���λ��ͬ����ͬ��λ���
	{
		REG1[0]=(v1>>4)&0x3F;
	  REG1[1]=((v1&0x0F)<<4)|((v2>>8)&0x03);
	  REG1[2]=v2&0xFF;
		REG2[0]=0x00;
		REG2[1]=0x00;
		REG2[2]=0x00;
	}
	else                               //��V1,V2���λ��ͬ���������λ���������
	{
		REG1[0]=(v1>>4)&0x3F;
		REG1[1]=(v1&0x0F)<<4;
		REG1[2]=0x00;
		REG2[0]=0x00;
		REG2[1]=(v2>>8)&0x03;
		REG2[2]=v2&0xFF;
	}
	
	IIC_Transmit(HX8915_SlaveAddr,0x02,(uint8_t *)&REG1,3);
	IIC_Transmit(HX8915_SlaveAddr,0x25,(uint8_t *)&REG2,3);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==huart1.Instance)
	{
    if(USER_Receive(Data)==1)
		{
			HX8915_SET(V1,V2);
			USER_PWM_SET(SCAL1,SCAL2,Puse);
		}
	}
	HAL_UART_Receive_IT(&huart1,(uint8_t *)&Data,12);
}

uint8_t USER_Receive(uint8_t data[])
{
	if(data[0]==0x19&&sta==0)    //֡ͷ1
	{
		sta++;
	}
	if(data[1]==0X65&&sta==1)    //֡ͷ2
	{
		sta++;
	}
	if(sta==2)
	{
		V1=(data[2]<<8)+data[3];
		V2=(data[4]<<8)+data[5];
		SCAL1=(data[6]<<8)+data[7];
		SCAL2=(data[8]<<8)+data[9];
		Puse=(data[10]<<8)+data[11];
		sta=0;
		return 1;
  }
	return 0;
}
