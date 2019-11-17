#include "HX8915.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

const uint8_t InitData[3]={0x00,0x00,0x00};  //寄存器初始化写入值
uint8_t REG1[3]={0x00,0x00,0x00};            //B1区寄存器值
uint8_t REG2[3]={0x00,0x00,0x00};            //B2区寄存器值
uint8_t Data[12];                            //串口接收数据缓存
uint8_t sta=0;                               //判断位
uint16_t SCAL1=0;                            //预分频
uint16_t SCAL2=0;                            //分频
uint16_t Puse=0;                             //占空比
uint16_t V1=0;                               //V1电压写入值
uint16_t V2=0;                               //V2电压写入值

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
	//IIC_Delay();
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
	//IIC_Transmit(HX8915_SlaveAddr,0x00,(uint8_t *)0x00,1);
	IIC_Transmit(HX8915_SlaveAddr,0x02,(uint8_t *)&InitData,3);
	IIC_Transmit(HX8915_SlaveAddr,0x25,(uint8_t *)&REG1,3);
	//IIC_Transmit(HX8915_SlaveAddr,0x00,(uint8_t *)0x02,1);
	//wIIC_Transmit(HX8915_SlaveAddr,0x00,(uint8_t *)0x02,1);
}

void HX8915_SET(uint16_t v1,uint16_t v2)
{
//	REG1[0]=(v1>>4)&0x3F;
//	REG1[1]=((v1&0x0F)<<4)|((v2>>8)&0x03);
//	REG1[2]=v2&0xFF;
//	//IIC_Transmit(HX8915_SlaveAddr,0x00,(uint8_t *)0x00,1);
//	IIC_Transmit(HX8915_SlaveAddr,0x02,(uint8_t *)&REG1,3);
//	IIC_Transmit(HX8915_SlaveAddr,0x00,(uint8_t *)0x02,1);
	
	if((v1>>15)==(v2>>15))            //若V1,V2最高位相同，则同相位输出
	{
		REG1[0]=(v1>>4)&0x3F;
	  REG1[1]=((v1&0x0F)<<4)|((v2>>8)&0x03);
	  REG1[2]=v2&0xFF;
		REG2[0]=0x00;
		REG2[1]=0x00;
		REG2[2]=0x00;
	}
	else                               //若V1,V2最高位不同，则输出相位相差半个周期
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
	//IIC_Transmit(HX8915_SlaveAddr,0x00,(uint8_t *)0x02,1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==huart1.Instance)
	{
    if(Data_R(Data)==1)
		{
			HX8915_SET(V1,V2);
			USER_PWM_SET(SCAL1,SCAL2,Puse);
		}
	}
	HAL_UART_Receive_IT(&huart1,(uint8_t *)&Data,12);
}

uint8_t Data_R(uint8_t data[])
{
	if(data[0]==0x19&&sta==0)    //帧头1
	{
		sta++;
	}
	if(data[1]==0X65&&sta==1)    //帧头2
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

//uint8_t USER_Receive(uint8_t data)
//{
//	if(data==0x19&&sta==0)
//	{
//		sum=sum+data;
//		sta++;
//	}
//	
//	else if(data==0x65&&sta==1)
//	{
//		sum=sum+data;
//		sta++;
//	}
//	
//	else if(data==0x20&&sta==2)
//	{
//		sum=sum+data;
//		sta++;
//	}
//	
//	else if(data==0x19&&sta==3)
//	{
//		sum=sum+data;
//		sta++;
//	}
//	
//	else if(sta==4)//判断改变值
//	{
//		if(data==0x01)//改变频率
//			return_flag=1;
//		else if(data==0x02)//改变幅值
//			return_flag=2;
//		else
//			return_flag=0;//错误值
//		
//		sum=sum+data;
//		sta++;
//	}
//	//////////////////////////////////////////////////FRQ
//	else if(return_flag==1&&sta==5)//预分频高八位
//	{
//		SCAL1=data<<8;
//		sum=sum+data;
//		sta++;
//	}
//	
//	else if(return_flag==1&&sta==6)//预分频低八位
//	{
//		SCAL1=SCAL1+data;
//		sum=sum+data;
//		sta++;
//	}
//	
//	else if(return_flag==1&&sta==7)//分频高八位
//	{
//		SCAL2=data<<8;
//		sum=sum+data;
//		sta++;
//	}
//	
//	else if(return_flag==1&&sta==8)//分频低八位
//	{
//		SCAL2=SCAL2+data;
//		sum=sum+data;
//		sta++;
//	}
//	
//	else if(return_flag==1&&sta==9)//占空比高八位
//	{
//		Puse=data<<8;
//		sum=sum+data;
//		sta++;
//	}
//	
//	else if(return_flag==1&&sta==10)//占空比低八位
//	{
//		Puse=Puse+data;
//		sum=sum+data;
//		sta++;
//	}
//	
//	else if(return_flag==1&&sta==11&&(data==(sum&0xFF)))
//	{
//		sta=0;
//		sum=0;
//		return_flag=0;
//		return 1;
//	}
//	//////////////////////////////////////////////////////////////////////幅值
//	else if(return_flag==2&&sta==5)//V1高八位
//	{
//		V1=data<<8;
//		sum=sum+data;
//		sta++;
//	}
//	
//	else if(return_flag==2&&sta==6)//V1低八位
//	{
//		V1=V1+data;
//		sum=sum+data;
//		sta++;
//	}
//	
//	else if(return_flag==2&&sta==7)//V2高八位
//	{
//		V2=data<<8;
//		sum=sum+data;
//		sta++;
//	}
//	
//	else if(return_flag==2&&sta==8)//V2低八位
//	{
//		V2=V2+data;
//		sum=sum+data;
//		sta++;
//	}
//	else if(return_flag==2&&sta==9&&(data==(sum&0xFF)))
//	{
//		sta=0;
//		sum=0;
//		return_flag=0;
//		return 2;
//	}
//	//////////////////////////////////////////////////////////////////////错误清零返回
//	else
//	{
//		SCAL1=0;
//		SCAL2=0;
//		Puse=0;
//		V1=0;
//		V2=0;
//		sum=0;
//		sta=0;
//		return_flag=0;
//		return 0;
//	}
//	
//	return 0;
//	/////////////////////////////////////////////////////////////////
////	else if(sta==5)//频率高八位
////	{
////		FRQ=data<<8;
////		sum=sum+data;
////		sta++;
////	}
////	
////	else if(sta==6)//频率低八位
////	{
////		FRQ=FRQ+data;
////		sum=sum+data;
////		sta++;
////	}
////	
////	else if(sta==7&data==0xEC)//分割
////	{
////		sum=sum+data;
////		sta++;
////	}
////	
////	else if(sta==8)//V1高八位
////	{
////		V1=data<<8;
////		sum=sum+data;
////		sta++;
////	}
////	
////	else if(sta==9)//V1低八位
////	{
////		V1=V1+data;
////		sum=sum+data;
////		sta++;
////	}
////	
////	else if(sta==10)//V2高八位
////	{
////		V2=data<<8;
////		sum=sum+data;
////		sta++;
////	}
////	
////	else if(sta==11)//V2低八位
////	{
////		V2=V2+data;
////		sum=sum+data;
////		sta++;
////	}
////	
////	else if(sta==12&&(data==(sum&0xFF)))//校验位
////	{
////		sta++;
////	}
////	
////	else if(sta==13&&data==0x13)
////	{
////		sta++;
////	}
////	
////	else if(sta==14&&data==0x14)
////	{
////		sta++;
////	}
////	
////	else
////	{
////		sta=0;
////		sum=0;
////		FRQ=0;
////		V1=0;
////		V2=0;
////	}
////	
////	if(sta==15)
////	{
////		sta=0;
////		sum=0;
////		return 1;
////	}
////	else return 0;
//}
