#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h" 
#include "max31865.h"
#include "string.h"
#include "math.h"
#include "gps.h"
#include "stm32f10x.h"  //包含需要的头文件
#include "main.h"       //包含需要的头文件
#include "delay.h"      //包含需要的头文件
#include "usart.h"     //包含需要的头文件
#include "uart4.h"     //包含需要的头文件
#include "timer1.h"     //包含需要的头文件
#include "timer2.h"     //包含需要的头文件
#include "timer3.h"     //包含需要的头文件
#include "timer4.h"     //包含需要的头文件
#include "wifi.h"	    //包含需要的头文件
#include "mqtt.h"       //包含需要的头文件
#include "dht11.h"
#include "iic.h"        //包含需要的头文件
#include "stdio.h"
#include "adc.h"


/************************************************
 ALIENTEK 战舰STM32F103开发板实验18
内部温度传感器 实验 
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/
double longtitude,latitude,humidity,temperature,oiltemper,speed,ic2;
float ic;
int pressure,gas,altitude;
u8 iaq,altitude1;

u16 oiltemper1,temperature1,humidity1,speed1,adcx1,ic1;
u32 gas1,latitude1,longtitude1,pressure1;


float temperatureTs;


u8 USART1_TX_BUF[USART3_MAX_RECV_LEN]; 					//串口1,发送缓存区
nmea_msg gpsx; 											//GPS信息
__align(4) u8 dtbuf[50];   								//打印缓存器
const u8*fixmode_tbl[4]={"Fail","Fail"," 2D "," 3D "};	//fix mode字符串 



static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_X;
  
  /* 4个抢占优先级，4个响应优先级 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  /*抢占优先级可打断中断级别低的中断*/
	/*响应优先级按等级执行*/
	NVIC_X.NVIC_IRQChannel = USART2_IRQn;//中断向量
  NVIC_X.NVIC_IRQChannelPreemptionPriority = 3;//抢占优先级
  NVIC_X.NVIC_IRQChannelSubPriority = 3;//响应优先级
  NVIC_X.NVIC_IRQChannelCmd = ENABLE;//使能中断响应
  NVIC_Init(&NVIC_X);
}


void send_Instruction(void)
{
	uint8_t send_data[4]={0};
	send_data[0]=0xa5;
	send_data[1]=0x55;
	send_data[2]=0x3F;
	send_data[3]=0x39;
	USART_Send_bytes(send_data,4);//发送
	
	delay_ms(100);
	
	send_data[0]=0xa5;
	send_data[1]=0x56;
	send_data[2]=0x02;
	send_data[3]=0xfd;
	USART_Send_bytes(send_data,4);//发送自动输出指令
	delay_ms(100);
}


//int fputc(int ch, FILE *f)
//{
// while (!(USART1->SR & USART_FLAG_TXE));
// USART_SendData(USART1, (unsigned char) ch);// USART1 可以换成 USART2 等
// return (ch);
//}


	  
//显示GPS定位信息 
void Gps_Msg_Show(void)
{

 	float tp;		   
	POINT_COLOR=BLUE;  	 
	tp=gpsx.longitude;	   
	sprintf((char *)dtbuf,"Longitude:%.5f %1c   ",tp/=100000,gpsx.ewhemi);	//得到经度字符串
	longtitude =(tp/=100000) ;
	longtitude1=gpsx.longitude;
 	LCD_ShowString(30,120,200,16,16,dtbuf);	 	   
	tp=gpsx.latitude;	   
	sprintf((char *)dtbuf,"Latitude:%.5f %1c   ",tp/=100000,gpsx.nshemi);	//得到纬度字符串
	latitude=(tp/=100000);
	latitude1=gpsx.latitude;
 	LCD_ShowString(30,140,200,16,16,dtbuf);	 	 
	tp=gpsx.altitude;	   
 	sprintf((char *)dtbuf,"Altitude:%.1fm     ",tp/=10);	    			//得到高度字符串
 	LCD_ShowString(30,160,200,16,16,dtbuf);	 			   
	tp=gpsx.speed;	
  speed1=	gpsx.speed;
 	sprintf((char *)dtbuf,"Speed:%.3fkm/h     ",tp/=1000);		    		//得到速度字符串	
	speed=tp/=1000;
 	LCD_ShowString(30,180,200,16,16,dtbuf);	 				    
	if(gpsx.fixmode<=3)														//定位状态
	{  
		sprintf((char *)dtbuf,"Fix Mode:%s",fixmode_tbl[gpsx.fixmode]);	
	  LCD_ShowString(30,200,200,16,16,dtbuf);			   
	}	 	   
	sprintf((char *)dtbuf,"GPS+BD Valid satellite:%02d",gpsx.posslnum);	 		//用于定位的GPS卫星数
 	LCD_ShowString(30,220,200,16,16,dtbuf);	    
	sprintf((char *)dtbuf,"GPS Visible satellite:%02d",gpsx.svnum%100);	 		//可见GPS卫星数
 	LCD_ShowString(30,240,200,16,16,dtbuf);
	
	sprintf((char *)dtbuf,"BD Visible satellite:%02d",gpsx.beidou_svnum%100);	 		//可见北斗卫星数
 	LCD_ShowString(30,260,200,16,16,dtbuf);
	
	sprintf((char *)dtbuf,"UTC Date:%04d/%02d/%02d   ",gpsx.utc.year,gpsx.utc.month,gpsx.utc.date);	//显示UTC日期
	LCD_ShowString(30,280,200,16,16,dtbuf);		    
	sprintf((char *)dtbuf,"UTC Time:%02d:%02d:%02d   ",gpsx.utc.hour,gpsx.utc.min,gpsx.utc.sec);	//显示UTC时间
  LCD_ShowString(30,300,200,16,16,dtbuf);		
  	
}


int main(void)
{	

	u16 i,rxlen;
	u16 lenx;
	u8 key=0XFF;
	u8 upload=0;	
	u16 adcx,ic;
	float temp;
	u16 temp5;
	double temp3;
	short temp4;
	uint8_t data_buf[50]={0},count=0;

  float Temperature,Humidity;
  uint32_t Gas;
  uint32_t Pressure;
  uint16_t IAQ;
	int16_t Altitude=0;
  uint8_t IAQ_accuracy;
	uint16_t temp1=0;
  int16_t temp2=0;
	
	//delay_init(72);//72M 
	extern int pressure;
	
	
	
	
	delay_init();                   //延时功能初始化              
	Uart4_Init(115200);
	TIM4_Init(300,7200);            //TIM4初始化，定时时间 300*7200*1000/72000000 = 30ms
	LED_Init();	                    //LED初始化
	KEY_Init();                     //按键初始化
	IIC_Init();                     //初始化IIC接口
	Adc_Init();
	
	delay_init();
	LED_Init();
	uart_init(9600);
	Usart_Int2(9600);
	NVIC_Configuration();//串口中断优先级配置
	send_Instruction();//向模块发送指令
	GPIO_SetBits(GPIOB,GPIO_Pin_5);

  MAX31865_Init();
	MAX31865_Cfg();  

			
	delay_init();	    	 //延时函数初始化	  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为115200
 //	usmart_dev.init(72);		//初始化USMART		
 	LED_Init();		  			//初始化与LED连接的硬件接口
	KEY_Init();					//初始化按键
	LCD_Init();			   		//初始化LCD   
	usart3_init(38400);		//初始化串口3 
	POINT_COLOR=RED;
	LCD_ShowString(30,20,200,16,16,"JIE");	  

	LCD_ShowString(30,60,200,16,16,"ENVIR TEST");
	LCD_ShowString(30,80,200,16,16,"KEY0:Upload NMEA Data SW");   	 										   	   
 
	if(SkyTra_Cfg_Rate(5)!=0)	//设置定位信息更新速度为5Hz,顺便判断GPS模块是否在位. 
	{
   	LCD_ShowString(30,120,200,16,16,"SkyTraF8-BD Setting...");
		do
		{
			usart3_init(9600);			//初始化串口3波特率为9600
	  	SkyTra_Cfg_Prt(3);			//重新设置模块的波特率为38400
			usart3_init(38400);			//初始化串口3波特率为38400
      key=SkyTra_Cfg_Tp(100000);	//脉冲宽度为100ms
		}while(SkyTra_Cfg_Rate(5)!=0&&key!=0);//配置SkyTraF8-BD的更新速率为5Hz
	  LCD_ShowString(30,120,200,16,16,"SkyTraF8-BD Set Done!!");
		delay_ms(500);
		LCD_Fill(30,120,30+200,120+16,WHITE);//清除显示 
	}
	
	WiFi_ResetIO_Init();            //初始化WiFi的复位IO
  MQTT_Buff_Init();               //初始化接收,发送,命令数据的 缓冲区 以及各状态参数
	AliIoT_Parameter_Init();	    //初始化连接阿里云IoT平台MQTT服务器的参数	

	while(1) 
	{		
/*--------------------------------------------------------------------*/
		/*   Connect_flag=1同服务器建立了连接,我们可以发布数据和接收推送了    */
		/*--------------------------------------------------------------------*/
		if(Connect_flag==1){ 

			
			
			/*-------------------------------------------------------------*/
			/*                     处理发送缓冲区数据                      */
			/*-------------------------------------------------------------*/
				if(MQTT_TxDataOutPtr != MQTT_TxDataInPtr){                //if成立的话，说明发送缓冲区有数据了
				//3种情况可进入if
				//第1种：0x10 连接报文
				//第2种：0x82 订阅报文，且ConnectPack_flag置位，表示连接报文成功
				//第3种：SubcribePack_flag置位，说明连接和订阅均成功，其他报文可发
				if((MQTT_TxDataOutPtr[2]==0x10)||((MQTT_TxDataOutPtr[2]==0x82)&&(ConnectPack_flag==1))||(SubcribePack_flag==1)){    
					printf("发送数据:0x%x\r\n",MQTT_TxDataOutPtr[2]);  //串口提示信息
					MQTT_TxData(MQTT_TxDataOutPtr);                       //发送数据
					MQTT_TxDataOutPtr += BUFF_UNIT;                       //指针下移
					if(MQTT_TxDataOutPtr==MQTT_TxDataEndPtr)              //如果指针到缓冲区尾部了
						MQTT_TxDataOutPtr = MQTT_TxDataBuf[0];            //指针归位到缓冲区开头
				} 				
			}//处理发送缓冲区数据的else if分支结尾
			
			/*-------------------------------------------------------------*/
			/*                     处理接收缓冲区数据                      */
			/*-------------------------------------------------------------*/
			if(MQTT_RxDataOutPtr != MQTT_RxDataInPtr){  //if成立的话，说明接收缓冲区有数据了														
				printf("接收到数据:");
				/*-----------------------------------------------------*/
				/*                    处理CONNACK报文                  */
				/*-----------------------------------------------------*/				
				//if判断，如果第一个字节是0x20，表示收到的是CONNACK报文
				//接着我们要判断第4个字节，看看CONNECT报文是否成功
				if(MQTT_RxDataOutPtr[2]==0x20){             			
				    switch(MQTT_RxDataOutPtr[5]){					
						case 0x00 : printf("CONNECT报文成功\r\n");                            //串口输出信息	
								    ConnectPack_flag = 1;                                        //CONNECT报文成功，订阅报文可发
									break;                                                       //跳出分支case 0x00                                              
						case 0x01 : printf("连接已拒绝，不支持的协议版本，准备重启\r\n");     //串口输出信息
									Connect_flag = 0;                                            //Connect_flag置零，重启连接
									break;                                                       //跳出分支case 0x01   
						case 0x02 : printf("连接已拒绝，不合格的客户端标识符，准备重启\r\n"); //串口输出信息
									Connect_flag = 0;                                            //Connect_flag置零，重启连接
									break;                                                       //跳出分支case 0x02 
						case 0x03 : printf("连接已拒绝，服务端不可用，准备重启\r\n");         //串口输出信息
									Connect_flag = 0;                                            //Connect_flag置零，重启连接
									break;                                                       //跳出分支case 0x03
						case 0x04 : printf("连接已拒绝，无效的用户名或密码，准备重启\r\n");   //串口输出信息
									Connect_flag = 0;                                            //Connect_flag置零，重启连接						
									break;                                                       //跳出分支case 0x04
						case 0x05 : printf("连接已拒绝，未授权，准备重启\r\n");               //串口输出信息
									Connect_flag = 0;                                            //Connect_flag置零，重启连接						
									break;                                                       //跳出分支case 0x05 		
						default   : printf("连接已拒绝，未知状态，准备重启\r\n");             //串口输出信息 
									Connect_flag = 0;                                            //Connect_flag置零，重启连接					
									break;                                                       //跳出分支case default 								
					}				
				}			
				//if判断，第一个字节是0x90，表示收到的是SUBACK报文
				//接着我们要判断订阅回复，看看是不是成功
				else if(MQTT_RxDataOutPtr[2]==0x90){ 
						switch(MQTT_RxDataOutPtr[6]){					
						case 0x00 :
						case 0x01 : printf("订阅成功\r\n");            //串口输出信息
							        SubcribePack_flag = 1;                //SubcribePack_flag置1，表示订阅报文成功，其他报文可发送
									Ping_flag = 0;                        //Ping_flag清零
   								    TIM3_ENABLE_30S();                    //启动30s的PING定时器
									TIM2_ENABLE_30S();                    //启动30s的上传数据的定时器
						          TempHumi_State();                     //先发一次数据
									break;                                //跳出分支                                             
						default   : printf("订阅失败，准备重启\r\n");  //串口输出信息 
									Connect_flag = 0;                     //Connect_flag置零，重启连接
									break;                                //跳出分支 								
					}					
				}
				//if判断，第一个字节是0xD0，表示收到的是PINGRESP报文
				else if(MQTT_RxDataOutPtr[2]==0xD0){ 
					printf("PING报文回复\r\n"); 		  //串口输出信息 
					if(Ping_flag==1){                     //如果Ping_flag=1，表示第一次发送
						 Ping_flag = 0;    				  //要清除Ping_flag标志
					}else if(Ping_flag>1){ 				  //如果Ping_flag>1，表示是多次发送了，而且是2s间隔的快速发送
						Ping_flag = 0;     				  //要清除Ping_flag标志
						TIM3_ENABLE_30S(); 				  //PING定时器重回30s的时间
					}				
				}	
				//if判断，如果第一个字节是0x30，表示收到的是服务器发来的推送数据
				//我们要提取控制命令
				else if((MQTT_RxDataOutPtr[2]==0x30)){ 
					printf("服务器等级0推送\r\n"); 		   //串口输出信息 
					MQTT_DealPushdata_Qs0(MQTT_RxDataOutPtr);  //处理等级0推送数据
				}				
								
				MQTT_RxDataOutPtr += BUFF_UNIT;                     //指针下移
				if(MQTT_RxDataOutPtr==MQTT_RxDataEndPtr)            //如果指针到缓冲区尾部了
					MQTT_RxDataOutPtr = MQTT_RxDataBuf[0];          //指针归位到缓冲区开头                        
			}//处理接收缓冲区数据的else if分支结尾
			
			/*-------------------------------------------------------------*/
			/*                     处理命令缓冲区数据                      */
			/*-------------------------------------------------------------*/
			if(MQTT_CMDOutPtr != MQTT_CMDInPtr){                             //if成立的话，说明命令缓冲区有数据了			       
				printf("命令:%s\r\n",&MQTT_CMDOutPtr[2]);                 //串口输出信息
				
				MQTT_CMDOutPtr += BUFF_UNIT;                             	 //指针下移
				if(MQTT_CMDOutPtr==MQTT_CMDEndPtr)           	             //如果指针到缓冲区尾部了
					MQTT_CMDOutPtr = MQTT_CMDBuf[0];          	             //指针归位到缓冲区开头				
			}//处理命令缓冲区数据的else if分支结尾	
		}//Connect_flag=1的if分支的结尾
		
		/*--------------------------------------------------------------------*/
		/*      Connect_flag=0同服务器断开了连接,我们要重启连接服务器         */
		/*--------------------------------------------------------------------*/
		else{ 
 
			printf("需要连接服务器\r\n");                 //串口输出信息
			TIM_Cmd(TIM4,DISABLE);                           //关闭TIM4 
			TIM_Cmd(TIM3,DISABLE);                           //关闭TIM3  
			WiFi_RxCounter=0;                                //WiFi接收数据量变量清零                        
			memset(WiFi_RX_BUF,0,WiFi_RXBUFF_SIZE);          //清空WiFi接收缓冲区 
			if(WiFi_Connect_IoTServer()==0){   			        //如果WiFi连接云服务器函数返回0，表示正确，进入if
				printf("建立TCP连接成功\r\n");               //串口输出信息
				Connect_flag = 1;                            //Connect_flag置1，表示连接成功	
				WiFi_RxCounter=0;                            //WiFi接收数据量变量清零                        
				memset(WiFi_RX_BUF,0,WiFi_RXBUFF_SIZE);      //清空WiFi接收缓冲区 
				MQTT_Buff_ReInit();                          //重新初始化发送缓冲区                    
			}				
		}
		
		
		//		
				if(!stata)
		 continue;
		 stata=0;
		delay_ms(1);
		
		if(USART3_RX_STA&0X8000)		//接收到一次数据了
		{
			rxlen=USART3_RX_STA&0X7FFF;	//得到数据长度
			for(i=0;i<rxlen;i++)USART1_TX_BUF[i]=USART3_RX_BUF[i];	   
 			USART3_RX_STA=0;		   	//启动下一次接收
			USART1_TX_BUF[i]=0;			//自动添加结束符
			GPS_Analysis(&gpsx,(u8*)USART1_TX_BUF);//分析字符串
			Gps_Msg_Show();				//显示信息	
			if(upload)printf("\r\n%s\r\n",USART1_TX_BUF);//发送接收到的数据到串口1
				if(CHeck(data_buf))
		{
			 count=0;
		   if(data_buf[2]&0x01) //Temperature
			 {
			 temp2=((uint16_t)data_buf[4]<<8|data_buf[5]);   
        Temperature=(float)temp2/100;
		//	  printf("Temperature: %.2f 'C", Temperature);
         count=2;
				 LCD_ShowString(30,350,200,16,16,"Temperate: 00.00C");
				 temp3=Temperature*100;
				 temp4=temp3;
				 temperature = Temperature;
				 temperature1=temp4;
				 if(temp4<0)
         {
				  temp4=-temp4;
					LCD_ShowString(30+10*8,350,16,16,16,"-");	//显示负号
				  }else 
				LCD_ShowString(30+10*8,350,16,16,16," ");	//无符号		
			  LCD_ShowxNum(30+11*8,350,temp4/100,2,16,0);		//显示整数部分
				LCD_ShowxNum(30+14*8,350,temp4%100,2,16, 0X80);	//显示小数部分
				//delay_ms(1000);
				 }
			  if(data_buf[2]&0x02) //Humidity
			 {  
				 temp3=0;
				 temp4=0;
			    temp1=((uint16_t)data_buf[4+count]<<8)|data_buf[5+count];
				  Humidity=(float)temp1/100; 
				//  printf(" ,Humidity: %.2f %% ", Humidity);
          count+=2;
				 LCD_ShowString(30,370,200,16,16,"Humidity: 00   %");
				 temp3=Humidity ; 
				 temp4=temp3;
				 humidity=temp3;
				 humidity1=temp1;
				// temp4=temp3;
//				 if(temp4<0)
//         {
//				  temp4=-temp4;
//					LCD_ShowString(30+10*8,170,16,16,16,"-");	//显示负号
//				  }else 
				LCD_ShowString(30+10*8,370,16,16,16," ");	//无符号		
			  LCD_ShowxNum(30+11*8,370,temp4,2,16,0);		//显示整数部分
			//	LCD_ShowxNum(30+14*8,170,temp4%10,2,16, 0X80);	//显示小数部分
				//delay_ms(1000);
			 }
			  if(data_buf[2]&0x04) //Pressure
			 {
			    Pressure=((uint32_t)data_buf[4+count]<<16)|((uint16_t)data_buf[5+count]<<8)|data_buf[6+count];
				//  printf(" ,Pressure: %d Pa", Pressure);
          count+=3;
				 LCD_ShowString(30,390,200,16,16,"Pressure: 000000 Pa");
				// temp4=Pressure ;
				 pressure=Pressure;
				 pressure1=Pressure;
				  LCD_ShowxNum(30+11*8,390,Pressure,6,16,0);		//显示整数部分
				 
			 }
			 
			  if(data_buf[2]&0x08) //IAQ_accuracy、IAQ
			 {
			   IAQ_accuracy=(data_buf[4+count]&0xf0)>>4;
				 IAQ=(((uint16_t)data_buf[4+count]&0x000f)<<8)|data_buf[5+count];
		
				// printf(" ,IAQ: %d ,IAQ_accuracy: %d",IAQ,IAQ_accuracy);
          count+=2;
				  LCD_ShowString(30,410,200,16,16,"IAQ: ");
				 iaq=IAQ;
				 LCD_ShowxNum(30+14*8,410,IAQ,4,16,0);		//显示整数部分
				 LCD_ShowString(30,430,200,18,16,"IAQ_accuracy: ");
				 LCD_ShowxNum(30+14*8,430,IAQ_accuracy,2,16,0);		//显示整数部分
			 }

			   if(data_buf[2]&0x10) //Gas
			 {
			    Gas =((uint32_t)data_buf[4+count]<<24)|((uint32_t)data_buf[5+count]<<16)|((uint16_t)data_buf[6+count]<<8)|data_buf[7+count]; 
			//	  printf(" ,Gas: %d ohm ", Gas);
          count+=4;
				 gas=Gas;
				 gas1=Gas;
				 LCD_ShowString(30,450,200,16,16,"Gas:              ohm");
				 //LCD_ShowString(30,260,200,18,16,"IAQ_accuracy: ");
				 LCD_ShowxNum(30+11*8,450,Gas,6,16,0);		//显示整数部分
			 }
			   if(data_buf[2]&0x10)//海拔
				 {
				    Altitude=((int16_t)data_buf[4+count]<<8)|data_buf[5+count];
					 altitude=Altitude;
					 altitude1=Altitude;
				  //  printf(" ,Altitude: %d m \r\n", Altitude);
					 LCD_ShowString(30,470,200,16,16,"Altitude:           m");
					 LCD_ShowxNum(30+14*8,470,Altitude,5,16,0);		//显示整数部分
				 }
		temp3=MAX31865_GetTemp();
		
		     //		temp2=MAX31865_GetTemp();
		temp4=temp3*100;
				 oiltemper1=temp4;
		//temp=temp1;
	//	printf(" temp=%d\r\n",temp1);
		LCD_ShowString(30,510,200,16,16,"OilTemper: 00.00C");
//		if(temp4<0)
//		{
//			temp4=-temp4;
//			LCD_ShowString(30+10*8,510,16,16,16,"-");	//显示负号
//		}else 
		LCD_ShowString(30+10*8,510,16,16,16," ");	//无符号		
		LCD_ShowxNum(30+11*8,510,temp4/100,2,16,0);		//显示整数部分
		LCD_ShowxNum(30+14*8,510,temp4%100,2,16, 0X80);	//显示小数部分;
			 oiltemper=temp3;
				 
				 
		}

 		}
		 	adcx=Get_Adc_Average(ADC_Channel_1,10);
					//LCD_ShowString(30,530,200,16,16,"ADCCH0AL:");	      
					LCD_ShowString(30,550,200,16,16,"ADCCH0VOL: 0.000A");	
					//LCD_ShowxNum(30+11*8,530,adcx,4,16,0);//显示ADC的值
					temp=(float)adcx*(3.3/4096);
		      adcx=temp;
					ic1=1000*temp;
		      ic2=temp;
					//adcx=temp;
					LCD_ShowxNum(30+11*8,550,adcx,1,16,0);//显示电压值
					temp-=adcx;
					temp*=1000;
					LCD_ShowxNum(46+11*8,550,temp,3,16,0X80);
				 
		}	 
					 
	}			 
						
	
		
		
		
		


/*-------------------------------------------------*/
/*函数名：采集温湿度，并发布给服务器               */
/*参  数：无                                       */
/*返回值：无                                       */
/*-------------------------------------------------*/
void TempHumi_State(void)
{

//	u8 EnvTemperature,RoomHumidity,oilTemp;	
//	char temp[256];  
//	//oilTemp =126;
//	//DHT11_Read_Data(&EnvTemperature,&RoomHumidity);	//读取温湿度值	
////	AHT10_Data(&tempdata,&humidata);
//	

////	sprintf(temp,"{\"method\":\"thing.event.property.post\",\"id\":\"203302322\",\"params\":{\"humidity\":%0.2f,\"longtitude\":%0.4f,\"latitude\":%0.4f,\"speed\":%0.2f,\"pressure\":%d,\"altitude\":%d}\"version\":\"1.0.0\"}",humidity,longtitude,latitude,speed,pressure,altitude);  //构建回复湿度温度数据
////	MQTT_PublishQs0(P_TOPIC_NAME,temp,strlen(temp));   //添加数据，发布给服务器	
//	
//	sprintf(temp,"{\"method\":\"thing.event.property.post\",\"id\":\"203302322\",\"params\":{\"iaq\":%2d}\"version\":\"1.0.0\"}",iaq);
//	MQTT_PublishQs0(P_TOPIC_NAME,temp,strlen(temp));
		//u8 EnvTemperature,RoomHumidity;	
	char temp[512];  
	
	//DHT11_Read_Data(&EnvTemperature,&RoomHumidity);	//读取温湿度值	
//	AHT10_Data(&tempdata,&humidata);
	//printf("温度：%d  湿度：%d\r\n",EnvTemperature,RoomHumidity);
  printf("湿度:%0.2f 东经:%0.4f 北纬:%0.4f 速度:%0.2f 气压:%d 海拔:%d 空气质量:%d 空气:%d 温度:%0.2f 油温:%0.2f\r\n",humidity,longtitude,latitude,speed,pressure,altitude,iaq,gas,temperature,oiltemper);
//	sprintf(temp,"{\"oiltemper100\":%2d,\"temperature100\":%2d,\"iaq\":%2d,\"gas\":%2d,\"pressure\":%2d,\"altitude\":%2d,\"humidity100\":%2d,\"longtitude100000\":%2d,\"latitude100000\":%2d,\"speed1000\":%2d,\"IC1000\":%2d}",oiltemper1,temperature1,iaq,gas1,pressure1,altitude,humidity1,longtitude1,latitude1,speed1,ic1);  //构建回复湿度温度数据
	sprintf(temp,"{\"method\":\"thing.event.property.post\",\"id\":\"1349184133\",\"params\":{\"CurrentTemperature\":%.2f,\"CurrentHumidity\":%.2f},\"version\":\"1.0.0\"}",temperature,humidity);
	MQTT_PublishQs0(P_TOPIC_NAME,temp,strlen(temp));   //添加数据，发布给服务器	
}

