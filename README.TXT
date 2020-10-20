器材:
	精英STM32F103开发板ZET6
	
目的:
	GYMCU680(环境温湿度+气压+空气质量)+ATK-BD-GPS+PT100油温+WSC电流传感器+ESP8266上传阿里云
	
硬件资源:
	1,ESP8266使用串口4通信，接线方式：
	vcc-5v；gnd-gnd；TXD-PC11;RXD-PC10;RST-PA12;I0_0-PA15.
	2,串口1(波特率:115200,PA9/PA10连接在板载USB转串口芯片CH340上面)用于打印数据至串口助手。
	3,PT100串口通信方式为SPI通信，接线方式：
	VIN-5V;GND-GND;SDO-PA6;CS-PA11;SCLK-PA8;SDI-PA7;DRDY-PA5.
	4,GYMCU680使用串口2通信，接线方式：
	VCC-5V;GND-GND;RX-PA2;TX-PA3.
	5,WSC电流传感器使用ADC1接线，使用ADC(STM32内部ADC1,通道16,即:ADC1_CH16).
	6,ATK-BD-GPS使用串口3通信，接线方式：
	PPS-KEY;RXD-RXD;TXD-TXD;GND-GND;VCC-5V.
	
现象:
	本项目通过串口2读取环境温湿度等，通过串口3连接BD监测经纬度，SPI检测PT100通道的电机绕组温度，通过串口4连接ESP8266将传感器数据上传至阿里云服务器。并通过阿里云公用版云智能APP下发数据，实现物联网的实时阈值控制。
	云端通信协议为MQTT协议。
	







































