#include "sys.h"
#include "delay.h"
#include "led.h"
#include "lcd.h"
#include "key.h"
#include "usmart.h"
#include "malloc.h"
#include "sdio_sdcard.h"
#include "w25qxx.h"
#include "ff.h"
#include "exfuns.h"
#include "JY901.h"
#include "misc.h"
#include "stdarg.h"
#include <string.h>
 
/************************************************
 基于 ALIENTEK战舰STM32开发板实验39 修改
************************************************/

/*******变量定义*****/
FIL fil;
FRESULT res;
UINT bww;
char buf[100];
//uint8_t len_str;
int len_str;
int len_ang;
int i;


// 创建基本数据类型
struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 		stcAngle;
struct SMag 		stcMag;
struct SDStatus 	stcDStatus;
struct SPress 		stcPress;
struct SLonLat 		stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ       	stcQ;



/* Private variables */
USART_InitTypeDef USART_InitStructure;
uint8_t RxBuffer3[250] = "Test";
__IO uint8_t RxCounter3 = 0x00;


/* Private function prototypes */
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void DISABLE_NVIC_Configuration(void);


void Delay(__IO uint32_t nCount);
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...);
char *itoa(int value, char *string, int radix);
void USART_Config(USART_TypeDef* USARTx);


GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStruct;
USART_ClockInitTypeDef USART_ClockInitStruct;

/****************************************************************************
* 功    能：配置串口
* 调用方法：例如: USART_Config(USART1)
****************************************************************************/
void USART_Config(USART_TypeDef* USARTx){
    USART_InitStructure.USART_BaudRate = 9600;						//速率9600bps
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据位8位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;			//停止位1位
    USART_InitStructure.USART_Parity = USART_Parity_No;				//无校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式

    /* Configure USART1 */
    USART_Init(USARTx, &USART_InitStructure);							//配置串口参数函数

    /* Enable USART1 Receive and Transmit interrupts */
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);                    //使能接收中断
    USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);						//使能发送缓冲空中断

    /* Enable the USART1 */
    USART_Cmd(USARTx, ENABLE);
}

//查询SD卡的内容
void show_sdcard_info(void)
{
	switch(SDCardInfo.CardType)
	{
		case SDIO_STD_CAPACITY_SD_CARD_V1_1:printf("Card Type:SDSC V1.1\r\n");break;
		case SDIO_STD_CAPACITY_SD_CARD_V2_0:printf("Card Type:SDSC V2.0\r\n");break;
		case SDIO_HIGH_CAPACITY_SD_CARD:printf("Card Type:SDHC V2.0\r\n");break;
		case SDIO_MULTIMEDIA_CARD:printf("Card Type:MMC Card\r\n");break;
	}
	printf("Card ManufacturerID:%d\r\n",SDCardInfo.SD_cid.ManufacturerID);	//制造商ID
	printf("Card RCA:%d\r\n",SDCardInfo.RCA);								//卡相对地址
	printf("Card Capacity:%d MB\r\n",(u32)(SDCardInfo.CardCapacity>>20));	//显示容量
	printf("Card BlockSize:%d\r\n\r\n",SDCardInfo.CardBlockSize);			//显示块大小
}



//CopeSerialData为串口3中断调用函数，串口每收到一个数据，调用一次这个函数。
//每个数据是1字节
void CopeSerial2Data(unsigned char ucData)
{

    RxBuffer3[RxCounter3++]=ucData;	//将收到的数据存入缓冲区中
    //全局变量：uint8_t RxBuffer3[250] = "Test";
    //全局变量：__IO uint8_t RxCounter3 = 0x00;
    if (RxBuffer3[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
    {
        RxCounter3=0;//串口每收到一个字节的数据调用一次，但是JY901的
        return;			//一组数据不止1个字节，因此如果数据头对不上就Rxcounter清零，并返回
    }
    //如果数据头对上了，则Rxcounter++，直到满11个字节，开始数据转换
    //由于JY901的一组数据是11个字节的
    if (RxCounter3<11) {return;}//数据不满11个，则返回

    else
    {//0 1 23456789 10 11
        switch(RxBuffer3[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
        {//0x51 加速度
            case 0x51:	memcpy(&stcAcc,&RxBuffer3[2],8);break;  //memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
                //即把串口接收到的数据拷贝到stcAcc这个结构体中
            case 0x52: memcpy(&stcGyro,&RxBuffer3[2],8);break;// 0x52：角速度
            case 0x53: memcpy(&stcAngle,&RxBuffer3[2],8);break;//角度
            case 0x54: memcpy(&stcMag,&RxBuffer3[2],8);break;//磁场
        }

        RxCounter3=0;//清空缓存区
    }
}


int main(void)
 {	 
 	u32 total,free;
	u8 t=0;	
	u8 res=0;	 
	int count = 4096;	 
	 
	char str_acc[100];
	char str_gyr[100];
	char str_ang[100];
	char str_mag[100];
  RCC_Configuration();											  //时钟配置
  NVIC_Configuration();											  //中断配置
	GPIO_Configuration();											  //GPIO配置
	
  USART_Config(USART1);											  //配置串口1
	USART_Config(USART3);											  //串口3
  
	GPIO_SetBits(GPIOB, GPIO_Pin_5);						//点亮LED
  USART_OUT(USART1,"IMU JY901\r\n");
	GPIO_ResetBits(GPIOB, GPIO_Pin_5);

	delay_init();	    	 //延时函数初始化	  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为115200
	usmart_dev.init(72);		//初始化USMART		
 	LED_Init();		  			//初始化与LED连接的硬件接口
	KEY_Init();					//初始化按键
	LCD_Init();			   		//初始化LCD   
	W25QXX_Init();				//初始化W25Q128
 	my_mem_init(SRAMIN);		//初始化内部内存池
 	POINT_COLOR=RED;			//设置字体为红色 


	while(SD_Init())//检测SD卡
	{
		printf("Card not found!");
		delay_ms(500);
		LED0=!LED0;//DS0闪烁
	}
	
	show_sdcard_info();	//打印SD卡相关信息
	
 	exfuns_init();							//为fatfs相关变量申请内存				 
  f_mount(fs[0],"0:",1); 					//mount SD card
 	res=f_mount(fs[1],"1:",1); 				//mount FLASH,get the file system type.
	
	
	if(res==0X0D)//FLASH磁盘,FAT文件系统错误,重新格式化FLASH
	{
    //格式化flash
		res=f_mkfs("1:",1,4096);//格式化FLASH,1,盘符;1,不需要引导区,8个扇区为1个簇
		if(res==0)
		{
			f_setlabel((const TCHAR *)"1:GaryBall");	//设置Flash磁盘的名字为：GaryBall
			printf("Flash Disk Format Finish");	//格式化完成
		}else printf("Flash Disk Format Error ");	//格式化失败
		delay_ms(1000);
	}


	while(exf_getfree("0",&total,&free))	//得到SD卡的总容量和剩余容量
	{
		printf("SD Card Fatfs Error!");
		delay_ms(200);
		LED0=!LED0;//DS0闪烁
	}

/********************start*************************/

	res=f_open (&fil,"0:/0723.txt", FA_CREATE_ALWAYS);	
	f_close(&fil);//这两行的作用是每次rest之后都重新创建一个新的这个文件
	//然后在while循环里面是有openalways
	for(i=0;i<=10;i++)
	{
		delay_ms(1000);
		
	}
	LED1 = 0;//0是亮
  while(count--)
    {
				delay_ms(200);
        USART_OUT(USART1,"\r\n");
        //输出加速度

				len_str = sprintf(str_acc,"ACC:%.3f,%.3f,%.3f\r\n",\
		(float)(stcAcc.a[0])/32768*16,(float)(stcAcc.a[1])/32768*16,(float)(stcAcc.a[2])/32768*16);

//        USART_OUT(USART1,str_acc);

//        sprintf(str_gyr,"Gyr:%.3f %.3f %.3f\r\n",\
//		(float)stcGyro.w[0]/32768*2000,(float)stcGyro.w[1]/32768*2000,(float)stcGyro.w[2]/32768*2000);
//        USART_OUT(USART1,str_gyr);

        len_ang=sprintf(str_ang,"Angle:%.3f,%.3f,%.3f\r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180,(float)stcAngle.Angle[2]/32768*180);
//        USART_OUT(USART1,str_ang);

//        sprintf(str_mag,"Mag:%.3f %.3f %.3f\r\n",\
//		(float)stcMag.h[0],(float)stcMag.h[1],(float)stcMag.h[2]);
//        USART_OUT(USART1,str_mag);
				
				
				USART_Cmd(USART3, DISABLE);
				res=f_open(&fil,"0:/0725.txt", FA_OPEN_ALWAYS|FA_WRITE);	
				f_lseek(&fil,fil.fsize);
				f_write(&fil, str_acc, len_str, &bww);
				f_write(&fil, str_ang, len_ang, &bww);
				f_close(&fil);
	
				res=f_open (&fil,"0:/0725.txt", FA_READ);
    
				f_read (&fil, buf,100,&bww);	
				f_close(&fil);
//				printf("Content of SD card: %s",buf);
				USART_Cmd(USART3, ENABLE);
    }
		
	LED1 = 1;

/********************end***************************/	

	while(1)
	{
		t++; 
		delay_ms(200);
		LED0 = !LED0;
	} 
}


/****************************************************************************
* 名    称：void Delay(__IO uint32_t nCount)
* 功    能：延时函数
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无
****************************************************************************/
void Delay(__IO uint32_t nCount)
{
    for(; nCount != 0; nCount--);
}

/****************************************************************************
* 名    称：void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...)
* 功    能：格式化串口输出函数
* 入口参数：USARTx:  指定串口
			Data：   发送数组
			...:     不定参数
* 出口参数：无
* 说    明：格式化串口输出函数
        	"\r"	回车符	   USART_OUT(USART1, "abcdefg\r")
			"\n"	换行符	   USART_OUT(USART1, "abcdefg\r\n")
			"%s"	字符串	   USART_OUT(USART1, "字符串是：%s","abcdefg")
			"%d"	十进制	   USART_OUT(USART1, "a=%d",10)
* 调用方法：无
****************************************************************************/
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...){
    const char *s;
    int d;
    char buf[16];
    va_list ap;
    va_start(ap, Data);

    while(*Data!=0){				                          //判断是否到达字符串结束符
        if(*Data==0x5c){									  //'\'
            switch (*++Data){
                case 'r':							          //回车符
                    USART_SendData(USARTx, 0x0d);

                    Data++;
                    break;
                case 'n':							          //换行符
                    USART_SendData(USARTx, 0x0a);
                    Data++;
                    break;

                default:
                    Data++;
                    break;
            }


        }
        else if(*Data=='%'){									  
            switch (*++Data){
                case 's':										  //字符串
                    s = va_arg(ap, const char *);
                    for ( ; *s; s++) {
                        USART_SendData(USARTx,*s);
                        while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                    }
                    Data++;
                    break;
                case 'd':										  //十进制
                    d = va_arg(ap, int);
                    itoa(d, buf, 10);
                    for (s = buf; *s; s++) {
                        USART_SendData(USARTx,*s);
                        while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                    }
                    Data++;
                    break;
                default:
                    Data++;
                    break;
            }
        }
        else USART_SendData(USARTx, *Data++);
        while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
    }
}

/******************************************************
		整形数据转字符串函数
        char *itoa(int value, char *string, int radix)
		radix=10 标示是10进制	非十进制，转换结果为0;

	    例：d=-379;
		执行	itoa(d, buf, 10); 后

		buf="-379"
**********************************************************/
char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} /* NCL_Itoa */

/****************************************************************************
* 名    称：void RCC_Configuration(void)
* 功    能：系统时钟配置为72MHZ， 外设时钟配置
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无
****************************************************************************/
void RCC_Configuration(void)
{
    SystemInit();
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 |  RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB
                            | RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO  , ENABLE);


    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3, ENABLE);   // USART3时钟在APB1上
}

/****************************************************************************
* 名    称：void GPIO_Configuration(void)
* 功    能：通用IO口配置
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：
****************************************************************************/
void GPIO_Configuration(void)
{

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				     //LED1控制--PB5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			 //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 默认复用功能 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         		 //USART1 TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    		 //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);		    		 //A端口
    /* 复用功能的输入引脚必须配置为输入模式（浮空/上拉/下拉的一种）*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         	 //USART1 RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   	 //复用浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);		         	 //A端口

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         		 //USART3 TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    		 //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);		    		 //B端口

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	         	 	 //USART3 RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   	 //复用浮空输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);		         	 //B端口

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; 			      //LCD背光控制
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);			              //LCD背光关闭
}

/****************************************************************************
* 名    称：void NVIC_Configuration(void)
* 功    能：中断源配置
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无
****************************************************************************/
void NVIC_Configuration(void)
{
    /*  结构声明*/
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure the NVIC Preemption Priority Bits */
    /* Configure one bit for preemption priority */
    /* 优先级组 说明了抢占优先级所用的位数，和响应优先级所用的位数
    0组：  抢占优先级占0位， 响应优先级占4位
    1组：  抢占优先级占1位， 响应优先级占3位
    2组：  抢占优先级占2位， 响应优先级占2位
    3组：  抢占优先级占3位， 响应优先级占1位
    4组：  抢占优先级占4位， 响应优先级占0位
    */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			     	//设置串口1中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	     	//抢占优先级 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;				//子优先级为1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//使能
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;			     	//设置串口3中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	     	//抢占优先级 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//子优先级为1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//使能
    NVIC_Init(&NVIC_InitStructure);

}
void DISABLE_NVIC_Configuration(void)
{
    /*  结构声明*/
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure the NVIC Preemption Priority Bits */
    /* Configure one bit for preemption priority */
    /* 优先级组 说明了抢占优先级所用的位数，和响应优先级所用的位数
    0组：  抢占优先级占0位， 响应优先级占4位
    1组：  抢占优先级占1位， 响应优先级占3位
    2组：  抢占优先级占2位， 响应优先级占2位
    3组：  抢占优先级占3位， 响应优先级占1位
    4组：  抢占优先级占4位， 响应优先级占0位
    */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			     	//设置串口1中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	     	//抢占优先级 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;				//子优先级为1
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;					//DIASBLE
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;			     	//设置串口3中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	     	//抢占优先级 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//子优先级为1
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;					//DISABLE
    NVIC_Init(&NVIC_InitStructure);

}

