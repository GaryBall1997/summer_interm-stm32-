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
 ���� ALIENTEKս��STM32������ʵ��39 �޸�
************************************************/

/*******��������*****/
FIL fil;
FRESULT res;
UINT bww;
char buf[100];
//uint8_t len_str;
int len_str;
int len_ang;
int i;


// ����������������
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
* ��    �ܣ����ô���
* ���÷���������: USART_Config(USART1)
****************************************************************************/
void USART_Config(USART_TypeDef* USARTx){
    USART_InitStructure.USART_BaudRate = 9600;						//����9600bps
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//����λ8λ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;			//ֹͣλ1λ
    USART_InitStructure.USART_Parity = USART_Parity_No;				//��У��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //��Ӳ������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//�շ�ģʽ

    /* Configure USART1 */
    USART_Init(USARTx, &USART_InitStructure);							//���ô��ڲ�������

    /* Enable USART1 Receive and Transmit interrupts */
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);                    //ʹ�ܽ����ж�
    USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);						//ʹ�ܷ��ͻ�����ж�

    /* Enable the USART1 */
    USART_Cmd(USARTx, ENABLE);
}

//��ѯSD��������
void show_sdcard_info(void)
{
	switch(SDCardInfo.CardType)
	{
		case SDIO_STD_CAPACITY_SD_CARD_V1_1:printf("Card Type:SDSC V1.1\r\n");break;
		case SDIO_STD_CAPACITY_SD_CARD_V2_0:printf("Card Type:SDSC V2.0\r\n");break;
		case SDIO_HIGH_CAPACITY_SD_CARD:printf("Card Type:SDHC V2.0\r\n");break;
		case SDIO_MULTIMEDIA_CARD:printf("Card Type:MMC Card\r\n");break;
	}
	printf("Card ManufacturerID:%d\r\n",SDCardInfo.SD_cid.ManufacturerID);	//������ID
	printf("Card RCA:%d\r\n",SDCardInfo.RCA);								//����Ե�ַ
	printf("Card Capacity:%d MB\r\n",(u32)(SDCardInfo.CardCapacity>>20));	//��ʾ����
	printf("Card BlockSize:%d\r\n\r\n",SDCardInfo.CardBlockSize);			//��ʾ���С
}



//CopeSerialDataΪ����3�жϵ��ú���������ÿ�յ�һ�����ݣ�����һ�����������
//ÿ��������1�ֽ�
void CopeSerial2Data(unsigned char ucData)
{

    RxBuffer3[RxCounter3++]=ucData;	//���յ������ݴ��뻺������
    //ȫ�ֱ�����uint8_t RxBuffer3[250] = "Test";
    //ȫ�ֱ�����__IO uint8_t RxCounter3 = 0x00;
    if (RxBuffer3[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
    {
        RxCounter3=0;//����ÿ�յ�һ���ֽڵ����ݵ���һ�Σ�����JY901��
        return;			//һ�����ݲ�ֹ1���ֽڣ�����������ͷ�Բ��Ͼ�Rxcounter���㣬������
    }
    //�������ͷ�����ˣ���Rxcounter++��ֱ����11���ֽڣ���ʼ����ת��
    //����JY901��һ��������11���ֽڵ�
    if (RxCounter3<11) {return;}//���ݲ���11�����򷵻�

    else
    {//0 1 23456789 10 11
        switch(RxBuffer3[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
        {//0x51 ���ٶ�
            case 0x51:	memcpy(&stcAcc,&RxBuffer3[2],8);break;  //memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
                //���Ѵ��ڽ��յ������ݿ�����stcAcc����ṹ����
            case 0x52: memcpy(&stcGyro,&RxBuffer3[2],8);break;// 0x52�����ٶ�
            case 0x53: memcpy(&stcAngle,&RxBuffer3[2],8);break;//�Ƕ�
            case 0x54: memcpy(&stcMag,&RxBuffer3[2],8);break;//�ų�
        }

        RxCounter3=0;//��ջ�����
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
  RCC_Configuration();											  //ʱ������
  NVIC_Configuration();											  //�ж�����
	GPIO_Configuration();											  //GPIO����
	
  USART_Config(USART1);											  //���ô���1
	USART_Config(USART3);											  //����3
  
	GPIO_SetBits(GPIOB, GPIO_Pin_5);						//����LED
  USART_OUT(USART1,"IMU JY901\r\n");
	GPIO_ResetBits(GPIOB, GPIO_Pin_5);

	delay_init();	    	 //��ʱ������ʼ��	  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
	usmart_dev.init(72);		//��ʼ��USMART		
 	LED_Init();		  			//��ʼ����LED���ӵ�Ӳ���ӿ�
	KEY_Init();					//��ʼ������
	LCD_Init();			   		//��ʼ��LCD   
	W25QXX_Init();				//��ʼ��W25Q128
 	my_mem_init(SRAMIN);		//��ʼ���ڲ��ڴ��
 	POINT_COLOR=RED;			//��������Ϊ��ɫ 


	while(SD_Init())//���SD��
	{
		printf("Card not found!");
		delay_ms(500);
		LED0=!LED0;//DS0��˸
	}
	
	show_sdcard_info();	//��ӡSD�������Ϣ
	
 	exfuns_init();							//Ϊfatfs��ر��������ڴ�				 
  f_mount(fs[0],"0:",1); 					//mount SD card
 	res=f_mount(fs[1],"1:",1); 				//mount FLASH,get the file system type.
	
	
	if(res==0X0D)//FLASH����,FAT�ļ�ϵͳ����,���¸�ʽ��FLASH
	{
    //��ʽ��flash
		res=f_mkfs("1:",1,4096);//��ʽ��FLASH,1,�̷�;1,����Ҫ������,8������Ϊ1����
		if(res==0)
		{
			f_setlabel((const TCHAR *)"1:GaryBall");	//����Flash���̵�����Ϊ��GaryBall
			printf("Flash Disk Format Finish");	//��ʽ�����
		}else printf("Flash Disk Format Error ");	//��ʽ��ʧ��
		delay_ms(1000);
	}


	while(exf_getfree("0",&total,&free))	//�õ�SD������������ʣ������
	{
		printf("SD Card Fatfs Error!");
		delay_ms(200);
		LED0=!LED0;//DS0��˸
	}

/********************start*************************/

	res=f_open (&fil,"0:/0723.txt", FA_CREATE_ALWAYS);	
	f_close(&fil);//�����е�������ÿ��rest֮�����´���һ���µ�����ļ�
	//Ȼ����whileѭ����������openalways
	for(i=0;i<=10;i++)
	{
		delay_ms(1000);
		
	}
	LED1 = 0;//0����
  while(count--)
    {
				delay_ms(200);
        USART_OUT(USART1,"\r\n");
        //������ٶ�

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
* ��    �ƣ�void Delay(__IO uint32_t nCount)
* ��    �ܣ���ʱ����
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�������
****************************************************************************/
void Delay(__IO uint32_t nCount)
{
    for(; nCount != 0; nCount--);
}

/****************************************************************************
* ��    �ƣ�void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...)
* ��    �ܣ���ʽ�������������
* ��ڲ�����USARTx:  ָ������
			Data��   ��������
			...:     ��������
* ���ڲ�������
* ˵    ������ʽ�������������
        	"\r"	�س���	   USART_OUT(USART1, "abcdefg\r")
			"\n"	���з�	   USART_OUT(USART1, "abcdefg\r\n")
			"%s"	�ַ���	   USART_OUT(USART1, "�ַ����ǣ�%s","abcdefg")
			"%d"	ʮ����	   USART_OUT(USART1, "a=%d",10)
* ���÷�������
****************************************************************************/
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...){
    const char *s;
    int d;
    char buf[16];
    va_list ap;
    va_start(ap, Data);

    while(*Data!=0){				                          //�ж��Ƿ񵽴��ַ���������
        if(*Data==0x5c){									  //'\'
            switch (*++Data){
                case 'r':							          //�س���
                    USART_SendData(USARTx, 0x0d);

                    Data++;
                    break;
                case 'n':							          //���з�
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
                case 's':										  //�ַ���
                    s = va_arg(ap, const char *);
                    for ( ; *s; s++) {
                        USART_SendData(USARTx,*s);
                        while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                    }
                    Data++;
                    break;
                case 'd':										  //ʮ����
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
		��������ת�ַ�������
        char *itoa(int value, char *string, int radix)
		radix=10 ��ʾ��10����	��ʮ���ƣ�ת�����Ϊ0;

	    ����d=-379;
		ִ��	itoa(d, buf, 10); ��

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
* ��    �ƣ�void RCC_Configuration(void)
* ��    �ܣ�ϵͳʱ������Ϊ72MHZ�� ����ʱ������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�������
****************************************************************************/
void RCC_Configuration(void)
{
    SystemInit();
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 |  RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB
                            | RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO  , ENABLE);


    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3, ENABLE);   // USART3ʱ����APB1��
}

/****************************************************************************
* ��    �ƣ�void GPIO_Configuration(void)
* ��    �ܣ�ͨ��IO������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����
****************************************************************************/
void GPIO_Configuration(void)
{

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				     //LED1����--PB5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			 //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Ĭ�ϸ��ù��� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         		 //USART1 TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    		 //�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);		    		 //A�˿�
    /* ���ù��ܵ��������ű�������Ϊ����ģʽ������/����/������һ�֣�*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         	 //USART1 RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   	 //���ø�������
    GPIO_Init(GPIOA, &GPIO_InitStructure);		         	 //A�˿�

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         		 //USART3 TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    		 //�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);		    		 //B�˿�

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	         	 	 //USART3 RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   	 //���ø�������
    GPIO_Init(GPIOB, &GPIO_InitStructure);		         	 //B�˿�

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; 			      //LCD�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);			              //LCD����ر�
}

/****************************************************************************
* ��    �ƣ�void NVIC_Configuration(void)
* ��    �ܣ��ж�Դ����
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�������
****************************************************************************/
void NVIC_Configuration(void)
{
    /*  �ṹ����*/
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure the NVIC Preemption Priority Bits */
    /* Configure one bit for preemption priority */
    /* ���ȼ��� ˵������ռ���ȼ����õ�λ��������Ӧ���ȼ����õ�λ��
    0�飺  ��ռ���ȼ�ռ0λ�� ��Ӧ���ȼ�ռ4λ
    1�飺  ��ռ���ȼ�ռ1λ�� ��Ӧ���ȼ�ռ3λ
    2�飺  ��ռ���ȼ�ռ2λ�� ��Ӧ���ȼ�ռ2λ
    3�飺  ��ռ���ȼ�ռ3λ�� ��Ӧ���ȼ�ռ1λ
    4�飺  ��ռ���ȼ�ռ4λ�� ��Ӧ���ȼ�ռ0λ
    */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			     	//���ô���1�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	     	//��ռ���ȼ� 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;				//�����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//ʹ��
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;			     	//���ô���3�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	     	//��ռ���ȼ� 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//�����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//ʹ��
    NVIC_Init(&NVIC_InitStructure);

}
void DISABLE_NVIC_Configuration(void)
{
    /*  �ṹ����*/
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure the NVIC Preemption Priority Bits */
    /* Configure one bit for preemption priority */
    /* ���ȼ��� ˵������ռ���ȼ����õ�λ��������Ӧ���ȼ����õ�λ��
    0�飺  ��ռ���ȼ�ռ0λ�� ��Ӧ���ȼ�ռ4λ
    1�飺  ��ռ���ȼ�ռ1λ�� ��Ӧ���ȼ�ռ3λ
    2�飺  ��ռ���ȼ�ռ2λ�� ��Ӧ���ȼ�ռ2λ
    3�飺  ��ռ���ȼ�ռ3λ�� ��Ӧ���ȼ�ռ1λ
    4�飺  ��ռ���ȼ�ռ4λ�� ��Ӧ���ȼ�ռ0λ
    */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			     	//���ô���1�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	     	//��ռ���ȼ� 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;				//�����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;					//DIASBLE
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;			     	//���ô���3�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	     	//��ռ���ȼ� 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//�����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;					//DISABLE
    NVIC_Init(&NVIC_InitStructure);

}

