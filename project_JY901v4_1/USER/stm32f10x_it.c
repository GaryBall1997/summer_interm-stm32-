/**
  ******************************************************************************
  * @file USART/Interrupt/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler and 
  *         peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_usart.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern uint8_t RxBuffer2[];
extern __IO uint8_t RxCounter2;

uint8_t ch;

extern void CopeSerial2Data(unsigned char ucData);



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles USART1 global interrupt request.
  * @param  None
  * @retval : None
  */


void USART1_IRQHandler(void)      //����1 �жϷ������
{
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET) //�������ݼĴ������ж�                 
  {
		USART_ITConfig(USART1, USART_IT_TXE, DISABLE); //ֱ�Ӷ�TXE���� 
	}
	
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)	  
  {			//�������ݼĴ����ǿ��ж�
		ch=USART_ReceiveData(USART1);//��ȡ���ݼĴ������Զ�RXNE����
  }
}

void USART2_IRQHandler(void)      //����2 �жϷ������
{

    if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
    {
        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }

    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {

        ch=USART_ReceiveData(USART2);//����USART3���յ�������
        //��ȡ���ݼĴ������Զ�RXNE����
        CopeSerial2Data(ch); //��������
    }

}

void USART3_IRQHandler(void)      //����2 �жϷ������
{
  
  if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
  {	
		USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
  }
	
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
  { 
		
		ch=USART_ReceiveData(USART3);//����USART3���յ�������
																//��ȡ���ݼĴ������Զ�RXNE����
		CopeSerial2Data(ch); //��������
  }	
	
}






/******************* (C) COPYRIGHT 2011 �ܶ�STM32 *****END OF FILE****/
