#include <stdio.h>
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "misc.h"

GPIO_InitTypeDef GPIO_InitStructure, GPIOA_InitStructure;

static __IO uint32_t DELAY_TIME;

uint8_t BUTTON_1_STATE = 0;
uint8_t BUTTON_2_STATE = 0;

// ������������� ����������� ������������ ������� ���������� ������������
// ----------------------------------------------------------------------------------
void SysTick_Handler(void) {
	// ���� �������� �� ����� �� ���� ��������������
	if (DELAY_TIME != 0x00) {
		DELAY_TIME--;
	}
}

void EXTI0_IRQHandler(void) {
  if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
	  BUTTON_1_STATE = 1;
  }

  // ������ ���������������� ���� ��������
  EXTI_ClearITPendingBit(EXTI_Line0);
}
void EXTI15_10_IRQHandler(void) {
  if (EXTI_GetITStatus(EXTI_Line15) != RESET) {
	  BUTTON_2_STATE = 1;
  }

  // ������ ���������������� ���� ��������
  EXTI_ClearITPendingBit(EXTI_Line15);
}
// ----------------------------------------------------------------------------------

void delay(__IO uint32_t nCount) {
	DELAY_TIME = nCount;

	while (DELAY_TIME != 0);
}

// ������� ��������� ������� ����������
void setup() {
	  // ��������� �������� ������� �������-��������� ���������������
	  RCC_ADCCLKConfig(RCC_PCLK2_Div4);

	  // ��������� ����������� ������� ������� � ������ DMA
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	  // ��������� GPIOA � ���������� ��������������� ���������
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	  // ������������� ����� PA0 � PA15
	  GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_15;

	  // ��������� ������ ������ � Input Pull Down
	  GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_IPD;

	  // ��������� ������������ ��������
	  GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	  GPIO_Init(GPIOA, &GPIOA_InitStructure);

	  // ��������� GPIOB
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	  // ������������� ����� PB6-PB9
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8  | GPIO_Pin_9;

	  // ����� ������ - Outpup Push-Pull
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, & GPIO_InitStructure);
}

void reset_diods() {
	GPIO_ResetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
}

void reset_buttons() {
	BUTTON_1_STATE = 0;
	BUTTON_2_STATE = 0;
}

void setup_interuption_config() {
	SysTick_Config(SystemCoreClock / 1000);

	GPIO_InitTypeDef GPIO_0_InitStructure, GPIO_15_InitStructure; // ������������� GPIOb init - ���������

	EXTI_InitTypeDef EXTI_0_InitStructure, EXTI_15_InitStructure;  // ������������� EXTI init - ���������

	NVIC_InitTypeDef NVIC_0_InitStructure, NVIC_15_InitStructure;  // ������������� NVIC init - ���������

	/* ������� GPIOA */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// ������������� ��� PA0 �� ������
	GPIO_0_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_0_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  // ����� -������ �����x
	GPIO_Init(GPIOA, &GPIO_0_InitStructure);  // �������������
	// ���������� WAKEUP_BUTTON GPIO � ����� EXTI
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
	// ������������� ������� ����� EXTI_0 ��� WKUP_BUTTON ����������
	EXTI_0_InitStructure.EXTI_Line = EXTI_Line0;
	// ����� ��������� � ������� �����
	EXTI_0_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  // P����-����������
	EXTI_0_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	// �������������� ������ �������
	EXTI_0_InitStructure.EXTI_LineCmd = ENABLE;  // ��������
	EXTI_Init(&EXTI_0_InitStructure);  // ������������� ����������� �����������
									 // ���������
	/*
	� ������ ����������� */
	/*��������� ����������
	WAKEUP_BUTTON_IRQn */
	NVIC_0_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_0_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_0_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_0_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_0_InitStructure);
	//
	// ��������� ����������� ��� PA15, EXTI_15
	//
	// ������������� ��� PA0 �� ������
	GPIO_15_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_15_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  // ����� -������ �����x
	GPIO_Init(GPIOA, &GPIO_15_InitStructure);  // �������������
	// ���������� WAKEUP_BUTTON GPIO � ����� EXTI
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);
	// ������������� ������� ����� EXTI_0 ��� WKUP_BUTTON ����������
	EXTI_15_InitStructure.EXTI_Line = EXTI_Line15;
	// ����� ��������� � ������� �����
	EXTI_15_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  // P����-����������
	EXTI_15_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	// �������������� ������ �������
	EXTI_15_InitStructure.EXTI_LineCmd = ENABLE;  // �������

	// ������������� ����������� ����������� ��������� � ������ �����������
	EXTI_Init(&EXTI_15_InitStructure);

	/*��������� ���������� USER_BUTTON */
	NVIC_15_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_15_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_15_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_15_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_15_InitStructure);
}

int main(void) {
	SystemInit();
	setup_interuption_config();
	setup();

	while (1) {
		// ���� ������ ���� ������
		if (BUTTON_1_STATE ^ BUTTON_2_STATE) {
			reset_diods();

			// �������� ��� ����� (��� ������) ����������
			GPIO_WriteBit(GPIOB, GPIO_Pin_6 | GPIO_Pin_7, (BitAction) BUTTON_1_STATE);
			GPIO_WriteBit(GPIOB, GPIO_Pin_8 | GPIO_Pin_9, (BitAction) BUTTON_2_STATE);

			reset_buttons();

			//printf("Button_1 (PA0): %d; Button_2 (PA15): %d\n\r", BUTTON_1_STATE, BUTTON_2_STATE);
		}
		// ���� ������ ��� ������
		else if (BUTTON_1_STATE && BUTTON_2_STATE) {
			reset_diods();

			// �������� ����������� �����������
			GPIO_WriteBit(GPIOB, GPIO_Pin_7, (BitAction) BUTTON_1_STATE);
			GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction) BUTTON_2_STATE);

			reset_buttons();

			//printf("Button_1 (PA0): %d; Button_2 (PA15): %d\n\r", BUTTON_1_STATE, BUTTON_2_STATE);
		}

		// delay
		delay(200);
	}
}
