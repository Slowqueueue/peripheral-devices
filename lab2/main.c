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

// Имплементация определений библиотечных функций управления прерываниями
// ----------------------------------------------------------------------------------
void SysTick_Handler(void) {
	// Если задержка не дошла до нуля декрементируем
	if (DELAY_TIME != 0x00) {
		DELAY_TIME--;
	}
}

void EXTI0_IRQHandler(void) {
  if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
	  BUTTON_1_STATE = 1;
  }

  // Чистка соответствующего бита ожидания
  EXTI_ClearITPendingBit(EXTI_Line0);
}
void EXTI15_10_IRQHandler(void) {
  if (EXTI_GetITStatus(EXTI_Line15) != RESET) {
	  BUTTON_2_STATE = 1;
  }

  // Чистка соответствующего бита ожидания
  EXTI_ClearITPendingBit(EXTI_Line15);
}
// ----------------------------------------------------------------------------------

void delay(__IO uint32_t nCount) {
	DELAY_TIME = nCount;

	while (DELAY_TIME != 0);
}

// Базовая установка рабочих параметров
void setup() {
	  // Установка тактовой частоты аналого-цифровому преобразователю
	  RCC_ADCCLKConfig(RCC_PCLK2_Div4);

	  // Включение контроллера прямого доступа к памяти DMA
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	  // Включение GPIOA и управление альтернативными функциями
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	  // Использование пинов PA0 и PA15
	  GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_15;

	  // Включение режима работы в Input Pull Down
	  GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_IPD;

	  // Установка максимальной скорости
	  GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	  GPIO_Init(GPIOA, &GPIOA_InitStructure);

	  // Включение GPIOB
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	  // Использование пинов PB6-PB9
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8  | GPIO_Pin_9;

	  // Режим работы - Outpup Push-Pull
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

	GPIO_InitTypeDef GPIO_0_InitStructure, GPIO_15_InitStructure; // Инициализация GPIOb init - структуры

	EXTI_InitTypeDef EXTI_0_InitStructure, EXTI_15_InitStructure;  // Инициализация EXTI init - структуры

	NVIC_InitTypeDef NVIC_0_InitStructure, NVIC_15_InitStructure;  // Инициализация NVIC init - структуры

	/* Вклчаем GPIOA */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// Конфигурируем пин PA0 на кнопке
	GPIO_0_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_0_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  // Режим -чтение данныx
	GPIO_Init(GPIOA, &GPIO_0_InitStructure);  // Инициализация
	// Подключаем WAKEUP_BUTTON GPIO к линии EXTI
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
	// Конфигурируем внешнюю линию EXTI_0 для WKUP_BUTTON прерываний
	EXTI_0_InitStructure.EXTI_Line = EXTI_Line0;
	// Вывод подключен к нулевой линии
	EXTI_0_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  // Pежим-прерывание
	EXTI_0_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	// Детектирование фронта имульса
	EXTI_0_InitStructure.EXTI_LineCmd = ENABLE;  // Включить
	EXTI_Init(&EXTI_0_InitStructure);  // Инициализация посредством заполненной
									 // структуры
	/*
	с низшим приоритетом */
	/*Разрешаем прерывания
	WAKEUP_BUTTON_IRQn */
	NVIC_0_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_0_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_0_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_0_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_0_InitStructure);
	//
	// Повторяем манипуляции для PA15, EXTI_15
	//
	// Конфигурируем пин PA0 на кнопке
	GPIO_15_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_15_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  // Режим -чтение данныx
	GPIO_Init(GPIOA, &GPIO_15_InitStructure);  // Инициализация
	// Подключаем WAKEUP_BUTTON GPIO к линии EXTI
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);
	// Конфигурируем внешнюю линию EXTI_0 для WKUP_BUTTON прерываний
	EXTI_15_InitStructure.EXTI_Line = EXTI_Line15;
	// Вывод подключен к нулевой линии
	EXTI_15_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  // Pежим-прерывание
	EXTI_15_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	// Детектирование фронта имульса
	EXTI_15_InitStructure.EXTI_LineCmd = ENABLE;  // Вклчить

	// Инициализация посредством заполненной структуры с низшим приоритетом
	EXTI_Init(&EXTI_15_InitStructure);

	/*Разрешаем прерывания USER_BUTTON */
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
		// Если нажата одна кнопка
		if (BUTTON_1_STATE ^ BUTTON_2_STATE) {
			reset_diods();

			// Включаем два левых (или правых) светодиода
			GPIO_WriteBit(GPIOB, GPIO_Pin_6 | GPIO_Pin_7, (BitAction) BUTTON_1_STATE);
			GPIO_WriteBit(GPIOB, GPIO_Pin_8 | GPIO_Pin_9, (BitAction) BUTTON_2_STATE);

			reset_buttons();

			//printf("Button_1 (PA0): %d; Button_2 (PA15): %d\n\r", BUTTON_1_STATE, BUTTON_2_STATE);
		}
		// Если нажаты обе кнопки
		else if (BUTTON_1_STATE && BUTTON_2_STATE) {
			reset_diods();

			// Включаем центральные светодоиоды
			GPIO_WriteBit(GPIOB, GPIO_Pin_7, (BitAction) BUTTON_1_STATE);
			GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction) BUTTON_2_STATE);

			reset_buttons();

			//printf("Button_1 (PA0): %d; Button_2 (PA15): %d\n\r", BUTTON_1_STATE, BUTTON_2_STATE);
		}

		// delay
		delay(200);
	}
}
