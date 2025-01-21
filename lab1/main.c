#include <stdio.h>
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

GPIO_InitTypeDef GPIO_InitStructure, GPIOA_InitStructure;

// Базовая установка рабочих параметров
inline void setup() {
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

inline void delay() {
	int i;

    for ( i = 0; i < 0x5000; i++);
}

inline void full_reset() {
	GPIO_ResetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
}

int main(void) {
  SystemInit();
  setup();

  int i;

  while (1) {
	// Чтение состояния кнопок PA0 PA15
    short int button1 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
    short int button2 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15);

	full_reset();

    // Если нажата одна кнопка
    if (button1 ^ button2) {
    	// Включаем два левых (или правых) светодиода
    	GPIO_WriteBit(GPIOB, GPIO_Pin_6 | GPIO_Pin_7, (BitAction) button1);
    	GPIO_WriteBit(GPIOB, GPIO_Pin_8 | GPIO_Pin_9, (BitAction) button2);

        printf("Button_1 (PA0): %d; Button_2 (PA15): %d\n\r", button1, button2);
    }
    // Если нажаты обе кнопки
    else if (button1 && button2) {
    	// Включаем центральные светодоиоды
    	GPIO_WriteBit(GPIOB, GPIO_Pin_7, (BitAction) button1);
    	GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction) button2);

        printf("Button_1 (PA0): %d; Button_2 (PA15): %d\n\r", button1, button2);
    }

    // delay
    delay();
  }
}
