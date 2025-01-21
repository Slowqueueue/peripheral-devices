#include <stdio.h>
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <stm32f10x_adc.h>
#include "stm32f10x_dma.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#define ADC1_DR_Address ((uint32_t)0x4001244C)

GPIO_InitTypeDef GPIO_InitStructure;
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
__IO uint16_t ADCConvertedValue[4]; // буффер со считанными значениями (с 4 каналов)

void delay() {
	int i;

    for ( i = 0; i < 0x5000; i++);
}

void full_reset() {
	GPIO_ResetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
}

int main(void) {
	// Инициализация RCC
	SystemInit();
	/* Установка тактовой частоты аналого-цифровому преобразователю */
	RCC_ADCCLKConfig(RCC_PCLK2_Div4);
	// enable ADC system clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
	/* Включение контроллера прямого доступа к памяти DMA*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	// Включение GPIOA и управление альтернативными функциями
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	// Использование пинов PA0 и PA15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_15 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	// Включение режима работы в Input
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	// Инициализация GPIOA
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	// Использование пинов PB6-PB9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	// Режим работы - Outpup Push-Pull
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // независимый режим работы
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; // Сканирование каналов включено 19
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 4; // Число каналов - 4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5); //конфигурирование первого канала АЦП
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_55Cycles5); //конфигурирование второго канала АЦП
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_55Cycles5); //конфигурирование третьего канала АЦП
	ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 4, ADC_SampleTime_55Cycles5);
	// конфигурирование четвертого канала АЦП
	ADC_Init(ADC1, &ADC_InitStructure); // инициализация
	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) & ADCConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 4; // размер буффера 4
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; // инкремент памяти включен
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	// включение АЦП
	ADC_Cmd(ADC1, ENABLE);
	// Калибровка АЦП
	ADC_ResetCalibration(ADC1); // Сброс предыдущей калибровки
	while (ADC_GetResetCalibrationStatus(ADC1))
	;
	ADC_StartCalibration(ADC1); // Запуск калибровки
	while (ADC_GetCalibrationStatus(ADC1))
	;
	ADC_Cmd(ADC1, ENABLE);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE); // Программный запуск непрерывного преобразования

	while (1) {
		int flag1, flag2;
		int _0 = ADCConvertedValue[0];
		int _1 = ADCConvertedValue[1];
		int _2 = ADCConvertedValue[2];
		int _3 = ADCConvertedValue[3];

		flag1 = ADCConvertedValue[1] > 1500;
		flag2 = ADCConvertedValue[2] > 1500;

		full_reset();

	    // Если поднят 1 флаг
	    if (flag1 ^ flag2) {
	    	// Включаем два левых (или правых) светодиода
	    	GPIO_WriteBit(GPIOB, GPIO_Pin_6 | GPIO_Pin_7, (BitAction) flag1);
	    	GPIO_WriteBit(GPIOB, GPIO_Pin_8 | GPIO_Pin_9, (BitAction) flag2);
	    }
	    // Если подняты оба флага
	    else if (flag1 && flag2) {
	    	// Включаем центральные светодоиоды
	    	GPIO_WriteBit(GPIOB, GPIO_Pin_7, (BitAction) flag1);
	    	GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction) flag2);
	    }
	    // delay
	    delay();
	}
}
