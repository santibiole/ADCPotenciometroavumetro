#include <stdint.h>
#include "stm32f4xx.h"			// Header del micro
#include "stm32f4xx_gpio.h"		// Perifericos de E/S
#include "stm32f4xx_rcc.h"		// Para configurar el (Reset and clock controller)
#include "stm32f4xx_tim.h"		// Modulos Timers
#include "stm32f4xx_exti.h"		// Controlador interrupciones externas
#include "stm32f4xx_syscfg.h"	// configuraciones Generales
#include "stm32f4xx_adc.h"		// Modulo analogico/digital
#include "misc.h"				// Vectores de interrupciones (NVIC)
#include "bsp.h"

#define LED_V GPIO_Pin_12
#define LED_N GPIO_Pin_13
#define LED_R GPIO_Pin_14
#define LED_A GPIO_Pin_15

#define BOTON GPIO_Pin_0

/* Puertos de los leds disponibles */
GPIO_TypeDef* leds_port[] = { GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD,
		GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOB, GPIOB, GPIOB };
/* Leds disponibles */
const uint16_t leds[] = { LED_V, LED_R, LED_N, LED_A, GPIO_Pin_0, GPIO_Pin_1,
		GPIO_Pin_2, GPIO_Pin_3, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_10,
		GPIO_Pin_11, GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_4 };

extern void APP_ISR_sw(void);
extern void APP_ISR_1ms(void);

volatile uint16_t bsp_contMS = 0;

void led_on(uint8_t led) {
	GPIO_SetBits(leds_port[led], leds[led]);
}

void led_off(uint8_t led) {
	GPIO_ResetBits(leds_port[led], leds[led]);
}

void led_toggle(uint8_t led) {
	GPIO_ToggleBits(leds_port[led], leds[led]);
}

uint8_t sw_getState(void) {
	return GPIO_ReadInputDataBit(GPIOA, BOTON);
}

void bsp_delayMs(uint16_t x) {
	bsp_contMS = x;

	while (bsp_contMS)
		;

}

/**
 * @brief Interrupcion llamada cuando se preciona el pulsador
 */
void EXTI0_IRQHandler(void) {

	if (EXTI_GetITStatus(EXTI_Line0) != RESET) //Verificamos si es la del pin configurado
			{
		EXTI_ClearFlag(EXTI_Line0); // Limpiamos la Interrupcion
		// Rutina:
		APP_ISR_sw();
	}
}

/**
 * @brief Interrupcion llamada al pasar 1ms
 */
void TIM2_IRQHandler(void) {

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		APP_ISR_1ms();

		if (bsp_contMS) {
			bsp_contMS--;
		}
	}
}

void bsp_led_init();
void bsp_sw_init();
void bsp_timer_config();
void InitAdc();

void bsp_init() {
	bsp_led_init();
	bsp_sw_init();
	bsp_timer_config();
	InitAdc();

}

/**
 * @brief Inicializa Leds
 */
void bsp_led_init() {
	GPIO_InitTypeDef GPIO_InitStruct;

	// Arranco el clock del periferico
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOB, ENABLE);

	//RGB
	//RCC_AHB1PeriphClockCmd(, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Pin |= GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Pin |= GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Pin |= GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Pin |= GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Pin |= GPIO_Pin_14 | GPIO_Pin_15;

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // (Push/Pull)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
 * @brief Inicializa SW
 */
void bsp_sw_init() {
	GPIO_InitTypeDef GPIO_InitStruct;

	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	// Arranco el clock del periferico
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configuro interrupcion

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	/* Configuro EXTI Line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Habilito la EXTI Line Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief Inicializa TIM2
 */
void bsp_timer_config(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Habilito la interrupcion global del  TIM2 */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM2 habilitado */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* Configuracion de la base de tiempo */
	TIM_TimeBaseStruct.TIM_Period = 1000; // 1 MHz bajado a 1 KHz (1 ms)
	TIM_TimeBaseStruct.TIM_Prescaler = (2 * 8000000 / 1000000) - 1; // 8 MHz bajado a 1 MHz
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);
	/* TIM habilitado */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	/* TIM2 contador habilitado */
	TIM_Cmd(TIM2, ENABLE);
}

void InitAdc() {
	// Config structs
	GPIO_InitTypeDef GPIO_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	ADC_InitTypeDef ADC1_InitStruct;
// Enable the clock for ADC and the ADC GPIOs

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
// Configure these ADC pins in analog mode using GPIO_Init();
	GPIO_StructInit(&GPIO_InitStruct); // Reset gpio init structure
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN; // Obvezno AIN !!!
	GPIO_Init(GPIOC, &GPIO_InitStruct);

// Common ADC init sets the prescaler
	ADC_CommonStructInit(&ADC_CommonInitStruct);
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInit(&ADC_CommonInitStruct);
	/* ADC1 Configuration */
	ADC_StructInit(&ADC1_InitStruct);
	ADC1_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_Init(ADC1, &ADC1_InitStruct);
	ADC_Cmd(ADC1, ENABLE);
	/* Now do the setup */
	ADC_Init(ADC1, &ADC1_InitStruct);
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
}

uint16_t readADC1() {
	ADC_RegularChannelConfig(ADC1, 12, 1, ADC_SampleTime_15Cycles);
	ADC_SoftwareStartConv(ADC1);
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != SET)
		;
	return ADC_GetConversionValue(ADC1);
}

