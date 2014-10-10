#ifndef _BSP_H
#define _BSP_H

/**
 * @brief Prende un led
 *
 * @param led Led a prender
 */
void led_on(uint8_t led);

/**
 * @brief Apaga un led
 *
 * @param led Led a apagar
 */
void led_off(uint8_t led);

/**
 * @brief Conmuta un led
 *
 * @param led Led a apagar
 */
void led_toggle(uint8_t led);

/**
 * @brief PWM
 *
 * @param led
 * @param value Ciclo de trabajo
 */
void led_set_bright (uint8_t led, uint8_t value);

/**
 * @brief Delay de x ms
 *
 * @param x cantidad de ms
 */
void bsp_delay_ms(uint16_t x);

/**
 * @brief Delay por software
 */
uint8_t sw_getState();

/**
 * @brief Lectura ADC
 */
uint16_t read_adc();

/**
 * @brief Valor Potenciometro a Leds
 */
uint16_t vumetro();

/**
 * @brief Inicializacion de los servicios de BSP
 */
void bsp_init();

#endif
