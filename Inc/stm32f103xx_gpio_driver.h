/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Sep 1, 2023
 *      Author: Gabriel Lopes
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_

#include "stm32f1xx.h"


/*
 * This is a configuration structure for a GPIO pin
 */


typedef struct{
	GPIO_TypeDef *port;
	uint32_t pin;
	uint32_t mode;
	uint32_t mode_type;
	uint32_t pull;
	uint32_t speed;
	uint32_t alt_func;
}GPIO_PinConfig_t;

typedef enum{
	RISING_EDGE,
	FALLING_EDGE,
	RISING_FALLING_EDGE
}edge_select;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0 				0
#define GPIO_PIN_NO_1 				1
#define GPIO_PIN_NO_2 				2
#define GPIO_PIN_NO_3 				3
#define GPIO_PIN_NO_4 				4
#define GPIO_PIN_NO_5 				5
#define GPIO_PIN_NO_6 				6
#define GPIO_PIN_NO_7 				7
#define GPIO_PIN_NO_8 				8
#define GPIO_PIN_NO_9 				9
#define GPIO_PIN_NO_10 				10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12 				12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */

//PIN MODE
#define OUTPUT_MODE						((uint32_t) 0x01)
#define INPUT_MODE						((uint32_t) 0x02)

//Define the pin mode
#define GPIO_MODE_IN     				((uint32_t)0x00)
#define GPIO_MODE_OUT_10MHz    			((uint32_t)0x01)
#define GPIO_MODE_OUT_2MHz  			((uint32_t)0x02)
#define GPIO_MODE_OUT_50MHz 			((uint32_t)0x03)

//if pin mode is defined as input
#define GPIO_ANALOG_MODE 				((uint32_t)0x00)
#define GPIO_FLOATING_IN 				((uint32_t)0x01)
#define GPIO_IN_PUPD 					((uint32_t)0x02)
#define GPIO_RESERVED 					((uint32_t)0x03)

//if pin mode is defined as output
#define GPIO_OUT_PUSHPULL 				((uint32_t)0x00)
#define GPIO_OUT_OPENDRAIN 				((uint32_t)0x01)
#define	GPIO_ALTFN_OUT_PUSHPULL 		((uint32_t)0x02)
#define GPIO_ALTFN_OUT_OPENDRAIN 		((uint32_t)0x03)

//HIGH BIT POSITIONS FOR CRH REGISTER CNFYG AND MODE
#define CNF_POS_BIT1					(PINPOS[pinNumber] + 2)
#define CNF_POS_BIT2					(PINPOS[pinNumber] + 3)

#define GPIO_PCLK_EN_ALT_FUNC			(RCC->APB2ENR |= (1 << 0))
#define GPIOA_PCLK_EN 					(RCC->APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN 					(RCC->APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN 					(RCC->APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN 					(RCC->APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN 					(RCC->APB2ENR |= (1 << 6))


#define GPIOA_PCLK_DI 					(RCC->APB2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI 					(RCC->APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI 					(RCC->APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI 					(RCC->APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI 					(RCC->APB2ENR &= ~(1 << 6))


/**********************************************************************************
 *   						API´s supported by this driver
 *         For more information about the API's the function definitions
 **********************************************************************************/

/*
 * Peripheral Clock setup
 */
//void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx, uint8_t EnorDi);

/*
 * GPIO CONFIGURATION FUNCTIONS
 */

void gpio_init(GPIO_PinConfig_t *pGPIOx, uint8_t EnorDi);
void config_pin (GPIO_TypeDef *pGPIOx, uint32_t pinNumber, uint32_t mode_type);
void config_pin_speed (GPIO_TypeDef *pGPIOx, uint32_t pinNumber, uint32_t pinSpeed, uint32_t mode);

//**************************************************************************
//                         GPIO PIN FUNCTIONS
void gpio_write(GPIO_TypeDef *pGPIOx, uint32_t pinNumber, uint8_t state);
uint8_t gpio_read(GPIO_TypeDef *pGPIOx, uint8_t PinNumber);
void gpio_toggle(GPIO_TypeDef *pGPIOx, uint32_t pinNumber);

//**************************************************************************
//                         INTERRUPT FUNCTIONS

void configure_gpio_interrupt(GPIO_TypeDef *pGPIOx, uint32_t pinNumber, edge_select edge);
void enable_gpio_interrupt(uint32_t pinNumber, IRQn_Type irqNumber);
void clear_gpio_interrupt(uint32_t pinNumber);


#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
