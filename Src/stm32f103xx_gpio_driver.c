/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Sep 1, 2023
 *      Author: Gabriel Lopes
 */

#include "stm32f103xx_gpio_driver.h"
//#include "stm32f103xx.h"
#include "stm32f1xx.h"
#include "core_cm3.h"
#include <stdint.h>


uint32_t PINPOS[16] = {
		(0x00), //pin0
		(0x04), //pin1
		(0x08), //pin2
		(0x0C), //pin3
		(0x10), //pin4
		(0x14), //pin5
		(0x18), //pin6
		(0x1C), //pin7
		(0x00), //pin8
		(0x04), //pin9
		(0x08), //pin10
		(0x0C), //pin11
		(0x10), //pin12
		(0x14), //pin13
		(0x18), //pin14
		(0x1C)  //pin15
};




/*
 * Peripheral Clock setup
 */
/**
  * @Func       : GPIO_PeriClockControl
  * @brief      : Enables and Disables peripheral clock for the given GPIO port
  * @parameters : Base address of the GPIO peripheral
  * @parameters : ENABLE or DISABLE macros
  * @return     : none
  * @note       : none
  *
  * GPIO_TypeDef
  */
/*void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)	{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
	}
}*/



/*
 * Init and DeInit
 */
/**
  * @Func       : GPIO_Init
  * @brief      : Initial configuration for the given GPIO port
  * @parameters : Base address of the GPIO peripheral
  * @return     : none
  * @note       : none
  */


void gpio_init(GPIO_PinConfig_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx->port == GPIOA){
			GPIOA_PCLK_EN;
		}else if(pGPIOx->port == GPIOB){
			GPIOB_PCLK_EN;
		}else if(pGPIOx->port == GPIOC){
			GPIOC_PCLK_EN;
		}else if(pGPIOx->port == GPIOD){
			GPIOD_PCLK_EN;
		}
	}else{
		if(pGPIOx->port == GPIOA){
			GPIOA_PCLK_DI;
		}else if(pGPIOx->port == GPIOB)	{
			GPIOB_PCLK_DI;
		}else if(pGPIOx->port == GPIOC){
			GPIOC_PCLK_DI;
		}else if(pGPIOx->port == GPIOD){
			GPIOD_PCLK_DI;
		}
	}

	config_pin(pGPIOx->port, pGPIOx->pin, pGPIOx->mode_type);
	config_pin_speed(pGPIOx->port, pGPIOx->pin, pGPIOx->speed, pGPIOx->mode);
}

void config_pin (GPIO_TypeDef *pGPIOx, uint32_t pinNumber, uint32_t mode_type){
	if(pinNumber >= 8){ // CONTROL HIGH REGISTER
		switch(mode_type){
			case GPIO_OUT_PUSHPULL | GPIO_ANALOG_MODE:
				pGPIOx->CRH &= ~((1 << CNF_POS_BIT1) | (1<<CNF_POS_BIT2));
				break;
			case GPIO_OUT_OPENDRAIN | GPIO_FLOATING_IN:
				pGPIOx->CRH &= ~( 1 << CNF_POS_BIT2);
				pGPIOx->CRH |= (1 << CNF_POS_BIT1);
				break;
			case GPIO_ALTFN_OUT_PUSHPULL | GPIO_IN_PUPD:
				pGPIOx->CRH |= GPIO_ALTFN_OUT_PUSHPULL << (CNF_POS_BIT1);
				break;
			case GPIO_ALTFN_OUT_OPENDRAIN:
					pGPIOx->CRH |= GPIO_ALTFN_OUT_OPENDRAIN << (CNF_POS_BIT1);
				break;
		}
	}else{ // CONTROL LOW REGISTER
		switch(mode_type){
			case GPIO_OUT_PUSHPULL | GPIO_ANALOG_MODE:
				pGPIOx->CRL &= ~((1 << CNF_POS_BIT1) | (1<<CNF_POS_BIT2));
				break;
			case GPIO_OUT_OPENDRAIN | GPIO_FLOATING_IN:
				pGPIOx->CRL &= ~( 1 << CNF_POS_BIT2);
				pGPIOx->CRL |= (1 << CNF_POS_BIT1);
				break;
			case GPIO_ALTFN_OUT_PUSHPULL | GPIO_IN_PUPD:
				pGPIOx->CRL |= GPIO_ALTFN_OUT_PUSHPULL << (CNF_POS_BIT1);
				break;
			case GPIO_ALTFN_OUT_OPENDRAIN:
				pGPIOx->CRL |= GPIO_ALTFN_OUT_OPENDRAIN << (CNF_POS_BIT1);
				break;
		}
	}
}

void config_pin_speed (GPIO_TypeDef *pGPIOx, uint32_t pinNumber, uint32_t pinSpeed, uint32_t mode){

	if(pinNumber >= 8){
		if(mode == INPUT_MODE)
			pGPIOx->CRH &= ~ (1 << (PINPOS[pinNumber]) |  1 << (PINPOS[pinNumber] +1) );
		else
			pGPIOx->CRH = (pinSpeed << (PINPOS[pinNumber]));
	}else{
		if(mode == INPUT_MODE)
			pGPIOx->CRL &= ~ (1 << (PINPOS[pinNumber]) |  1 << (PINPOS[pinNumber] +1) );
		else
			pGPIOx->CRL |= (pinSpeed << (PINPOS[pinNumber]));
	}
}

void gpio_write(GPIO_TypeDef *pGPIOx, uint32_t pinNumber, uint8_t state){

	if(state){
		pGPIOx->BSRR = (1 << pinNumber);
	}else{
		pGPIOx->BSRR = (1 << (pinNumber +16));
	}

}


uint8_t gpio_read(GPIO_TypeDef *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & (0x00000001));
	return value;
}



void gpio_toggle(GPIO_TypeDef *pGPIOx, uint32_t pinNumber){

	pGPIOx->ODR ^=(1<<pinNumber);

}


void configure_gpio_interrupt(GPIO_TypeDef *pGPIOx, uint32_t pinNumber, edge_select edge){
	//GPIO_PCLK_EN_ALT_FUNC();
	RCC->APB2ENR |= (1<<0);

	if(pGPIOx == GPIOA){
		switch(pinNumber)
		{
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PA;
				break;
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PA;
				break;
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PA;
				break;
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PA;
				break;
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PA;
				break;
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PA;
				break;
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PA;
				break;
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PA;
				break;
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PA;
				break;
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PA;
				break;
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PA;
				break;
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PA;
				break;
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PA;
				break;
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PA;
				break;
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PA;
				break;
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PA;
				break;
			default:
				break;
		}
	}

	if(pGPIOx == GPIOB){
		switch(pinNumber)
		{
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PB;
				break;
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PB;
				break;
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PB;
				break;
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PB;
				break;
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PB;
				break;
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PB;
				break;
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PB;
				break;
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PB;
				break;
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PB;
				break;
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PB;
				break;
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PB;
				break;
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PB;
				break;
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PB;
				break;
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PB;
				break;
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PB;
				break;
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PB;
				break;
			default:
				break;
		}
	}

	if(pGPIOx == GPIOC){
		switch(pinNumber)
		{
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PC;
				break;
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PC;
				break;
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PC;
				break;
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PC;
				break;
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PC;
				break;
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PC;
				break;
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PC;
				break;
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PC;
				break;
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PC;
				break;
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PC;
				break;
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PC;
				break;
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PC;
				break;
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PC;
				break;
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PC;
				break;
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PC;
				break;
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PC;
				break;
			default:
				break;
		}
	}

	if(pGPIOx == GPIOD){
		switch(pinNumber)
		{
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PD;
				break;
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PD;
				break;
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PD;
				break;
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PD;
				break;
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PD;
				break;
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PD;
				break;
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PD;
				break;
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PD;
				break;
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PD;
				break;
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PD;
				break;
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PD;
				break;
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PD;
				break;
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PD;
				break;
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PD;
				break;
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PD;
				break;
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PD;
				break;
			default:
				break;
		}


	}


	if(edge == RISING_EDGE)
		EXTI->RTSR |= 1 << pinNumber;
	if(edge == FALLING_EDGE)
		EXTI->FTSR |= 1 << pinNumber;
	if(edge == RISING_FALLING_EDGE){
		EXTI->RTSR |= 1 << pinNumber;
		EXTI->FTSR |= 1 << pinNumber;
	}


}

void enable_gpio_interrupt(uint32_t pinNumber, IRQn_Type irqNumber){

	//enable interrupt in EXTI
	EXTI->IMR |= 1 << pinNumber;
	//enable interrupt in NVIC
	NVIC_EnableIRQ(irqNumber);
}

void clear_gpio_interrupt(uint32_t pinNumber){

	EXTI->PR |= (1 << pinNumber);
}
