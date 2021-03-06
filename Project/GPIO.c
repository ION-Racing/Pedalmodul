#include "stm32f4xx.h"
#include "GPIO.h"

void InitGPIO(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		
	// Setup GPIO LED pins
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	// Setup outputs
	/*
		PB13	RTDS
		PB14	Bremselys (BL)
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void LED_SetState(uint8_t led, FunctionalState state){
	uint32_t pins;
	switch(led){
		case LED_GREEN:
			pins = GPIO_Pin_6;
			break;
		case LED_RED:
			pins = GPIO_Pin_7;
			break;
		case LED_BLUE:
			pins = GPIO_Pin_8;
			break;
		default:
			return;			
	}
	
	if(state == ENABLE){
		GPIOC->ODR |= pins;
	}
	else {
		GPIOC->ODR &= ~pins;		
	}
}