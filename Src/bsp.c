/*
 * bsp.c
 *
 *  Created on: Jun 27, 2023
 *      Author: hussamaldean
 */


#include "bsp.h"



volatile uint32_t ticks;



void GPIOA_CLOCK_ENABLE(void)
{
	RCC->AHB1ENR|= RCC_AHB1ENR_GPIOAEN;
}

void GPIOC_CLOCK_ENABLE(void)
{
	RCC->AHB1ENR|= RCC_AHB1ENR_GPIOCEN;
}

void GPIO_Initialization(GPIO_TypeDef *GPIO,GPIO_Configure_Typedef * GPIO_Config)
{
	if (GPIO_Config->Mode !=Alternate_function)
	{


		GPIO->MODER &=~((0x03<<(GPIO_Config->PinNumber*2)));
		GPIO->MODER|= (GPIO_Config->Mode<< GPIO_Config->PinNumber*2);

		GPIO->OTYPER|= (GPIO_Config->OutputType<<GPIO_Config->PinNumber);

		GPIO->OSPEEDR|= (GPIO_Config->OutPutspeed<< GPIO_Config->PinNumber*2);

		GPIO->PUPDR|= (GPIO_Config->PullUp_PullDown<< GPIO_Config->PinNumber*2);
	}

	if (GPIO_Config->Mode == Alternate_function)
	{
		GPIO->MODER &=~((0x03<<(GPIO_Config->PinNumber*2)));
		GPIO->MODER|= (GPIO_Config->Mode<< GPIO_Config->PinNumber*2);

		GPIO->OTYPER|= (GPIO_Config->OutputType<<GPIO_Config->PinNumber);

		GPIO->OSPEEDR|= (GPIO_Config->OutPutspeed<< GPIO_Config->PinNumber*2);

		GPIO->PUPDR|= (GPIO_Config->PullUp_PullDown<< GPIO_Config->PinNumber*2);

		if (GPIO_Config->PinNumber <=pin7)
		{
			GPIO->AFR[0]= (GPIO_Config->AlternateType << GPIO_Config->PinNumber *4);
		}

		if (GPIO_Config->PinNumber>pin7 && GPIO_Config->PinNumber<=pin15)
		{
			GPIO->AFR[1]= (GPIO_Config->AlternateType << (GPIO_Config->PinNumber-8) *4);
		}

	}

}

void GPIO_WritePin(GPIO_TypeDef *GPIO,GPIO_Output_Typedef *GPIOPin, GPIO_State_Typedef state )
{

	switch (state)
	{
		case Reset:
			GPIO->BSRR = (1<<((GPIOPin->pinNumber) +16));
			GPIOPin->state=Reset;
			break;

		case Set:
			GPIO->BSRR = (1<<(GPIOPin->pinNumber));
			GPIOPin->state=Set;
			break;

		default: break;
	}
}

void GPIO_TogglePin(GPIO_TypeDef *GPIO,GPIO_Output_Typedef *GPIOPin)
{
	switch (GPIOPin->state)
	{
		case Reset:
			GPIO->BSRR = (1<<(GPIOPin->pinNumber));
			GPIOPin->state=Set;
			break;

		case Set:
			GPIO->BSRR = (1<<((GPIOPin->pinNumber) +16));
			GPIOPin->state=Reset;
			break;
	}
}

GPIO_State_Typedef GPIO_ReadPin(GPIO_TypeDef *GPIO,GPIO_Input_Typedef *pin)
{
	uint8_t temp;
	temp= (GPIO->IDR & (1<<pin->pinNumber))>>pin->pinNumber;
	return temp;
}


Clock_Config_State_Typedef  Clock_Configuration(Clock_Config_Typedef * config)
{
	if (config->clockSourc==External_Oscillator)
	{
		__IO uint32_t StartUpCounter = 0, HSEStatus = 0;


		RCC->CR |= ((uint32_t)RCC_CR_HSEON);


		do
		{
			HSEStatus = RCC->CR & RCC_CR_HSERDY;
			StartUpCounter++;
		} while((HSEStatus == 0) && (StartUpCounter != 3000));

		if ((RCC->CR & RCC_CR_HSERDY) != RESET) //HSE enabled
		{
			RCC->APB1ENR |= RCC_APB1ENR_PWREN; //enable power regulator
			PWR->CR &= (uint32_t)~(PWR_CR_VOS); //reset VOS (mode 3 selected)

			RCC->CFGR |=(config->AHB1Prescaler<<RCC_CFGR_HPRE_Pos); /*Configure AHB bus prescaler*/

			RCC->CFGR |= (config->APB2Prescaler<<RCC_CFGR_PPRE2_Pos); /*Configure APB2 bus prescaler*/

			RCC->CFGR |= (config->APB1Prescaler<<RCC_CFGR_PPRE1_Pos); /*Configure APB1 bus prescaler*/

		    RCC->PLLCFGR = config->PLL_M | (config->PLL_N << 6) | (((config->PLL_P >> 1) -1) << 16) | //set PLL_M,PLL_N,PLL_P
		                   (RCC_PLLCFGR_PLLSRC_HSE) /*Set PLL clock source to be external oscillator*/;

		    RCC->CR |= RCC_CR_PLLON; //turn on the PLL


		    while((RCC->CR & RCC_CR_PLLRDY) == 0) //wait untill PLL is active
		    {
		    }

		    /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
		       FLASH->ACR = FLASH_ACR_ICEN |FLASH_ACR_DCEN |(config->flash_latency<<FLASH_ACR_LATENCY_Pos);

		       /* Select the main PLL as system clock source */
		       RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		       RCC->CFGR |= RCC_CFGR_SW_PLL;

		       /* Wait till the main PLL is used as system clock source */
		       while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL)
		       {;}

		}
		else
		{
			return failed;
		}



	}

	else
	{
		RCC->APB1ENR |= RCC_APB1ENR_PWREN; //enable power regulator
		PWR->CR &= (uint32_t)~(PWR_CR_VOS); //reset VOS (mode 3 selected)

		/*Configure AHB bus prescaler*/
		RCC->CFGR |=(config->AHB1Prescaler<<RCC_CFGR_HPRE_Pos);

		RCC->CFGR |= (config->APB2Prescaler<<RCC_CFGR_PPRE2_Pos);

		RCC->CFGR |= (config->APB1Prescaler<<RCC_CFGR_PPRE1_Pos);

		RCC->PLLCFGR = config->PLL_M | (config->PLL_N << 6) | (((config->PLL_P >> 1) -1) << 16); //set PLL_M,PLL_N,PLL_P


		RCC->CR |= RCC_CR_PLLON; //turn on the PLL


		while((RCC->CR & RCC_CR_PLLRDY) == 0) //wait untill PLL is active
		{
		}

		/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
		FLASH->ACR = FLASH_ACR_ICEN |FLASH_ACR_DCEN |(config->flash_latency<<FLASH_ACR_LATENCY_Pos);

		/* Select the main PLL as system clock source */
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= RCC_CFGR_SW_PLL;

		/* Wait till the main PLL is used as system clock source */
		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL)
		{;}
	}



	return success;
}

void BSP_Ticks_Init(uint32_t freq)
{
	/*Load the SysTick value to be the core frequency over 1000
	 *
	 * Since the core frequency is in MHz, dividing it by 1000 will get 1ms period
	 * */
	SysTick->LOAD=(freq/1000)-1;

	/*Set the source to be internal core clock*/
	SysTick->CTRL=(1<<SysTick_CTRL_CLKSOURCE_Pos);

	/*Enable The interrupt */

	SysTick->CTRL|=(1<<SysTick_CTRL_TICKINT_Pos);

	/*Enable Systick Interrupt in NIVC*/

	NVIC_EnableIRQ(SysTick_IRQn);

	/*Enable Systick*/
	SysTick->CTRL|=(1<<SysTick_CTRL_ENABLE_Pos);

}

void SysTick_Handler(void)
{
	ticks++;
}

uint32_t BSP_Get_Ticks(void)
{
	uint32_t current_ticks;
	__disable_irq();
	current_ticks=ticks;
	__enable_irq();
	/*Return the counter value*/
	return current_ticks;
}

/*Spin lock the CPU to force delay*/
void BSP_Delay(uint32_t delay_ms)
{

	uint32_t ticks_start=BSP_Get_Ticks();

	while(BSP_Get_Ticks()-ticks_start<delay_ms);
}
