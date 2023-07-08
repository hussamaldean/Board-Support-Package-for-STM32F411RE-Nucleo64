/*
 * bsp.h
 *
 *  Created on: Jun 27, 2023
 *      Author: hussamaldean
 */

#ifndef BSP_H_
#define BSP_H_


#include "stdint.h"
#include "stm32f4xx.h"

/**
 * @brief This enum represents the Pin number of GPIO.
 * Starts from GPIOx0 to GPIOx15 where x is the port.
 */

typedef enum
{
	pin0=0, /*Pin0*/
	pin1,
	pin2,
	pin3,
	pin4,
	pin5,
	pin6,
	pin7,
	pin8,
	pin9,
	pin10,
	pin11,
	pin12,
	pin13,
	pin14,
	pin15 /*Pin15*/
}Pins_Typedef;


/**
 * @brief This enum represents the different GPIO mode.
 */

typedef enum
{
	INPUT=0, /*Pin in input mode*/
	OUTPUT, /*Pin in output mode*/
	Alternate_function, /*Pin in alternate function mode*/
	Analog_Mode /*Pin in analog  input/output mode*/

}GPIO_ModeTypedef;


/**
 * @brief This enum represents the output mode type.
 */

typedef enum
{
	Push_Pull=0,  /*Pin in push pull mode*/
	Open_Drain	 /*Pin in open drain mode*/
}GPIO_Output_Type_Typedef;


/**
 * @brief This enum represents the speed of GPIO output.
 */
typedef enum
{
	Low_Speed=0, /*Pin in low speed*/
	Medium_Speed, /*Pin in medium speed*/
	Fast_Speed, /*Pin in fast speed*/
	High_Speed /*Pin in high speed*/
}GPIO_OutputSpeed_Typedef;


/**
 * @brief This enum represents the Pullup Pulldown mode.
 */
typedef enum
{
	No_Pullup_PullDown=0, /*No pullup nor pulldown*/
	PullUp, /* pullup Activated*/
	PullDown /* pulldown Activated*/
}GPIO_PullUPPullDown_Typedef;


/**
 * @brief This enum represents the type of alternate function.
 */
typedef enum
{
	AF0=0, /*AF0 is selected*/
	AF1, /*AF1 is selected*/
	AF2, /*AF2 is selected*/
	AF3, /*AF3 is selected*/
	AF4, /*AF4 is selected*/
	AF5, /*AF5 is selected*/
	AF6, /*AF6 is selected*/
	AF7, /*AF7 is selected*/
	AF8, /*AF8 is selected*/
	AF9, /*AF9 is selected*/
	AF10, /*AF10 is selected*/
	AF11, /*AF11 is selected*/
	AF12, /*AF12 is selected*/
	AF13, /*AF13 is selected*/
	AF14, /*AF14 is selected*/
	AF15  /*AF15 is selected*/

}Alternate_Type_Typedef;

/**
 * @brief This enum represents the GPIO state for input and output.
 */

typedef enum
{
	Reset, /*GPIO is in low state*/
	Set /*GPIO is in high state*/

}GPIO_State_Typedef;

/**
 * @brief This enum represents the AHB1 prescaller.
 */

typedef enum
{
	AHB1_Prescaler1=0, /*HCLK divided by 1*/
	AHB1_Prescaler2=0x8, /*HCLK divided by 2*/
	AHB1_Prescaler4=0x09, /*HCLK divided by 4*/
	AHB1_Prescaler8=0x0A, /*HCLK divided by 8*/
	AHB1_Prescaler16=0x0B, /*HCLK divided by 16*/
	AHB1_Prescaler64=0x0C, /*HCLK divided by 64*/
	AHB1_Prescaler128=0x0D, /*HCLK divided by 128*/
	AHB1_Prescaler256=0x0E, /*HCLK divided by 256*/
	AHB1_Prescaler512=0x0F /*HCLK divided by 512*/

}AHB1_Prescalers_Typedef;


/**
 * @brief This enum represents the APB1 prescaller.
 */

typedef enum
{
	APB1_Prescaler1=0x00, /*AHB1 divided by 1*/
	APB1_Prescaler2=0x04, /*AHB1 divided by 2*/
	APB1_Prescaler4=0x05, /*AHB1 divided by 4*/
	APB1_Prescaler8=0x06, /*AHB1 divided by 8*/
	APB1_Prescaler16=0x07 /*AHB1 divided by 16*/
}APB1_Prescalers_Typedef;


/**
 * @brief This enum represents the APB2 prescaller.
 */
typedef enum
{
	APB2_Prescaler1=0x00, /*AHB1 divided by 1*/
	APB2_Prescaler2=0x04, /*AHB1 divided by 2*/
	APB2_Prescaler4=0x05, /*AHB1 divided by 4*/
	APB2_Prescaler8=0x06, /*AHB1 divided by 8*/
	APB2_Prescaler16=0x07 /*AHB1 divided by 16*/
}APB2_Prescalers_Typedef;


/**
 * @brief This enum represents oscillator source.
 */
typedef enum
{
	Internal_Oscillator=0, /*Internal oscillator is selected*/
	External_Oscillator=1 /*External oscillator is selected*/
}Clock_Source_Typedef;



/**
 * @brief This enum represents flash latency.
 */

typedef enum

{
	Zero_wait_state=0, /*Zero wait state*/
	One_wait_state,
	Two_wait_state,
	Three_wait_state,
	Four_wait_state,
	five_wait_state,
	six_wait_state,
	seven_wait_state,
	eight_wait_state,
	nine_wait_state,
	Ten_wait_state,
	Eleven_wait_state,
	Twelve_wait_state,
	Thirteen_wait_state,
	Fourteen_wait_state,
	Fifteen_wait_state /*Fifteen wait state*/
}Flash_Latency_Typedef;


/**
 * @brief This enum represents clock setup state.
 */

typedef enum
{
	success=0,
	failed=1

}Clock_Config_State_Typedef;

/**
 * @brief This represents GPIO configuration.
 */

typedef struct
{
	uint8_t PinNumber; /* The pin number*/
	uint8_t Mode;  /*Pin Mode*/
	uint8_t OutputType; /*Output Type*/
	uint8_t OutPutspeed; /*Output speed*/
	uint8_t PullUp_PullDown; /*Pullup/pulldown*/
	uint8_t AlternateType; /*Alternate type number*/

}GPIO_Configure_Typedef;

/**
 * @brief This represents GPIO control output mode.
 */

typedef struct
{
	uint8_t pinNumber; /*Pin number*/
	uint8_t state; /*State of the pin*/
}GPIO_Output_Typedef;


/**
 * @brief This represents GPIO input.
 */

typedef struct
{
	uint8_t pinNumber; /*Pin number*/
}GPIO_Input_Typedef;


/**
 * @brief This represents Clock configuration.
 */

typedef struct
{
	uint8_t clockSourc; /*Clcok Source*/

	/*PLL_M, PLL_N and PLL_P values*/
	uint16_t PLL_M;
	uint16_t PLL_N;
	uint16_t PLL_P;

	/*Flash latency value*/
	uint8_t flash_latency;

	/*AHB1, APB1 and PA2 prescalelr values*/
	uint8_t AHB1Prescaler;
	uint8_t APB1Prescaler;
	uint8_t APB2Prescaler;


}Clock_Config_Typedef;


/**
 * @brief Enable clock access to GPIOA
 *
 * This function takes no argument and returns nothing.
 *
 */
void GPIOA_CLOCK_ENABLE();


/**
 * @brief Enable clock access to GPIOC
 *
 * This function takes no argument and returns nothing.
 *
 */
void GPIOC_CLOCK_ENABLE();

/**
 * @brief Initialize the pin according to requirement.
 *
 * This function will initialize the pin to the required mode.
 * i.e. Input mode, analog mode etc.
 * It takes pointer to GPIO_Typedef and pointer to GPIO_Config_Typedef
 * and returns nothing
 *
 * @param GPIO pointer to GPIO_Typedef.
 * @param GPIO_Config pointer to GPIO_Configure_Typedef.
 */

void GPIO_Initialization(GPIO_TypeDef *GPIO,GPIO_Configure_Typedef *GPIO_Config);


/**
 * @brief Write the state to GPIO pin in output state.
 *
 * This function will set/reset the pin according the passed state.
 * i
 * It takes pointer to GPIO_Typedef, pointer to GPIO_Config_Typedef
 * and the desired state and  returns nothing
 *
 * @param GPIO pointer to GPIO_Typedef.
 * @param GPIO_Config pointer to GPIO_Output_Typedef.
 * @param value to be set (set/reset)
 */

void GPIO_WritePin(GPIO_TypeDef *GPIO,GPIO_Output_Typedef *GPIOPin, GPIO_State_Typedef state );


/**
 * @brief Toggle the state to GPIO pin in output state.
 *
 * This function will toggle the state of the pin.
 * i
 * It takes pointer to GPIO_Typedef, pointer to GPIO_Config_Typedef
 * and returns nothing
 *
 * @param GPIO pointer to GPIO_Typedef.
 * @param GPIO_Config pointer to GPIO_Output_Typedef.
 *
 */

void GPIO_TogglePin(GPIO_TypeDef *GPIO,GPIO_Output_Typedef *GPIOPin);


/**
 * @brief Read the GPIO Pin in input mode.
 *
 * This function will read the state of the GPIO pin.

 * It takes pointer to GPIO_Typedef and pointer to GPIO_Input_Typedef
 * and returns GPIO_State_Typedef.
 *
 * @param GPIO pointer to GPIO_Typedef.
 * @param GPIO_Config pointer to GPIO_Input_Typedef.
 * @return GPIO_State_Typedef
 */

GPIO_State_Typedef GPIO_ReadPin(GPIO_TypeDef *GPIO,GPIO_Input_Typedef *pin);


/**
 * @brief Configure the clock.
 *
 * This function will configure the clock of STM32F4.
 *
 * It takes pointer to Clock_Config_Typedef
 * and Clock_Config_State_Typedef
 *
 * @param GPIO pointer to Clock_Config_Typedef.
 * @return Clock_Config_State_Typedef
 */

Clock_Config_State_Typedef Clock_Configuration(Clock_Config_Typedef * config);

/**
 * @brief Initialize the SysTick for timing managment.
 *
 * This function will initialize SysTick to generate
 * interrupt each 1ms. It takes the core frequency as
 * parameter and return nothing
 *
 * @param Core frequency in Hz.
 */

void BSP_Ticks_Init(uint32_t freq);

/**
 * @brief Get the current ticks of the system.
 *
 * This function will return number of the ticks since
 * the system has booted
 * It takes nothing and returns number of ticks passed.
 *
 * @return Number of ticks.
 */

uint32_t BSP_Get_Ticks(void);

/**
 * @brief Delay the code by x amounts in milliseconds.
 *
 * This function will spin lock the CPU to achieve
 * the required delay
 * It takes amount to be delayed and returns nothing.
 *
 * @param Amount to be delayed in milliseconds.
 */


void BSP_Delay(uint32_t delay_ms);

#endif /* BSP_H_ */
