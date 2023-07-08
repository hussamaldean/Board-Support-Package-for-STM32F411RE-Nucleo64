#include "bsp.h"

GPIO_Output_Typedef LED;


Clock_Config_Typedef clockConfig;



int main()
{



	GPIOA_CLOCK_ENABLE();

	GPIO_Configure_Typedef LED_Config;

	LED_Config.PinNumber=pin5;
	LED_Config.Mode=OUTPUT;

	LED.pinNumber=pin5;

	GPIO_Initialization(GPIOA,&LED_Config);

	clockConfig.PLL_M= 4;
	clockConfig.PLL_N= 200;
	clockConfig.PLL_P= 4;

	clockConfig.AHB1Prescaler=AHB1_Prescaler1;
	clockConfig.APB1Prescaler=APB1_Prescaler2;
	clockConfig.APB2Prescaler=APB2_Prescaler1;

	clockConfig.clockSourc=External_Oscillator;
	clockConfig.flash_latency= Three_wait_state;

	Clock_Configuration(&clockConfig);

	GPIO_Configure_Typedef PA2_ALT;

	PA2_ALT.PinNumber=pin2;
	PA2_ALT.Mode=Alternate_function;
	PA2_ALT.AlternateType=AF7;

	GPIO_Initialization(GPIOA,&PA2_ALT);


	GPIO_Configure_Typedef PA9_ALT;

	PA9_ALT.PinNumber=pin9;
	PA9_ALT.Mode=Alternate_function;
	PA9_ALT.AlternateType=AF7;

	GPIO_Initialization(GPIOA,&PA9_ALT);



	BSP_Ticks_Init(100000000);


	while(1)
	{

		GPIO_TogglePin(GPIOA, &LED);
		BSP_Delay(1000);
	}

}


