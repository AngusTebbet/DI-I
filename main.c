/* 
	Pete Hubbard 2019
	Loughborough University
	WSC055 Lab 1
	V.2.0
	
	The following 'c' code presents an outline for you to adapt during the laboratory
	
	*/

#include "stm32f3xx.h"                  // Device header
int counter;

void delay(int a); // prototype for delay function

int main(void)
{
	// Enable clock on GPIO port E
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	
	TIM3->DIER |= TIM_DIER_UIE; //Set DIER register to watch out for an update
	NVIC_EnableIRQ(TIM3_IRQn);  // Enable timer3 interrupt requests in NVIC
	
	//Lab 2: Connect to the system clock
	RCC->AHBENR |=RCC_APB1ENR_DAC1EN;

	//Disable the buffer funciton.
	DAC1->CR |= DAC_CR_BOFF1;
	DAC1->CR |= DAC_CR_EN1;
	DAC->DHR12R1 |=DAC_DHR12L1_DACC1DHR;
	
	//Interrupt fpr task 3.2
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->PSC = 799; //Prescalor vale in timer3 set as 719999
	TIM3->ARR = 99; //Auto resent register of timer3 set as 99
	TIM3->CR1 |= TIM_CR1_CEN;
	
	
	
	// GPIOE is a structure defined in stm32f303xc.h file
	// Define settings for each output pin using GPIOE structure
	GPIOE->MODER |= 0x55550000; // Set mode of each pin in port E 8 - 15
	
	
	GPIOE->OTYPER |= ~(0x00000100); // Set output type for pin 8 required in Port E
	GPIOE->OTYPER &= ~(0x00000200); // Set output type for pin 9 
	GPIOE->OTYPER &= ~(0x00000400); // Set output type for pin 10
	GPIOE->OTYPER &= ~(0x00000800); // Set output type for pin 11 
	GPIOE->OTYPER &= ~(0x00001000); // Set output type for pin 12 
	GPIOE->OTYPER &= ~(0x00002000); // Set output type for pin 13 
	GPIOE->OTYPER &= ~(0x00004000); // Set output type for pin 14 
	GPIOE->OTYPER &= ~(0x00008000); // Set output type for pin 15 
	
	GPIOE->PUPDR |= ~(0x00010000); // Set Pull up/Pull down resistor configuration for Port E pin 8
	GPIOE->PUPDR &= ~(0x00080000); // Set Pull up/Pull down resistor configuration for Port E pin 9
	GPIOE->PUPDR &= ~(0x00200000); // Set Pull up/Pull down resistor configuration for Port E pin 10
	GPIOE->PUPDR &= ~(0x00800000); // Set Pull up/Pull down resistor configuration for Port E pin 11
	GPIOE->PUPDR &= ~(0x01000000); // Set Pull up/Pull down resistor configuration for Port E pin 12
	GPIOE->PUPDR &= ~(0x08000000); // Set Pull up/Pull down resistor configuration for Port E pin 13
	GPIOE->PUPDR &= ~(0x20000000); // Set Pull up/Pull down resistor configuration for Port E pin 14
	GPIOE->PUPDR &= ~(0x80000000); // Set Pull up/Pull down resistor configuration for Port E pin 15

	// Main programme loop - make LED 4 (attached to pin PE.0) turn on and off	
	while (1)
  {
		GPIOE->BSRRL = 0x0100; // Set delay turn on for LED8
		delay(700000);
		GPIOE->BSRRL = 0x0200; // Set delay turn on for LED9
		delay(700000);
		GPIOE->BSRRL = 0x0400;// Set delay turn on for LED10
		delay(700000);
		GPIOE->BSRRL = 0x0800;// Set delay turn on for LED11
		delay(700000);
		GPIOE->BSRRL = 0x1000;// Set delay turn on for LED12
		delay(700000);
		GPIOE->BSRRL = 0x2000;// Set delay turn on for LED13
		delay(700000);
		GPIOE->BSRRL = 0x4000;// Set delay turn on for LED14
		delay(700000);
		GPIOE->BSRRL = 0x8000;// Set delay turn on for LED15
		delay(700000);
		
		
		GPIOE->BSRRH = 0x0100; // Set delay turn off do LED8
		delay(700000);
		GPIOE->BSRRH = 0x0200; // Set delay turn off do LED9
		delay(700000);
		GPIOE->BSRRH = 0x0400; // Set delay turn off do LED10
		delay(700000);
		GPIOE->BSRRH = 0x0800; // Set delay turn off do LED11
		delay(700000);
		GPIOE->BSRRH = 0x1000; // Set delay turn off do LED12
		delay(700000);
		GPIOE->BSRRH = 0x2000; // Set delay turn off do LED13
		delay(700000);
		GPIOE->BSRRH = 0x4000; // Set delay turn off do LED14
		delay(700000);
		GPIOE->BSRRH = 0x8000; // Set delay turn off do LED15
		delay(700000);
	}

}

// Delay function to occupy processor
void delay (int a)
	{
    volatile int i,j;

    for (i=0 ; i < a ; i++)
    {
        j++;
    }

    return;
	}

//Enable interrupt to be set as Update interrupt on timer 3

	void TIM3_IRQHandler()
	{
	if ((TIM3->SR & TIM_SR_UIF) !=0) // Check interrupt source is from the ‘Update’ interrupt flag
	{
		counter++;
		if(counter & 0x0001)
		//turns the LED on
			GPIOE->BSRRL = 0x0200; // sets pin 9 to flash on
		else
			GPIOE->BSRRH = 0x0200; //sets pin 9 not to flash
	}
	TIM3->SR &= ~TIM_SR_UIF; // Reset ‘Update’ interrupt flag in the SR register
	}

