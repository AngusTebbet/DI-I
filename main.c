/* 
	Pete Hubbard 2019
	Loughborough University
	WSC055 Lab 1
	V.2.0
	
	The following 'c' code presents an outline for you to adapt during the laboratory
	
	*/

#include "stm32f3xx.h"                  // Device header

int count =255; 
void SubFunction(void);
void EXTI0_IRQHandler(void);
volatile int portAPin0percent = 0;	//Initialise variable at 

int main(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;	// Enable clock on GPIO port E
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; //Enable timer clock. Timer 4 is connected to APB1

	GPIOE->MODER |= 0x08880000; // Set mode of each pin in port E
	GPIOE->OTYPER &= ~(0x0000FFFF); // Set output type for each pin required in Port E
	GPIOE->PUPDR |= 0x55555555; // Set Pull up/Pull down resistor configuration for Port E

	GPIOE->AFR[1]  |= 0x00202020; //Sets PE.2 to TIM3_CH1

	// Initialise the timer
	TIM1->PSC = 799; // Sets prescalor 
	TIM1->ARR = 9; // 

	TIM1->CCMR1 |= 0x00000060; // Set the CCR for the associated channel
	TIM1->CCR1 = 10; //Sets on time to 10 clock pulses
	
	TIM1->BDTR |= TIM_BDTR_MOE; //Enable the output mode in the BDTR
	TIM1->CCER |= TIM_CCER_CC1E; // Enable channel 1 to be output in the CCE1 register
	
	TIM1->CR1 |= TIM_CR1_CEN; // Enable the timer
	
	// GPIOE is a structure defined in stm32f303xc.h file
	// Define settings for each output pin using GPIOE structure


	TIM3->DIER |= TIM_DIER_UIE; // Set DIER register to watch out for an ‘Update’ Interrupt Enable (UIE) – or 0x00000001
	NVIC_EnableIRQ(TIM3_IRQn); // Enable Timer ‘x’ interrupt request in NVIC

	TIM3->CR1 |= TIM_CR1_CEN;
	// Main programme loop - make LED 4 (attached to pin PE.0) turn on and off	
	
	SubFunction(); // Function call for ADC Initialisation
	
	
	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //Enable system configuration controller
	EXTI->IMR |= EXTI_IMR_MR0; //Remove the mask to enable the interrupt. Unmask EXTI0. Set the mask register bit.
	EXTI->RTSR |= EXTI_RTSR_TR0; // EXTI0 to generate interrupt from rising edge
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // Confifgure multiplexing options
	
	NVIC_SetPriority(EXTI0_IRQn, 0); // Sets the priority of the interrupt
	
	EXTI0_IRQHandler();
	
	while(1)
	{

		
	if( count>65535){	
		//resets LEDs on overflow
		GPIOE->BSRRH = 0xFF00;
		count = 255;
		}
	}

}

void TIM3_IRQHandler()
{
if ((TIM3->SR & TIM_SR_UIF) !=0) // Check interrupt source is from the ‘Update’ interrupt flag
{
		GPIOE->BSRRH |= ~count; 	
		GPIOE->BSRRL |= count; 
		count=count+256;

}
TIM3->SR &= ~TIM_SR_UIF; // Reset ‘Update’ interrupt flag in the SR register
}

void SubFunction(void) //setting up the ADC
{
ADC1->CR &= 0xCFFFFFFF; //Reset ADC
ADC1->CR |= 0x10000000; //Enable ADC
	
//count 100 times
int j = 0;
while(j<100)
{
j++;
}
//Calibrate ADC
ADC1->CR &= 0xBFFFFFFF; //single-ended calibration
ADC1->CR |= 0x80000000; //start calibration

//Enable Clock connections to ADC Peripheral
RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV2;
RCC->AHBENR |= RCC_AHBENR_ADC12EN;
ADC1_2_COMMON->CCR |= 0x00010000;

// Select Input pin & Configure GPIO Port
GPIOA->MODER &= 0xFFFFFFFE; //Mode of PA0 in Port A to analogue
GPIOA->MODER |= 0x00000002; //Mode of PA0 in Port A to analogue

//Configure ADC;
ADC1->CFGR |= 0x00000010; // SET RESOLUTION TO 8 BIT
ADC1->CFGR &= 0xFFFFFFF7; // SET RESOLUTION TO 8 BIT
ADC1->CFGR &= 0xFFFFFFDF; // SET ALIGNMENT TO RIGHT
ADC1->CFGR &= 0xFFFFDFFF; // SET OPERATION TO NON-CONTINUOUS

//Configure Multiplexing Options
ADC1->SQR1 |= 0x00000040; //sampling channel 1 setting SQ1 to 00001
ADC1->SQR1 |= 0x00000001; //length of sequence is 1
ADC1->SMPR1 |= 0x00000018; // sample time is 7.5 clock sample, setting SMP1 to 011
ADC1->SMPR1 &= 0xFFFFFFDF; // sample time is 7.5 clock sample, setting SMP1 to 011

//Enable ADC
ADC1->CR |= 0x00000001;

//Wait for ADRDY Flag to go high

while (ADC1->ISR &= 0xFFFFFFFE)

{
}
}

void EXTI0_IRQHandler()
{
	if (EXTI->PR & EXTI_PR_PR0) // check source
	{
	
	EXTI->PR |= EXTI_PR_PR0; // clear flag*
	
		portAPin0percent = portAPin0percent + 25; //Increment duty cycle by 25%
	
	TIM1->CCR1 |= ~portAPin0percent; //Write percentage of duty cycle into the CCR register
		
	if(portAPin0percent == 100) //Reset the variable to 0 once it exceeds 100
		{ 
		portAPin0percent = 0;
		}
	
	}
};


