#include "stm32f3xx.h"                  // Device header 	 


enum output(potentiometer, encoder, average);
enum output current_output;
enum output nextOutput = potentiometer;

volatile int InterruptPercentage = 0; //Initialise the interrupt variable at 0. 
void EXTI0_IRQHandler(void);
void enable_DAC(void);
void enable_ADC(void);
void configure_DAC_out(void);


int main(void)
{
	enable_DAC();
	EXTI0_IRQHandler();
	enable_ADC();
	
	
	while(1){
		configure_DAC_out();
		
		ADC1->CR |= ADC_CR_ADSTART;
		while(!ADC1->ISR & ADC_ISR_EOC)
		GPIOE->BSRRH = 0xFF00; //Turn off the LEDs from the previous count
		GPIOE->BSRRL = (ADC1->DR * 0x0100); //Turn on the New count LEDs
	}
	
	
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;	// Enable clock on GPIO port E
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; //Enable timer clock. Timer 4 is connected to APB1
	
	TIM1->PSC = 100; 
	TIM1->ARR = 1000; // Period
	TIM1 ->CCR1;
	TIM1 ->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; //Sets PE.9 to OC register 1 Channel 1
	TIM1 ->CCER |= TIM_CCER_CC1E; //Output mode enable
	TIM1->CR1 |= TIM_CR1_CEN | TIM_CR1_CMS_0; //Sets counter to centre alligned mode to form the triangle wave. Counts up and down.
	
	GPIOE->MODER |= 0x08880000; // Set mode of each pin in port E
	GPIOE->OTYPER &= ~(0x0000FFFF); // Set output type for each pin required in Port E
	GPIOE->PUPDR |= 0x55555555; // Set Pull up/Pull down resistor configuration for Port E.
	
	//USR button interrupt
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //Enable system configuration controller
	EXTI->IMR |= EXTI_IMR_MR0; //Remove the mask to enable the interrupt. Unmask EXTI0. Set the mask register bit.
	EXTI->RTSR |= EXTI_RTSR_TR0; // EXTI0 to generate interrupt from rising edge
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // Confifgure multiplexing options
	
	
}


//Enable ADC
void enable_ADC(void) 
{
	ADC1->CR &= 0xCFFFFFFF; //This resets the ADC
	ADC1->CR |= 0x10000000; //Enables the ADC
	
//code to count 100 cycles
	int j = 0;
while(j<100)
	{
	j++;
	}
//This code calibrates the ADC
	ADC1->CR &= 0xBFFFFFFF; //Uses a single-ended calibration
	ADC1->CR |= 0x80000000; // This line starts the calibration

//This enables clock connections to ADC Peripheral
	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV2;
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	ADC1_2_COMMON->CCR |= 0x00010000;

// This selects the input pin & Configure GPIO Port. These lines set the mode to analogue. 
	GPIOA->MODER &= 0xFFFFFFFE; 
	GPIOA->MODER |= 0x00000002; 

//This configures the ADC;
	ADC1->CFGR |= 0x00000010; // Sets an 8 bit resolution.
	ADC1->CFGR &= 0xFFFFFFF7; 
	ADC1->CFGR &= 0xFFFFFFDF; // Right allignment
	ADC1->CFGR &= 0xFFFFDFFF; // Sets operation to be non continuous

//This configures the multiplexing options
	ADC1->SQR1 |= 0x00000040;
	ADC1->SQR1 |= 0x00000001; //Sequence length of 1
	ADC1->SMPR1 |= 0x00000018;
	ADC1->SMPR1 &= 0xFFFFFFDF; 

//Enable ADC
	ADC1->CR |= 0x00000001;

//Wait for ADRDY Flag to go high

	while (ADC1->ISR &= 0xFFFFFFFE)
	{
	}
}

//Enable DAC

void enableDac(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //Enable GPIOA pins for DAC output
	RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;
	GPIOA->MODER |= 0x0200; //PA.4 to analogue mode
	DAC1->CR |= DAC_CR_BOFF1; //Disable the ‘buffer’ function in the DAC control register
	DAC1->CR |= DAC_CR_EN1; //Enable DAC peripheral
}

//Configure DAC output
void configure_DAC_out(void)
{
	DAC1->DHR12R1 = InterruptPercentage;
}

//Programme interrupt
void EXTI0_IRQHandler(void) 
{
	if (EXTI->PR & EXTI_PR_PR0) // check source
	{
	EXTI->PR |= EXTI_PR_PR0; // clear flag*
	InterruptPercentage = InterruptPercentage + 333.3; //Increment duty cycle by 33%
	TIM1->CCR1 = InterruptPercentage; //Write percentage of duty cycle into the CCR register
		
	if(InterruptPercentage == 1000) //Reset the variable to 0 once it exceeds 1098
		{ 
		InterruptPercentage = 0;
		}
	}
}


