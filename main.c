#include "stm32f3xx.h"   

//These variables handle the button press

enum output{potentiometer, encoder, average};
enum output currentOutput;
enum output nextOutput = potentiometer;

//Throttle position
int Ascending = 1; //sets direction
int maxAmp = 511; //sets the max amplitude of the triangle wave
int throttlePos = 1;
int resolution = 1;

//Encoder/decoder
int prevCharA , prevCharB, curCharA, curCharB; //store current and previous cycles for decoded channel A & B
int encoderOutput = 10;
int encoderCounter = 1;

int main(void)
{
	//Initialise LEDs
	
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;	// Enable clock on GPIO port E
	GPIOE->MODER |= 0x55550000;	//Set E.8-15 (onboard LEDs) to output mode
	GPIOE->OTYPER &= ~(0x0000FF00);//Set output type to push/pull
	GPIOE->PUPDR |= 0x55550000;	//Configure as Pull-up

	// Initialise DAC
	
	RCC->APB1ENR |= RCC_APB1ENR_DAC1EN; // Enable DAC Clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA Clock
	GPIOA->MODER |= 0x00000200; // Set PA.4 to analogue
	DAC1->CR |= DAC_CR_BOFF1;	// Disable DAC buffer
	DAC1->CR |= DAC_CR_EN1; // Enable DAC peripheral
	
	// Initialise Encoder
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable clock on GPIOC
	GPIOC->MODER |= 0x00000005;	// Set output mode
	GPIOC->OTYPER &= ~(0x00000003); // Set to push/pull
	GPIOC->PUPDR |= 0x000000005; // Set as pull up
	
	// Initialise Decoder
	GPIOC->MODER &= ~(0x000000F0); // Set pins to input mode
	
	// Initialise the trianlge wave
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable timer clock
	TIM3->PSC = 799; 
	TIM3->ARR = 19; 
	TIM3->DIER |= TIM_DIER_UIE; // Set DIER register to watch for update
	NVIC_EnableIRQ(TIM3_IRQn); // Enable timer 3 interrupt request in NVIC
	TIM3->CR1 |= TIM_CR1_CEN; // Start timer 3
	
	//Enable external interrupt with PA.0 button
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable clock on system configuration controller
	EXTI->IMR |= EXTI_IMR_MR0; // Unmask EXTI0
	EXTI->IMR |= EXTI_IMR_MR2;
	EXTI->IMR |= EXTI_IMR_MR3;
	
	//External triggered Interrupt service routine on NVIC
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI2_TSC_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);
	
	//PA.0 multiplexer interrupts
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PC;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PC;
	
	// User button rising edge interrupt
	EXTI->RTSR |= EXTI_RTSR_TR0 + EXTI_RTSR_TR2 + EXTI_RTSR_TR3;
	EXTI->FTSR |= EXTI_FTSR_TR2 + EXTI_FTSR_TR3;
	
	while(1)
	{
	}
}
	
	
	
