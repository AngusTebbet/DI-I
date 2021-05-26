#include "stm32f3xx.h"                  // Device header
#include <stdbool.h>

void delay(int a); // prototype for delay function
void Blink_delay(void); //prototype for the Blinker with delay function
void Blink_interrupt(void);
void enable_LEDs(void);
void enable_DAC(void);
void configure_DAC_output(void);
void enable_ADC(void);
void enable_Timer(void);
void Count_bits(volatile uint32_t count);
int interrupt_counter = 0;
bool reverse = false;

int main(void){

enable_LEDs();
enable_DAC();
enable_ADC();
enable_Timer();


	
	while(1){
		
		configure_DAC_output();
		ADC1->CR |= ADC_CR_ADSTART;
		while(!ADC1->ISR & ADC_ISR_EOC)
		GPIOE->BSRRH = 0xFF00; //Turn off the LEDs from the previous count
		GPIOE->BSRRL = (ADC1->DR * 0x0100); //Turn on the New count LEDs
		
	}
}


void Count_bits(volatile uint32_t count){
				GPIOE->BSRRH = 0xFF00; //Turn off the LEDs from the previous count
				GPIOE->BSRRL = (count * 0x0100); //Turn on the New count LEDs
}
void enable_LEDs(void){
	
	// Enable clock on GPIO port E
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	
	// GPIOE is a structure defined in stm32f303xc.h file
	// Define settings for each output pin using GPIOE structure
	GPIOE->MODER |= 0x55550000; // Set to output mode for PE.15-8
	GPIOE->OTYPER &= ~(0xFF00); // Set Open Drain output type for PE.15-8
	GPIOE->PUPDR &= ~(0xFFFF0000); // Set Pull up/Pull down resistor configuration for Port E
}
void enable_DAC(void){
	
	// Enable clock on GPIO port A
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	// The DAC is connected to the system clock via the APB1 peripheral clock bus
	RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;
	GPIOA->MODER |= 0x0200; //Set analogue output mode for pin PA.4
	DAC1->CR |= DAC_CR_BOFF1; //Disable the ‘buffer’ function in the DAC control register
	DAC1->CR |= DAC_CR_EN1; //Enable DAC peripheral
}
void configure_DAC_output(void){
	
	DAC1->DHR12R1 = interrupt_counter;
	
}
void enable_ADC(void){
	int i = 0;
	
	ADC1->CR &= ~ADC_CR_ADVREGEN; // reset the voltage reguletor
	ADC1->CR |= ADC_CR_ADVREGEN_0; // enable the voltage regulator
	
	while(i < 110){ i++;}  // wait for voltage regulator to be enabled for approx 10 useconds
	
	ADC1->CR &= ~ADC_CR_ADCALDIF;  // set differential mode for calibration
	ADC1->CR |= ADC_CR_ADCAL; // run the calibration
	
	while(ADC1->CR & ADC_CR_ADCAL); // wait for calibration to finish
	
	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV2; // divide the clock freq to make the readings easier
	RCC->AHBENR |= RCC_AHBENR_ADC12EN; // enable clock connection to adc peripheral
	ADC1_2_COMMON->CCR |= 0x00010000;  // do something with pointy thing to the ADC
	
	GPIOA->MODER |= 0x02; // Set analogue output mode for pin PA.0
	
	ADC1->CFGR &= ~ADC_CFGR_CONT; //disable ADC continuous mode
	ADC1->CFGR &= ~ADC_CFGR_ALIGN; // right aligned
	ADC1->CFGR &= ~ADC_CFGR_RES; 
	ADC1->CFGR |= ADC_CFGR_RES_1; // 8 bit resolution
	
	
	ADC1->SQR1 &= ~ADC_SQR1_L; // sequence length of 1
	ADC1->SQR1 |= ADC_SQR1_SQ1_0; // Channel 1
	
	ADC1->SMPR1 |=ADC_SMPR1_SMP1_1 | ADC_SMPR1_SMP1_0; // 7.5 smaple rate
	
	ADC1->CR |= ADC_CR_ADEN; 
	
	while (ADC1->ISR & ADC_ISR_ADRD); // is the ADC ready?
	
	
}
void enable_Timer(void){
	
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;	
	TIM3->PSC = 7999; //prescalor value in Timer ‘3’ 
  TIM3->ARR = 99; //Auto-Reset Register of Timer ‘3’ to make 1 delay between interrupts
	TIM3->DIER |= TIM_DIER_UIE; //Set DIER register to watch out for an ‘Update’ Interrupt Enable (UIE) – or 0x00000001
  NVIC_EnableIRQ(TIM3_IRQn); //Enable Timer ‘3’ interrupt request in NVIC
	TIM3->CR1 |= TIM_CR1_CEN;
}
void delay (int a){
    volatile int i,j;

    for (i=0 ; i < a ; i++)
    {
        j++;
    }

    return;
}

void TIM3_IRQHandler(){
	
		if ((TIM3->SR & TIM_SR_UIF) !=0) // Check interrupt source is from the ‘Update’ interrupt flag
		{
				interrupt_counter++;
				if (interrupt_counter == 256) {
				interrupt_counter = 0;
				}
		}
		
		TIM3->SR &= ~TIM_SR_UIF; // Reset ‘Update’ interrupt flag in the SR register
}