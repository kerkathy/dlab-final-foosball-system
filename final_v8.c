/*
 * At TeraTerm:
 * Set baud rate as 9600
 * area as 'cht'
 * data bit 8
 * stop bit 1
 * connect UART line RX (PA9), TX(PA10), GND
 * PC2: ADC1 (light sensor 1 - score)
 * connect max7219 to PA5-7
 * button is PC13
 * PB0: motor energy output
 * PC3: ADC2 (light sensor 2 - ball elevation)
 * PA8: testing signal
 * PC1: ADC3
 * */

#include "stm32l476xx.h"
#include "cmsis_gcc.h"
#include "core_cm4.h"
#include "core_cmFunc.h"
#include "core_cmInstr.h"
#include "core_cmSimd.h"
#include "system_stm32l4xx.h"
#include <string.h>

#define SET_REG(REG,SELECT,VAL) {((REG)=((REG)&(~(SELECT)))|(VAL));};
#define MSI_freq 4000000

extern void max7219_init();
extern void max7219_send(unsigned char address, unsigned char data);

int score_1_flag = 0;
int score_2_flag = 0;
int score_1 = 0;
int score_2 = 0;
int motor_flag = 0;

void GPIO_Init(void) {
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);

	// PA5-7 for max7219 (output: 01)
	SET_REG(GPIOA->MODER, GPIO_MODER_MODE5, GPIO_MODER_MODE5_0);							// Set PA5 as output mode
	SET_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED5, GPIO_OSPEEDR_OSPEED5_0);		// Set PA5 as medium speed mode
	SET_REG(GPIOA->MODER, GPIO_MODER_MODE6, GPIO_MODER_MODE6_0);							// Set PA6 as output mode
	SET_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED6, GPIO_OSPEEDR_OSPEED6_0);		// Set PA6 as medium speed mode
	SET_REG(GPIOA->MODER, GPIO_MODER_MODE7, GPIO_MODER_MODE7_0);							// Set PA7 as output mode
	SET_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED7, GPIO_OSPEEDR_OSPEED7_0);		// Set PA6 as medium speed mode

	// temp: PA8 for testing
	SET_REG(GPIOA->MODER, GPIO_MODER_MODE8, GPIO_MODER_MODE8_0);							// Set PA8 as output mode
	SET_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED8, GPIO_OSPEEDR_OSPEED8_0);		// Set PA8 as medium speed mode

	// UART:PA9 & PA10
	GPIOA->MODER=(GPIOA->MODER&0xFFF3FFFF)|(0b10<<18);
	GPIOA->OTYPER=(GPIOA->OTYPER&0xFFFFFDFF)|0;
	GPIOA->PUPDR=GPIOA->PUPDR&0xFFF3FFFF;
	GPIOA->OSPEEDR=GPIOA->OSPEEDR&0xFFF3FFFF;

	GPIOA->MODER=(GPIOA->MODER&0xFFCFFFFF)|(0b10<<20);
	GPIOA->OTYPER=(GPIOA->OTYPER&0xFFFFFBFF)|0;
	GPIOA->PUPDR=GPIOA->PUPDR&0xFFCFFFFF;
	GPIOA->OSPEEDR=GPIOA->OSPEEDR&0xFFCFFFFF;

	//turn on LED (CAN'T USE NOW! BC PA5 is now for max7219)
//	RCC->AHB2ENR = RCC->AHB2ENR|0x1;											// enable GPIOA
//	SET_REG(GPIOA->MODER, GPIO_MODER_MODE5, 1U<<10);							// Set PA5 as output mode
//	SET_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED5, GPIO_OSPEEDR_OSPEED5_0);		// Set PA5 as medium speed mode

	// AF7() for pin9&10
	GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0x00000FF0)) | 0x00000770;
	//(0111 0111)000000

	// BUTTON: PC13
	GPIOC->MODER=(GPIOC->MODER&0xF3FFFFFF);
	GPIOC->OTYPER=(GPIOC->OTYPER&0xFFFFDFFF)|0;
	GPIOC->PUPDR=GPIOC->PUPDR&0xF3FFFFFF;
	GPIOC->OSPEEDR=(GPIOC->OSPEEDR&0xF3FFFFFF)|(0b01<<26);

	//PC2: analog mode (read signal)
	GPIOC->MODER |= GPIO_MODER_MODER2_1;
	GPIOC->MODER |= GPIO_MODER_MODER2_0;
	GPIOC->OTYPER  &=~ GPIO_OTYPER_OT2; //PP
	GPIOC->PUPDR  &=~ GPIO_PUPDR_PUPD2_1;
	GPIOC->PUPDR  &=~ GPIO_PUPDR_PUPD2_0;//00 no
	GPIOC->OSPEEDR &=~ GPIO_OSPEEDR_OSPEED2_1;
	GPIOC->OSPEEDR &=~ GPIO_OSPEEDR_OSPEED2_0;//00 low
	GPIOC->ASCR |= GPIO_ASCR_ASC2;	//switch ctrl

	// PC3: analog mode
	GPIOC->MODER |= GPIO_MODER_MODER3_1;
	GPIOC->MODER |= GPIO_MODER_MODER3_0;
	GPIOC->OTYPER  &=~ GPIO_OTYPER_OT3; //PP
	GPIOC->PUPDR  &=~ GPIO_PUPDR_PUPD3_1;
	GPIOC->PUPDR  &=~ GPIO_PUPDR_PUPD3_0;//00 no
	GPIOC->OSPEEDR &=~ GPIO_OSPEEDR_OSPEED3_1;
	GPIOC->OSPEEDR &=~ GPIO_OSPEEDR_OSPEED3_0;//00 low
	GPIOC->ASCR |= GPIO_ASCR_ASC3;	//switch ctrl

	// PC1:
	GPIOC->MODER |= GPIO_MODER_MODER1_1;
	GPIOC->MODER |= GPIO_MODER_MODER1_0;
	GPIOC->OTYPER  &=~ GPIO_OTYPER_OT1; //PP
	GPIOC->PUPDR  &=~ GPIO_PUPDR_PUPD1_1;
	GPIOC->PUPDR  &=~ GPIO_PUPDR_PUPD1_0;//00 no
	GPIOC->OSPEEDR &=~ GPIO_OSPEEDR_OSPEED1_1;
	GPIOC->OSPEEDR &=~ GPIO_OSPEEDR_OSPEED1_0;//00 low
	GPIOC->ASCR |= GPIO_ASCR_ASC1;	//switch ctrl
}


/* For Motor signal at PB0 ---------------------------------------*/

void GPIO_init_AF(){
	//TODO: Initial GPIO pin as alternate function for pwm.
	// PB0 AF2 : TIM3_CH3
	SET_REG(GPIOB->MODER, GPIO_MODER_MODER0, GPIO_MODER_MODER0_1);				// set PB0 as AF mode (01)
	GPIOB->AFR[0] = 0x2;														// AF2
}

void Timer_init(){
	//TODO: Initialize timer
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;										// enable timer3
}

void PWM_channel_init(){
	//TODO: Initialize timer PWM channel
	TIM3->CR1 |= TIM_CR1_ARPE;															// auto reload preload enable
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;				// CH3,4: CCMR2 (we are using CH2)
	// OC3PE: output compare 3 preload enable
	// OC3M: output compare 3 mode
	// if OCxM[2:0] is set 110, that means pwm mode 1
	int duration;
	int pwm_dur;
	duration = 4000000/1000;		// default 1k Hz
	pwm_dur = duration * 50 / 100;

	TIM3->PSC = 0;
	TIM3->ARR = duration;

	TIM3->CCER |= TIM_CCER_CC3E;														// compare 3 output enable
}

void set_duty(int freq, int pwm_percent)
{
	int duration;
	int pwm_dur;
	duration = 4000000/freq;
	pwm_dur = duration * pwm_percent / 100;

	TIM3->PSC = 0;
	TIM3->ARR = duration;
	TIM3->CCR3 = pwm_dur;																// decide the duty cycle (to compare with CNT)
	TIM3->EGR |= TIM_EGR_UG;															// reinitialize (generate update event)
	TIM3->CR1 |= TIM_CR1_CEN;															// enable counter
}

/* --------------------------------------- For Motor */

//PA9 & PA10 => USART1
void USART1_Init(void) {
	/* Enable clock for USART1 */
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	// CR1
	SET_REG(USART1->CR1, USART_CR1_M, USART_CR1_M0);//00: 1 Start bit, 8+1 data bits, n stop bits

	SET_REG(USART1->CR1, USART_CR1_PS, 0);//0: Even parity
	//
	SET_REG(USART1->CR1, USART_CR1_PCE, USART_CR1_PCE);//1: the computed parity is inserted at the MSB position(9th bit if M=1)

	SET_REG(USART1->CR1, USART_CR1_RE, 0);//0: Receiver is disabled
	SET_REG(USART1->CR1, USART_CR1_OVER8, 0);//0: Oversampling by 16; 1: Oversampling by 8

// CR2
	SET_REG(USART1->CR2, USART_CR2_STOP, 0);//00: 1 stop bit

// CR3
	//MODIFY_REG(USART1->CR3, (USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT), 0x0); // none flow control
	SET_REG(USART1->CR3, USART_CR3_RTSE, 0);//0
	SET_REG(USART1->CR3, USART_CR3_CTSE, 0);//0
	SET_REG(USART1->CR3, USART_CR3_ONEBIT, 0);//0

// BRR
	//MODIFY_REG(USART1->BRR, 0xFF, 4000000L/115200);
	SET_REG(USART1->BRR, 0xFF, 4000000/9600);//0
	//clk rate = 4MHz (default MSI)

/*
In asynchronous mode, the following bits must be kept cleared:
- LINEN and CLKEN bits in the USART_CR2 register,
- SCEN, HDSEL and IREN bits in the USART_CR3 register.
*/
	USART1->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
	USART1->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);

// Finally, enable UART
	USART1->CR1 |= (USART_CR1_UE);
}

void EXTI_config()
{
	SET_REG(RCC->APB2ENR,RCC_APB2ENR_SYSCFGEN,RCC_APB2ENR_SYSCFGEN);
	//connect EXTI13(in EXTICR4) with pc13
	SET_REG(SYSCFG->EXTICR[3],SYSCFG_EXTICR4_EXTI13,SYSCFG_EXTICR4_EXTI13_PC);
	//Interrupt mask(in IMR1)
	SET_REG(EXTI->IMR1,EXTI_IMR1_IM13,EXTI_IMR1_IM13);
	//trigger: rise edge(in FTSR1)
	SET_REG(EXTI->FTSR1,EXTI_FTSR1_FT13,EXTI_FTSR1_FT13);
}

void NVIC_config()
{
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_EnableIRQ(ADC3_IRQn);
	NVIC_EnableIRQ(ADC1_2_IRQn);
}

char* change_to_ASCII(int value) {
	/* Convert int to ASCII char in order to display */
	// temp
	if(value<1) return "0 to 1";
	else if (value<0) return "negative";
	else {
		static char s[12];
		int digit = 0;
		int temp;
		temp = value;
		while(temp > 0){
			temp /= 10;
			digit = digit + 1;
		}
		temp = value;
		int i;
		s[digit] = 32;		// space
		for(i=digit-1;i>=0;i--){
			s[i] = temp % 10 + 48;		// ASCII 48: 0
			temp /= 10;
		}
		return s;
	}
}

/* Show data on 7-seg via max7219_send */
int display (int data) {
	// from LSB to MSB
	int bits = 0;
	do {
		unsigned int num = data % 10;
		bits++;
		max7219_send( bits, num );
		data /= 10;
	} while(data != 0);

	for (int i = bits+1; i<=8; i++) {
		// decode blank
		max7219_send( i, 0xF );
	}
	return 0;
}


int UART_Transmit(uint8_t *arr, uint32_t size) {
//TODO: Send str to UART and return how many bytes are successfully transmitted.
	SET_REG(USART1->CR1, USART_CR1_TE, USART_CR1_TE);//1: Transmitter is enabled
	int i;
	while ( ( USART1->ISR & USART_ISR_TXE )== 0 );
	for (i = 0; i < size; i++) {
		//The TE bit should not be reset(0) during transmission of data.

		USART1->TDR = arr[i]; //Transmit data register
		while ( ( USART1->ISR & USART_ISR_TXE )== 0 );
		//wait until TXE=1.
		//This indicates that the transmission of the last frame is complete.
	}
	//while (!IS_UART_TRANS_DONE);//lock
	while ( ( USART1->ISR & USART_ISR_TC )== 0 );

	return i;
}

int ADC1ConvertedValue;
int ADC1ConvertedVoltage;

//automatic handler
//void EXTI15_10_IRQHandler(void)
//{
//	if((EXTI->PR1 & EXTI_PR1_PIF13) == EXTI_PR1_PIF13)
//	{
//		EXTI->PR1 = EXTI_PR1_PIF13;
//
//		while(!ADC1->ISR & ADC_ISR_ADRDY );
//		ADC1ConvertedValue = ADC1->DR;
//		//clean flag
//		ADC1->ISR |= ADC_ISR_EOC;	// clear
//		ADC1ConvertedVoltage = ADC1ConvertedValue;
//		char *s;
//		s = change_to_ASCII(ADC1ConvertedVoltage);
//		int d = strlen(s);
//		UART_Transmit( s, d );
//
//		if(ADC1ConvertedVoltage > 4000) {
//			GPIOA->BSRR = (1 << 5);
//		}
//		else GPIOA->BRR = (1 << 5);
//
//		// delay detection
//		int a=0;
//		while ( a<500000 ) a++;
//	}
//	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);//clear pending
//}

void ADC1_2_IRQHandler(void){
	// ADC 1 watchdog 1
	if((ADC1->ISR & ADC_ISR_AWD1) == ADC_ISR_AWD1){
		if(score_1_flag == 0){
			score_1_flag = 1;
		}
		ADC1->ISR |= ADC_ISR_AWD1;			// clear watchdog 1 event (not sure)
	}

	// ADC 2 watchdog 1
	else if((ADC2->ISR & ADC_ISR_AWD1) == ADC_ISR_AWD1){
		if(motor_flag == 0){
			motor_flag = 1;
		}
		ADC2->ISR |= ADC_ISR_AWD1;			// clear watchdog 1 event (not sure)
	}
	NVIC_ClearPendingIRQ(ADC1_2_IRQn);		// clear pending
}

void ADC3_IRQHandler(void){
	// ADC 3 watchdog 1
	if((ADC3->ISR & ADC_ISR_AWD1) == ADC_ISR_AWD1){
		if(score_2_flag == 0){
			score_2_flag = 1;
		}
		ADC3->ISR |= ADC_ISR_AWD1;			// clear watchdog 1 event (not sure)
	}
	NVIC_ClearPendingIRQ(ADC3_IRQn);		// clear pending
}

void ADC_1_Init(){
	//ADC1: one of ADC's instances
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN; //
	//ADC peripheral clock
	RCC->CCIPR |= (RCC_CCIPR_ADCSEL_1|RCC_CCIPR_ADCSEL_0); //LSE clock selected as USART1 clock

	//PC2 is the output of ADC"1" regular channel"3" configuration
	//(See datasheet Production data Table 16. STM32L476xx pin definitions)
	ADC1->SQR1 &= ~ADC_SQR1_L; //0000: Regular channel sequence length = 1
	// 1 conversion => sq1
	ADC1->SQR1 &=~ ADC_SQR1_SQ1_3;
	ADC1->SQR1 &=~ ADC_SQR1_SQ1_2;
	ADC1->SQR1 |= ADC_SQR1_SQ1_1;
	ADC1->SQR1 |= ADC_SQR1_SQ1_0;	//0011 = channel"3"

	ADC1->SMPR1 |= (ADC_SMPR1_SMP3_1 |ADC_SMPR1_SMP3_0);	//0x03:sampling time 7.5 clock cycle

	/*ADC1 calibration, Ensure DEEPPWD=0, ADVREGEN=1, ADEN = 0 */
	ADC1->CR &=~ ADC_CR_ADEN;	// ADC enable
	ADC1->CR &=~ ADC_CR_DEEPPWD;	// Deep-power-down enable
	ADC1->CR |= ADC_CR_ADVREGEN;	// ADC voltage regulator enable

	/* The software must wait for the startup time of the ADC voltage regulator
	  (TADCVREG_STUP) before launching a calibration or enabling the ADC.*/
	//TADCVREG_STUP
	int a;
	a = 0;
	while(a < 100)a++;
//	while(a < 1000)a++;

	/*ADC1 configuration*/
	ADC1->CFGR |= ADC_CFGR_CONT; //continuous mode (won't stop after ADSTART)

	ADC1->CFGR &=~ ADC_CFGR_RES_0;
	ADC1->CFGR |= ADC_CFGR_RES_1;	//[live coding] 10: 8-bit resolution

	// ADC1->CFGR &=~ ADC_CFGR_RES_1;
	// ADC1->CFGR &=~ ADC_CFGR_RES_0;	//00: 12-bit resolution

	ADC1->CFGR |= ADC_CFGR_ALIGN; //Left alignment
//	ADC1->CFGR &= ~ADC_CFGR_ALIGN; //Right alignment

	ADC1->ISR |= ADC_ISR_ADRDY;
	ADC1->CR |= ADC_CR_ADEN;
	while(!ADC1->ISR & ADC_ISR_ADRDY );
	ADC1->CR |= ADC_CR_ADSTART;

	/*
	DEEPPWD=0.
	It is mandatory to enable the ADC internal voltage regulator by
	setting the bit ADVREGEN=1
	*/
}

void ADC_2_Init(){
			//ADC1: one of ADC's instances
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
		RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN; //
		//ADC peripheral clock
		RCC->CCIPR |= (RCC_CCIPR_ADCSEL_1|RCC_CCIPR_ADCSEL_0); //as system clock


		ADC2->CFGR |= ADC_CFGR_OVRMOD;//1:Overrun mode

		//PC3 is the output of ADC"2" regular channel"4" configuration
		//(See Production data Table 16. STM32L476xx pin definitions)
		ADC2->SQR1 &= ~ADC_SQR1_L; //0000: Regular channel sequence length = 1
		// 1 conversion => sq1
		ADC2->SQR1 &=~ ADC_SQR1_SQ1_4;
		ADC2->SQR1 &=~ ADC_SQR1_SQ1_3;
		ADC2->SQR1 |= ADC_SQR1_SQ1_2;
		ADC2->SQR1 &=~ ADC_SQR1_SQ1_1;
		ADC2->SQR1 &=~ ADC_SQR1_SQ1_0;//0100 = channel"4"

		ADC2->SMPR1 |= (ADC_SMPR1_SMP4_1 |ADC_SMPR1_SMP4_0);
		// channel"3"
		//0x03:sampling time 7.5 clock cycle


		/*ADC2 calibration, Ensure DEEPPWD=0, ADVREGEN=1, ADEN = 0 */
		ADC2->CR &=~ ADC_CR_ADEN;
		ADC2->CR &=~ ADC_CR_DEEPPWD;
		ADC2->CR |= ADC_CR_ADVREGEN;
		/* The software must wait for the startup time of the ADC voltage regulator
		  (TADCVREG_STUP) before launching a calibration or enabling the ADC.*/
		//TADCVREG_STUP
		int a;
		a = 0;
		while(a <100)a++;

	 	// calibration
		ADC2->CR &= ~ADC_CR_ADCALDIF; //single-ended inputs mode
		ADC2->CR |= ADC_CR_ADCAL; //start ADC calibration
		//read in 1 means that a calibration in progress
		while(ADC2->CR & ADC_CR_ADCAL);//wait until calibration done

		/*ADC2 configuration*/
		ADC2->CFGR |= ADC_CFGR_CONT; //continuous mode (won't stop)
		ADC2->CFGR &= ~ADC_CFGR_RES_0; //10:8-bit data resolution
		ADC2->CFGR |= ADC_CFGR_RES_1;
		//ADC2->CFGR &= ~ADC_CFGR_ALIGN; //Right alignment4

		ADC2->CFGR |= ADC_CFGR_ALIGN;

		ADC2->ISR |= ADC_ISR_ADRDY;
		ADC2->CR |= ADC_CR_ADEN;
		while(!ADC2->ISR & ADC_ISR_ADRDY );
		ADC2->CR |= ADC_CR_ADSTART;

	/*
	DEEPPWD=0.
	It is mandatory to enable the ADC internal voltage regulator by
	setting the bit ADVREGEN=1
	*/
}

void ADC_3_Init(){
		//ADC3: one of ADC's instances
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
		RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
		//ADC peripheral clock
		RCC->CCIPR |= (RCC_CCIPR_ADCSEL_1|RCC_CCIPR_ADCSEL_0); //as system clock

		ADC3->CFGR |= ADC_CFGR_OVRMOD;//1:Overrun mode

		//PC1 is the output of ADC"3" regular channel "2" configuration
		//(See Production data Table 16. STM32L476xx pin definitions)
		ADC3->SQR1 &= ~ADC_SQR1_L; //0000: Regular channel sequence length = 1
		// 1 conversion => sq1

		ADC3->SQR1 &=~ ADC_SQR1_SQ1_4;
		ADC3->SQR1 &=~ ADC_SQR1_SQ1_3;
		ADC3->SQR1 &=~ ADC_SQR1_SQ1_2;
		ADC3->SQR1 |= ADC_SQR1_SQ1_1;
		ADC3->SQR1 &=~ ADC_SQR1_SQ1_0;//00010 = channel"2"

		ADC3->SMPR1 |= (ADC_SMPR1_SMP2_1 |ADC_SMPR1_SMP2_0);
		//0x03:sampling time 7.5 clock cycle


		/*ADC3 calibration, Ensure DEEPPWD=0, ADVREGEN=1, ADEN = 0 */
		ADC3->CR &=~ ADC_CR_ADEN;
		ADC3->CR &=~ ADC_CR_DEEPPWD;
		ADC3->CR |= ADC_CR_ADVREGEN;
		/* The software must wait for the startup time of the ADC voltage regulator
		  (TADCVREG_STUP) before launching a calibration or enabling the ADC.*/
		//TADCVREG_STUP
		int a;
		a = 0;
		while(a <100)a++;
//		while(a <1000)a++;

	 	// calibration
		ADC3->CR &= ~ADC_CR_ADCALDIF; //single-ended inputs mode
		ADC3->CR |= ADC_CR_ADCAL; //start ADC calibration
		//read in 1 means that a calibration in progress
		while(ADC3->CR & ADC_CR_ADCAL);//wait until calibration done

		/*ADC3 configuration*/
		ADC3->CFGR |= ADC_CFGR_CONT; //continuous mode (won't stop)
		ADC3->CFGR &= ~ADC_CFGR_RES_0; //10:8-bit data resolution
		ADC3->CFGR |= ADC_CFGR_RES_1;
		//ADC3->CFGR &= ~ADC_CFGR_ALIGN; //Right alignment4

		ADC3->CFGR |= ADC_CFGR_ALIGN;

		ADC3->ISR |= ADC_ISR_ADRDY;
		ADC3->CR |= ADC_CR_ADEN;
		while(!ADC3->ISR & ADC_ISR_ADRDY );
		ADC3->CR |= ADC_CR_ADSTART;

	/*
	DEEPPWD=0.
	It is mandatory to enable the ADC internal voltage regulator by
	setting the bit ADVREGEN=1
	*/
}

void ADC_1_WatchDog_1_Init(){
	// single regular channel
	ADC1->CFGR |= ADC_CFGR_AWD1SGL | ADC_CFGR_AWD1EN;
	ADC1->CFGR &= ~ADC_CFGR_JAWD1EN;

	// channel 3
	SET_REG(ADC1->CFGR, ADC_CFGR_AWD1CH, ADC_CFGR_AWD1CH_0 | ADC_CFGR_AWD1CH_1);	//00011

	// enable watchdog 1
	ADC1->IER |= ADC_IER_AWD1IE;

	// set high and low threshold
	SET_REG(ADC1->TR1, ADC_TR1_HT1, 0xff0<<16);		// not sure
	SET_REG(ADC1->TR1, ADC_TR1_LT1, 247<<4);
}

void ADC_2_WatchDog_1_Init(){
	// single regular channel
	ADC2->CFGR |= ADC_CFGR_AWD1SGL | ADC_CFGR_AWD1EN;
	ADC2->CFGR &= ~ADC_CFGR_JAWD1EN;

	// channel 4
	SET_REG(ADC2->CFGR, ADC_CFGR_AWD1CH, ADC_CFGR_AWD1CH_2);	//00100

	// enable watchdog 1
	ADC2->IER |= ADC_IER_AWD1IE;

	// set high and low threshold
	SET_REG(ADC2->TR1, ADC_TR1_HT1, 0xff0<<16);		// not sure
	SET_REG(ADC2->TR1, ADC_TR1_LT1, 247<<4);
}

void ADC_3_WatchDog_1_Init(){
	// single regular channel
	ADC3->CFGR |= ADC_CFGR_AWD1SGL | ADC_CFGR_AWD1EN;
	ADC3->CFGR &= ~ADC_CFGR_JAWD1EN;

	// channel 2
	SET_REG(ADC3->CFGR, ADC_CFGR_AWD1CH, ADC_CFGR_AWD1CH_1);	//00010

	// enable watchdog 1
	ADC3->IER |= ADC_IER_AWD1IE;

	// set high and low threshold
	SET_REG(ADC3->TR1, ADC_TR1_HT1, 0xff0<<16);		// not sure
	SET_REG(ADC3->TR1, ADC_TR1_LT1, 247<<4);
}

void clear_led() {
	for (int i = 1; i<=8; i++) {
		// decode blank
		max7219_send( i, 0xF );
	}
}


/* Timer ------------------------------------------------------------------------------------------- */

int n_us;

void SysTick_config (uint32_t ticks) {
	// set reload register
	SysTick->LOAD = ticks - 1;
	// Load the SysTick Counter Value
	SysTick->VAL = 0; 															// reset as 0
	// CLKSOURCE processor clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
	return;
}

// Interrupt every 100us
void SysTick_Handler () {
	if (n_us != 0) n_us-- ;
	return;
}

void Delay_ms(int n_ms) {
	n_us = 10 * n_ms;
	//	n_us = 100 * n_ms;
	while ( n_us > 0 );															// wait here for 100*n_ms* 100 us (ex. if n_ms = 3000, wait here for 3s)
	return;
}

/* ------------------------------------------------------------------------------------------- Timer */



int main(void){
	GPIO_Init();
	EXTI_config();	// to detect btn press
	NVIC_config();
	USART1_Init();
	max7219_init();
	clear_led();
	display(0);

	GPIO_init_AF();
	Timer_init();
	PWM_channel_init();

	ADC_1_Init();
	ADC_1_WatchDog_1_Init();
	ADC_2_Init();
	ADC_2_WatchDog_1_Init();
	ADC_3_Init();
	ADC_3_WatchDog_1_Init();

	SysTick_config(MSI_freq * 0.0001);	// 100us
//	SysTick_config(MSI_freq * 0.00001);	// 10us


	int pwm = 15;
	set_duty(333, 0);

	while(1){
		//detect
		//resistance = 2.2k
		//resolution = 2^12 = 4096
		if (score_1_flag) {
			score_1++;
			display(score_1);

//			int a = 500;
////			int a = 5000;
//			while(a>0) a--;

			Delay_ms(1000);					// wait for rsps

			
//			char *s = "score_1: ";
//			int d = strlen(s);
//			UART_Transmit( s, d );
//
//			char *s2;
//			s2 = change_to_ASCII(score_1);
//			int d2 = strlen(s2);
//			UART_Transmit( s2, d2 );

			score_1_flag = 0;
		}

		else if (score_2_flag) {
			score_2++;
			display(score_2);

			// temp
//			GPIOA->BSRR = (1 << 8);			// testing
//			Delay_ms(score_2 * 1000);					// wait for rsps
//			GPIOA->BRR = (1 << 8);			// testing
			// temp

//			int a = 500;
//			int a = 5000;
//			while(a>0) a--;
			Delay_ms(1000);					// wait for rsps

			
			
//			char *s = "score_2: ";
//			int d = strlen(s);
//			UART_Transmit( s, d );
//
//			char *s2;
//			s2 = change_to_ASCII(score_2);
//			int d2 = strlen(s2);
//			UART_Transmit( s2, d2 );

			score_2_flag = 0;
		}

		else if (motor_flag) {
			motor_flag = 0;

//			// temp
//			GPIOA->BSRR = (1 << 8);			// testing
//			Delay_ms(5000);					// wait for rsps
//			GPIOA->BRR = (1 << 8);			// testing

			set_duty(333, pwm);
			Delay_ms(6000);					// wait for rsps
			set_duty(333, 0);

//			int freq = 333, pwm_percent = pwm;
//			int duration = 4000000/freq;
//			int pwm_dur;
//			pwm_dur = duration * pwm_percent / 100;
//
//			TIM3->PSC = 0;
//			TIM3->ARR = duration;
//			TIM3->CCR3 = pwm_dur;																// decide the duty cycle (to compare with CNT)
//			TIM3->EGR |= TIM_EGR_UG;															// reinitialize (generate update event)
//			TIM3->CR1 |= TIM_CR1_CEN;															// enable counter
//
//			Delay_ms(3000);					// wait for rsps
//
//			freq = 333, pwm_percent = 0;
//			duration = 4000000/freq;
//			pwm_dur = duration * pwm_percent / 100;
//
//			TIM3->PSC = 0;
//			TIM3->ARR = duration;
//			TIM3->CCR3 = pwm_dur;																// decide the duty cycle (to compare with CNT)
//			TIM3->EGR |= TIM_EGR_UG;															// reinitialize (generate update event)
//			TIM3->CR1 |= TIM_CR1_CEN;															// enable counter
//
		}
	}
}
