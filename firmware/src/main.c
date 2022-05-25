/*
 * main.c
 *
 *  Created on: May 6, 2022
 *      Author: AnhThu
 */

// ADC Example
#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/iom128.h>
#include "myLCD.h"

#define F_CPU 7372800UL
// Output Port pin LED_O
#define PORT_LED_O      PORTB
#define DDR_LED_O       DDRB
#define BIT_LED_O       6

// Output Port pin Buzzer
#define PORT_BUZ       PORTB
#define DDR_BUZ        DDRB
#define BIT_BUZ        7

// Output Port pin LED_O
#define PORT_LED_O         PORTB
#define DDR_LED_O          DDRB
#define BIT_LED_O          6

// Output Port Warning Led
#define PORT_WARNING 	PORTE
#define DDR_WARNING		DDRE
#define HOT				1
#define COLD			2
#define COOL			3
// Define baud rate
#define USART0_BAUD         115200ul
#define USART0_UBBR_VALUE   ((F_CPU/(USART0_BAUD<<4))-1)



void Init_IO() {
	// LED & Buzzer
	DDR_LED_O |= (1 << BIT_LED_O);
	DDR_BUZ |= (1 << BIT_BUZ);
	PORT_BUZ |= (1 << BIT_BUZ);
	// LED warning
	DDR_WARNING |= (1<<HOT)|(1<<COLD)|(1<<COOL);

}

void TMR_vInit(void) {
	/* Start timer 1 with clock prescaler CLK/1024 */
	/* Resolution is 139 us */
	/* Maximum time is 9.1 s */
	TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0)
			| (0 << COM1C1) | (0 << COM1C0) | (0 << WGM11) | (0 << WGM10);

	TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12)
			| (1 << CS12) | (0 << CS11) | (1 << CS10);
}

void TMR_vDelay(uint16_t u16DelayMs) {
	// Calculate and set delay
	TCNT1 = (uint16_t) (0x10000 - ((F_CPU / 1024) * u16DelayMs) / 1000);

	// Clear timer overflow flag
	TIFR = (1 << TOV1);

	// Wait until timer overflow flag is set
	while ((TIFR & (1 << TOV1)) == 0) {
		;
	}
}

void ADC_vInit(void) {
	/*
	 Select AVCC as reference with external capacitor at AREF pin
	 ADC1 as the single-ended input channel with 1x gain
	 */
	ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (0 << MUX4)
			| (0 << MUX3) | (0 << MUX2) | (0 << MUX1) | (1 << MUX0);

	/*
	 Enable ADC; Select Prescaler of 128 (clock frequency of 57.6 kHz)
	 */
	ADCSRA = (1 << ADEN) | (0 << ADSC) | (0 << ADFR) | (0 << ADIF) | (0 << ADIE)
			| (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADC_u16GetSample(void) {
	// Start conversion
	ADCSRA |= (1 << ADSC);
	// Wait until conversion is finished... 13 ADC clock cycles
	while (ADCSRA & (1 << ADSC)) {
		;
	}

	// Return sampled value
	return ADC;
}

void USART0_vInit(void)
{
	// Set baud rate
	UBRR0H = (uint8_t)(USART0_UBBR_VALUE>>8);
	UBRR0L = (uint8_t)USART0_UBBR_VALUE;

	// Set frame format to 8 data bits, no parity, 1 stop bit
	UCSR0C = (0<<USBS)|(1<<UCSZ1)|(1<<UCSZ0);

	// Enable receiver and transmitter
	UCSR0B = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);
}

void USART_tx(uint8_t temp){	// Transmit data to NodeMCU
	while(bit_is_clear(UCSR0A,UDRE0)){};

	UDR0 = temp;
}
uint8_t USART_rx()
{
	// Wait until a byte has been received
	while((UCSR0A&(1<<RXC0)) == 0)
	{
		;
	}

	// Return received data
	return UDR0;
}

int aboveThreshold = 35;
int belowThreshold = 20;


int main(void) {
	//uint16_t u16AdcValue;
	int Temperature = 25;
	// Initialise Interrupt
	// Init_Interrupt();
	sei();

	// Initialise IO
	Init_IO();

	// Initialise timer
	TMR_vInit();

	// Initialise ADC
	//ADC_vInit();

	// Initialise LCD
	init_LCD();
	clr_LCD();

	// Initialise USART
	USART0_vInit();
	// Repeat indefinitely
	for (;;) {
		// Retrieve a sample
		//u16AdcValue = ADC_u16GetSample();

		// Calculate voltage
		//Temperature = (int) ((u16AdcValue / 1023) * 5 * 100);

		// Display LCD
		move_LCD(1, 1);
		printf_LCD("Temperature:%d", Temperature);
		move_LCD(2, 1);
		printf_LCD("Threshold:%d-%d", belowThreshold, aboveThreshold);

		// Compare threshold
		if (Temperature > aboveThreshold) {
			// Hot temperature warning
			PORT_WARNING |= (1<<HOT);

			// LED and Buzzer ON/OFF 400ms
			PORT_LED_O |= (1 << BIT_LED_O);
			PORT_BUZ &= ~(1 << BIT_BUZ);   // Wait 400 milisecond
			TMR_vDelay(400);
			PORT_LED_O &= ~(1 << BIT_LED_O);
			PORT_BUZ |= (1 << BIT_BUZ);   // Wait 400 milisecond
			TMR_vDelay(400);
		}
		if(Temperature > belowThreshold && Temperature < aboveThreshold){
				// Cool temperature warning
				PORT_WARNING |= (1<<COOL);
			}
		if (Temperature < belowThreshold) {
			// Cold temperature warning
			PORT_WARNING |= (1<<COLD);
			PORT_LED_O |= (1 << BIT_LED_O);
			PORT_BUZ &= ~(1 << BIT_BUZ);   // Wait 700 milisecond
			TMR_vDelay(700);
			PORT_LED_O &= ~(1 << BIT_LED_O);
			PORT_BUZ |= (1 << BIT_BUZ);   // Wait 700 milisecond
			TMR_vDelay(700);
		}

		// USART transmits current temperature to NodeMCU
		USART_tx(Temperature);
		// Wait 1 second
		TMR_vDelay(1000);

//		USART_tx(belowThreshold);
//		TMR_vDelay(1000);
//		USART_tx(aboveThreshold);
//		TMR_vDelay(1000);
	}
}

// USART recieve Threshold from NodeMCU
//ISR(USART0_RX_vect){
//	uint8_t thr = USART_rx();
//	if(thr){
//		aboveThreshold = thr;
//	}else{
//		belowThreshold = thr;
//	}
//}
