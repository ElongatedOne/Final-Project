/*
 * FinalProject.c
 *
 * Created: 10/11/2022 12:42:54
 * Author : Adam Grecner
 * H00323152
 */ 


#define F_CPU 16000000UL
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/common.h>


static volatile int pulse = 0;
static volatile int i = 0;
static volatile int newread = 0;
static volatile int rising = 1;
static volatile uint8_t cm = 0;
static volatile int power = 0;
static volatile int setup_done = 0;

/*******************************************INITALIZE PORTS, TIMER, AND INTURRUPTS*******************************************/
void init() {

	DDRC |= (1<<0);
	DDRC |= (1<<1);
	DDRC |= (1<<2);

	DDRD |= (1<<6);
	DDRD |= (1<<4);
	DDRD |= (1<<7);
	PORTD |= (1<<3); //enable pull up
	PORTD |= (1<<2); //enable pull up
	PORTD |= (1<<7);
	TCCR0B = 0x05;
	TCCR0A = 0x83;
	OCR0A = 1;
	
	DDRB |= 0xFF;
	
	EIMSK |= (1<<INT1); //enable INT1
	EIMSK |= (1<<INT0); //enable INT0
	EICRA |= (1<<ISC10); //INT1 any logic change	
	EICRA |= (1<<ISC01); //INT0 falling edge trigger				
	sei();									// Enable Global Interrupts
}

void USART_init(void){
	UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8); //set baud rate
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); //enable transmit
	UCSR0C = (3<<UCSZ00); //set 8-bit (default)
}

void waiting_USART() {
	unsigned char string[100] = "Press button to start ranging ";
	
	i = 0;
	while(string[i] != 0) 
	{
		while (!( UCSR0A & (1<<UDRE0))); //Wait for empty transmit buffer
		UDR0 = string[i];            // Put data into buffer, sends the data
		i++;                             // increment counter           
	}
}

void starting_USART() {
	unsigned char string[100] = "Starting Ranging ";
	
	i = 0;
	while(string[i] != 0) 
	{
		while (!( UCSR0A & (1<<UDRE0))); 
		UDR0 = string[i];           
		i++;                                      
	}
}

void shutdown_USART() {
	unsigned char string[100] = "Stopping Ranging ";
	
	i = 0;
	while(string[i] != 0) 
	{
		while (!( UCSR0A & (1<<UDRE0))); 
		UDR0 = string[i];
		i++;                             
	}
}


void initADC() {
	ADMUX |= (1 << REFS0); //reference voltage on AVCC, and use ADC0
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //ADC clock prescaler / 128
	ADCSRA |= (1<<ADEN);
	DDRB = 0xFF;
}

void doADC() {
	   //read from the potentiometer to determine what mode the user has selected

	   uint16_t potVal;
	   uint16_t threshold = 5;

	   ADMUX = (ADMUX & 0xF0) | (0 & 0x0F);
	   ADCSRA |= (1 << ADSC); //start ADC conversion
	   while((ADCSRA & (1 << ADSC))) //wait until ADSC bit is clear, i.e., ADC conversion is done
	   {}
	   //read ADC value in
	   uint8_t theLowADC = ADCL;
	   potVal = ADCH << 8 | theLowADC;
	   if (potVal > threshold) {
		   PORTB = 0x00;
	   }
	   else {
		   PORTB = 0xFF;
	   }
}

void wait_1ms(){
	TCNT2 = 0x00;  //reset timer
	OCR2A = 0x60;
	TCCR2A |= (1<<WGM21)|(1<<COM2A1)|(1<<COM2A0); //CTC, compare with OCR2A
	TCCR2B |= (1<<CS20)|(1<<CS22); //prescale clock to 1/1024 * 16MHz ~= 16kHz
	while((TIFR2&(1<<OCF2A))==0)
	{}
	TCCR2B = 0; //stop timer
	TIFR2 = 1<<OCF2A;
	return;
}

void delay(int ms){
	for(int i = 0; i < ms; i++){
		wait_1ms();
	}
	return;
}

void toggleLight() {
	//turn on the LEDs
	PORTB |= (1<<0);
	PORTB |= (1<<1);
	PORTB |= (1<<2);
	return;
}

int alternative_duty_change(int sensor_read) {
	if (sensor_read<70) {
		return 250-sensor_read;
	}
	else {
		return 1;
	}
}

int change_duty_cycle(int sensor_read) {
	//change the duty cycle based on the distance
	if (sensor_read > 60) {
		return 1;
	}
	else if (sensor_read > 65) {
		return 150;
	}
	else if (sensor_read > 55) {
		return 160;
	}
	else if (sensor_read > 45) {
		return 190;
	}
	else if (sensor_read > 30) {
		return 210;
	}
	else if (sensor_read > 25) {
		return 230;
	}
	else if (sensor_read > 15) {
		return 250;
	}
	else {
		return 1;
	}
}

void setOCRA(int value)
{
	TCCR0B = 0;
	TCNT0 = 0;
	OCR0A = value;
	TCCR0B = 0x05;
}

void startup_sequence() {
	for (int i = 0; i<2; i++) {
		setOCRA(200);
		delay(100);
		setOCRA(150);
		delay(100);
		setOCRA(50);
		delay(200);
		setOCRA(250);
		delay(1000);
		setOCRA(0);
		delay(500);
	}
	return;
}

int main() {
	
	init();
	USART_init();
	while (power==0) {
		waiting_USART();
		delay(200);
	}
	TCCR0B = 0x05;
	TCCR0A = 0x83;
	startup_sequence();
	initADC();
	starting_USART();
 	while (power==1) {
		 setup_done = 1;
		 if (power==0) {
			 TCCR0B = 0x00;
			 TCCR0A = 0x83;
			 PORTD &= ~(1<<6);
			 
		 }
		
		doADC();
		
		_delay_ms(200); 						// To allow sufficient time between queries (60ms min)
		PORTD |= (1<<4);						// Set trigger high
		_delay_us(10);							// for 10uS
		PORTD &= ~(1<<4);
			
	}
	shutdown_USART();
	setup_done = 0;
	power = 0;
	main();
	
}

void update_value(int value) 
{
	TCCR0B = 0;
	TCNT0 = 0;
	OCR0A = change_duty_cycle(value);
	TCCR0B = 0x05;
	TCCR0A = 0x83;
	return;
}

ISR(INT1_vect)
{
	if(i == 0)
	{
		TCCR1B = 2;
		i = 1;
		
	}
	else
	{
		TCCR1B = 0;
		uint16_t pulse = TCNT1;
		cm = (pulse/58);
		update_value(cm);
		TCNT1 = 0;
		i = 0;
		
	}
}
ISR (INT0_vect) {
	if (setup_done==0) {
		power = 1;
	}
	else {
		power = 0;
	}
	
}



