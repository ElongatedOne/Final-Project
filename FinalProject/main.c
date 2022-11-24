/*
 * FinalProject.c
 *
 * Created: 10/11/2022 12:42:54
 * Author : Adam
 */ 


/*
 *   
 *      Program uses a ATMega328p MCU to query a HC-SR04 ultrasonic module. Then Calculates and displays the distance to a object in cm on the NewHaven display.
 *
 *      The ultrasonic module is triggered every 60 ms by setting pin PC4 (the trigger) high for 10 us. A change in state on pin PC5 (the echo)
 *      causes a interrupt. The interrupt checks if PC5 is high. If it is a timer is started that increments every micro second. Once PC5 triggers a
 *      interrupt again and is low the result is calculated and displayed in centimeters on the NewHaven display. Distance in cm is calculated
 *      by the amount of microseconds PC5 (the echo) is active high divided by 58.
 *
 *      Pin placement of ATMega328p:
 *      Port D[7-0]			NewHavenDisplay DB[7-0]
 *      Pin PC0				NewHavenDisplay RW
 *      Pin PC1				NewHavenDisplay RS
 *      Pin PC2				NewHavenDisplay E
 *      Pin PC3				Debugging LED Active High
 *      Pin PC4				HC-SR04 Trig
 *      Pin PC5				HC-SR04 Echo
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
static volatile int trunk_pulse = 0;
static volatile int i = 0;
static volatile int newread = 0;
static volatile int rising = 1;
static volatile uint8_t cm = 0;
static volatile int power = 1;

/*******************************************INITALIZE PORTS, TIMER, AND INTURRUPTS*******************************************/
void init() {
	//DDRD = 0xFF;							// Port D all output. Display: DB0 - DB7 as PD0 - PD7
	//DDRC = 0xFF;							// Port C all output. PC0: RW		PC1: RS		PC2: E
	//DDRC &= ~(1<<5);						// Set Pin C5 as input to read Echo
	//PORTC |= (1<<5);					// Enable pull up on C5
	//PORTC &= ~(1<<4);						// Init C4 as low (trigger)

	//PRR &= ~(1<<PRTIM1);					// To activate timer1 module
	//TCNT1 = 0;								// Initial timer value
	//TCCR1B |= (1<<CS10);					// Timer without prescaller. Since default clock for atmega328p is 1Mhz period is 1uS
	//TCCR1B |= (1<<ICES1);					// First capture on rising edge

	DDRD |= (1<<6);
	DDRD |= (1<<4);
	DDRD |= (1<<7);
	PORTD |= (1<<3); //enable pull up
	PORTD |= (1<<2); //enable pull up
	PORTD |= (1<<7);
	TCCR0B = 0x00;
	TCCR0A = 0x00;
	OCR0A = 0;
	
	//PCICR = (1<<PCIE1);						// Enable PCINT[14:8] we use pin C5 which is PCINT13
	//PCMSK1 = (1<<PCINT13);
	EIMSK |= (1<<INT1); //enable INT1
	EIMSK |= (1<<INT0); //enable INT0
	//EICRA |= (1<<ISC11);
	EICRA |= (1<<ISC10); //INT1 any logic change	
	EICRA |= (1<<ISC00); //INT0 falling edge trigger				
	sei();									// Enable Global Interrupts
}

void wait1ms(){
	TCNT2 = 0x00;  //reset timer
	OCR2A = 0x60;
	TCCR2A |= (1<<WGM21)|(1<<COM2A1)|(1<<COM2A0); //CTC, compare with OCR1A
	TCCR2B |= (1<<CS20)|(1<<CS22); //prescale clock to 1/1024 * 16MHz ~= 16kHz
	while((TIFR2&(1<<OCF2A))==0)
	{}
	TCCR2B = 0; //stop timer
	TIFR2 = 1<<OCF2A;
}

void delay(int ms){
	for(int i = 0; i < ms; i++){
		wait1ms();
	}
}


int change_duty_cycle(int sensor_read) {
	if (sensor_read > 55) {
		return 1;
	}
	else if (sensor_read > 45) {
		return 70;
	}
	else if (sensor_read > 35) {
		return 100;
	}
	else if (sensor_read > 20) {
		return 130;
	}
	else if (sensor_read > 15) {
		return 150;
	}
	else if (sensor_read > 5) {
		return 200;
	}
	else {
		return 250;
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
}

int main() {
	int setup_done = 0;
	int state = 0;
	//toggleMotorPower();
	init();
	while (power==1) {
		//wait
	}
	TCCR0B = 0x05;
	TCCR0A = 0x83;
	startup_sequence();
 	while (1) {
		 
		
		_delay_ms(400); 						// To allow sufficient time between queries (60ms min)
		PORTD |= (1<<4);						// Set trigger high
		_delay_us(10);							// for 10uS
		PORTD &= ~(1<<4);
			
	}
	
}

void update_value(int value) 
{
	TCCR0B = 0;
	TCNT0 = 0;
	OCR0A = change_duty_cycle(value);
	TCCR0B = 0x05;
}

ISR(INT1_vect)
{
	if(i == 0)
	{
		TCCR1B = 2;
		i = 1;
		//rising = 0;
		//OCR0A = 50;
	}
	else
	{
		TCCR1B = 0;
		uint16_t pulse = TCNT1;
		uint8_t trunk_pulse = pulse;
		cm = (pulse/58);
		//OCR0A = cm;
		//OCR0A = change_duty_cycle(cm)
		update_value(cm);
		TCNT1 = 0;
		i = 0;
		newread = 1;
		//OCR0A = 255;
	}
}
ISR (INT0_vect) {
	power = (power+1)%2;
}



