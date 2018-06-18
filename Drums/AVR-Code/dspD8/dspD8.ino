// dsp-D8 Drum Chip (c) DSP Synthesizers 2015
// Free for non commercial use

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "drum_samples.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 

// Standard Arduino Pins
/*
#define digitalPinToPortReg(P) \
	(((P) >= 0 && (P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define digitalPinToDDRReg(P) \
	(((P) >= 0 && (P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define digitalPinToPINReg(P) \
	(((P) >= 0 && (P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define digitalPinToBit(P) \
	(((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P) - 8 : (P) - 14))


	#define digitalReadFast(P) bitRead(*digitalPinToPINReg(P), digitalPinToBit(P))

#define digitalWriteFast(P, V) bitWrite(*digitalPinToPortReg(P), digitalPinToBit(P), (V))
*/

const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);


#define dataPin 3
#define clockPin 4


#define beatPrescaler 15
#define beatMidpoint 8

//--------- Ringbuf parameters ----------
uint8_t Ringbuffer[256];
uint8_t RingWrite=0;
uint8_t RingRead=0;
volatile uint8_t RingCount=0;
uint8_t beatClock;
volatile uint8_t beatClockCounter = 0;

//-----------------------------------------
ISR(TIMER1_COMPA_vect) {

	//-------------------  Ringbuffer handler -------------------------
	if (RingCount) {                            //If entry in FIFO..
		OCR2A = Ringbuffer[(RingRead++)];       //Output LSB of 16-bit DAC
		RingCount--;
	}
	//-----------------------------------------------------------------

}

ISR(TIMER0_COMPA_vect) {
	beatClockCounter++;
	if (beatClockCounter == beatPrescaler) {
		beatClockCounter = 0;
		beatClock = 0xff;
	}
}



void setup() 
{
	OSCCAL=0xFF;

	//Drumtrigger inputs
	
	pinMode(9,INPUT_PULLUP);
	pinMode(10,INPUT_PULLUP);
	pinMode(12,INPUT_PULLUP);
	pinMode(8,INPUT_PULLUP);
	pinMode(A4, OUTPUT);
	pinMode(A5, OUTPUT);
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);

	// LED beat-indicator
	pinMode(dataPin, OUTPUT);
	pinMode(clockPin, OUTPUT);
	ledAnimation();
	shiftOut(dataPin, clockPin, MSBFIRST, 0x00);	// Clear LEDS
	

	//8-bit PWM DAC pin
	pinMode(11,OUTPUT);

	// Set up Timer 1 to send a sample every interrupt.
	cli();
	// Set CTC mode
	// Have to set OCR1A *after*, otherwise it gets reset to 0!
	TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
	TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));    
	// No prescaler
	TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
	// Set the compare register (OCR1A).
	// OCR1A is a 16-bit register, so we have to do this with
	// interrupts disabled to be safe.
	//OCR1A = F_CPU / SAMPLE_RATE; 
	// Enable interrupt when TCNT1 == OCR1A
	TIMSK1 |= _BV(OCIE1A);   
	OCR1A = 400; //40KHz Samplefreq

	// Set up Timer 2 to do pulse width modulation on D11

	// Use internal clock (datasheet p.160)
	ASSR &= ~(_BV(EXCLK) | _BV(AS2));

	// Set fast PWM mode  (p.157)
	TCCR2A |= _BV(WGM21) | _BV(WGM20);
	TCCR2B &= ~_BV(WGM22);

	// Do non-inverting PWM on pin OC2A (p.155)
	// On the Arduino this is pin 11.
	TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
	TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
	// No prescaler (p.158)
	TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

	// Set initial pulse width to the first sample.
	OCR2A = 128;

	//set timer0 interrupt at 61Hz
	TCCR0A = 0;// set entire TCCR0A register to 0
	TCCR0B = 0;// same for TCCR0B
	TCNT0  = 0;//initialize counter value to 0
	// set compare match register for 62hz increments
	OCR0A = 255;// = 61Hz
	// turn on CTC mode
	TCCR0A |= (1 << WGM01);
	// Set CS01 and CS00 bits for prescaler 1024
	TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00);  //1024 prescaler 

	TIMSK0 |= (1 << OCIE0A);


	// set up the ADC
	ADCSRA &= ~PS_128;  // remove bits set by Arduino library
	// Choose prescaler PS_128.
	ADCSRA |= PS_128;
	ADMUX = 64;
	sbi(ADCSRA, ADSC);


	sei();
}

uint8_t phaccBD,phaccCH,phaccCL,phaccCR,phaccOH,phaccRD,phaccRS,phaccSD;
uint8_t pitchBD=128;
uint8_t pitchCH=64;
uint8_t pitchCL=64;
uint8_t pitchCR=16;
uint8_t pitchOH=64;
uint8_t pitchRD=16;
uint8_t pitchRS=64;
uint8_t pitchSD=64;
uint16_t samplecntBD,samplecntCH,samplecntCL,samplecntCR,samplecntOH,samplecntRD,samplecntRS,samplecntSD;
uint16_t samplepntBD,samplepntCH,samplepntCL,samplepntCR,samplepntOH,samplepntRD,samplepntRS,samplepntSD;

uint8_t bar, led;

int16_t total;
uint8_t divider;
uint8_t MUX=0;
uint8_t lastPortState = 0;
uint8_t currentPortState = 0;

uint32_t pattern1, pattern2, pattern3, pattern4, currentBar;

void loop()
{

#pragma region ringbuffer
	//------ Add current sample word to ringbuffer FIFO --------------------  
	if (RingCount < 255) {  //if space in ringbuffer
		total = 0;
		if (samplecntBD) {
			phaccBD += pitchBD;
			if (phaccBD & 128) {
				phaccBD &= 127;
				samplepntBD++;
				samplecntBD--;

			}
			total += (pgm_read_byte_near(BD + samplepntBD) - 128);
		}
		else
		{
			//PORTD &= ~(1 << PD5);
			PORTD &= 0xBF;
		}
		if (samplecntSD) {
			phaccSD += pitchSD;
			if (phaccSD & 128) {
				phaccSD &= 127;
				samplepntSD++;
				samplecntSD--;

			}
			total += (pgm_read_byte_near(SN + samplepntSD) - 128);
		}
		else
		{
			//PORTD &= ~(1 << PD4);
			PORTD &= 0xDF;
		}
		/*
		if (samplecntCL) {
			phaccCL += pitchCL;
			if (phaccCL & 128) {
				phaccCL &= 127;
				samplepntCL++;
				samplecntCL--;

			}
			total += (pgm_read_byte_near(CL + samplepntCL) - 128);
		}
		*/
		/*
		if (samplecntRS) {
			phaccRS += pitchRS;
			if (phaccRS & 128) {
				phaccRS &= 127;
				samplepntRS++;
				samplecntRS--;

			}
			total += (pgm_read_byte_near(RS + samplepntRS) - 128);
		}
		*/
		if (samplecntCH) {
			phaccCH += pitchCH;
			if (phaccCH & 128) {
				phaccCH &= 127;
				samplepntCH++;
				samplecntCH--;

			}
			total += (pgm_read_byte_near(CH + samplepntCH) - 128);
		}
		else
		{
			//PORTC &= ~(1 << PC5);
			PORTC &= 0xEF;
		}
		if (samplecntOH) {
			phaccOH += pitchOH;
			if (phaccOH & 128) {
				phaccOH &= 127;
				samplepntOH++;
				samplecntOH--;

			}
			total += (pgm_read_byte_near(OH + samplepntOH) - 128);
		}
		else
		{
			//PORTC &= ~(1 << PC4);
			PORTC &= 0xDF;
		}
		/*
		if (samplecntCR) {
			phaccCR += pitchCR;
			if (phaccCR & 128) {
				phaccCR &= 127;
				samplepntCR++;
				samplecntCR--;

			}
			total += (pgm_read_byte_near(CR + samplepntCR) - 128);
		}
		if (samplecntRD) {
			phaccRD += pitchRD;
			if (phaccRD & 128) {
				phaccRD &= 127;
				samplepntRD++;
				samplecntRD--;

			}
			total += (pgm_read_byte_near(RD + samplepntRD) - 128);
		}*/
		total >>= 1;
		if (!(PINB & 4)) total >>= 1;
		total += 128;
		if (total > 255) total = 255;

		cli();
		Ringbuffer[RingWrite] = total;
		RingWrite++;
		RingCount++;
		sei();
	}

	//----------------------------------------------------------------------------
#pragma endregion

#pragma region beatHandler
	//--------------- BEAT Tempo ------------------------------------
	if (beatClock) {
		bar++;
		beatClock = 0;
		if (bar == 16) bar = 0;
		currentBar = (1 << bar);

		if (!(bar & 0x01)) { // each 1/2th beat
			led++;
			if (led > 7) { led = 0; }
			shiftOut(dataPin, clockPin, MSBFIRST, (1 << led));
		}

		if (pattern1 & currentBar) {
			samplepntBD = 0;
			samplecntBD = 2154;
			PORTD |= 0x40;
		}
		if (pattern2 & currentBar) {
			samplepntSD = 0;
			samplecntSD = 3482;
			PORTD |= 0x20;
		}
		if (pattern3 & currentBar) {
			samplepntCH = 0;
			samplecntCH = 482;
			PORTC |= 0x10;
		}
		if (pattern4 & currentBar) {
			samplepntOH = 0;
			samplecntOH = 2572;
			PORTC |= 0x20;
		}
	}
#pragma endregion

#pragma region Triggers
	//----------------- Handle Triggers ------------------------------
	currentPortState = (PINB & 0b00010111);
	if (currentPortState == lastPortState) {
	}
	if ((lastPortState & 0b00000010) && !(currentPortState & 0b00000010)) {
		if (beatClockCounter < beatMidpoint) {
			pattern1 |= currentBar;
		}
		else
		{
			pattern1 |= (currentBar << 1);
		}
		samplepntBD = 0;
		samplecntBD = 2154;
		PORTD |= 0x40;
	}
	if ((lastPortState & 0b00000100) && !(currentPortState & 0b00000100)) {		
		if (beatClockCounter < beatMidpoint) {
			pattern2 |= currentBar;
		}
		else
		{
			pattern2 |= (currentBar << 1);
		}
		samplepntSD = 0;
		samplecntSD = 3482;
		PORTD |= 0x20;
	}
	if ((lastPortState & 0b00010000) && !(currentPortState & 0b00010000)) {
		if (beatClockCounter < beatMidpoint) {
			pattern3 |= currentBar;
		}
		else
		{
			pattern3 |= (currentBar << 1);
		}
		samplepntCH = 0;
		samplecntCH = 482;
		PORTC |= 0x10;
	}
	if ((lastPortState & 0b00000001) && !(currentPortState & 0b00000001)) {
		if (beatClockCounter < beatMidpoint) {
			pattern4 |= currentBar;
		}
		else
		{
			pattern4 |= (currentBar << 1);
		}
		samplepntOH = 0;
		samplecntOH = 2572;
		PORTC |= 0x20;
	}
	lastPortState = currentPortState;
	//-----------------------------------------------------------------
#pragma endregion	

#pragma region ADCMeasurements
	//--------------- ADC block ---------------------------------------
	if (!(divider++)) {
		if (!(ADCSRA & 64)) {
			uint16_t pitch = ((ADCL + (ADCH << 8)) >> 3) + 1;
			if (MUX == 0) pitchOH = pitch;
			if (MUX == 1) pitchCH = pitch;
			if (MUX == 2) pitchSD = pitch;
			if (MUX == 3) pitchBD = pitch;
			MUX++;
			if (MUX == 5) MUX = 0;
			ADMUX = 64 | MUX; //Select MUX
			sbi(ADCSRA, ADSC); //start next conversation
		}
	}
	//---------------------------------------------------------------
#pragma endregion

}

void ledAnimation() {
	for (byte i = 0; i < 2; i++)
	{
		for (byte i = 0; i < 8; i++)
		{
			shiftOut(dataPin, clockPin, MSBFIRST, (1 << i));
			delay(50);
		}
		for (byte i = 0; i < 8; i++)
		{
			shiftOut(dataPin, clockPin, MSBFIRST, (1 << 7 - i));
			delay(50);
		}
	}

	PORTD |= 0x40;
	delay(100);
	PORTD |= 0x20;
	delay(100);
	PORTC |= 0x10;
	delay(100);
	PORTC |= 0x20;
	delay(500);
	digitalWrite(5, LOW);
	delay(100);
	digitalWrite(6, LOW);
	delay(100);
	digitalWrite(A4, LOW);
	delay(100);
	digitalWrite(A5, LOW);
	delay(100);

	
}