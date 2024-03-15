#include <stdlib.h>         // header of C general-purpose standard library
#include <avr/io.h>         // AVR input output header, enabling pins
#include <avr/interrupt.h>  // AVR interrupt header, enabling use and editing of interrupts
#include "lcd.h"            // LCD functions header
#include "group2_utility.h" // our library

/*
- Initializes Timer 1 in CTC mode, timer clock 125 kHz
takes parameter milliseconds_to_delay, which is the number of ms to delay the program
note this is evaluated as "milliseconds_to_delay = number of times to milliseconds_to_delay to 125"
*/
void mTimer(int milliseconds_to_delay)
{
    // this function leaves timer 1 in CTC mode

    // scales clock by /64, making the timer clock 250 kHz
    // therefore, there are 250 timer cycles per 1 ms
    TCCR1B &= ~_BV(CS12); // 0
    TCCR1B |= _BV(CS11);  // 1
    TCCR1B |= _BV(CS10);  // 1

    // set WGM1[3:0] to 0100, enabling CTC mode (clear timer on compare)
    // this means TCNT1 will be cleared when it reaches set value of OCR1A
    TCCR1B &= ~_BV(WGM13); // 0
    TCCR1B |= _BV(WGM12);  // 1
    TCCR1A &= ~_BV(WGM11); // 0
    TCCR1A &= ~_BV(WGM10); // 0

    // setting values for TCNT1 and OCRA1, the timer and maximum value respectively
    // datasheet says it's best to set the timer value before the max value to avoid errors
    TCNT1 = 0;
    OCR1A = 250;

    for (int ms_delayed = 0; ms_delayed < milliseconds_to_delay; ms_delayed++)
    {
        TIFR1 |= _BV(OCF1A); // start timer. clears the interrupt flag by writing 1 to its bit

        // wait for OCF1A to trigger (bit 1)
        while (!(TIFR1 & 0b00000010))
        {
            // do nothing
        }
        // once triggered, increment to show 1 ms passed and repeat
    }

    // reset timer 1 scaler back to 000
    // CS12 already set to 0
    // ! may not be necessary in final project code
    TCCR1B &= ~_BV(CS11);
    TCCR1B &= ~_BV(CS10);
}

/*
- starts ADC in free run mode, left adjusted, voltage reference AVCC
! note use of sei(), which polls for pending interrupts and runs them after the last line
*/
void startADC()
{
    cli(); // disable all interrupts

    // - config ADC
    // by default, ADC input is set to ADC0 / PORTF0 / 97
    ADCSRA |= _BV(ADEN); // enable ADC
    ADCSRA |= _BV(ADIE); // enable ADC interrupt
    // ADTS2:0 = 0 (default) means ADC is in free running mode

    // - set result to present left-adjusted in ADCH and ADCL
    ADMUX |= _BV(ADLAR);
    // ADCH: [ ADC9 ADC8 ADC7 ADC6 ADC5 ADC4 ADC3 ADC2 ]
    // ADCL: [ ADC1 ADC0  --   --   --   --   --   --  ]
    /*
    If left adjusted and no more than 8-bit precision is required, it is sufficient to read ADCH.
    however, only the 8 MSBs are read, so the output will be slightly stepped.
    Otherwise, ADCL must be read first, then ADCH, to ensure content is from the same conversion.
    */

    // - sets voltage reference to AVCC
    ADMUX |= _BV(REFS0);
    /*
    a capacitor should be placed between the AREF pin and GND for this mode
    AVCC should also be connected to Vcc through a lowpass (fig. 26-9)
        connect Vcc => 10 uH inductor => 100 nF capacitor => GND
        connect AVCC between inductor and capacitor
    */

    sei(); // sets the Global Enable for all interrupt

    // - initialize ADC, start the first conversion
    // ! first conversion not to be trusted!!
    ADCSRA |= _BV(ADSC);

    // because of sei above, pending interrupts will execute here
}

/*
- sends a HIGH signal to PORTL for 200 ms if PORTL set to output
repeats "times" parameter amount of times
depends on mTimer function
! continues forever if called with times = 0
*/
void flashPortL(unsigned char times)
{
    if (!times)
    {
        while (1)
        {
            PORTL = 0xFF;
            mTimer(200);
            PORTL = 0x00;
            mTimer(200);
        }
    }

    while (times > 0)
    {
        PORTL = 0xFF;
        mTimer(250);
        PORTL = 0x00;
        mTimer(250);
        times--;
    }
}

/*
- starts Timer 0 in Fast PWM Mode at 7,812.50 Hz
parameter duty_cycle takes duty cycle of PWM in % as a float
ex. duty_cycle = 50 => 50% duty cycle
values < 0 are treated as 0, values > 100 are treated as 100
*/
void startPWM(float duty_cycle)
{
    // repair duty_cycle parameter
    if (duty_cycle < 0)
    {
        duty_cycle = 0;
    }

    if (duty_cycle > 100)
    {
        duty_cycle = 100;
    }

    // set Timer 0 (8-bit timer) to Fast PWM mode
    // also sets TOP to 0xFF, OCR0A update at TOP, and TOV flag set on MAX
    TCCR0B &= ~_BV(WGM02);
    TCCR0A |= _BV(WGM01);
    TCCR0A |= _BV(WGM00);

    // clear OC0A on Compare Match, set at bottom
    TCCR0A |= _BV(COM0A1);
    TCCR0A &= ~_BV(COM0A0);

    // set timer 0 clock prescalar to 8
    // timer clock 2 MHz, timer period 128 microseconds
    // therefore PWM frequency 7,812.50 Hz
    TCCR0B &= ~_BV(CS02);
    TCCR0B |= _BV(CS01);
    TCCR0B &= ~_BV(CS00);

    // set timer compare (duty cycle) using duty_cycle parameter
    OCR0A = (int)((duty_cycle / 100) * (0xFF));

    // ! RUN THIS IN MAIN IF COMMENTED
    // DDRB = 0xFF; // set output on PORTB7, which is also OC0A

    TCNT0 = 0x00;        // set timer 0 value to 0, top already set to 0xFF
    TIFR0 |= _BV(OCF0A); // sets the output compare A match flag, starting timer
}

void nightriderPortL()
{
	while(1)
	{
		PORTL = 0b00010000;
		mTimer(125);
		PORTL = 0b00110000;
		mTimer(125);
		PORTL = 0b01100000;
		mTimer(125);
		PORTL = 0b11000000;
		mTimer(125);
		PORTL = 0b10000000;
		mTimer(125);
		PORTL = 0b11000000;
		mTimer(125);
		PORTL = 0b01100000;
		mTimer(125);
		PORTL = 0b00110000;
		mTimer(125);
	}
}