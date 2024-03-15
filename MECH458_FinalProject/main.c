/*
* PROJECT: FINAL
* GROUP: 2
- NAME 1: Ben Say, V00927942
- NAME 2: Bryce Larson, V00947112
# DESC:
	Wiring:
		lower 4 bits of PORTL (42:45) to debugging LEDs
		PORTC (30:37) to 8 output LEDs (and LCD)

		a capacitor should be placed between the AREF pin and GND to reduce noise

		kill switch active LOW on INT2/PORTD2 (19)

		- SENSOR WIRING
		Through a lowpass:
		OR -> PINA0 (22)
		EX -> PINA1 (23)
		HE -> PINA2 (24)
		RL -> ADC   (A0)

		- STEPPER MOTOR WIRING
				[ x , x , A13 , A12 , A11 , A10 , A9 , A8 ]
		PORTK = [ x , x , EN0 , L1  , L2  , EN1 , L3 , L4 ]

		- CONVEYOR BELT WIRING
				[  13 , x , x , x , 50 , 51 , 52 , 53 ]
		PORTB = [ PWM , x , x , x , IA , IB , EA , EB ]
		50: [IA IB]
		52: [EA EB]



- working: 29Mar2023
*/

//*########################################## INCLUDES ###########################################*//

#include <stdlib.h>		    // header of C general-purpose standard library
#include <avr/io.h>		    // AVR input output header, enabling pins
#include <avr/interrupt.h>  // AVR interrupt header, enabling use and editing of interrupts
#include "lcd.h"		    // LCD functions header
#include "group2_utility.h" // our library

//*################################### MACROS AND DEFINITIONS ####################################*//

#define BELT_STOP() PORTB=0b00001111
#define BELT_MOVE() PORTB=0b00000111
#define DISABLE_MOTORS() PORTB=0b00001100; PORTK=0b00000000;

#define PI 3.1415926535897932384626
#define TOTAL_SORTED a_sorted + s_sorted + w_sorted + b_sorted

// material average min ADC values
#define ALUMINUM_AVG 232
#define STEEL_AVG 679
#define WHITE_AVG 935
#define BLACK_AVG 963

// motor speed defining constants
#define PERCENT_BELT_SPEED 65
#define MIN_STEP_DELAY 4
#define MAX_STEP_DELAY 15

//*###################################### GLOBAL VARIABLES #######################################*//

// - midpoint values can be set manually here
// ! if set to 0, value computed from definitions above (useful for classification in final test)
unsigned short ALUMINUM_STEEL_MIDPOINT = 456;
unsigned short STEEL_WHITE_MIDPOINT = 830;
unsigned short WHITE_BLACK_MIDPOINT = 958; // 949

// ADC system variables
unsigned short ADC_output = 0;				// holds the output of the ADC until next conversion
unsigned short ADCH_output = 0;	// holds the output of ADCH until next conversion
unsigned short ADCL_output = 0;	// holds the output of ADCL until next conversion
unsigned char new_ADC_output_available = 0; // new conversion ready flag

// stores the minimum ADC value while a part is in front of the prox sensor
// otherwise stores 1025, which is just over the maximum possible ADC output
unsigned short current_part_max_reflection = 1025;

// buffer for polling PINA (sensor PIN)
unsigned char PINA_input = 0;
unsigned char previous_PINA_input = 0;

unsigned char most_recent_part = 0; // 1=A, 2=S, 3=W, 4=B

// contains the current state of the stepper
// the choice to say the motor is starting at state 1 is arbitrary
unsigned char current_stepper_state = 1;

// contains the currently selected bucket after initialization to black via the hall effect sensor
// ! not related to current_stepper_state variable
unsigned char current_bucket = 4; // 1=A, 2=S, 3=W, 4=B
unsigned char last_moving_CW = 1; // 1 if true (stepper is initialized CW, so this set CW at start)

unsigned char a_sorted = 0;
unsigned char s_sorted = 0;
unsigned char w_sorted = 0;
unsigned char b_sorted = 0;
unsigned char rampdown = 0;
unsigned char rampdown_indicated = 0;
unsigned char pause = 0;
unsigned char was_paused = 1; // initially set to 1 to run LCD setup and start system

// FIFO queue variables
link *queue_head = NULL;
link *queue_tail = NULL;

//*########################################## FUNCTIONS ##########################################*//

// steps is how many steps, CW = 0 is CCW
// delay indicates how much time is between each step
void basic_step(unsigned char steps, unsigned char CW, unsigned short delay)
{
	for (unsigned char i = 0; i < steps; i++)
	{
		// move current_stepper_state to its next value based on CW parameter
		if (CW)
		{
			if (++current_stepper_state > 4) current_stepper_state = 1;
		}
		else if (--current_stepper_state < 1) current_stepper_state = 4;

		// determine what state to send the stepper motor
		     if (current_stepper_state == 1) PORTK = 0b00110110;
		else if (current_stepper_state == 2) PORTK = 0b00101110;
		else if (current_stepper_state == 3) PORTK = 0b00101101;
		else if (current_stepper_state == 4) PORTK = 0b00110101;

		mTimer(delay); // delay between each step
	}

	last_moving_CW = CW;
}

void stepper_move(unsigned char desired_bucket)
{
	if (desired_bucket < 1 || desired_bucket > 4)
	{
		DISABLE_MOTORS();
		LCDWriteStringXY(0, 0, "BUCKET REQ ERR  ");
		LCDWriteStringXY(0, 1, "CB=   DB=       ");
		LCDWriteIntXY(3,1,current_bucket,2);
		LCDWriteIntXY(9,1,desired_bucket,2);
		flashPortL(0);
	}

	// set defaults for move_CW and # steps to move
	unsigned char move_CW = last_moving_CW;
	unsigned char steps_to_move = 50;

	if (current_bucket == desired_bucket) return;

	else if (current_bucket == 1) // ALUMINUM
	{
		     if (desired_bucket == 2) steps_to_move = 100; // S
		else if (desired_bucket == 3) move_CW = 1; // W
		else if (desired_bucket == 4) move_CW = 0; // B
	}

	else if (current_bucket == 2) // STEEL
	{
		     if (desired_bucket == 1) steps_to_move = 100; // A
		else if (desired_bucket == 3) move_CW = 0; // W
		else if (desired_bucket == 4) move_CW = 1; // B
	}

	else if (current_bucket == 3) // WHITE
	{
		     if (desired_bucket == 4) steps_to_move = 100; // B
		else if (desired_bucket == 1) move_CW = 0; // A
		else if (desired_bucket == 2) move_CW = 1; // S
	}

	else if (current_bucket == 4) // BLACK
	{
		     if (desired_bucket == 3) steps_to_move = 100; // W
		else if (desired_bucket == 1) move_CW = 1; // A
		else if (desired_bucket == 2) move_CW = 0; // S
	}

	/*
	// - 2 move cases
	else if (current_bucket == 1 && desired_bucket == 2) steps_to_move = 100;
	else if (current_bucket == 2 && desired_bucket == 1) steps_to_move = 100;
	else if (current_bucket == 3 && desired_bucket == 4) steps_to_move = 100;
	else if (current_bucket == 4 && desired_bucket == 3) steps_to_move = 100;

	// - 1 move cases
	else if (current_bucket == 1 && desired_bucket == 3) move_CW = 1; // A -> W
	else if (current_bucket == 1 && desired_bucket == 4) move_CW = 0; // A -> B

	else if (current_bucket == 2 && desired_bucket == 3) move_CW = 0; // S -> W
	else if (current_bucket == 2 && desired_bucket == 4) move_CW = 1; // S -> B

	else if (current_bucket == 3 && desired_bucket == 1) move_CW = 0; // W -> A
	else if (current_bucket == 3 && desired_bucket == 2) move_CW = 1; // W -> S

	else if (current_bucket == 4 && desired_bucket == 1) move_CW = 1; // B -> A
	else if (current_bucket == 4 && desired_bucket == 2) move_CW = 0; // B -> S
	*/

	else
	{
		DISABLE_MOTORS();
		LCDWriteStringXY(0, 0, "STEPPER CASE ERR");
		LCDWriteStringXY(0, 1, "CB=   DB=       ");
		LCDWriteIntXY(3,1,current_bucket,2);
		LCDWriteIntXY(9,1,desired_bucket,2);
		flashPortL(0);
	}

	// move stepper
	unsigned char delay = MAX_STEP_DELAY;
	for (unsigned char i = 0; i < steps_to_move; i++)
	{
		// calculate delay in an S curve
		delay = ((MAX_STEP_DELAY-MIN_STEP_DELAY)/2)*sin(2*i*PI/steps_to_move+PI/2) + (MAX_STEP_DELAY+MIN_STEP_DELAY)/2;

		// move current_stepper_state to its next value based on CW parameter
		if (move_CW)
		{
			if (++current_stepper_state > 4) current_stepper_state = 1;
		}
		else if (--current_stepper_state < 1) current_stepper_state = 4;

		// determine what state to send the stepper motor
		     if (current_stepper_state == 1) PORTK = 0b00110110;
		else if (current_stepper_state == 2) PORTK = 0b00101110;
		else if (current_stepper_state == 3) PORTK = 0b00101101;
		else if (current_stepper_state == 4) PORTK = 0b00110101;

		mTimer(delay); // delay between each step
	}

	last_moving_CW = move_CW;
	current_bucket = desired_bucket;
}

// - adds a value to a predefined global fifo queue
unsigned char add_to_queue(unsigned char value)
{
	// if number was not valid, return 0
	if (value < 1 || value > 4) return 0;

	// add value to the queue
	link *newLink = malloc(sizeof(link));
	newLink->piece_code = value;
	newLink->next = NULL;

	if (queue_head != NULL && queue_tail != NULL)
	{
		queue_tail->next = newLink;
		queue_tail = newLink;
	}
	else
	{
		queue_head = newLink;
		queue_tail = newLink;
	}

	// if it worked, return 1
	return 1;
}

unsigned short sizeofqueue()
{
	unsigned short return_value = 0;
	link *temp_ptr = queue_head;

	while(temp_ptr != NULL)
	{
		return_value++;
		temp_ptr = temp_ptr->next;
	}

	return return_value;
}

//*############################################ MAIN #############################################*//

int main()
{
	// - calculate part classifying variables if not manually declared in global variables
	if (!ALUMINUM_STEEL_MIDPOINT) ALUMINUM_STEEL_MIDPOINT = (ALUMINUM_AVG + STEEL_AVG) / 2;
	if (!STEEL_WHITE_MIDPOINT) STEEL_WHITE_MIDPOINT = (STEEL_AVG + WHITE_AVG) / 2;
	if (!WHITE_BLACK_MIDPOINT) WHITE_BLACK_MIDPOINT = (WHITE_AVG + BLACK_AVG) / 2;

	// - SET SYSTEM CLOCK TO: 8 MHz
	//CLKPR = 0x80;
	//CLKPR = 0x01; // set clock prescalar to 2

	// - CONFIGURE EXT INTERRUPTS
	EIMSK |= _BV(INT2); // enable INT2 (PD2 / 19)
	EIMSK |= _BV(INT3); // enable INT3 (PD3 / 18)
	// set falling edge of INT2 and INT3 to generate an interrupt request
	EICRA = 0b10100000; // ISC3 = 0b10 and ISC2 = 0b10

	// - SET PORT MODES
	DDRL = 0xFF; // sets all pins on PORTL to output (for debug)
	DDRA = 0x00; // sets all pins on PINA  to  input (for sensors)
	DDRD = 0x00; // sets all pins on PIND  to  input (for interrupts)
	DDRB = 0xFF; // sets all pins on PORTB to output (for PWM and belt control)
	DDRK = 0xFF; // sets all pins on PORTK to output (for stepper motor control)

	// - VALIDATE LCD MODULE
	InitLCD(LS_BLINK | LS_ULINE);
	LCDClear();
	LCDWriteStringXY(0, 0, "--INITIALIZING--");
	LCDWriteStringXY(0, 1, "-----BUCKET-----");

	// - initialize stepper motor (unplug HE to bypass)
	unsigned char d = 20;
	basic_step(30,1,d);
	// move stepper until hall effect sensor activates (LOW)
	PINA_input = PINA;
	while ((PINA_input & (1 << 2)))
	{
		basic_step(1,1,d);
		if ((d /= 1.1) < 8) d = 8;
		PINA_input = PINA;
	}
	// ! bucket now at black

	LCDWriteStringXY(0, 0, "-^------^-------");
	LCDWriteStringXY(0, 1, "END    PAUSE    ");
	mTimer(200);

	// start the ADC in free run mode
	startADC();

	// start the conveyor belt
	startPWM(PERCENT_BELT_SPEED);
	BELT_STOP();
	mTimer(5);

	// * main loop

	while (1)
	{
		if (!pause)
		{
			PINA_input = PINA; // read PINA

			if (was_paused)
			{
				LCDWriteStringXY(0, 0, "last=X-#### QD=#");
				LCDWriteStringXY(0, 1, "next=  sorted=  ");
				LCDWriteIntXY(5, 0, most_recent_part, 1);
				LCDWriteIntXY(15, 0, sizeofqueue(), 1);
				LCDWriteIntXY(14, 1, TOTAL_SORTED, 2);
				if (queue_head != NULL)
				{
					LCDWriteIntXY(5, 0, queue_head->piece_code, 1);
					LCDWriteIntXY(5, 1, queue_tail->piece_code, 1);
				}
				else
				{
					LCDWriteStringXY(5, 0, "N");
					LCDWriteStringXY(5, 1, "N");
				}
				was_paused = 0;
				BELT_MOVE();
			}

			// - SORT PART AT FRONT (EX active LOW)
			if (!(PINA_input & (1 << 1))) // if EX active
			{
				if (queue_head != NULL && queue_head->piece_code != current_bucket)
				{
					BELT_STOP();
					//mTimer(10);
					stepper_move(queue_head->piece_code);
					BELT_MOVE();
				}
			}
			else if (!(previous_PINA_input & (1 << 1)) && queue_head != NULL) // if EX inactive but was active last time
			{
				// front of queue has been sorted
					 if (queue_head->piece_code == 1) a_sorted++;
				else if (queue_head->piece_code == 2) s_sorted++;
				else if (queue_head->piece_code == 3) w_sorted++;
				else if (queue_head->piece_code == 4) b_sorted++;

				// remove piece from queue
				link *temp_link = queue_head;
				queue_head = queue_head->next;
				free(temp_link);

				if (queue_head != NULL)
				{
					LCDWriteIntXY(5, 1, queue_head->piece_code, 1);
					LCDWriteIntXY(15, 0, sizeofqueue(), 1);
				}
				else
				{
					queue_tail = NULL;
					LCDWriteStringXY(5, 1, "N");
					LCDWriteIntXY(15, 0, 0, 1);
				}

				LCDWriteIntXY(14, 1, TOTAL_SORTED, 2);
			}

			// - classify a part if nothing in front of prox sensor (but there was last time)
			if (!(PINA_input & (1 << 0)) && (previous_PINA_input & (1 << 0)))
			{
				if (current_part_max_reflection < ALUMINUM_STEEL_MIDPOINT)
					most_recent_part = 1; // aluminum
				else if (current_part_max_reflection < STEEL_WHITE_MIDPOINT)
					most_recent_part = 2; // steel
				else if (current_part_max_reflection < WHITE_BLACK_MIDPOINT)
					most_recent_part = 3; // white
				else if (current_part_max_reflection < 1025)
					most_recent_part = 4; // black
				else
				{
					DISABLE_MOTORS();
					LCDWriteStringXY(0, 0, "CLASSIFIER ERROR");
					LCDWriteStringXY(0, 1, "final ADC =     ");
					LCDWriteIntXY(11, 1, current_part_max_reflection, 4);
					flashPortL(0);
				}

				if (!add_to_queue(most_recent_part))
				{
					DISABLE_MOTORS();
					LCDWriteStringXY(0, 0, "QUEUE ADD ERROR ");
					LCDWriteStringXY(0, 1, "NO FURTHER INFO ");
					flashPortL(0);
				}

				LCDWriteIntXY(15, 0, sizeofqueue(), 1);
				LCDWriteIntXY(5, 0, most_recent_part, 1);
				LCDWriteIntXY(7, 0, current_part_max_reflection, 4);
				LCDWriteIntXY(5, 1, queue_head->piece_code, 1);
				current_part_max_reflection = 1025;	// reset ADC output for part
			}

			// - deal with an ADC output only if OR sees something, except first read of a part
			if (new_ADC_output_available && (PINA_input & (1 << 0)) )
			{
				ADC_output = (ADCH_output << 2) | (ADCL_output >> 6);
				new_ADC_output_available = 0;
				ADCSRA |= _BV(ADSC); // start the next conversion
				if (ADC_output < current_part_max_reflection && (previous_PINA_input & (1 << 0)))
					current_part_max_reflection = ADC_output;
			}

			previous_PINA_input = PINA_input;
		}

		if (rampdown && queue_head == NULL)
		{
			EIMSK &= ~_BV(INT3); // enable INT3 to disable pause
			EIMSK &= ~_BV(INT2); // enable INT2 to disable rampdown re-trigger
			DISABLE_MOTORS();
			LCDWriteStringXY(0, 0, "rampdown.       ");
			LCDWriteStringXY(0, 1, "a   s   w   b   ");
			LCDWriteIntXY( 1, 1, a_sorted, 2);
			LCDWriteIntXY( 5, 1, s_sorted, 2);
			LCDWriteIntXY( 9, 1, w_sorted, 2);
			LCDWriteIntXY(13, 1, b_sorted, 2);
			nightriderPortL();
		}
		else if (rampdown && !rampdown_indicated)
		{
			PORTL = 0b11110000;
			rampdown_indicated = 1;
		}
	}

	return 0;
}

//*################################### INTERRUPT DEFINITIONS #####################################*//

// - interrupt for kill switch
ISR(INT2_vect)
{
	rampdown = 1;
}

// - interrupt for pause button
ISR(INT3_vect)
{
	if (!(PIND & 1<<3))
	{
		BELT_STOP();
		mTimer(20);
		pause = !pause;
		while(!(PIND & 1<<3));
		mTimer(20);
		LCDWriteStringXY(0, 0, "PAUSED  queue=  ");
		LCDWriteStringXY(0, 1, "a   s   w   b   ");
		LCDWriteIntXY(14, 0, sizeofqueue(), 2);
		LCDWriteIntXY( 1, 1, a_sorted, 2);
		LCDWriteIntXY( 5, 1, s_sorted, 2);
		LCDWriteIntXY( 9, 1, w_sorted, 2);
		LCDWriteIntXY(13, 1, b_sorted, 2);
		was_paused = 1;
	}
}

// - interrupt for ADC complete
ISR(ADC_vect)
{
	ADCL_output = ADCL;	// read ADCL, locking ADC data registers
	ADCH_output = ADCH; // read ADCH, unlocking ADC data registers
	new_ADC_output_available = 1;
}
