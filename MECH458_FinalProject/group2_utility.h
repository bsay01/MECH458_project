#ifndef GROUP2UTILITY_H

#define GROUP2UTILITY_H

// defining a type for an element of the FIFO queue
typedef struct link
{
    char piece_code;
    struct link *next; // points to the next element in the list
} link;

void mTimer(int milliseconds_to_delay);
void startADC();
void flashPortL(unsigned char times);
void startPWM(float duty_cycle);
void nightriderPortL();

#endif
