//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Robot.h"
//static function prototypes, functions only called in this file

int main(void)
{
//main function initialization
serial2_init();
serial0_init();
milliseconds_init();
adc_init();
uint32_t current_ms, last_send_ms;
uint8_t battery_voltage;
static int16_t lm = 0;
static int16_t rm = 0;
static int16_t fc;
static int16_t rc;
char buffer[60];
char serial_string[60];
uint8_t recievedData[2]; //recieved data array
uint16_t joystick;
DDRF = 0x00;//put PORTF into input mode
PORTF |= (1<<PF0); //enable the internal pull-up resistor on PF0 and leave the rest of PORTF alone
DDRB |= (1<<PB5) | (1<<PB6); // OC1A and OC1B as outputs
DDRA |= (1<<PA0)|(1<<PA1)|(1<<PA2)|(1<<PA3); //put A0-A3 into low impedance output mode
// Confuguring Timer1
TCCR1A = 0; TCCR1B = 0;
TCCR1A |= (1 << WGM11);
TCCR1B |= (1 << WGM13) | (1 << WGM12); // Fast PWM, TOP = ICR1
TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
TCCR1B |= (1 << CS11); // Prescaler = 8

ICR1 = 10000; // Set TOP value for 20ms period (50Hz)

while(1)
{
	//main loop
    if(serial2_available()) //Returns true if new data available on serial buffer
	{
        //Function takes the array to return data to and the number of bytes to be read.
        serial2_get_data(recievedData,2); 
        fc = ((int16_t)recievedData[0]*5)/4;
        rc = ((int16_t)recievedData[1]*5)/4;
        sprintf(serial_string,"Joystick 1: %3u Joystick 2: %3u \n", fc, rc); //Format string
    }

    lm = fc - 127;
    rm = rc - 127;
    if (lm > 127) lm = 127;
    if (lm < -127) lm = -127;
    if (rm > 127) rm = 127;
    if (rm < -127) rm = -127;
    sprintf(serial_string,"Joystick 1: %3d Joystick 2: %3d \n", lm, rm); //Format string
    serial0_print_string(serial_string); //Print string to terminal
    OCR1A = (int32_t)abs(lm)*10000/127;
    OCR1B = (int32_t)abs(rm)*10000/127;
    if(lm>=0)
    {
        //set direction of left motor forward
        PORTA |= (1<<PA0);
        PORTA &= ~(1<<PA1);
    }
    else
    {
        //set direction of left motor in reverse
        PORTA |= (1<<PA1);
        PORTA &= ~(1<<PA0);
    }
    if(rm>=0)
    {
        //set direction of right motor forward
        PORTA |= (1<<PA2);
        PORTA &= ~(1<<PA3);
    }
    else
    {
        //set direction of right motor in reverse
        PORTA |= (1<<PA3);
        PORTA &= ~(1<<PA2);
    }
    current_ms = milliseconds_now();
	
	//sending section
	if( (current_ms-last_send_ms) >= 100) //sending rate controlled here
	{
        battery_voltage = adc_read(7)/5; //read battery voltage on ADC channel 7, convert to voltage
        serial2_write_bytes(1, battery_voltage);
        last_send_ms = current_ms;
  }
}
}
