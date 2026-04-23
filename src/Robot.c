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
char reading[10];
uint32_t adc_value;
uint32_t volt_value;
uint32_t range;
serial0_init();
milliseconds_init();
char buffer[60];
char serial_string[60];
uint8_t command;
uint8_t recievedData[2]; //recieved data array
uint32_t current_reading;
uint32_t current_ms, last_send_ms;
uint16_t joystick;
DDRF = 0x00;//put PORTF into input mode
PORTF |= (1<<PF0); //enable the internal pull-up resistor on PF0 and leave the rest of PORTF alone
DDRB = 0xFF;
// Confuguring Timer1
TCCR1A = 0; TCCR1B = 0;
TCCR1A |= (1 << WGM11);
TCCR1B |= (1 << WGM13); // Mode 10, Phase correct
TCCR1A |= (1 << COM1A1)|(1<<COM1B1); // Clear down, set on upcount
TCCR1B |= (1 << CS11); // Prescaler = 8

ICR1 = 20000; // Set TOP value for 20ms period (50Hz)

while(1)
{
	//main loop
  if(serial2_available()) //Returns true if new data available on serial buffer
	{
		//Function takes the array to return data to and the number of bytes to be read.
		serial2_get_data(recievedData,2); 
    sprintf(serial_string,"Joystick 1: %3u Joystick 2: %3u", recievedData[0], recievedData[1]); //Format string
    serial0_print_string(serial_string); //Print string to terminal
    OCR1A = 1000 + ((4 * (uint32_t)recievedData[0]) * 1000) / 1023;
    OCR1B = 1000 + ((4 * (uint32_t)recievedData[1]) * 1000) / 1023;
	}
  }
}
}
