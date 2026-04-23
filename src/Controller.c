#include "Controller.h"
#include <avr/io.h>
#include <avr/interrupt.h>

int main(void)
{
//main function initialization
serial2_init();
char reading1[24], reading2[24];
uint32_t joystick1_value, joystick2_value;
uint32_t current_ms, last_send_ms;
serial0_init();
milliseconds_init();
adc_init();

while(1)
{
	//main loop
  if(serial2_available()) //Returns true if new data available on serial buffer
	{
	}
  current_ms = milliseconds_now();
	
	//sending section
	if( (current_ms-last_send_ms) >= 100) //sending rate controlled here
	{
  joystick1_value = adc_read(0)/4;
  joystick2_value = adc_read(1)/4;
  sprintf(reading1, "Joystick 1 Value: %lu", joystick1_value);
  sprintf(reading2, "Joystick 2 Value: %lu", joystick2_value);
  serial0_print_string(reading1);
  serial0_print_string("\n");
  serial0_print_string(reading2);
  serial0_print_string("\n");
  serial2_write_bytes(2, reading1, reading2);
  _delay_ms(1000);
  last_send_ms = current_ms;
  }
}
}
