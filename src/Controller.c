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
uint8_t recievedData;
uint32_t battery_percentage;
serial0_init();
milliseconds_init();
adc_init();
lcd_init();
DDRD = 0x00; //put PORTD into input mode
PORTD |= (1<<PD0); //enable the internal pull-up resistor on PD0

while(1)
{
	//main loop
  if(serial2_available()) //Returns true if new data available on serial buffer
	{
    serial2_get_data(recievedData,1); //Function takes the array to return data to and the number of bytes to be read.
	}
  current_ms = milliseconds_now();

  battery_percentage = recievedData*100/118;
  lcd_home();
  lcd_puts(battery_percentage);
  sprintf(reading1,"Battery: %3u%% \n", battery_percentage); //Format string
  serial0_print_string(reading1); //Print string to terminal

	if ((current_ms-last_send_ms) >= 100) //sending rate controlled here
	{
  joystick1_value = abs((adc_read(15)/5)-204);
  joystick2_value = abs((adc_read(0)/5)-204);
  if (joystick1_value > 255) joystick1_value = 255;
  if (joystick2_value > 255) joystick2_value = 255;
  uint8_t data_to_send[2] = {joystick1_value, joystick2_value};
  serial2_write_bytes(2, data_to_send[0], data_to_send[1]);
  last_send_ms = current_ms;
  }
}
}
