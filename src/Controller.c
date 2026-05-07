#include "Controller.h"
#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t button_state = 0;
volatile uint8_t mode = 0;

int main(void)
{
//main function initialization
serial2_init();
char reading1[24], reading2[24];
char mode_str[16];
uint32_t joystick1_value, joystick2_value;
uint32_t current_ms, last_send_ms;
uint8_t recievedData[2];
uint8_t battery_percentage;
uint8_t ldr_value = 0;
uint8_t front_range = 0;
serial0_init();
milliseconds_init();
adc_init();
lcd_init();
DDRD = 0x00; //put PORTD into input mode
PORTD |= (1<<PD0); //enable the internal pull-up resistor on PD0
// Set PD1 as input with pull-up
DDRD &= ~(1 << PD1);
PORTD |= (1 << PD1);

// Configure INT1 — trigger on rising edge
EICRA |= (1 << ISC11);
EICRA &= ~(1 << ISC10);

// Enable INT1
EIMSK |= (1 << INT1);

DDRD &= ~(1 << PD0);
PORTD |= (1 << PD0); // pull-up

// Falling edge on INT2
EICRA |= (1 << ISC01);
EICRA &= ~(1 << ISC00);

EIMSK |= (1 << INT0);

// Enable global interrupts
sei();

while(1)
{
	//main loop
  if(serial2_available()) //Returns true if new data available on serial buffer
	{
    serial2_get_data(recievedData,3); //Function takes the array to return data to and the number of bytes to be read.
    char buf[16];
  }

  if (mode == 0) {
    sprintf(mode_str, "User");
  }
  else if (mode == 1) {
    sprintf(mode_str, "Auto");
  }
  else {
    sprintf(mode_str, "Beacon");
  }
  front_range = recievedData[2];
  current_ms = milliseconds_now();
  battery_percentage = recievedData[0]*100/118;
  ldr_value = recievedData[1];
  char buf[16];
  sprintf(buf, "Pwr:%3u%% %s  ", battery_percentage, mode_str);
  lcd_home();
  lcd_puts(buf);
  sprintf(buf, "LDR:%3u Dist:%3ucm", ldr_value, front_range);
  lcd_goto(0x40);
  lcd_puts(buf);

	if ((current_ms-last_send_ms) >= 100) //sending rate controlled here
	{
  joystick1_value = abs((adc_read(15)/5)-204);
  joystick2_value = abs((adc_read(0)/5)-204);
  if (joystick1_value > 255) joystick1_value = 255;
  if (joystick2_value > 255) joystick2_value = 255;
  uint8_t data_to_send[2] = {joystick1_value, joystick2_value};
  serial2_write_bytes(4, data_to_send[0], data_to_send[1], button_state, mode);
  last_send_ms = current_ms;
  }
}
}

ISR(INT1_vect)
{
    static uint32_t last_press = 0;
    uint32_t now = milliseconds_now();
    if ((now - last_press) > 200) // 200ms debounce
    {
      if (button_state == 0) {
        button_state = 1;
        last_press = now;
        serial0_print_string("Button Pressed\n");
      }
      else {
        button_state = 0;
        last_press = now;
        serial0_print_string("Button Released\n");
      }
    }
}

ISR(INT0_vect)
{
    static uint32_t last_press = 0;
    uint32_t now = milliseconds_now();
    if ((now - last_press) > 200)
    {
      if (mode == 0) {
        mode = 1;
      }
      else if (mode == 1) {
        mode = 2;
      }
      else {
        mode = 0;
      }
    }
    last_press = now;
}
