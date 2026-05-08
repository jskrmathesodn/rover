#include "Robot.h"

int main(void)
{
    serial2_init();
    serial0_init();
    milliseconds_init();
    adc_init();

    uint32_t current_ms, last_send_ms = 0;
    uint8_t servo_on = 0;
    uint8_t battery_voltage = 0;
    uint8_t mode = 0;
    uint8_t mode0 = 1;
    uint16_t front_range = 0;
    uint16_t left_range = 0;
    uint16_t right_range = 0;
    uint16_t raw_front = 0;
    uint16_t raw_left = 0;
    uint16_t raw_right = 0;
    uint16_t front_stop = 20;
    uint16_t side_wall = 15;
    static int16_t lm = 0;
    static int16_t rm = 0;
    static int16_t fc = 0;
    static int16_t rc = 0;
    char serial_string[60];
    uint8_t recievedData[4] = {0, 0, 0, 0};

    static uint8_t last_ldr_state = 0;
    static uint32_t last_rise_ms = 0;
    static uint8_t frequency = 0;
    static uint16_t ldr_min = 1023;
    static uint16_t ldr_max = 0;
    static uint32_t last_calibrate_ms = 0;
    static uint32_t period_ms = 0;
    static uint8_t freq_history[5] = {0};
    static uint8_t freq_index = 0;
    static uint8_t stable_frequency = 0;
    static uint16_t ldr_min_short = 1023;
    static uint16_t ldr_max_short = 0;
    static uint32_t last_short_ms = 0;

    DDRF = 0x00;
    DDRB |= (1 << PB5) | (1 << PB6);
    DDRA |= (1 << PA0) | (1 << PA1) | (1 << PA2) | (1 << PA3);

    // Timer1 — DC motors
    TCCR1A = 0; TCCR1B = 0;
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);
    TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
    TCCR1B |= (1 << CS11);
    ICR1 = 1000;

    // Timer3 — Servo
    TCCR3A = 0; TCCR3B = 0;
    TCCR3A |= (1 << WGM31) | (1 << COM3A1);
    TCCR3B |= (1 << WGM33) | (1 << WGM32) | (1 << CS31);
    ICR3 = 20000;
    DDRE |= (1 << PE3);

while (1)
    {
        // LDR frequency detection — runs every loop for accuracy
        uint16_t ldr = adc_read(1);
        uint8_t current_ldr_state;

        if (ldr < ldr_min_short) ldr_min_short = ldr;
        if (ldr > ldr_max_short) ldr_max_short = ldr;

        uint32_t now2 = milliseconds_now();
        if ((now2 - last_short_ms) > 500)
        {
            ldr_min = ldr_min_short;
            ldr_max = ldr_max_short;
            ldr_min_short = 1023;
            ldr_max_short = 0;
            last_short_ms = now2;
        }

        uint16_t ldr_low = ldr_min + (ldr_max - ldr_min) / 3;
        uint16_t ldr_high = ldr_min + (ldr_max - ldr_min) * 2 / 3;

        if (ldr < ldr_low)
        {
            current_ldr_state = 1;
        }
        else if (ldr > ldr_high)
        {
            current_ldr_state = 0;
        }
        else
        {
            current_ldr_state = last_ldr_state;
        }

        if (current_ldr_state == 1 && last_ldr_state == 0)
        {
            uint32_t now = milliseconds_now();
            uint32_t period_ms = now - last_rise_ms;
            if (period_ms > 40 && period_ms < 600)
            {
                frequency = (uint8_t)(1000 / period_ms);
                freq_history[freq_index] = frequency;
                freq_index = (freq_index + 1) % 5;

                uint8_t sorted[5];
                memcpy(sorted, freq_history, 5);
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 4 - i; j++)
                    {
                        if (sorted[j] > sorted[j+1])
                        {
                            uint8_t temp = sorted[j];
                            sorted[j] = sorted[j+1];
                            sorted[j+1] = temp;
                        }
                    }
                }
                stable_frequency = sorted[2];
            }
            last_rise_ms = now;
        }
        last_ldr_state = current_ldr_state;

        // Reset frequency if no edge detected for 600ms
        uint32_t now = milliseconds_now();
        if ((now - last_rise_ms) > 600)
        {
            frequency = 0;
            stable_frequency = 0;
            memset(freq_history, 0, 5);
        }

        // Receive data from controller
        if (serial2_available())
        {
            serial2_get_data(recievedData, 4);
            fc = ((int16_t)recievedData[0] * 5) / 4;
            rc = ((int16_t)recievedData[1] * 5) / 4;
            servo_on = recievedData[2];
            mode = recievedData[3];
        }

        // Mode 0 — Manual
        if (mode0 == 0)
        {
            lm = fc - 127;
            rm = rc - 127;
            if (lm > 127) lm = 127;
            if (lm < -127) lm = -127;
            if (rm > 127) rm = 127;
            if (rm < -127) rm = -127;

            OCR1A = (int32_t)abs(lm) * 1000 / 127;
            OCR1B = (int32_t)abs(rm) * 1000 / 127;

            if (lm >= 0)
            {
                PORTA |= (1 << PA0);
                PORTA &= ~(1 << PA1);
            }
            else
            {
                PORTA |= (1 << PA1);
                PORTA &= ~(1 << PA0);
            }
            if (rm >= 0)
            {
                PORTA |= (1 << PA2);
                PORTA &= ~(1 << PA3);
            }
            else
            {
                PORTA |= (1 << PA3);
                PORTA &= ~(1 << PA2);
            }

            if (servo_on)
            {
                OCR3A = 2000;
            }
            else
            {
                OCR3A = 1000;
            }
        }
        else if (mode0 == 1)
        {
            if (front_range > (front_stop + 5) && (left_range - right_range) < 5)
            {
                OCR1A = 1000;
                OCR1B = 1000;
                PORTA |= (1 << PA0) | (1 << PA2);
                PORTA &= ~((1 << PA1) | (1 << PA3));
            }
            else if (front_range <= (front_stop + 5))
            {
                if (left_range > right_range)
                {
                    OCR1A = 0;
                    OCR1B = 1000;
                    PORTA |= (1 << PA2);
                    PORTA &= ~((1 << PA1) | (1 << PA3) | (1 << PA0));
                }
                else
                {
                    OCR1A = 1000;
                    OCR1B = 0;
                    PORTA |= (1 << PA0);
                    PORTA &= ~((1 << PA1) | (1 << PA3) | (1 << PA2));
                }
            }
        }
        else if (mode0 == 2)
        {
        }
         raw_front = adc_read(0);
        if (raw_front > 20)
        {
            front_range = 4800 / (raw_front - 20);
        }
        else
        {
            front_range = 255;
        }

        raw_left = adc_read(2);
        if (raw_left > 11)
        {
            left_range = 2076 / (raw_left - 11);
        }
        else
        {
            left_range = 255;
        }

        raw_right = adc_read(3);
        if (raw_right > 11)
        {
            right_range = 2076 / (raw_right - 11);
        }
        else
        {
            right_range = 255;
        }

        // Send data and read sensors at 100ms
        current_ms = milliseconds_now();
        if ((current_ms - last_send_ms) >= 100)
        {
            battery_voltage = (uint8_t)(adc_read(7) / 5);
            uint8_t freq_byte = (uint8_t)stable_frequency;
            uint8_t range_byte = (uint8_t)(front_range > 255 ? 255 : front_range);
            serial2_write_bytes(3, battery_voltage, freq_byte, range_byte);
            sprintf(serial_string, "Range: %u cm Left: %u cm Right: %u cm\n", front_range, left_range, right_range);
            serial0_print_string(serial_string);
            last_send_ms = current_ms;
        }
    }
}