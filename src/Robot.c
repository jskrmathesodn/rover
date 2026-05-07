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
    uint16_t front_range = 0;
    uint16_t raw_read = 0;
    static int16_t lm = 0;
    static int16_t rm = 0;
    static int16_t fc = 0;
    static int16_t rc = 0;
    char serial_string[60];
    uint8_t recievedData[4] = {0, 0, 0, 0};

    static uint8_t last_ldr_state = 0;
    static uint32_t last_rise_ms = 0;
    static uint8_t frequency = 0;
    uint16_t ldr_threshold = 160;

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
        // LDR frequency detection
        uint16_t ldr = adc_read(1);
        uint8_t current_ldr_state;
        if (ldr < ldr_threshold)
        {
            current_ldr_state = 1;
        }
        else
        {
            current_ldr_state = 0;
        }

        if (current_ldr_state == 1 && last_ldr_state == 0)
        {
            uint32_t now = milliseconds_now();
            uint32_t period_ms = now - last_rise_ms;
            if (period_ms > 5 && period_ms < 1100)
            {
                frequency = (uint8_t)(2000 / period_ms);
            }
            last_rise_ms = now;
        }
        last_ldr_state = current_ldr_state;

        // Reset frequency if no edge detected for 500ms
        uint32_t now = milliseconds_now();
        if ((now - last_rise_ms) > 500)
        {
            frequency = 0;
        }

        // Range sensor read
        raw_read = adc_read(0);
        if (raw_read > 20)
        {
            front_range = 4800 / (raw_read - 20);
        }
        else
        {
            front_range = 255;
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
        if (mode == 0)
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
        else if (mode == 1)
        {
            if (front_range > 10)
            {
                OCR1A = 1000;
                OCR1B = 1000;
                PORTA |= (1 << PA0) | (1 << PA2);
                PORTA &= ~((1 << PA1) | (1 << PA3));
            }
            else
            {
                OCR1A = 0;
                OCR1B = 0;
            }
        }
        else if (mode == 2)
        {
        }

        // Send data to controller
        current_ms = milliseconds_now();
        if ((current_ms - last_send_ms) >= 100)
        {
            battery_voltage = (uint8_t)(adc_read(7) / 5);
            uint8_t freq_byte = (uint8_t)frequency;
            uint8_t range_byte = (uint8_t)(front_range > 255 ? 255 : front_range);
            serial2_write_bytes(3, battery_voltage, freq_byte, range_byte);
            sprintf(serial_string, "Mode: %u Freq: %u Hz Bat: %u Range: %u cm\n", mode, frequency, battery_voltage, front_range);
            serial0_print_string(serial_string);
            last_send_ms = current_ms;
        }
    }
}
