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
    uint16_t ldr1 = 0;
    uint16_t ldr2 = 0;
    uint16_t ldr_max1 = 512;
    uint16_t ldr_min1 = 512;
    uint16_t ldr_max2 = 512;
    uint16_t ldr_min2 = 512;
    uint16_t frequency1 = 0;
    uint32_t period1 = 0;
    uint32_t period1_history[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t period2_history[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t pi1 = 0;
    uint8_t pi2 = 0;
    uint32_t last_toggle1 = 0;
    uint16_t frequency2 = 0;
    uint32_t period2 = 0;
    uint32_t last_toggle2 = 0;
    uint16_t ldr_history1[10] = {512, 512, 512, 512, 512, 512, 512, 512, 512, 512};
    uint16_t ldr_history2[10] = {512, 512, 512, 512, 512, 512, 512, 512, 512, 512};
    uint8_t i = 0;
    uint8_t check = 0;
    uint16_t sensitivity = 30;

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
        ldr1 = adc_read(1);
        ldr2 = adc_read(4);

        // LDR1 frequency detection with averaging
        if (ldr1 > (ldr_min1 + sensitivity))
        {
            uint32_t now1 = milliseconds_now();
            period1 = now1 - last_toggle1;
            last_toggle1 = now1;
            if (period1 > 40 && period1 < 600)
            {
                period1_history[pi1] = period1;
                pi1 = (pi1 + 1) % 10;
                uint32_t sum1 = 0;
                uint8_t count1 = 0;
                for (uint8_t k = 0; k < 10; k++)
                {
                    if (period1_history[k] > 0)
                    {
                        sum1 += period1_history[k];
                        count1++;
                    }
                }
                if (count1 > 0)
                {
                    frequency1 = (uint16_t)(1000 / (sum1 / count1));
                }
            }
        }

        if ((milliseconds_now() - last_toggle1) > 600)
        {
            frequency1 = 0;
        }

        // LDR2 frequency detection with averaging
        if (ldr2 > (ldr_min2 + sensitivity))
        {
            uint32_t now2 = milliseconds_now();
            period2 = now2 - last_toggle2;
            last_toggle2 = now2;
            if (period2 > 40 && period2 < 600)
            {
                period2_history[pi2] = period2;
                pi2 = (pi2 + 1) % 10;
                uint32_t sum2 = 0;
                uint8_t count2 = 0;
                for (uint8_t k = 0; k < 10; k++)
                {
                    if (period2_history[k] > 0)
                    {
                        sum2 += period2_history[k];
                        count2++;
                    }
                }
                if (count2 > 0)
                {
                    frequency2 = (uint16_t)(1000 / (sum2 / count2));
                }
            }
        }

        if ((milliseconds_now() - last_toggle2) > 600)
        {
            frequency2 = 0;
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
                OCR3A = 1000;
            }
            else
            {
                OCR3A = 3000;
            }
        }
        else if (mode == 1)
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
            else if (front_range > (front_stop + 5) && (left_range - right_range) > 5 && left_range > right_range)
            {
                OCR1A = 900;
                OCR1B = 1000;
                PORTA |= (1 << PA0) | (1 << PA2);
                PORTA &= ~((1 << PA1) | (1 << PA3));
            }
            else if (front_range > (front_stop + 5) && (left_range - right_range) > 5 && left_range < right_range)
            {
                OCR1A = 1000;
                OCR1B = 900;
                PORTA |= (1 << PA0) | (1 << PA2);
                PORTA &= ~((1 << PA1) | (1 << PA3));
            }
        }
        else if (mode == 2)
        {
            // Stop if front obstacle detected
            if (front_range < 10)
            {
                OCR1A = 0;
                OCR1B = 0;
                check = 0;
            }
            // Beacon detected and centred — drive forward
            else if (frequency1 > 0 && frequency2 > 0)
            {
                OCR1A = 1000;
                OCR1B = 1000;
                PORTA |= (1 << PA0) | (1 << PA2);
                PORTA &= ~((1 << PA1) | (1 << PA3));
                check = 3;
            }
            // Beacon detected on LDR1 — turn toward LDR1
            else if (frequency1 > 0 && frequency2 == 0)
            {
                OCR1A = 0;
                OCR1B = 1000;
                PORTA |= (1 << PA2);
                PORTA &= ~((1 << PA1) | (1 << PA3) | (1 << PA0));
                check = 1;
            }
            // Beacon detected on LDR2 — turn toward LDR2
            else if (frequency2 > 0 && frequency1 == 0)
            {
                OCR1A = 1000;
                OCR1B = 0;
                PORTA |= (1 << PA0);
                PORTA &= ~((1 << PA1) | (1 << PA3) | (1 << PA2));
                check = 2;
            }
            // No beacon detected — slow scan right to search
            else
            {
                OCR1A = 900;
                OCR1B = 0;
                PORTA |= (1 << PA0);
                PORTA &= ~((1 << PA1) | (1 << PA3) | (1 << PA2));
                check = 4;
            }
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

        // Send data at 100ms
        current_ms = milliseconds_now();
        if ((current_ms - last_send_ms) >= 100)
        {
            ldr_history1[i] = ldr1;
            ldr_history2[i] = ldr2;
            i = (i + 1) % 10;

            ldr_max1 = 0;
            ldr_max2 = 0;
            ldr_min1 = 1023;
            ldr_min2 = 1023;
            for (uint8_t j = 0; j < 10; j++)
            {
                if (ldr_history1[j] > ldr_max1)
                {
                    ldr_max1 = ldr_history1[j];
                }
                if (ldr_history1[j] < ldr_min1)
                {
                    ldr_min1 = ldr_history1[j];
                }
                if (ldr_history2[j] > ldr_max2)
                {
                    ldr_max2 = ldr_history2[j];
                }
                if (ldr_history2[j] < ldr_min2)
                {
                    ldr_min2 = ldr_history2[j];
                }
            }

            battery_voltage = (uint8_t)(adc_read(7) / 5);
            uint8_t freq_byte = (uint8_t)frequency1;
            uint8_t range_byte = (uint8_t)(front_range > 255 ? 255 : front_range);
            serial2_write_bytes(3, battery_voltage, freq_byte, range_byte);
            sprintf(serial_string, "LDR1: %u LDR2: %u Max1: %u Min1: %u F1: %u F2: %u Chk: %u\n", ldr1, ldr2, ldr_max1, ldr_min1, frequency1, frequency2, check);
            serial0_print_string(serial_string);
            last_send_ms = current_ms;
        }
    }
}