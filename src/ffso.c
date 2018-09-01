/*
 * Generates two PWM-encoded sine waveforms with 90-degree phase shift
 * on OC0A and OC0B. These signals are used to drive a synchronous
 * motor of a turntable.
 * Sine frequency may be changed between two hard-coded setting (50Hz,
 * 67.5Hz) via INT0; change is only done when the current period of
 * signal is finished (see comments near *sintab_next). Due to the
 * low frequency of the signals, this also solves the debouncing of INT0.
 * "sintabs.inc" stores sine cache tables for these two frequencies. It is
 * assumed that PWM frequency will be 62500Hz (F_CPU=16MHz); to use another
 * frequency, update util/generate-sintabs.py and pipe its output
 * to src/sintabs.inc.
 *
 * I/O map:
 * Pin	Direction	Description
 * --------------------------------------------------------------------------
 * B0	Output		OC0A - PWM signal #1
 * B1	Output		OC0B - PWM signal #2 (90 degrees shift)
 * B2	Input		INT0 - Select Frequency (high - 50 Hz; low - 67.5 Hz)
 * B5	Input		!RESET
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include "sintabs.inc"

static volatile uint8_t *sintab;
static volatile uint16_t sintab_len;

/*
 * Select the current sine table to use based on frequency select pin's state
 */
static void select_sintab(void) {
    if (PINB & (1 << PB2)) {
        sintab = (uint8_t *) &sintab_50;
        sintab_len = SINTAB_50_LEN;
    } else {
        sintab = (uint8_t *) &sintab_67_5;
        sintab_len = SINTAB_67_5_LEN;
    }
}

/*
 * Tick interrupt (every 256clk):
 * - set new sample values for PWM outputs
 * - change current frequency as needed
 */
ISR(SIG_OVERFLOW0) {
    static uint16_t phi = 0; /* index on current sintab */

    if (++phi == sintab_len) {
    	phi = 0;
        select_sintab();
    }
    OCR0A = pgm_read_byte(sintab + phi);
    OCR0B = pgm_read_byte((sintab + phi + (sintab_len / 4) % sintab_len));
}

int main(void) {
    /* Enable pull-up resistors on PB5 (!RESET) and PB2 (INT0) */
    PORTB |= (1 << DDB5)
           | (1 << DDB2);

    select_sintab();

    /* Configure PWM outputs */
    DDRB = (1 << DDB1)
         | (1 << DDB0);
    TCCR0A = (1 << COM0A1)
           | (1 << COM0B1)
           | (1 << WGM01)
           | (1 << WGM00);  /* Timer0: A&B non-inverting fast pwm */
    TCCR0B = (1 << CS00);   /* Timer0: no prescale for clock */
    TIMSK |= (1 << TOIE0);  /* Timer0: overflow interrupt enable */

    /* Enable sleep; sleep mode is idle */
    MCUCR |= (1 << SE);
    MCUCR &= ~(1 << SM1);

    sei();

    /* Sleep forever, everything's handled by interrupts */
    while (1) __asm__ __volatile__ ("sleep");
    return 42;
}
