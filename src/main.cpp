#include <avr/io.h>
#include <avr/interrupt.h>

#include "utils.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

const uint16_t MSEC_IN_SEC    = 1000;
const uint16_t MICROS_IN_MSEC = 1000;

const byte CLOCK_PRESCALER = 8;
const byte TIMER_SCALE = (F_CPU / MSEC_IN_SEC / MICROS_IN_MSEC / CLOCK_PRESCALER);

const byte PWM_CHANNEL_MAX = 8;

#include "config.h"

#define PWM_PORT  PORTD
#define PWM_PINS  PIND
#define PWM_DDR   DDRD
#define PWM_PCMSK PCMSK2
#define PWM_PCIE  PCIE2

#define PPM_DDR DDRB
#define PPM_PIN PB1

#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_PIN  PB0

enum PWM_CHANNEL
{
  PWM_CH0 = 0,
  PWM_CH1,
  PWM_CH2,
  PWM_CH3,
  PWM_CH4,
  PWM_CH5,
  PWM_CH6,
  PWM_CH7
};

uint32_t timer_switches[PWM_CHANNEL_MAX] = { 0, 0, 0, 0, 0, 0, 0, 0 };
uint16_t values[PWM_CHANNEL_MAX] =
{
  CHANNEL_VALUE_INIT,
  CHANNEL_VALUE_INIT,
  CHANNEL_VALUE_INIT,
  CHANNEL_VALUE_INIT,
  CHANNEL_VALUE_INIT,
  CHANNEL_VALUE_INIT,
  CHANNEL_VALUE_INIT,
  CHANNEL_VALUE_INIT
};
uint16_t filter_array[FILTER_DEPTH][PWM_CHANNEL_MAX];
byte     filterIdx[FILTER_DEPTH];

volatile uint16_t timer_counter = 0;
volatile uint32_t PWM_last_update = 0;

volatile bool PWM_haveSignal = false;

void init()
{
  // Init global vars

  for (byte i = 0; i < FILTER_DEPTH; ++i)
  {
    filterIdx[i] = 0;
    for (byte j = 0; j < PWM_CHANNEL_MAX; ++j)
      filter_array[i][j] = CHANNEL_VALUE_INIT;
  }

  // Init LED

  sbi(LED_DDR, LED_PIN);

  // Init PWM input

  PWM_DDR   = 0x00;
  PWM_PORT  = 0xFF;
  PWM_PCMSK = 0xFF;
  sbi(PCICR, PWM_PCIE);

  // Init PPM output

  sbi(PPM_DDR, PPM_PIN);

  TCCR1A =
    (0 << WGM10)  |
    (0 << WGM11)  |
    (0 << COM1A1) |
    (0 << COM1A0) |
    (0 << COM1B1) | //
    (0 << COM1B0);  // By start PPM disabled

  TCCR1B =
    (0 << ICNC1) | //
    (0 << ICES1) | //
    (0 << CS10)  | //
    (1 << CS11)  | // Prescale 8
    (0 << CS12)  | //
    (0 << WGM13) |
    (1 << WGM12);  // CTC mode (Clear timer on compare match) with OCR1A as top.

  TIMSK1 =
    (0 << OCIE1A) | // Disable interrupt on compare A
    (0 << OCIE1B) | // Disable interrupt on compare B
    (0 << TOIE1);

  // Init clock time

  TCCR2A = 0;
  TCCR2B = (1 << CS21);  // Prescale 8
  TCNT2  = 0;

  TIMSK2 = (1 << TOIE2);

  // Start work

  sei();
}

int main()
{
  init();

  while (true) ; // Nothing to do
}


//
// PWM input
//

#define NOW ( (((uint32_t)timer_counter) << 8) | TCNT2 )

inline uint16_t inrange(const uint32_t val, const uint16_t prev)
{
  return (val >= CHANNEL_VALUE_MIN && val <= CHANNEL_VALUE_MAX) ? val : prev;
}

inline void checkPortPins(const uint8_t pins, const uint8_t pin, uint8_t *state, const uint8_t pwm_ch)
{
  const uint8_t diff = pins ^ *state;
  const uint8_t mask = 1U << pin;

  if (mask & diff)
  {
    if (pins & mask)
    {
      timer_switches[pwm_ch] = NOW;
      sbi(*state, pin);
    }
    else
    {
      byte& Idx = filterIdx[pwm_ch];
      filter_array[Idx][pwm_ch] = (timer_switches[pwm_ch] == 0) ?
                                    CHANNEL_VALUE_INIT :
                                    inrange(NOW - timer_switches[pwm_ch], values[pwm_ch]);

      if (++Idx == FILTER_DEPTH)
        Idx = 0;

      cbi(*state, pin);
    }
  }
}

inline void PWM_getSignal()
{
  PWM_last_update = NOW;

  if (PWM_haveSignal)
    return;

  cli();

  PWM_haveSignal  = true;

  sbi(LED_PORT, LED_PIN);

  cbi(TCCR1A, COM1A1); //
  sbi(TCCR1A, COM1A0); // Toggle PPM pin on compare-match
  sbi(TIMSK1, OCIE1A); // Enable interrupt on compare A

  OCR1A = PWM_LOST_TIME;

  sei();
}

inline void PWM_lostSignal()
{
  if (!PWM_haveSignal)
    return;

  PWM_haveSignal = false;

  cbi(LED_PORT, LED_PIN);

  // PPM interrupt will disable in ISR(TIMER1_COMPA_vect) by onsync
}

ISR(PCINT2_vect)
{
  static byte PORT_state = 0;

  checkPortPins(PIND, PD0, &PORT_state, PWM_CH0);
#if PWM_NUMBER_OF_CHANNELS >= 2
  checkPortPins(PWM_PINS, PD1, &PORT_state, PWM_CH1);
#endif
#if PWM_NUMBER_OF_CHANNELS >= 3
  checkPortPins(PWM_PINS, PD2, &PORT_state, PWM_CH2);
#endif
#if PWM_NUMBER_OF_CHANNELS >= 4
  checkPortPins(PWM_PINS, PD3, &PORT_state, PWM_CH3);
#endif
#if PWM_NUMBER_OF_CHANNELS >= 5
  checkPortPins(PWM_PINS, PD4, &PORT_state, PWM_CH4);
#endif
#if PWM_NUMBER_OF_CHANNELS >= 6
  checkPortPins(PWM_PINS, PD5, &PORT_state, PWM_CH5);
#endif
#if PWM_NUMBER_OF_CHANNELS >= 7
  checkPortPins(PWM_PINS, PD6, &PORT_state, PWM_CH6);
#endif
#if PWM_NUMBER_OF_CHANNELS == 8
  checkPortPins(PWM_PINS, PD7, &PORT_state, PWM_CH7);
#endif

  PWM_getSignal();
}

//
// Clock time and check PWM lost
//

ISR(TIMER2_OVF_vect)
{
  timer_counter++;

  if (NOW - PWM_last_update > PWM_LOST_TIME)
    PWM_lostSignal();
}

//
// Generate PPM
//

inline uint16_t getChannel(uint8_t ch, uint16_t vv)
{
  int32_t v = 0;
  for (uint8_t i = 0; i < FILTER_DEPTH; i++)
    v += filter_array[i][ch];
  v = v / FILTER_DEPTH;

  return (abs(v - vv) > TRESHHOLD) ? (uint16_t)v : vv;
}

ISR(TIMER1_COMPA_vect)
{
  static bool     onsync         = true;
  static bool     ondelay        = false;
  static uint32_t frame_spent    = 0;
  static uint8_t  channel_number = 0;

  if (onsync)
  {
    if (!PWM_haveSignal)
    {
      // Don't start new PPM generation, disable interrupt on compare A
      cbi(TCCR1A, COM1A1);
      cbi(TCCR1A, COM1A0);
      cbi(TIMSK1, OCIE1A);
      return;
    }

    sbi(TCCR1A, COM1A1); //
    sbi(TCCR1A, COM1A0); // Set PPM pin on Compare Match (Set output to high level).

    channel_number = 0;
    onsync  = false;
    ondelay = true;

    OCR1A       = DEAD_TIME;
    frame_spent = DEAD_TIME;
  }
  else
  {
    if (channel_number == 0)
    {
      cbi(TCCR1A, COM1A1); //
      sbi(TCCR1A, COM1A0); // After first time, when PPM pin have been set high, we toggle the pin at each interrupt
    }
    if(channel_number < PWM_NUMBER_OF_CHANNELS)
    {
      if(ondelay)
      {
        ondelay = false;

        values[channel_number] = getChannel(channel_number, values[channel_number]);

        OCR1A        = values[channel_number];
        frame_spent += values[channel_number];

        ++channel_number;
      }
      else
      {
        ondelay = true;

        OCR1A        = DEAD_TIME;
        frame_spent += DEAD_TIME;
      }
    }
    else if(!ondelay)
    {
      ondelay = true;

      OCR1A        = DEAD_TIME;
      frame_spent += DEAD_TIME;
    }
    else
    {
      onsync = true;
      OCR1A  = FRAME_TOTAL_LENGTH - frame_spent;
    }
  }
}
