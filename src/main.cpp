#include <avr/io.h>
#include <avr/interrupt.h>

#include "utils.h"


#define TIMER_SCALE 2

// Dead-time between each channel in the PPM-stream. (in microseconds)
#define DEAD_TIME 150 * TIMER_SCALE

// Minimal and maximal value of channel (in microseconds)
#define CHANNEL_VALUE_MIN 850  * TIMER_SCALE
#define CHANNEL_VALUE_MAX 2200 * TIMER_SCALE

// Start value of channel (in microseconds)
#define CHANNEL_VALUE_INIT 1200 * TIMER_SCALE

// Maximal count of channels
#define CHANNEL_MAX 8

// Number of PPM channels out. [1, CHANNEL_MAX] channels supported (both incl).
#define NUMBER_OF_CHANNELS 6
#define TRESHHOLD 100

// Set frame-length depending on channels (in microseconds)
#define FRAME_TOTAL_LENGTH 18400 * TIMER_SCALE
#define FILTER_DEPTH 7

#define LED_PORT PORTB
#define LED_PIN  PB5
#define LED_DDR  DDRB

uint32_t timer_switches[CHANNEL_MAX] = { 0, 0, 0, 0, 0, 0, 0, 0 };
uint16_t values[CHANNEL_MAX] =
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
uint16_t filter_array[FILTER_DEPTH][8];
uint8_t  filterIdx[FILTER_DEPTH];

volatile uint16_t tcount = 0;

void init()
{
  for (byte i = 0; i < FILTER_DEPTH; ++i)
  {
    filterIdx[i] = 0;
    for (byte j = 0; j < CHANNEL_MAX; ++j)
      filter_array[i][j] = CHANNEL_VALUE_INIT;
  }



#define PINS_SETUP_D ((1<<2) | (1<<4) | (1<<5) | (1<<6) | (1<<7));
  DDRD   &= ~PINS_SETUP_D;
  PCMSK2 |=  PINS_SETUP_D;
  PORTD   =  PINS_SETUP_D;
  PCICR  |= (1<<PCIE2);


#define PINS_SETUP_B ((1 << 0) | (1 << 3) | (1<<4))
  DDRB   &= ~PINS_SETUP_B;
  PCMSK0 |=  PINS_SETUP_B;
  PORTB   =  PINS_SETUP_B;
  PCICR  |= (1<<PCIE0);

  TCCR1A =
    (0<<WGM10)  |
    (0<<WGM11)  |
    (0<<COM1A1) |
    (1<<COM1A0) | // Toggle pin om compare-match
    (0<<COM1B1) |
    (0<<COM1B0);

  TCCR1B =
    (0<<ICNC1) | //
    (0<<ICES1) | //
    (0<<CS10)  | //Prescale 8
    (1<<CS11)  | //Prescale 8
    (0<<CS12)  | //Prescale 8
    (0<<WGM13) |
    (1<<WGM12); // CTC mode (Clear timer on compare match) with OCR1A as top.

  TIMSK1 =
    (1<<OCIE1A) | // Interrupt on compare A
    (0<<OCIE1B) | // Disable interrupt on compare B
    (0<<TOIE1);

  DDRB |= (( 1 << 1 ) | ( 1 << 2));

  sbi(LED_DDR, LED_PIN);

  OCR1A = FRAME_TOTAL_LENGTH;

  TCCR2A = 0;
  TCCR2B = (1<<CS21);  // presacle 8
  TCNT2  = 0;

  TIMSK2 = (1 << TOIE2);

  sei();
}

int main()
{
  init();

  while (true)
  {
    // Nothing to do
  }
}


// ============================================
// Read PWM
// ============================================



#define DIFF_OVFL(v1, v2) (v1 - v2) & 0xFFFF

#define NOW ( (((uint32_t)tcount) << 8) | TCNT2 )

#define TIME_DIFF(idx) ( (timer_switches[idx] == 0) ? CHANNEL_VALUE_INIT : DIFF_OVFL(NOW, timer_switches[idx]) )

inline uint16_t CHECKVAL(uint32_t val, uint32_t prev)
{
  return (val > CHANNEL_VALUE_MIN && val < CHANNEL_VALUE_MAX) ? val : prev;
}

inline void checkPortPins(uint8_t pin, uint8_t channel, uint8_t *state, uint8_t pins)
{
  const uint8_t diff = pins ^ *state;
  const uint8_t mask = 1 << pin;

  if (mask & diff)
  {
    if (pins & mask)
    {
      timer_switches[channel] = NOW;
      sbi(*state, pin);
    }
    else
    {
      filter_array[filterIdx[channel]][channel] = CHECKVAL(TIME_DIFF(channel), values[channel]);

      ++filterIdx[channel];
      if (filterIdx[channel] == FILTER_DEPTH)
        filterIdx[channel] = 0;

      cbi(*state, pin);
    }
  }
}

ISR(PCINT0_vect)
{
  static byte state = 0;

  sbi(LED_PORT, LED_PIN);

  checkPortPins(0, 5, &state, PINB);
  checkPortPins(3, 6, &state, PINB);
  checkPortPins(4, 7, &state, PINB);

  cbi(LED_PORT, LED_PIN);
}

ISR(PCINT2_vect)
{
  static byte state = 0;

  sbi(LED_PORT, LED_PIN);

  checkPortPins(2, 0, &state, PIND);
  checkPortPins(4, 1, &state, PIND);
  checkPortPins(5, 2, &state, PIND);
  checkPortPins(6, 3, &state, PIND);
  checkPortPins(7, 4, &state, PIND);

  cbi(LED_PORT, LED_PIN);
}

ISR(TIMER2_OVF_vect)
{
  tcount++;
}

// ========================================
// Generate PPM
// ========================================

inline uint16_t getChannel(uint8_t ch, int16_t vv)
{
  int32_t v = 0;
  for (uint8_t i = 0; i < FILTER_DEPTH; i++)
    v += filter_array[i][ch];
  v = v / FILTER_DEPTH;

  return (abs(v - vv) > TRESHHOLD) ? v : vv;
}

ISR(TIMER1_COMPA_vect)
{
  static bool     onsync         = true;
  static bool     ondelay        = false;
  static uint32_t frame_spent    = 0;
  static uint8_t  channel_number = 0;

  if (onsync)
  {
    TCCR1A =
      (0<<WGM10)  |
      (0<<WGM11)  |
      (1<<COM1A1) |
      (1<<COM1A0) |
      (0<<COM1B1) |
      (0<<COM1B0);

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
      // After first time, when pin have been set hgih, we toggle the pin at each interrupt
      TCCR1A =
        (0<<WGM10)  |
        (0<<WGM11)  |
        (0<<COM1A1) |
        (1<<COM1A0) |
        (0<<COM1B1) |
        (0<<COM1B0);
    }
    if(channel_number < NUMBER_OF_CHANNELS)
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
