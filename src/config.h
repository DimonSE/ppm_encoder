#ifndef __CONFIG_H__
#define __CONFIG_H__

// Dead-time between each channel in the PPM-stream. (in microseconds)
const uint16_t DEAD_TIME = 300 * TIMER_SCALE;

// Minimal and maximal value of channel (in microseconds)
const uint16_t CHANNEL_VALUE_MIN = 900  * TIMER_SCALE;
const uint16_t CHANNEL_VALUE_MAX = 2100 * TIMER_SCALE;

// Start value of channel (in microseconds)
const uint16_t CHANNEL_VALUE_INIT = 1200 * TIMER_SCALE;

// Number of PPM channels out. [1, PWM_CHANNEL_MAX] channels supported (both included).
#define PWM_NUMBER_OF_CHANNELS 6

// Sensitivity on small changes in PWM value
const uint16_t TRESHHOLD = 100;

// Set frame-length depending on channels (in microseconds)
const uint16_t FRAME_TOTAL_LENGTH = 18400U * TIMER_SCALE;

// Count of average filtration steps
const byte FILTER_DEPTH = 7;

//
const uint16_t PWM_LOST_TIME = 50 * MICROS_IN_MSEC * MSEC_IN_SEC * TIMER_SCALE;

#endif // __CONFIG_H__