#ifndef INPUT_EVENTS_H
#define INPUT_EVENTS_H

#include <stdint.h>  //for int32_t

typedef enum {
  EVENT_BUTTON_SELECT,
  EVENT_BUTTON_STOP,
  EVENT_BUTTON_LEFT,
  EVENT_BUTTON_RIGHT,
  EVENT_ENCODER_LEFT,
  EVENT_ENCODER_RIGHT,
} InputEventType_t;

typedef struct {
  InputEventType_t type;
  int value;  //Can be used for delta, e.g. +1/-1 from encoder
} InputEvent_t;

//IO ISR flags/variable values
volatile uint8_t button_select_flag		= 0;
volatile uint8_t button_stop_flag		= 0;
volatile uint8_t button_left_flag		= 0;
volatile uint8_t button_right_flag		= 0;
volatile int encoder_left_delta			= 0;		//CW = +1, CCW = -1
volatile int encoder_right_delta		= 0;
volatile uint8_t lastEncLeftState 		= 0;
volatile uint8_t lastEncRightState 		= 0;

uint8_t currentState, combined;

#endif // INPUT_EVENTS_H
