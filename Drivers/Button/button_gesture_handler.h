#ifndef APPLICATION_BUTTONGESTUREHANDLER_H_
#define APPLICATION_BUTTONGESTUREHANDLER_H_

#include <stdint.h>

typedef enum ButtonGestureType
{
	INITIAL = 0,
	SINGLE_PRESS,
	DOUBLE_PRESS,
	TRIPLE_PRESS,
	SINGLE_LONG_PRESS,
	SINGLE_LONG_PRESS_RELEASED,
	NUMBER_OF_GESTURES,
} ButtonGestureType;


typedef void(*button_gesture_callback)(void);

void button_gesture_handler_init(const uint8_t button_index);
void button_gesture_handler_save_callback(ButtonGestureType gesture, button_gesture_callback callback);
void button_gesture_handler_iterate();


#endif /* APPLICATION_BUTTONGESTUREHANDLER_H_ */
