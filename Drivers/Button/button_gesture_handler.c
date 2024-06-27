#include "button_gesture_handler.h"
#include "utils.h"
#include "button.h"

static const ButtonGestureType next_gesture_map[] = {SINGLE_PRESS, DOUBLE_PRESS,
		TRIPLE_PRESS, SINGLE_PRESS};

static button_gesture_callback callback_map[NUMBER_OF_GESTURES] = {0};

static uint8_t current_button_index = 0;

static const uint64_t button_debouncing_timeout_in_milliseconds = 50;
static const uint64_t button_consecutive_press_timeout_in_milliseconds = 300;
static const uint64_t button_long_press_limit_in_milliseconds = 3000;
static uint64_t button_press_time_in_milliseconds = 0;
static uint64_t button_release_time_in_milliseconds = 0;

static uint64_t last_event_time_in_milliseconds = 0;
static uint8_t event_consumed = 0;
static uint8_t long_press_occured = 0;

static ButtonGestureType current_gesture = INITIAL;
static ButtonGestureType waiting_gesture = INITIAL;

static void button_release_cb();
static void button_press_cb();

static int check_bouncing_timeout(int current_time_in_milliseconds)
{
	if(	current_time_in_milliseconds - button_release_time_in_milliseconds >
	     button_debouncing_timeout_in_milliseconds &&
		current_time_in_milliseconds - button_press_time_in_milliseconds >
	     button_debouncing_timeout_in_milliseconds)
	{
        return 1;
	}
	else
	{
		return 0;
	}
}

static int check_consecutive_press_timeout(int current_time_in_milliseconds)
{
	if(	current_time_in_milliseconds - last_event_time_in_milliseconds
			> button_consecutive_press_timeout_in_milliseconds)
	{
        return 1;
	}
	else
	{
		return 0;
	}
}

static void button_release_cb()
{
	const uint64_t current_time = utils_get_time_ms();

	if(check_bouncing_timeout(current_time))
    {
    	button_release_time_in_milliseconds = current_time;
    	event_consumed = 1;

    	if(current_time - button_press_time_in_milliseconds >
    		button_long_press_limit_in_milliseconds)
    	{
    		long_press_occured = 1;
    	}

    	button_register_cb(current_button_index, button_press_cb, BUTTON_FALLING_EDGE_INT);
    }
}

static void button_press_cb()
{
    const uint64_t current_time = utils_get_time_ms();

    if(check_bouncing_timeout(current_time))
    {
    	button_press_time_in_milliseconds = current_time;
        button_register_cb(current_button_index, button_release_cb, BUTTON_RISING_EDGE_INT);
    }
}

static ButtonGestureType check_gesture_status(const int current_time_in_milliseconds)
{
	if(event_consumed == 1)
	{
		event_consumed = 0;

		if(long_press_occured == 0)
		{
			if(check_consecutive_press_timeout(current_time_in_milliseconds))
			{
				current_gesture = INITIAL;
			}

			last_event_time_in_milliseconds = current_time_in_milliseconds;

			current_gesture = next_gesture_map[current_gesture];

			return current_gesture;
		}
		else
		{
			long_press_occured = 0;
			current_gesture = INITIAL;
			return SINGLE_LONG_PRESS;
		}
	}
	else
	{
	    return INITIAL;
	}
}

void button_gesture_handler_init(const uint8_t button_index)
{
	current_button_index = button_index;
	button_register_cb(current_button_index, button_press_cb, BUTTON_FALLING_EDGE_INT);
}


void button_gesture_handler_save_callback(ButtonGestureType gesture, button_gesture_callback callback)
{
    if(gesture < NUMBER_OF_GESTURES && gesture > INITIAL)
    {
    	callback_map[gesture] = callback;
    }
}

void button_gesture_handler_iterate()
{
	const uint64_t current_time = utils_get_time_ms();
	ButtonGestureType reported_gesture = check_gesture_status(current_time);

	if (waiting_gesture != INITIAL && check_consecutive_press_timeout(current_time))
	{
		callback_map[waiting_gesture]();
		waiting_gesture = INITIAL;
	}

	if(reported_gesture == SINGLE_PRESS || reported_gesture == DOUBLE_PRESS)
	{
		waiting_gesture = reported_gesture;
	}
	else if(reported_gesture != INITIAL)
	{
		 if (callback_map[reported_gesture] != 0)
		 {
			 callback_map[reported_gesture]();
		 }

		 waiting_gesture = INITIAL;
	}
}


