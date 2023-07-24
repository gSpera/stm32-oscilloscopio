#ifndef H_TOUCH
#define H_TOUCH

#include <stdint.h>
#include <stdbool.h>

#define TS_LEFT   10000
#define TS_RIGHT  60000
#define TS_TOP    10000
#define TS_BOT    60000
#define TS_THRESH 20000

void touch_init();
void touch_enable();
void touch_disable();
bool touch_is_detected();
uint16_t touch_read_x();
uint16_t touch_read_y();
uint16_t touch_read_z();

#endif