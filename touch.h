#ifndef H_TOUCH
#define H_TOUCH

#include <stdint.h>
#include <stdbool.h>

#define TS_LEFT   10000
#define TS_RIGHT  60000
#define TS_TOP    10000
#define TS_BOT    60000
#define TS_THRESH 20000
#define BUTTON_SIZE 1024
#define TOUCH_BTN_RADIOUS 1000

typedef void (*ButtonCallback)();

typedef struct {
    uint8_t x;
    uint8_t y;
    ButtonCallback fn; 
} Button;


// Le definizioni dei pulsanti è salvata nella flash, read-only
extern const Button touch_btns[BUTTON_SIZE];
extern uint8_t  touch_btns_enabled[BUTTON_SIZE/8];

void touch_init();
void touch_enable();
void touch_disable();
bool touch_is_detected();
uint16_t touch_read_x();
uint16_t touch_read_y();
uint16_t touch_read_z();

void touch_btn_press(int x, int y);
void touch_btn_enable(int index);
void touch_btn_disable(int index);
int touch_btn_is_enabled(int index);

#endif