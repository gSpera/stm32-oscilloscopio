#ifndef H_TFT
#define H_TFT

#include <stdbool.h>
#include <stdint.h>

#define TFT_RS_CMD 0
#define TFT_RS_DAT 1

// TFT Commands
#define TFT_WRITE_GRAM 0x22
#define TFT_WIN_HSTART 0x50
#define TFT_WIN_HEND 0x51
#define TFT_WIN_VSTART 0x52
#define TFT_WIN_VEND 0x53
#define TFT_GRAM_HSET 0x20
#define TFT_GRAM_VSET 0x21

void tft_init();
void tft_write8(uint8_t value, bool rs);
void tft_write16(uint16_t value, bool rs);
uint8_t tft_read8(bool rs);
uint16_t tft_read16(bool rs);

#endif
