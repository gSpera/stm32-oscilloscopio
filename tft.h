#ifndef H_TFT
#define H_TFT

#include <stdbool.h>
#include <stdint.h>

#define TFT_RS_CMD 0
#define TFT_RS_DAT 1

void tft_init();
void tft_write8(uint8_t value, bool rs);
void tft_write16(uint16_t value, bool rs);
uint8_t tft_read8();
uint16_t tft_read16();

#endif
