#include <stm32f30x.h>

#include "delay.h"
#include "tft.h"
#include "usart.h"
#include "gfx.h"

extern const unsigned int DATA_START;
extern const unsigned int DATA_LEN;
extern const unsigned int DATA_LOAD_ADDR;

// GPIOB TFT
// GPIOC USART
// GPIOD TFT

int main() {
    // Copy .data from flash to ram
    unsigned int *from = (unsigned int *)&DATA_LOAD_ADDR;
    unsigned int *to = (unsigned int *)&DATA_START;
    for (unsigned int i = 0; i < ((unsigned int)&DATA_LEN); i++) {
        to[i] = from[i];
    }

    SystemInit();
    usart_init(9600);
    delay_init();

    tft_write16(0x0, TFT_RS_CMD);
    usart_puthex(tft_read16(TFT_RS_CMD));
    usart_putc('\n');

    usart_puts("Oscilloscopio 2000\n");

    tft_init();
    usart_puts("TFT Init done\n");
    usart_puthex(SystemCoreClock);
    usart_putc('\n');

    tft_write16(0x3, TFT_RS_CMD);
    tft_write16((1<<12), TFT_RS_DAT);

    gfx_fill_rect(0, 0, 240, 320, gfx_color(31, 0, 31));
    gfx_fill_rect(0, 0, 240, 320, gfx_color(31, 10, 31));

    gfx_fill_rect(50, 50, 50, 50, gfx_color(31, 0, 0));
    gfx_fill_rect(100, 50, 50, 50, gfx_color(0, 31, 0));
    gfx_fill_rect(150, 50, 50, 50, gfx_color(0, 0, 31));

    /*
    tft_write16(TFT_WIN_HSTART, TFT_RS_CMD);
    tft_write16(0, TFT_RS_DAT);
    tft_write16(TFT_WIN_HEND, TFT_RS_CMD);
    tft_write16(239, TFT_RS_DAT);
    tft_write16(TFT_WIN_VSTART, TFT_RS_CMD);
    tft_write16(0, TFT_RS_DAT);
    tft_write16(TFT_WIN_VEND, TFT_RS_CMD);
    tft_write16(319, TFT_RS_DAT);

    tft_write16(TFT_GRAM_HSET, TFT_RS_CMD);
    tft_write16(0, TFT_RS_DAT);
    tft_write16(TFT_GRAM_VSET, TFT_RS_CMD);
    tft_write16(0, TFT_RS_DAT);

    tft_write16(TFT_WRITE_GRAM, TFT_RS_CMD);
    for (int y=0;y<320;y++) {
        for (int x=0;x<240;x++) {
            tft_write16(0xA0, TFT_RS_DAT);
        }
    }

    tft_write16(TFT_WRITE_GRAM, TFT_RS_CMD);
    for (int y=0;y<320;y++) {
        for (int x=0;x<240;x++) {
            tft_write16(0xADAD*y, TFT_RS_DAT);
        }
    }
    gfx_fill_rect(0, 0, 240, 320, gfx_color(31, 0, 31));

    tft_write16(TFT_WIN_HSTART, TFT_RS_CMD);
    tft_write16(10, TFT_RS_DAT);
    tft_write16(TFT_WIN_HEND, TFT_RS_CMD);
    tft_write16(99, TFT_RS_DAT);
    tft_write16(TFT_WIN_VSTART, TFT_RS_CMD);
    tft_write16(10, TFT_RS_DAT);
    tft_write16(TFT_WIN_VEND, TFT_RS_CMD);
    tft_write16(99, TFT_RS_DAT);

    tft_write16(TFT_GRAM_HSET, TFT_RS_CMD);
    tft_write16(10, TFT_RS_DAT);
    tft_write16(TFT_GRAM_VSET, TFT_RS_CMD);
    tft_write16(10, TFT_RS_DAT);

    uint16_t arr[3] = {0xFF, 0xAA, 0xA0F0};
    for (int i=0;i<100;i++) {
        tft_write16(TFT_WRITE_GRAM, TFT_RS_CMD);
        for (int y=10;y<100;y++) {
            for (int x=10;x<100;x++) {
                tft_write16(arr[i % 3], TFT_RS_DAT);
            }
        }
    }
    gfx_fill_rect(20, 20, 50, 50, gfx_color(31, 0, 0));
    gfx_fill_rect(50, 20, 70, 70, gfx_color(31, 31, 0));

    tft_write16(TFT_GRAM_HSET, TFT_RS_CMD);
    tft_write16(200, TFT_RS_DAT);
    tft_write16(TFT_GRAM_VSET, TFT_RS_CMD);
    tft_write16(200, TFT_RS_DAT);
    tft_write16(TFT_WRITE_GRAM, TFT_RS_CMD);
    for (int i=0;i<100;i++) {
        tft_write16(0xAAFF, TFT_RS_DAT);
    }
    */

    gfx_draw_button(100, 100, 30, 90, 5, GFX_GRAY(15), GFX_BLACK);

    gfx_draw_text(0, 0, GFX_RED, GFX_GREEN, "This is at 0");

    /*
    tft_write16(0x0, TFT_RS_CMD);
    usart_puthex(tft_read16(TFT_RS_CMD));
    usart_putc('\n');
    */


    usart_puts("Finished drawing\n");

    while (1)
        ;
}
