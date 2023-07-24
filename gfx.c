#include "gfx.h"
#include "tft.h"
#include "usart.h"

#include "font.h"

GfxColor gfx_color(int red, int green, int blue) {
    if (red > 31) {
        usart_puts("Gfx: Red > 31\n");
        red = 31;
    }
    if (green > 31) {
        usart_puts("Gfx: Green > 31\n");
        green = 31;
    }
    if (blue > 31) {
        usart_puts("Gfx: Blue > 31\n");
        blue = 31;
    }

    red &= 0x1F;
    green &= 0x1F;
    blue &= 0x1F;
    int cyan = 31 - red;
    int magenta = 31 - green;
    int yellow = 31 - blue;

    return (cyan << 11) | (magenta << 6) | yellow;
}

void gfx_set_pixel(int x, int y, GfxColor color) {
    tft_write16(TFT_GRAM_HSET, TFT_RS_CMD);
    tft_write16(x, TFT_RS_DAT);
    tft_write16(TFT_GRAM_VSET, TFT_RS_CMD);
    tft_write16(y, TFT_RS_DAT);
    tft_write16(TFT_WRITE_GRAM, TFT_RS_CMD);
    tft_write16(color, TFT_RS_DAT);
}

void gfx_fill_rect(int x, int y, int w, int h, GfxColor color) {
    tft_write16(TFT_WIN_HSTART, TFT_RS_CMD);
    tft_write16(x, TFT_RS_DAT);
    tft_write16(TFT_WIN_HEND, TFT_RS_CMD);
    tft_write16(x+w-1, TFT_RS_DAT);
    tft_write16(TFT_WIN_VSTART, TFT_RS_CMD);
    tft_write16(y, TFT_RS_DAT);
    tft_write16(TFT_WIN_VEND, TFT_RS_CMD);
    tft_write16(y+h-1, TFT_RS_DAT);

    tft_write16(TFT_WRITE_GRAM, TFT_RS_CMD);
    for (int yi=y;yi<y+h;yi++) {
        for (int xi=x;xi<x+w;xi++) {
            tft_write16(color, TFT_RS_DAT);
        }
    }
}

void gfx_draw_button(int x, int y, int w, int h, int stroke, GfxColor border, GfxColor fill) {
    tft_write16(TFT_WIN_HSTART, TFT_RS_CMD);
    tft_write16(x, TFT_RS_DAT);
    tft_write16(TFT_WIN_HEND, TFT_RS_CMD);
    tft_write16(x+w-1, TFT_RS_DAT);
    tft_write16(TFT_WIN_VSTART, TFT_RS_CMD);
    tft_write16(y, TFT_RS_DAT);
    tft_write16(TFT_WIN_VEND, TFT_RS_CMD);
    tft_write16(y+h-1, TFT_RS_DAT);

    tft_write16(TFT_GRAM_HSET, TFT_RS_CMD);
    tft_write16(x, TFT_RS_DAT);
    tft_write16(TFT_GRAM_VSET, TFT_RS_CMD);
    tft_write16(y, TFT_RS_DAT);

    tft_write16(TFT_WRITE_GRAM, TFT_RS_CMD);
    for (int i=0;i<stroke;i++) {
        for (int xi=0;xi<w;xi++) tft_write16(border, TFT_RS_DAT);
    }

    for (int yi=0;yi<h-2*stroke;yi++) {
        for(int xi=0;xi<stroke;xi++) tft_write16(border, TFT_RS_DAT);

        for (int xi=0;xi<w-2*stroke;xi++) {
            tft_write16(fill, TFT_RS_DAT);
        }
        for(int xi=0;xi<stroke;xi++) tft_write16(border, TFT_RS_DAT);
    }

    for (int i=0;i<stroke;i++) {
        for (int xi=0;xi<w;xi++) tft_write16(border, TFT_RS_DAT);
    }
}

void gfx_draw_char(int x, int y, GfxColor fg, GfxColor bg, int scale, char ch) {
    tft_write16(TFT_WIN_HSTART, TFT_RS_CMD);
    tft_write16(x, TFT_RS_DAT);
    tft_write16(TFT_WIN_HEND, TFT_RS_CMD);
    tft_write16(x+scale*8-1, TFT_RS_DAT);
    tft_write16(TFT_WIN_VSTART, TFT_RS_CMD);
    tft_write16(y, TFT_RS_DAT);
    tft_write16(TFT_WIN_VEND, TFT_RS_CMD);
    tft_write16(y+scale*8-1, TFT_RS_DAT);

    tft_write16(TFT_GRAM_HSET, TFT_RS_CMD);
    tft_write16(x, TFT_RS_DAT);
    tft_write16(TFT_GRAM_VSET, TFT_RS_CMD);
    tft_write16(y, TFT_RS_DAT);

    tft_write16(TFT_WRITE_GRAM, TFT_RS_CMD);
    for (int y=0;y<scale*8;y++) {
        for (int x=0;x<scale*8;x++) {
            int yscaled = y / scale;
            int xscaled = x / scale;

            int row = font8x8_basic[ch][yscaled];
            int bit = row & (1<< (7-xscaled));

            GfxColor c = bg;

            if (bit != 0) {
                c = fg;
            }

            tft_write16(c, TFT_RS_DAT);
        }
    }
}

void gfx_draw_char_hor(int x, int y, GfxColor fg, GfxColor bg, int scale, char ch) {
    tft_write16(TFT_WIN_HSTART, TFT_RS_CMD);
    tft_write16(x, TFT_RS_DAT);
    tft_write16(TFT_WIN_HEND, TFT_RS_CMD);
    tft_write16(x+scale*8-1, TFT_RS_DAT);
    tft_write16(TFT_WIN_VSTART, TFT_RS_CMD);
    tft_write16(y, TFT_RS_DAT);
    tft_write16(TFT_WIN_VEND, TFT_RS_CMD);
    tft_write16(y+scale*8-1, TFT_RS_DAT);

    tft_write16(TFT_GRAM_HSET, TFT_RS_CMD);
    tft_write16(x, TFT_RS_DAT);
    tft_write16(TFT_GRAM_VSET, TFT_RS_CMD);
    tft_write16(y, TFT_RS_DAT);

    tft_write16(TFT_WRITE_GRAM, TFT_RS_CMD);
    for (int x=0;x<scale*8;x++) {
        for (int y=0;y<scale*8;y++) {
            int yscaled = y / scale;
            int xscaled = x / scale;

            int row = font8x8_basic[ch][7-yscaled];
            int bit = row & (1<< (7-xscaled));

            GfxColor c = bg;

            if (bit != 0) {
                c = fg;
            }

            tft_write16(c, TFT_RS_DAT);
        }
    }
}

void gfx_draw_text_scaled(int x, int y, GfxColor fg, GfxColor bg, int scale, const char *text) {
    for (int i=0;text[i]!='\0';i++) {
        gfx_draw_char(x+scale*8*i,y, fg, bg, scale, text[i]);
    }
}
void gfx_draw_text_scaled_hor(int x, int y, GfxColor fg, GfxColor bg, int scale, const char *text) {
    for (int i=0;text[i]!='\0';i++) {
        gfx_draw_char_hor(x, y+scale*8*i, fg, bg, scale, text[i]);
    }
}

void gfx_draw_text(int x, int y, GfxColor fg, GfxColor bg, const char *text) { gfx_draw_text_scaled(x, y, fg, bg, 2, text); }
void gfx_draw_text_hor(int x, int y, GfxColor fg, GfxColor bg, const char *text) { gfx_draw_text_scaled_hor(x, y, fg, bg, 2, text); }
