#ifndef H_GFX
#define H_GFX

#include <stdint.h>

#define GFX_RES_X 320
#define GFX_RES_Y 240

#define GFX_RED gfx_color(31, 0, 0)
#define GFX_GREEN gfx_color(0, 31, 0)
#define GFX_BLUE gfx_color(0, 0, 31)
#define GFX_WHITE gfx_color(0, 0, 0)
#define GFX_BLACK gfx_color(31, 31, 31)
#define GFX_GRAY(X) gfx_color(X, X, X)

typedef uint16_t GfxColor;

GfxColor gfx_color(int red, int green, int blue);
void gfx_set_pixel(int x, int y, GfxColor color);
void gfx_fill_rect(int x, int y, int w, int h, GfxColor color);

void gfx_draw_button(int x, int y, int w, int h, int stroke, GfxColor border, GfxColor fill);
void gfx_draw_text(int x, int y, GfxColor fg, GfxColor bg, const char *text);

#endif