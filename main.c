#include <stm32f30x.h>

#include "delay.h"
#include "tft.h"
#include "usart.h"
#include "gfx.h"
#include "tft-dma.h"
#include "touch.h"

extern const unsigned int DATA_START;
extern const unsigned int DATA_LEN;
extern const unsigned int DATA_LOAD_ADDR;

// GPIOB, GPIOD TFT
// GPIOC USART
// TIM6 DELAY
// TIM4, DMA1CH4, DMA1CH5 TFT-DMA
// GPIOB, GPIOD, ADC3 TOUCH
// ADC1 OSCILLOSCOPE

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
    touch_init();
    osc_init();

    // test_touch();
    // while(1);

    tft_write16(0x0, TFT_RS_CMD);
    usart_puthex(tft_read16(TFT_RS_CMD));
    usart_putc('\n');

    usart_puts("Oscilloscopio 2000\n");

    tft_init();
    usart_puts("TFT Init done\n");
    usart_puthex(SystemCoreClock);
    usart_putc('\n');

    tft_write16(0x3, TFT_RS_CMD);
    tft_write16((1<<12), TFT_RS_DAT); // BGR

    gfx_fill_rect(0, 0, 240, 320, GFX_BLACK);
    gfx_fill_rect(50, 50, 240-100, 320-100, GFX_RED);
    gfx_draw_text_hor(60, 60, GFX_WHITE, GFX_RED, "Oscilloscopio");
    gfx_draw_text_scaled_hor(90, 90, GFX_WHITE, GFX_RED, 3, "2000");

    delay_ms(500);

    // gfx_fill_rect(0, 0, 240, 320, GFX_GRAY(16));

    gfx_fill_rect(0, 0, 240, 320, gfx_color(31, 0, 31));
    gfx_fill_rect(0, 0, 240, 320, gfx_color(31, 10, 31));

    gfx_fill_rect(50, 50, 50, 50, gfx_color(31, 0, 0));
    gfx_fill_rect(100, 50, 50, 50, gfx_color(0, 31, 0));
    gfx_fill_rect(150, 50, 50, 50, gfx_color(0, 0, 31));

    gfx_draw_button(100, 100, 30, 90, 5, GFX_GRAY(15), GFX_BLACK);

    gfx_draw_text(0, 0, GFX_RED, GFX_GREEN, "This is at 0");

    usart_puts("Finished drawing\n");

    /*
    for (int i=0;i<1024;i+=2) {
        tftdma_buffer_rs[i] = i;
        tftdma_buffer_rs[i+1] = 0xF-i;
    }

    tftdma_init();

    while(1) {
        if (DMA1->ISR & DMA_ISR_TCIF4 != 0) break;
        usart_puthex(TIM4->CNT);
        usart_putc(' ');
        usart_puthex(DMA1_Channel4->CNDTR);
        usart_putc('\n');
    }
    usart_puts("Finished dma");
    */

    usart_puts("Touch screen\n");
    usart_putdec(123);
    usart_putdec(0xFFFF);
    usart_putc('\n');
    touch_init();
    test_touch();
    while(1);

    while(1) {
        touch_enable();
        uint8_t v = touch_read_y();
        usart_puthex(v);
        usart_putc('\n');
        touch_disable();
        delay_ms(100);
    }

    while (1)
        ;
}

void test_touch() {
    usart_puts("Touch screen");
    touch_enable();

    // XP PB0
    // XM PD6
    // YP PB1
    // YM PD7

    while(1) {
        if (touch_is_detected()) {
        uint32_t x=0;
        //for (int i=0;i<10;i++) {
            x += touch_read_x();
        //}
        //v /= 10;

        usart_puts("X:");
        usart_putdec(x);
        usart_puts(" Y:");

        uint32_t y = 0;
        //for (int i=0;i<10;i++) {
            y += touch_read_y();
        //}
        //v /= 10;

        usart_putdec(y);
        usart_puts(" Z:");

        uint32_t z = 0;
        //for (int i=0;i<10;i++) {
            z += touch_read_z();
        //}
        //v /= 10;

        usart_putdec(z);
        usart_putc('\n');
        }
        delay_ms(10);
    }
}
