#include <stm32f30x.h>

#include "delay.h"
#include "tft.h"
#include "usart.h"
#include "gfx.h"
#include "tft-dma.h"

extern const unsigned int DATA_START;
extern const unsigned int DATA_LEN;
extern const unsigned int DATA_LOAD_ADDR;

// GPIOB,GPIOD TFT
// GPIOC USART
// TIM6 DELAY
// TIM4, DMA1CH4,DMA1CH5 TFT-DMA

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

    gfx_draw_button(100, 100, 30, 90, 5, GFX_GRAY(15), GFX_BLACK);

    gfx_draw_text(0, 0, GFX_RED, GFX_GREEN, "This is at 0");

    usart_puts("Finished drawing\n");

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

    while (1)
        ;
}
