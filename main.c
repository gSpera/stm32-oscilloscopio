#include <stm32f30x.h>

#include "delay.h"
#include "tft.h"
#include "usart.h"

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

    usart_puts("Oscilloscopio 2000\n");

    tft_init();
    usart_puts("TFT Init done\n");
    usart_puthex(SystemCoreClock);
    usart_putc('\n');

    tft_write16(0x0, TFT_RS_CMD);
    usart_puthex(tft_read16());
    usart_putc('\n');

    while (1)
        ;
}
