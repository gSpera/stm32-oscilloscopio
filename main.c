#include <stm32f30x.h>

#include "usart.h"
extern const unsigned int DATA_START;
extern const unsigned int DATA_LEN;
extern const unsigned int DATA_LOAD_ADDR;

int main() {
    // Copy .data from flash to ram
    unsigned int *from = (unsigned int *)&DATA_LOAD_ADDR;
    unsigned int *to = (unsigned int *)&DATA_START;
    for (unsigned int i = 0; i < ((unsigned int)&DATA_LEN); i++) {
        to[i] = from[i];
    }

    SystemInit();

    usart_init(9600);
    usart_putc('A');
    usart_puthex(0xABCDEF01);
    usart_putc('B');
    usart_puts("ABC\n");

    while (1)
        ;
}