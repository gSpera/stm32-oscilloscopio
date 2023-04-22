#include <stm32f30x.h>

#include "dma.h"

void dma_init() {
    RCC->APB1 |= RCC_APB2ENR_TIM3EN;
}