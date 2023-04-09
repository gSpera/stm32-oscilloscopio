#include "delay.h"

#include <stm32f30x.h>

void delay_init() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->CR1 |= TIM_CR1_OPM | TIM_CR1_ARPE;
}

void delay_ms(unsigned int ms) {
    TIM6->CR1 &= ~(TIM_CR1_CEN);
    TIM6->ARR = ms * 10;
    TIM6->CNT = 0;

    // We want a frequency of 10_000hz, period of 0.1ms
    TIM6->PSC = 7200 - 1;

    TIM6->CR1 |= TIM_CR1_CEN;

    while (TIM6->SR == 0)
        ;
    // Clear UIF
    TIM6->SR = 0;
}

void delay_us(unsigned int us) {
    TIM6->CR1 &= ~(TIM_CR1_CEN);
    TIM6->ARR = us;
    TIM6->CNT = 0;

    // We want a frequency of 1_000_000, period of 1us
    TIM6->PSC = 72 -1;
    TIM6->CR1 |= TIM_CR1_CEN;

    while (TIM6->SR == 0)
        ;
    // Clear UIF
    TIM6->SR = 0;
}