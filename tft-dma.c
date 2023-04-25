#include <stm32f30x.h>

#include "tft-dma.h"

uint8_t tftdma_buffer_data[TFTDMA_BUFFER_SIZE];
uint8_t tftdma_buffer_rs[TFTDMA_BUFFER_SIZE];

void tftdma_init() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER9);
    GPIOB->MODER |= GPIO_MODER_MODER9_1;
    GPIOB->AFR[1] = 1<<5; // GPIOB P9 Alternate Function 2 (TIM4 Ch4)
    GPIOE->MODER = GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0 |
                   GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0;

    // DMA1 Ch4 (TIM4 Ch2) -> TFT RS
    // DMA1 Ch5 (TIM4 Ch3) -> TFT D0-D7

    // Channel 4 - RS
    // 8bit Memory -> 8bit Peripheral, Memory Increment
    DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_DIR;
    DMA1_Channel4->CPAR = ((uintptr_t) &GPIOB->ODR)+1;
    DMA1_Channel4->CMAR = (uintptr_t) tftdma_buffer_rs;
    DMA1_Channel4->CNDTR = 1000;
    DMA1_Channel4->CCR |= DMA_CCR_EN;

    // Channel 5 - Data
    // 8bit Memory -> 8bit Peripheral, Memory Increment
    DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_DIR;
    DMA1_Channel5->CPAR = ((uintptr_t) &GPIOD->ODR);
    DMA1_Channel5->CMAR = (uintptr_t) tftdma_buffer_data;
    DMA1_Channel5->CNDTR = 1000;
    // DMA1_Channel5->CCR |= DMA_CCR_EN;

    // Timer
    TIM4->DIER = TIM_DIER_CC3DE | TIM_DIER_CC2DE;
    // CCMR2 = PWM (Ch 4) MODE 2
    TIM4->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0;
    TIM4->CCER = TIM_CCER_CC4E;
    TIM4->CCR3 = 0;
    TIM4->CCR2 = 0;
    TIM4->CCR4 = 5000; // 50% Duty cycle
    TIM4->PSC = 7200;
    TIM4->ARR = 10000;
    TIM4->CR1 = TIM_CR1_CEN;
}