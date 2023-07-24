#include "oscilloscope.h"
#include "delay.h"

#include <stm32f30x.h>

void osc_init() {
    RCC->AHBENR |= RCC_AHBENR_ADC12EN | RCC_AHBENR_GPIOCEN;
    ADC1_2->CCR |= ADC34_CCR_CKMODE_0;
    ADC1->CFGR &= ~(ADC_CFGR_RES);
    ADC1->CFGR |= ADC_CFGR_ALIGN; // 12 bit allineati a sinistra

    // Abilitazione Regolatore di Tensione
    ADC1->CR &= ~(ADC_CR_ADVREGEN); // ADVREGEN = 00
    ADC1->CR |= ADC_CR_ADVREGEN_0;  // ADVREGEN = 01

    // Aspettiamo 10us
    delay_ms(10);

    // Calibrazione
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL != 0); // Aspettiamo che ADCAL == 0
}

void osc_main() {
}