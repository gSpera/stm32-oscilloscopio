#include <stm32f30x.h>
#include "touch.h"
#include "tft.h"
#include "delay.h"

uint8_t  touch_btns_enabled[BUTTON_SIZE/8];
const Button touch_btns[BUTTON_SIZE] = {
};

// TFT_WR -> A1 (Arduino) -> PB6 (STM32) -> Y+
// TFT_RS -> A2           -> PB0         -> X+
// D6 -> X-
// D7 -> Y-

#define XP GPIO_ODR_0 // Port B ADC3_IN12
#define YP GPIO_ODR_6 // Port B

#define XM GPIO_ODR_6 // Port D D6
#define YM GPIO_ODR_7 // Port D D7

#define LCD_CS GPIO_ODR_4

// Enable deve essere chiamata
// per poter leggere il touch screen
// Modifica le porte dati e controllo
void touch_enable() {
    tft_data_out();
    // Il bit 0 è già impostato, poichè questi moder o sono in output (0b01)
    // o in analogico (0b11)
    // WR e RS
    GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER0_1;
    GPIOB->ODR |= LCD_CS; // Disabilita lo schermo
}

// Disable deve essere chiamato
// dopo aver letto il touch screen
// Modifica le porte dati e controllo
void touch_disable() {
    // Il realtà non sappiamo lo stato precedente dalla porta dati
    GPIOB->MODER &= ~(GPIO_MODER_MODER6_1 | GPIO_MODER_MODER0_1);
    GPIOB->ODR &= ~(LCD_CS);
}

void touch_init() {
    RCC->AHBENR |= RCC_AHBENR_ADC34EN | RCC_AHBENR_GPIOBEN;
    ADC3_4->CCR |= ADC34_CCR_CKMODE_0;
    ADC3->CFGR &= ~(ADC_CFGR_RES);
    ADC3->CFGR |= ADC_CFGR_RES_0 | ADC_CFGR_ALIGN; // 10 bit allineati a sinistra

    // Abilitazione Regolatore di Tensione
    ADC3->CR &= ~(ADC_CR_ADVREGEN); // ADVREGEN = 00
    ADC3->CR |= ADC_CR_ADVREGEN_0;  // ADVREGEN = 01

    // Aspettiamo 10us
    delay_ms(10);

    // Calibrazione
    ADC3->CR |= ADC_CR_ADCAL;
    while (ADC3->CR & ADC_CR_ADCAL != 0); // Aspettiamo che ADCAL == 0

    // ADC3->SMPR2 &= ~(ADC_SMPR2_SMP12);
    // ADC3->SMPR2 |= 5 << 6;
}

bool touch_is_detected() {
    uint16_t z = touch_read_z();
    return z < TS_THRESH;
}
uint16_t touch_read_raw_x() {
    GPIOB->MODER |= GPIO_MODER_MODER1; // Y+ Input
    GPIOD->MODER |= GPIO_MODER_MODER7; // Y- Input
    GPIOB->MODER &= ~(GPIO_MODER_MODER0);
    GPIOB->MODER |= GPIO_MODER_MODER0_0; // X+ Output
    GPIOD->MODER &= ~(GPIO_MODER_MODER6);
    GPIOD->MODER |= GPIO_MODER_MODER6_0; // X- Ouput
    GPIOB->BSRR = GPIO_BSRR_BS_0;
    GPIOD->BSRR = GPIO_BSRR_BR_6;

    delay_us(20);

    // Leggiamo YP
    ADC3->SQR1 = 1 << 6; // 1 Campionamento sull'input 1 (PB1)

    // Abilitiamo l'ADC
    ADC3->CR |= ADC_CR_ADEN;
    while(ADC3->CR & ADC_ISR_ADRD == 0); // Aspettiamo sia pronto

    ADC3->CR |= ADC_CR_ADSTART;
    while (ADC3->ISR & ADC_ISR_EOS == 0);

    uint16_t value = ADC3->DR;
    return value;
}
uint16_t touch_read_raw_y() {
    GPIOB->MODER |= GPIO_MODER_MODER0; // X+ Input
    GPIOD->MODER |= GPIO_MODER_MODER6; // X- Input
    GPIOB->MODER &= ~(GPIO_MODER_MODER1);
    GPIOB->MODER |= GPIO_MODER_MODER1_0; // Y+ Output
    GPIOD->MODER &= ~(GPIO_MODER_MODER7);
    GPIOD->MODER |= GPIO_MODER_MODER7_0; // Y- Output
    GPIOB->BSRR = GPIO_BSRR_BS_1;
    GPIOD->BSRR = GPIO_BSRR_BR_7;

    delay_us(20);

    // Leggiamo XP
    ADC3->SQR1 = 12 << 6; // 1 Campionamento sull'input 12 (PB0)

    // Abilitiamo l'ADC
    ADC3->CR |= ADC_CR_ADEN;
    while(ADC3->CR & ADC_ISR_ADRD == 0); // Aspettiamo sia pronto

    ADC3->CR |= ADC_CR_ADSTART;
    while (ADC3->ISR & ADC_ISR_EOS == 0);

    uint16_t value = ADC3->DR;
    return value;
}

uint16_t touch_read_raw_z() {
    // TFT_WR -> A1 (Arduino) -> PB6 (STM32) -> Y+
    // TFT_RS -> A2           -> PB0         -> X+
    // D6 -> X-
    // D7 -> Y-

    // X- GND
    // Y- VCC
    // X+ Y+ Input

    GPIOD->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOD->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0; // X- Y- Output
    GPIOB->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1; // X+ Y+ Input
    GPIOD->BSRR = GPIO_BSRR_BS_7 | GPIO_BSRR_BR_6;
    delay_us(20);

    // Leggiamo XP
    ADC3->SQR1 = 12 << 6; // 1 Campionamento sull'input 12 (PB0)

    // Abilitiamo l'ADC
    ADC3->CR |= ADC_CR_ADEN;
    while(ADC3->CR & ADC_ISR_ADRD == 0); // Aspettiamo sia pronto

    ADC3->CR |= ADC_CR_ADSTART;
    while (ADC3->ISR & ADC_ISR_EOS == 0);

    uint16_t z1 = ADC3->DR;

    // Leggiamo YP
    ADC3->SQR1 = 1 << 6; // 1 Campionamento sull'input 12 (PB0)

    // Abilitiamo l'ADC
    ADC3->CR |= ADC_CR_ADEN;
    while(ADC3->CR & ADC_ISR_ADRD == 0); // Aspettiamo sia pronto

    ADC3->CR |= ADC_CR_ADSTART;
    while (ADC3->ISR & ADC_ISR_EOS == 0);

    uint16_t z2 = ADC3->DR;

    return z1+z2;
}

uint32_t map(uint32_t t, uint32_t f0, uint32_t f1, uint32_t t0, uint32_t t1) {
    return (t - 1000) / 100;
    if (t > f1) t = f1;
    else if (t < f0) t = f0;
    float slope = (t1- t0) / (f1 - f0);
    float v = (t - f0) * slope + t0;
    return v;
}

uint16_t touch_read_x() {
    uint32_t v = 0;
    for (int i=0;i<10;i++) {
        v += touch_read_raw_x();
    }
    v /= 10;
    v = map(v, TS_LEFT, TS_RIGHT, 0, TFT_WIDTH);
    return v;
}
uint16_t touch_read_y() {
    uint32_t v = 0;
    for (int i=0;i<10;i++) {
        v += touch_read_raw_y();
    }
    v /= 10;
    v = map(v, TS_TOP, TS_BOT, 0, TFT_HEIGHT);
    return v;
}
uint16_t touch_read_z() {
    uint32_t v = 0;
    for (int i=0;i<10;i++) {
        v += touch_read_raw_z();
    }
    v /= 10;
    return v;
}

#define ABS(A) (A) > 0 ? (A) : -(A)
void touch_btn_press(int x, int y) {
    for (int i=0;i<BUTTON_SIZE;i++) {
        if (!touch_btn_is_enabled(i)) {
            continue;
        }
        Button btn = touch_btns[i];
        int dx = ABS(btn.x - x);
        int dy = ABS(btn.y - y);
        if (dx < TOUCH_BTN_RADIOUS && dy < TOUCH_BTN_RADIOUS) {
            btn.fn();
            return;
        }
    }
}
void touch_btn_enable(int index) {
    int position = index / 8;
    int offset = index & 8;
    touch_btns_enabled[position] |= 1<<offset;
}
void touch_btn_disable(int index) {
    int position = index / 8;
    int offset = index & 8;
    touch_btns_enabled[position] &= ~(1<<offset);
}
int touch_btn_is_enabled(int index) {
    int position = index / 8;
    int offset = index & 8;
    return touch_btns_enabled[position] != 0;
}