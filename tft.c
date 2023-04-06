#include "tft.h"

#include <stm32f30x.h>

#include "usart.h"

#define LCD_RST GPIO_ODR_3
#define LCD_CS GPIO_ODR_4
#define LCD_WR GPIO_ODR_6
#define LCD_RD GPIO_ODR_7
#define LCD_RS GPIO_ODR_5
#define TFTLCD_DELAY 0xFFFF

void tft_data_out() {
    GPIOD->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 |
                    GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 |
                    GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 |
                    GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0;
}
void tft_data_in() {
    GPIOD->MODER &=
        ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 |
          GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 |
          GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);
}

void tft_init_regs();
void tft_init() {
    RCC->AHBENR |= RCC_AHBENR_GPIODEN | RCC_AHBENR_GPIOBEN;

    // GPIOD Moder set to Output from D0 to D7 (Data)
    // GPIOB Moder set to Output from B3 to B7 (Control)
    tft_data_out();
    GPIOB->MODER |= GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 |
                    GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 |
                    GPIO_MODER_MODER7_0;

    // LCD_RST to high
    // LCD_CS  to low
    // LCD_WR  to high
    // LCD_RD  to high
    GPIOB->ODR &= ~(LCD_CS);
    GPIOB->ODR |= LCD_RST | LCD_WR | LCD_RD;

    for (int i = 0; i < 50 * 100; i++)
        ;
    GPIOB->ODR &= ~(LCD_RST);
    for (int i = 0; i < 1000; i++)
        ;
    GPIOB->ODR |= LCD_RST;

    tft_init_regs();
}

/*
void lcdWrite(uint8_t v, bool rs) {
#define B(X) (v & (1 << X))
    digitalWrite(LCD_RS, rs);
    // digitalWrite(LCD_CS, LOW);
    digitalWrite(DATA0, B(0));
    digitalWrite(DATA1, B(1));
    digitalWrite(DATA2, B(2));
    digitalWrite(DATA3, B(3));
    digitalWrite(DATA4, B(4));
    digitalWrite(DATA5, B(5));
    digitalWrite(DATA6, B(6));
    digitalWrite(DATA7, B(7));

    digitalWrite(LCD_WR, LOW);
    digitalWrite(LCD_WR, HIGH);

    // digitalWrite(LCD_CS, HIGH);
    digitalWrite(LCD_RS, DATA);
#undef B
}
*/

void tft_write8(uint8_t v, bool rs) {
    tft_data_out();

    // Set or reset GPIOB_5 (LCD_RS) based on rs
    GPIOB->BSRR = 1 << (rs == TFT_RS_CMD ? 21 : 5);
    // Set data bus to v
    GPIOD->ODR = v;
    // Write Strobe

    // WR low
    GPIOB->BSRR = GPIO_BSRR_BR_6;
    // Wait 45ns
    for (int i = 0; i < 45 * 100; i++) {
    }
    // WR high
    // Wait 70ns
    GPIOB->BSRR = GPIO_BSRR_BS_6;
    for (int i = 0; i < 70 * 100; i++) {
    }
}

void tft_write16(uint16_t v, bool rs) {
    tft_write8(v >> 8, rs);
    tft_write8(v & 0xFF, rs);
}

void tft_init_regs() {
    static const uint16_t registers[] = {
        0x00e5,
        0x8000,
        0x0000,
        0x0001,
        0x0001,
        0x100,
        0x0002,
        0x0700,
        0x0003,
        0x1030,
        0x0004,
        0x0000,
        0x0008,
        0x0202,
        0x0009,
        0x0000,
        0x000A,
        0x0000,
        0x000C,
        0x0000,
        0x000D,
        0x0000,
        0x000F,
        0x0000,
        //-----Power On sequence-----------------------
        0x0010,
        0x0000,
        0x0011,
        0x0007,
        0x0012,
        0x0000,
        0x0013,
        0x0000,
        TFTLCD_DELAY,
        50,
        0x0010,
        0x17B0,  // SAP=1, BT=7, APE=1, AP=3
        0x0011,
        0x0007,  // DC1=0, DC0=0, VC=7
        TFTLCD_DELAY,
        10,
        0x0012,
        0x013A,  // VCMR=1, PON=3, VRH=10
        TFTLCD_DELAY,
        10,
        0x0013,
        0x1A00,  // VDV=26
        0x0029,
        0x000c,  // VCM=12
        TFTLCD_DELAY,
        10,
        //-----Gamma control-----------------------
        0x0030,
        0x0000,
        0x0031,
        0x0505,
        0x0032,
        0x0004,
        0x0035,
        0x0006,
        0x0036,
        0x0707,
        0x0037,
        0x0105,
        0x0038,
        0x0002,
        0x0039,
        0x0707,
        0x003C,
        0x0704,
        0x003D,
        0x0807,
        //-----Set RAM area-----------------------
        0x0060,
        0xA700,  // GS=1
        0x0061,
        0x0001,
        0x006A,
        0x0000,
        0x0021,
        0x0000,
        0x0020,
        0x0000,
        //-----Partial Display Control------------
        0x0080,
        0x0000,
        0x0081,
        0x0000,
        0x0082,
        0x0000,
        0x0083,
        0x0000,
        0x0084,
        0x0000,
        0x0085,
        0x0000,
        //-----Panel Control----------------------
        0x0090,
        0x0010,
        0x0092,
        0x0000,
        0x0093,
        0x0003,
        0x0095,
        0x0110,
        0x0097,
        0x0000,
        0x0098,
        0x0000,
        //-----Display on-----------------------
        0x0007,
        0x0173,
        TFTLCD_DELAY,
        50,
    };

    uint16_t size = sizeof(registers);
    usart_puts("Start writing registers\n");
    for (int i = 0; i < size / 8;) {
        uint16_t cmd = registers[i++];
        uint16_t d = registers[i++];
        usart_puts("Register: ");
        usart_puthex(i);
        usart_putc(' ');
        usart_puthex(cmd);
        usart_putc(' ');
        usart_puthex(d);
        usart_putc('\n');

        if (cmd == TFTLCD_DELAY) {
            for (int j = 0; j < d * 1000; j++)
                ;
        } else {
            tft_write16(cmd, TFT_RS_CMD);
            tft_write16(d, TFT_RS_DAT);
        }
    }

    tft_write16(0x61, TFT_RS_CMD);
    tft_write16(0x2, TFT_RS_DAT);
    tft_write16(0x6A, TFT_RS_CMD);
    tft_write16(0, TFT_RS_DAT);
    tft_write16(0x60, TFT_RS_CMD);
    tft_write16(0xA700, TFT_RS_DAT);
}

uint8_t tft_read8() {
    tft_data_in();
    GPIOB->ODR &= ~(LCD_RD);
    for (int i = 0; i < 170; i++)
        ;
    uint8_t data = GPIOD->IDR & 0xFF;
    GPIOB->ODR |= LCD_RD;
    for (int i = 0; i < 150; i++)
        ;
    return data;
}
uint16_t tft_read16() { return (((uint16_t)tft_read8()) << 8) | tft_read8(); }