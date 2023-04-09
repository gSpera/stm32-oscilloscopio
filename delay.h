#ifndef H_DELAY
#define H_DELAY

// Delay uses TIM6
void delay_init();

void delay_ms(unsigned int ms);
void delay_us(unsigned int us);

#endif