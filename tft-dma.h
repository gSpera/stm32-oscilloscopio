#ifndef H_TFT_DMA
#define H_TFT_DMA

#define TFTDMA_BUFFER_SIZE 2048

extern uint8_t tftdma_buffer_data[TFTDMA_BUFFER_SIZE];
extern uint8_t tftdma_buffer_rs[TFTDMA_BUFFER_SIZE];

// Requires tft_init to be called
void tftdma_init();

#endif