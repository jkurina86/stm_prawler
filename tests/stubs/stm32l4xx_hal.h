#ifndef TEST_STM32_HAL_H
#define TEST_STM32_HAL_H
#include <stdint.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct {
    struct {
        uint32_t Mode, Direction, PeriphInc, MemInc;
        uint32_t PeriphDataAlignment, MemDataAlignment;
    } Init;
    uint32_t counter;
} DMA_HandleTypeDef;
typedef struct {
    struct {
        uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl;
    } Init;
    DMA_HandleTypeDef *hdmarx;
    uint32_t ErrorCode;
} UART_HandleTypeDef;

#define HAL_MAX_DELAY UINT32_MAX
#define DMA_NORMAL 0U
#define DMA_CIRCULAR 1U
#define DMA_PERIPH_TO_MEMORY 0U
#define DMA_PINC_DISABLE 0U
#define DMA_MINC_ENABLE 1U
#define DMA_PDATAALIGN_BYTE 0U
#define DMA_MDATAALIGN_BYTE 0U
#define DMA_IT_HT 1U
#define UART_WORDLENGTH_8B 0U
#define UART_STOPBITS_1 0U
#define UART_PARITY_NONE 0U
#define UART_MODE_TX_RX 3U
#define UART_HWCONTROL_NONE 0U
#define HAL_UART_ERROR_NONE 0U
#define UART_RXDATA_FLUSH_REQUEST 1U
#define __DMB() ((void)0)
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->counter)
#define __HAL_DMA_DISABLE_IT(dma, flag) ((void)(dma), (void)(flag))
#define __HAL_UART_CLEAR_OREFLAG(uart) ((void)(uart))
#define __HAL_UART_CLEAR_NEFLAG(uart) ((void)(uart))
#define __HAL_UART_CLEAR_FEFLAG(uart) ((void)(uart))
#define __HAL_UART_CLEAR_PEFLAG(uart) ((void)(uart))
#define __HAL_UART_SEND_REQ(uart, request) ((void)(uart), (void)(request))

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
#endif
