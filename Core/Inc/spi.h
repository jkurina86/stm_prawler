/**
  ******************************************************************************
  * @file    spi.h
  * @brief   SPI handle declarations
  ******************************************************************************
  */

#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */
