/* HAL mínima para compilar no PC o eps.c e os drivers de EPS (teste test_eps.c). */
#ifndef STUB_STM32L4XX_HAL_H
#define STUB_STM32L4XX_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef struct { int instancia; } I2C_HandleTypeDef;
#define I2C_MEMADD_SIZE_8BIT 1U
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *h, uint16_t addr, uint8_t *d, uint16_t n, uint32_t t);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *h, uint16_t addr, uint8_t *d, uint16_t n, uint32_t t);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t addr, uint16_t reg, uint16_t rs, uint8_t *d, uint16_t n, uint32_t t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t addr, uint16_t reg, uint16_t rs, uint8_t *d, uint16_t n, uint32_t t);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t addr, uint32_t tentativas, uint32_t t);
void HAL_Delay(uint32_t ms);
#endif
