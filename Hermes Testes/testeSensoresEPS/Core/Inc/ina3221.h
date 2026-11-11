#ifndef INA_3221_H
#define INA_3221_H

#include "stdint.h"
#include "stdbool.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"


// Constants
// Registers
#define INA3221_REG_CFG              (0x00) // read and write
#define INA3221_REG_CHAN_1_SHUNT     (0x01) // only read
#define INA3221_REG_CHAN_1_BUS       (0x02) // only read
#define INA3221_REG_CHAN_2_SHUNT     (0x03) // only read
#define INA3221_REG_CHAN_2_BUS       (0x04) // only read
#define INA3221_REG_CHAN_3_SHUNT     (0x05) // only read
#define INA3221_REG_CHAN_3_BUS       (0x06) // only read
#define INA3221_REG_CHAN_1_CRIT_LIM  (0x07) // read and write
#define INA3221_REG_CHAN_1_WARN_LIM  (0x08) // read and write 
#define INA3221_REG_CHAN_2_CRIT_LIM  (0x09) // read and write 
#define INA3221_REG_CHAN_2_WARN_LIM  (0x0A) // read and write 
#define INA3221_REG_CHAN_3_CRIT_LIM  (0x0B) // read and write 
#define INA3221_REG_CHAN_3_WARN_LIM  (0x0C) // read and write 
#define INA3221_REG_SHUNT_SUM        (0x0D) // only read
#define INA3221_REG_SHUNT_SUM_LIM    (0x0E) // read and write
#define INA3221_REG_MASK_ENABLE      (0x0F) // read and write 
#define INA3221_REG_PV_UPPER_LIM     (0x10) // read and write
#define INA3221_REG_PV_LOWER_LIM     (0x11) // read and write
#define INA3221_REG_MANUFAC_ID       (0xFE) // only read 
#define INA3221_REG_DIE_ID           (0xFF) // only read
#define INA3221_REG_NONE             (0xFD) // doesn't exist

// Configuration register bits
#define INA3221_CFG_RESET                           (1u<<15)
#define INA3221_CFG_CHAN_1                          (1u<<14)
#define INA3221_CFG_CHAN_2                          (1u<<13)
#define INA3221_CFG_CHAN_3                          (1u<<12)
#define INA3221_CFG_AVG_1                           (0u<<9)
#define INA3221_CFG_AVG_4                           (1u<<9)
#define INA3221_CFG_AVG_16                          (2u<<9)
#define INA3221_CFG_AVG_64                          (3u<<9)
#define INA3221_CFG_AVG_128                         (4u<<9)
#define INA3221_CFG_AVG_256                         (5u<<9)
#define INA3221_CFG_AVG_512                         (6u<<9)
#define INA3221_CFG_AVG_1024                        (7u<<9)
#define INA3221_CFG_TIME_BUS_140_US                 (0u<<6)
#define INA3221_CFG_TIME_BUS_204_US                 (1u<<6)
#define INA3221_CFG_TIME_BUS_332_US                 (2u<<6)
#define INA3221_CFG_TIME_BUS_588_US                 (3u<<6)
#define INA3221_CFG_TIME_BUS_1100_US                (4u<<6)
#define INA3221_CFG_TIME_BUS_2116_US                (5u<<6)
#define INA3221_CFG_TIME_BUS_4156_US                (6u<<6)
#define INA3221_CFG_TIME_BUS_8244_US                (7u<<6)
#define INA3221_CFG_TIME_SHUNT_140_US               (0u<<3)
#define INA3221_CFG_TIME_SHUNT_204_US               (1u<<3)
#define INA3221_CFG_TIME_SHUNT_332_US               (2u<<3)
#define INA3221_CFG_TIME_SHUNT_588_US               (3u<<3)
#define INA3221_CFG_TIME_SHUNT_1100_US              (4u<<3)
#define INA3221_CFG_TIME_SHUNT_2116_US              (5u<<3)
#define INA3221_CFG_TIME_SHUNT_4156_US              (6u<<3)
#define INA3221_CFG_TIME_SHUNT_8244_US              (7u<<3)
#define INA3221_CFG_MODE_POWER_DOWN                 (0u<<0)
#define INA3221_CFG_MODE_SHUNT_SINGLE_SHOT          (1u<<0)
#define INA3221_CFG_MODE_BUS_SINGLE_SHOT            (2u<<0)
#define INA3221_CFG_MODE_SHUNT_AND_BUS_SINGLE_SHOT  (3u<<0)
#define INA3221_CFG_MODE_SHUNT_CONTINUOUS           (5u<<0)
#define INA3221_CFG_MODE_BUS_CONTINUOUS             (6u<<0)
#define INA3221_CFG_MODE_SHUNT_AND_BUS_CONTINUOUS   (7u<<0)

// Addresses (hardcoded)
#define INA3221_ADDR_GND (0x40)
#define INA3221_ADDR_VS  (0x41)
#define INA3221_ADDR_SDA (0x42)
#define INA3221_ADDR_SCL (0x43)

typedef enum INA3221_Chan {
    INA3221_CHAN_1 = 0,
    INA3221_CHAN_2,
    INA3221_CHAN_3 
} INA3221_Chan;

typedef enum INA3221_VoltageType {
    INA3221_VOLTAGE_TYPE_BUS = 0,
    INA3221_VOLTAGE_TYPE_SHUNT
} INA3221_VoltageType;

typedef enum INA3221_Error {
    INA3221_OK = 0,
    INA3221_ERROR_I2C,
    INA3221_ERROR_NULL
} INA3221_Error;

// Types

typedef uint8_t INA3221_Reg;
typedef uint8_t INA3221_Addr;
typedef uint16_t INA3221_Cfg;

typedef I2C_HandleTypeDef INA3221_I2C_Handle;

typedef struct INA3221_Handle {
    INA3221_I2C_Handle *i2c_handle;
    uint16_t dev_addr;
    INA3221_Reg last_reg;
    uint32_t timeout_ms;
} INA3221_Handle;

// Prototypes

// Initializes INA handle with I2C handle, device address, and timeout in millisecond
INA3221_Error
INA3221_init(INA3221_Handle *h,
    INA3221_I2C_Handle *i2c_handle,
    INA3221_Addr addr,
    uint32_t timeout_ms
);

// 
INA3221_Error
INA3221_set_reg(INA3221_Handle *h,
    INA3221_Reg reg
);

INA3221_Error
INA3221_read_reg(INA3221_Handle *h,
    INA3221_Reg reg,
    uint16_t *out_val
);

INA3221_Error
INA3221_write_reg(INA3221_Handle *h,
    INA3221_Reg reg,
    uint16_t val
);

INA3221_Error
INA3221_get_cfg(INA3221_Handle *h,
    INA3221_Cfg *out_cfg
);

INA3221_Error
INA3221_set_cfg(INA3221_Handle *h,
    INA3221_Cfg cfg
);

INA3221_Error
INA3221_read_voltage(INA3221_Handle *h,
    INA3221_Chan chan,
    INA3221_VoltageType type,
    int16_t *out_voltage
);

#endif // INA_3221_H