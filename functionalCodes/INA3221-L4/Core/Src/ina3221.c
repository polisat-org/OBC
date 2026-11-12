#include "ina3221.h"
#include <stm32l4xx_hal_def.h>
#include <stm32l4xx_hal_i2c.h>


INA3221_Error
INA3221_init(INA3221_Handle *h,
    INA3221_I2C_Handle *i2c_handle,
    INA3221_Addr addr,
    uint32_t timeout_ms
)
{
    if (h == NULL || i2c_handle == NULL) {
        return INA3221_ERROR_NULL;
    }

    h->i2c_handle = i2c_handle;
    h->dev_addr = addr << 1u;
    h->timeout_ms = timeout_ms;

    return INA3221_OK;
}

INA3221_Error
INA3221_set_reg(INA3221_Handle *h,
    INA3221_Reg reg
)
{
    if (h == NULL) {
        return INA3221_ERROR_NULL;
    }
    if (reg == INA3221_REG_NONE) {
        return INA3221_OK; // no-op
    }

    uint8_t data[1] = { reg };

    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Master_Transmit(h->i2c_handle,
        h->dev_addr,
        data,
        sizeof(data),
        h->timeout_ms
    );

    if (ret != HAL_OK) {
        return INA3221_ERROR_I2C;
    }

    return INA3221_OK;
}

INA3221_Error
INA3221_read_reg(INA3221_Handle *h,
    INA3221_Reg reg,
    uint16_t *out_val
)
{
    if (h == NULL || out_val == NULL || reg == INA3221_REG_NONE) {
        return INA3221_ERROR_NULL;
    }

    INA3221_Error err;
    err = INA3221_set_reg(h, reg);
    if (err != INA3221_OK){
        return err;
    }
    
    uint8_t data[2];
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Master_Receive(
        h->i2c_handle,
        h->dev_addr,
        data,
        sizeof(data),
        h->timeout_ms
    );

    if (ret != HAL_OK) {
        return INA3221_ERROR_I2C;
    }

    *out_val = (data[0] << 8u) | data[1];
    return INA3221_OK;
}

INA3221_Error
INA3221_write_reg(INA3221_Handle *h,
    INA3221_Reg reg,
    uint16_t val
)
{
    if (h == NULL || reg == INA3221_REG_NONE) {
        return INA3221_ERROR_NULL;
    }

    INA3221_Error err;
    err = INA3221_set_reg(h, reg);
    if (err != INA3221_OK) {
        return err;
    }

    uint8_t data[2] = {
        ((val & 0xFF00) >> 8u),
        ((val & 0x00FF) >> 0u)
    };
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(
        h->i2c_handle,
        h->dev_addr,
        data,
        sizeof(data),
        h->timeout_ms
    );

    if (ret != HAL_OK) {
        return INA3221_ERROR_I2C;
    }

    return INA3221_OK;
}

INA3221_Error
INA3221_get_cfg(INA3221_Handle *h,
    INA3221_Cfg *out_cfg
)
{
    if (h == NULL || out_cfg == NULL) {
        return INA3221_ERROR_NULL;
    }

    uint16_t cfg = 0;
    INA3221_Error err = INA3221_read_reg(h, INA3221_REG_CFG, &cfg);
    if (err != INA3221_OK) {
        return err;
    }

    *out_cfg = cfg;
    return INA3221_OK;
}

INA3221_Error
INA3221_set_cfg(INA3221_Handle *h,
    INA3221_Cfg cfg
)
{
    if (h == NULL) {
        return INA3221_ERROR_NULL;
    }

    return INA3221_write_reg(h, INA3221_REG_CFG, cfg);
}

INA3221_Error
INA3221_read_voltage(INA3221_Handle *h,
    INA3221_Chan chan,
    INA3221_VoltageType type,
    int16_t *out_voltage
)
{
    if (h == NULL || out_voltage == NULL) {
        return INA3221_ERROR_NULL;
    }

    INA3221_Reg reg;
    switch (chan) {
        case INA3221_CHAN_1:
            reg = (type == INA3221_VOLTAGE_TYPE_BUS) ? INA3221_REG_CHAN_1_BUS : INA3221_REG_CHAN_1_SHUNT;
            break;
        case INA3221_CHAN_2:
            reg = (type == INA3221_VOLTAGE_TYPE_BUS) ? INA3221_REG_CHAN_2_BUS : INA3221_REG_CHAN_2_SHUNT;
            break;
        case INA3221_CHAN_3:
            reg = (type == INA3221_VOLTAGE_TYPE_BUS) ? INA3221_REG_CHAN_3_BUS : INA3221_REG_CHAN_3_SHUNT;
            break;
        default:
            return INA3221_ERROR_NULL;
    }

    uint16_t val = 0u;
    INA3221_Error err = INA3221_read_reg(h, reg, &val);
    if (err != INA3221_OK) {
        return err;
    }

    uint32_t raw = val >> 3u;
    
    // check for sign bit
    if (raw & (1u<<12u)) {
        raw = raw & 0xFFF;
    }

    return INA3221_OK;
}