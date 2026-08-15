#ifndef __MP2733_H
#define __MP2733_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdio.h>

// Endereço I2C do MP2733
#define MP2733_ADDR 0x6C

// Registradores do MP2733
#define MP2733_REG_INPUT_CURRENT_LIMIT      0x00
#define MP2733_REG_INPUT_VOLTAGE_REG        0x01
#define MP2733_REG_TEMP_PROTECTION          0x02
#define MP2733_REG_ADC_OTG_CTRL             0x03
#define MP2733_REG_CHARGE_VSYS_CTRL         0x04
#define MP2733_REG_CHARGE_CURRENT           0x05
#define MP2733_REG_PRECHARGE_TERMINATION    0x06
#define MP2733_REG_CHARGE_VOLTAGE           0x07
#define MP2733_REG_TIMER_CONFIG             0x08
#define MP2733_REG_BANDGAP                  0x09
#define MP2733_REG_BATTFET_CONFIG           0x0A
#define MP2733_REG_INT_MASK_USB             0x0B
#define MP2733_REG_STATUS                   0x0C
#define MP2733_REG_FAULT                    0x0D
#define MP2733_REG_ADC_BATTERY_VOLTAGE      0x0E
#define MP2733_REG_ADC_SYS_VOLTAGE          0x0F
#define MP2733_REG_ADC_THERMISTOR           0x10
#define MP2733_REG_ADC_INPUT_VOLTAGE        0x11
#define MP2733_REG_ADC_CHARGE_CURRENT       0x12
#define MP2733_REG_ADC_INPUT_CURRENT        0x13
#define MP2733_REG_POWER_STATUS             0x14
#define MP2733_REG_DPM_MASK                 0x15
#define MP2733_REG_JEITA_CONFIG             0x16
#define MP2733_REG_SAFETY_TIMER_STATUS      0x17

// Estrutura para configuracoes de carregamento
typedef struct {
    uint8_t input_current_limit;      // REG00h
    uint8_t input_voltage_reg;        // REG01h
    uint8_t temp_protection;          // REG02h
    uint8_t adc_otg_ctrl;             // REG03h
    uint8_t charge_vsys_ctrl;         // REG04h
    uint8_t charge_current;           // REG05h
    uint8_t precharge_termination;    // REG06h
    uint8_t charge_voltage;           // REG07h
    uint8_t timer_config;             // REG08h
    uint8_t battfet_config;           // REG0Ah
    uint8_t int_mask_usb;             // REG0Bh
    uint8_t dpm_mask;                 // REG15h
    uint8_t jeita_config;             // REG16h
} MP2733_Config_t;

// Estrutura para status do carregador
typedef struct {
    uint8_t vsys_status;
    uint8_t thermal_reg_status;
    uint8_t ntc_float_status;
    uint8_t charge_status;
    uint8_t input_source;
} MP2733_Status_t;

// Estrutura para falhas
typedef struct {
    uint8_t ntc_fault;
    uint8_t battery_ov;
    uint8_t thermal_shutdown;
    uint8_t input_fault;
    uint8_t otg_fault;
    uint8_t watchdog_fault;
} MP2733_Fault_t;

// Estrutura para handle do MP2733
typedef struct {
    // Handle I2C, depende da conexao no STM32
    I2C_HandleTypeDef *hi2c;

    // Configuracoes de carregamento
    MP2733_Config_t config;

    // Struct do status
    MP2733_Status_t status;
    // Struct de falhas
    MP2733_Fault_t fault;
} MP2733_HandleTypeDef;

#define MP2733_I2C_TIMEOUT 100

// Funções para o MP2733
uint8_t MP2733_WriteRegister(MP2733_HandleTypeDef * MP2733, uint8_t reg, uint8_t value);
uint8_t MP2733_ReadRegister(MP2733_HandleTypeDef * MP2733, uint8_t reg, uint8_t *value);
uint8_t MP2733_Init(MP2733_HandleTypeDef * MP2733, I2C_HandleTypeDef *hi2c);
uint8_t MP2733_WriteConfig(MP2733_HandleTypeDef * MP2733);
uint8_t MP2733_GetStatus(MP2733_HandleTypeDef * MP2733);
uint8_t MP2733_GetFault(MP2733_HandleTypeDef * MP2733);
uint8_t MP2733_ReadADC(MP2733_HandleTypeDef * MP2733, uint8_t adc_reg, uint16_t *adc_value);
void MP2733_PrintStatus(MP2733_HandleTypeDef * MP2733);
void MP2733_PrintFault(MP2733_HandleTypeDef * MP2733);


#ifdef __cplusplus
}
#endif

#endif /* __MP2733_H */
