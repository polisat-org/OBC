/*
Biblioteca para MP2733
OBC - Polisat
Atualizada em: 13/08/2026
Autores: Gabriel Prodossimo, Sofia Jayanthi
*/

#include "mp2733.h"

// Função para escrever em um registrador do MP2733
uint8_t MP2733_WriteRegister(MP2733_HandleTypeDef * MP2733, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    if (HAL_I2C_Master_Transmit(MP2733->hi2c, MP2733_ADDR << 1, data, 2, MP2733_I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }
    return 1;
}

// Função para ler um registrador do MP2733
uint8_t MP2733_ReadRegister(MP2733_HandleTypeDef * MP2733, uint8_t reg, uint8_t *value)
{
    if (HAL_I2C_Master_Transmit(MP2733->hi2c, MP2733_ADDR << 1, &reg, 1, MP2733_I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }

    if (HAL_I2C_Master_Receive(MP2733->hi2c, MP2733_ADDR << 1, value, 1, MP2733_I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }
    return 1;
}

// Inicialização do handle MP2733 com valores iniciais (status e fault) e da planilha (config)
uint8_t MP2733_Init(MP2733_HandleTypeDef * MP2733, I2C_HandleTypeDef *hi2c)
{
    // Atribui handle I2C certo ao handle MP2733
    MP2733->hi2c = hi2c;

    // Verifica se o chip responde lendo status e fault
    // Os valores inicias sao colocados no handle MP2733
    MP2733_GetStatus(MP2733);
    MP2733_GetFault(MP2733);

    // Configuração padrao dos registradores
    MP2733->config.input_current_limit   = 0x48;  // 0b01001000
    MP2733->config.input_voltage_reg     = 0x06;  // 0b00000110
    MP2733->config.temp_protection       = 0xD9;  // 0b11011001
    MP2733->config.adc_otg_ctrl          = 0x10;  // 0b00010000
    MP2733->config.charge_vsys_ctrl      = 0xDB;  // 0b11011011
    MP2733->config.charge_current        = 0xA6;  // 0b10100110
    MP2733->config.precharge_termination = 0x22;  // 0b00100010
    MP2733->config.charge_voltage        = 0xA0;  // 0b10100000
    MP2733->config.timer_config          = 0x95;  // 0b10010101
    MP2733->config.battfet_config        = 0x58;  // 0b01011000
    MP2733->config.int_mask_usb          = 0xC0;  // 0b11000000
    MP2733->config.dpm_mask              = 0xE0;  // 0b11100000 (assumindo R=1)
    MP2733->config.jeita_config          = 0xEE;  // 0b11101110

    mp2733_initialized = 1;
    return 1;
}

// Escreve valores definidos no handle MP2733 nos registradores do chip
uint8_t MP2733_WriteConfig(MP2733_HandleTypeDef * MP2733)
{
    if (!MP2733_WriteRegister(MP2733, MP2733_REG_INPUT_CURRENT_LIMIT, MP2733->config.input_current_limit)) return 0;
    if (!MP2733_WriteRegister(MP2733, MP2733_REG_INPUT_VOLTAGE_REG, MP2733->config.input_voltage_reg)) return 0;
    if (!MP2733_WriteRegister(MP2733, MP2733_REG_TEMP_PROTECTION, MP2733->config.temp_protection)) return 0;
    if (!MP2733_WriteRegister(MP2733, MP2733_REG_ADC_OTG_CTRL, MP2733->config.adc_otg_ctrl)) return 0;
    if (!MP2733_WriteRegister(MP2733, MP2733_REG_CHARGE_VSYS_CTRL, MP2733->config.charge_vsys_ctrl)) return 0;
    if (!MP2733_WriteRegister(MP2733, MP2733_REG_CHARGE_CURRENT, MP2733->config.charge_current)) return 0;
    if (!MP2733_WriteRegister(MP2733, MP2733_REG_PRECHARGE_TERMINATION, MP2733->config.precharge_termination)) return 0;
    if (!MP2733_WriteRegister(MP2733, MP2733_REG_CHARGE_VOLTAGE, MP2733->config.charge_voltage)) return 0;
    if (!MP2733_WriteRegister(MP2733, MP2733_REG_TIMER_CONFIG, MP2733->config.timer_config)) return 0;
    if (!MP2733_WriteRegister(MP2733, MP2733_REG_BATTFET_CONFIG, MP2733->config.battfet_config)) return 0;
    if (!MP2733_WriteRegister(MP2733, MP2733_REG_INT_MASK_USB, MP2733->config.int_mask_usb)) return 0;
    if (!MP2733_WriteRegister(MP2733, MP2733_REG_DPM_MASK, MP2733->config.dpm_mask)) return 0;
    if (!MP2733_WriteRegister(MP2733, MP2733_REG_JEITA_CONFIG, MP2733->config.jeita_config)) return 0;

    return 1; // Sucesso
}

// Ler status do MP2733
uint8_t MP2733_GetStatus(MP2733_HandleTypeDef * MP2733)
{
    uint8_t reg_value;

    if (!MP2733_ReadRegister(MP2733, MP2733_REG_STATUS, &reg_value))
        return 0;

    MP2733->status.vsys_status = (reg_value >> 0) & 0x01;
    MP2733->status.thermal_reg_status = (reg_value >> 1) & 0x01;
    MP2733->status.ntc_float_status = (reg_value >> 2) & 0x01;
    MP2733->status.charge_status = (reg_value >> 3) & 0x03;
    MP2733->status.input_source = (reg_value >> 5) & 0x07;

    return 1;
}

// Ler falhas do MP2733
uint8_t MP2733_GetFault(MP2733_HandleTypeDef * MP2733)
{
    uint8_t reg_value;

    if (!MP2733_ReadRegister(MP2733, MP2733_REG_FAULT, &reg_value))
        return 0;

    MP2733->fault.ntc_fault = (reg_value >> 0) & 0x07;
    MP2733->fault.battery_ov = (reg_value >> 3) & 0x01;
    MP2733->fault.thermal_shutdown = (reg_value >> 4) & 0x01;
    MP2733->fault.input_fault = (reg_value >> 5) & 0x01;
    MP2733->fault.otg_fault = (reg_value >> 6) & 0x01;
    MP2733->fault.watchdog_fault = (reg_value >> 7) & 0x01;

    return 1;
}

// Ler valores ADC
uint8_t MP2733_ReadADC(MP2733_HandleTypeDef * MP2733, uint8_t adc_reg, uint16_t *adc_value)
{
    uint8_t lsb, msb;

    // Primeiro habilita a leitura ADC
    if (!MP2733_WriteRegister(MP2733, MP2733_REG_ADC_OTG_CTRL, 0x11)) // Bit 0 = 1 para ler ADC
        return 0;

    HAL_Delay(1); // Pequeno delay para conversão

    // Lê o registrador ADC especificado
    if (!MP2733_ReadRegister(MP2733, adc_reg, &lsb))
        return 0;

    if (!MP2733_ReadRegister(MP2733, adc_reg + 1, &msb)) // Próximo registrador contém MSB
        return 0;

    *adc_value = (msb << 8) | lsb;

    // Desabilita leitura ADC
    if (!MP2733_WriteRegister(MP2733, MP2733_REG_ADC_OTG_CTRL, 0x10)) // Bit 0 = 0
        return 0;

    return 1;
}

// Função para imprimir status (para debug)
void MP2733_PrintStatus(MP2733_HandleTypeDef * MP2733)
{
    const char *charge_states[] = {"Não carregando", "Trickle Charge", "Constant Current", "Charge Done"};
    const char *input_sources[] = {"Sem input", "USB SDP", "USB CDP", "USB DCP", "Adaptador", "OTG", "Reservado", "Reservado"};

    printf("VSYS Status: %s\r\n", MP2733->status.vsys_status ? "Normal" : "Regulando");
    printf("Regulação Térmica: %s\r\n", MP2733->status.thermal_reg_status ? "Ativa" : "Inativa");
    printf("NTC Float: %s\r\n", MP2733->status.ntc_float_status ? "Ativo" : "Inativo");
    printf("Status Carga: %s\r\n", charge_states[MP2733->status.charge_status]);
    printf("Fonte Input: %s\r\n", input_sources[MP2733->status.input_source]);
}

// Função para imprimir falhas (para debug)
void MP2733_PrintFault(MP2733_HandleTypeDef * MP2733)
{
    const char *ntc_faults[] = {"Normal", "Reservado", "Warm", "Cool", "Reservado", "Cold", "Hot", "Reservado"};

    printf("Falha NTC: %s\r\n", ntc_faults[MP2733->fault.ntc_fault]);
    printf("Overvoltage Bateria: %s\r\n", MP2733->fault.battery_ov ? "Sim" : "Não");
    printf("Shutdown Térmico: %s\r\n", MP2733->fault.thermal_shutdown ? "Sim" : "Não");
    printf("Falha Input: %s\r\n", MP2733->fault.input_fault ? "Sim" : "Não");
    printf("Falha OTG: %s\r\n", MP2733->fault.otg_fault ? "Sim" : "Não");
    printf("Watchdog: %s\r\n", MP2733->fault.watchdog_fault ? "Expirado" : "OK");
}
