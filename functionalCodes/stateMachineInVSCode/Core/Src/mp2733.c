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

    // Verifica se o chip responde 
    if(HAL_I2C_IsDeviceReady(MP2733->hi2c, MP2733_ADDR << 1, 3, MP2733_I2C_TIMEOUT) != HAL_OK)
    {
        return 0; // Falha na comunicação
    }

    // Configuração validada na bancada (bancada/eps_nano/eps_nano.ino, MP_CFG; datasheet MP2733 Rev. 1.1)
    MP2733->config.input_current_limit   = 0x52;  // EN_LIM=1 (respeita o pino ILIM), IIN_LIM = 100 + 18*50 = 1000 mA
    MP2733->config.input_voltage_reg     = 0x00;  // VIN_MIN = 3,7 V
    MP2733->config.temp_protection       = 0xC9;  // EN_CHG_NTC=0: a placa não tem termistor ("NTC hot" suspenderia a carga)
    MP2733->config.adc_otg_ctrl          = 0x10;  // 0b00010000
    MP2733->config.charge_vsys_ctrl      = 0x5D;  // BAT_LOADEN=0, carga ligada, VSYS_MIN = 3,675 V (+VTRACK 150 mV = ~3,82 V)
    MP2733->config.charge_current        = 0x91;  // VBATT_PRE=3,0 V, ICC = 320 + 17*40 = 1000 mA
    MP2733->config.precharge_termination = 0x22;  // 0b00100010
    MP2733->config.charge_voltage        = 0xA0;  // 0b10100000
    MP2733->config.timer_config          = 0x95;  // 0b10010101
    MP2733->config.battfet_config        = 0x58;  // 0b01011000
    MP2733->config.int_mask_usb          = 0xC0;  // 0b11000000
    MP2733->config.dpm_mask              = 0xE0;  // 0b11100000 (assumindo R=1)
    MP2733->config.jeita_config          = 0xEE;  // 0b11101110
    
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
    /* Datasheet Rev. 1.1: cada resultado de ADC (REG0Eh..REG13h) é 1 byte. A versão anterior escrevia
     * 0x11/0x10 no REG03h, o que não liga o ADC (ADC_START é o bit 7, ADC_RATE o bit 6) e mexia na
     * configuração do OTG, e lia adc_reg + 1 como byte alto (é outro canal). Liga a conversão contínua
     * preservando os bits do OTG e devolve o byte cru (escalas em eps.c). */
    uint8_t ctrl, v;
    if (!MP2733_ReadRegister(MP2733, MP2733_REG_ADC_OTG_CTRL, &ctrl))
        return 0;
    if ((ctrl & 0x40U) == 0U && !MP2733_WriteRegister(MP2733, MP2733_REG_ADC_OTG_CTRL, (uint8_t)(ctrl | 0x40U)))
        return 0;
    if (!MP2733_ReadRegister(MP2733, adc_reg, &v))
        return 0;
    *adc_value = v;
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
