#include "INA3221.h"
I2C_HandleTypeDef i2cty;

void INA3221_wireWriteRegister(INA3221 *ina3221, uint8_t reg, uint16_t value) {
	uint8_t cmd_buff[3];
    cmd_buff[0] = reg;
	cmd_buff[1] = ((value >> 8) & 0xFF);       // Upper 8-bits
	cmd_buff[2] = (value & 0xFF);

	HAL_I2C_Master_Transmit(&i2cty,ina3221->ADDR,(uint8_t*)cmd_buff,3,1000);
}

void INA3221_wireReadRegister(INA3221 *ina3221, uint8_t reg, uint16_t *value) {
	uint8_t cmd_buff[2];
	HAL_I2C_Master_Transmit(&i2cty, ina3221->ADDR, &reg, 1, 1000);
	HAL_I2C_Master_Receive(&i2cty, ina3221->ADDR, cmd_buff, 2, 1000);

    *value = ((cmd_buff[0] << 8) | cmd_buff[1]);
}

void INA3221_Begin(I2C_HandleTypeDef *_td, INA3221 *ina3221, uint8_t DEFAULT) {
    i2cty=*_td;

    uint16_t config;

	if(DEFAULT){
	for(uint8_t i = 0; i < 3; i++) ina3221->SHUNTR[i] = SHUNT_100m;

	ina3221->ADDR = INA3221_ADDRESS0;
	config = INA3221_CONFIG_ENABLE_CHAN1 |
			 INA3221_CONFIG_ENABLE_CHAN2 |
			 INA3221_CONFIG_ENABLE_CHAN3 |
			 INA3221_CONFIG_AVG_1        |
			 INA3221_CONFIG_VBUS_CT5     |
			 INA3221_CONFIG_VSH_CT5      |
	         INA3221_CONFIG_MODE_2       |
	         INA3221_CONFIG_MODE_1       |
	         INA3221_CONFIG_MODE_0;
	} else {
    config = ina3221->CHANNEL1_ENABLE    |
    		 ina3221->CHANNEL2_ENABLE    |
			 ina3221->CHANNEL3_ENABLE    |
             ina3221->AVG                |
             ina3221->VBUS_CT            |
			 ina3221->VSH_CT             |
             INA3221_CONFIG_MODE_2       |
             INA3221_CONFIG_MODE_1       |
             INA3221_CONFIG_MODE_0;
    for(uint8_t i = 0; i < 3; i++){
    	INA3221_setCritCurrent(ina3221, i+1, ina3221->CRIT_CURR[i]);
    	INA3221_setWarnCurrent(ina3221, i+1, ina3221->WARN_CURR[i]);
   	   }
	}
    INA3221_wireWriteRegister(ina3221, INA3221_REG_CONFIG, config);


}

int16_t INA3221_getBusVoltage_raw(INA3221 *ina3221, uint8_t channel) {
    uint16_t value;
    INA3221_wireReadRegister(ina3221, INA3221_REG_VBUS_1 + (channel - 1) * 2, &value);
    return (int16_t)(value);
}

int16_t INA3221_getShuntVoltage_raw(INA3221 *ina3221, uint8_t channel) {
    uint16_t value;
    INA3221_wireReadRegister(ina3221, INA3221_REG_VSH_1 + (channel - 1) * 2, &value);
    return value;
}

void INA3221_read(INA3221 *ina3221) {
	int16_t VBUS_raw;
	int16_t VSH_raw;
	float VSH;

	uint16_t MASK_ENABLE_Read;

	INA3221_wireReadRegister(ina3221, INA3221_REG_MASK_ENABLE, &MASK_ENABLE_Read);

	for(uint8_t i = 0; i < 3; i++){
		VBUS_raw = INA3221_getBusVoltage_raw(ina3221, i+1);
		VSH_raw  = INA3221_getShuntVoltage_raw(ina3221, i+1);
		VSH      = ((float)((VSH_raw-16)>>3))*(40.0/1000.0);
		ina3221->VBUS_raw[i] = VBUS_raw;
		ina3221->VSHUNT_raw[i] = VSH_raw;
		ina3221->VSHUNT[i] = VSH;
		ina3221->CURR[i] = VSH/(ina3221->SHUNTR[i]);
		ina3221->POW[i] = VSH*(ina3221->CURR[i]);
		ina3221->CRIT_FLAG[i] = ((INA3221_BIT_CRIT_FLAG_CHAN1 >> i) & MASK_ENABLE_Read) && 1;
		ina3221->WARN_FLAG[i] = ((INA3221_BIT_WARN_FLAG_CHAN1 >> i) & MASK_ENABLE_Read) && 1;
	}
	ina3221->SUMM_FLAG = (INA3221_BIT_SUMM_FLAG & MASK_ENABLE_Read) && 1;
}

void INA3221_setCritCurrent(INA3221 *ina3221, uint8_t channel, uint16_t current){
	uint16_t VSH_max = ((uint16_t)(current*(ina3221->SHUNTR[channel-1])*25) << 3) + 16;

	INA3221_wireWriteRegister(ina3221, INA3221_REG_SHUNT_CRIT_LIM_1+(channel-1)*2, VSH_max);
}

void INA3221_setWarnCurrent(INA3221 *ina3221, uint8_t channel, uint16_t current){
	uint16_t VSH_max = ((uint16_t)(current*(ina3221->SHUNTR[channel-1])*25) << 3) + 16;

	INA3221_wireWriteRegister(ina3221, INA3221_REG_SHUNT_WARN_LIM_1+(channel-1)*2, VSH_max);
}
