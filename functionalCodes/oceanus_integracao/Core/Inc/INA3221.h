/*

    Arduino library for INA3221 current and voltage sensor.

    MIT License

    Copyright (c) 2020 Beast Devices, Andrejs Bondarevs

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.

*/

#ifndef _INA3221_H_
#define _INA3221_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>


/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define INA3221_CHANNEL_1						(0x01)
#define INA3221_CHANNEL_2						(0x02)
#define INA3221_CHANNEL_3						(0x03)

#define INA3221_ADDRESS0                        (0x40<<1)    // 1000000 (A0=GND)
#define INA3221_ADDRESS1                        (0x41<<1)    // 1000001 (A0=VS+)
#define INA3221_ADDRESS2                        (0x42<<1)    // 1000010 (A0=SDA)
#define INA3221_ADDRESS3                        (0x43<<1)    // 1000011 (A0=SCL)
#define INA3221_READ                            (0x01)
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER (R/W)
    -----------------------------------------------------------------------*/
#define INA3221_REG_CONFIG                      (0x00)
#define INA3221_CONFIG_RESET                    (0x8000)  // Reset Bit

#define INA3221_CONFIG_ENABLE_CHAN1             (0x4000)  // Enable Channel 1
#define INA3221_CONFIG_ENABLE_CHAN2             (0x2000)  // Enable Channel 2
#define INA3221_CONFIG_ENABLE_CHAN3             (0x1000)  // Enable Channel 3

#define INA3221_CONFIG_AVG_1                    (0x0000<<9)  // AVG Samples Bit 1    - See table 3 spec
#define INA3221_CONFIG_AVG_4                    (0x0001<<9)  // AVG Samples Bit 4    - See table 3 spec
#define INA3221_CONFIG_AVG_16                   (0x0002<<9)  // AVG Samples Bit 16   - See table 3 spec
#define INA3221_CONFIG_AVG_64                   (0x0003<<9)  // AVG Samples Bit 64   - See table 3 spec
#define INA3221_CONFIG_AVG_128                  (0x0004<<9)  // AVG Samples Bit 128  - See table 3 spec
#define INA3221_CONFIG_AVG_256                  (0x0005<<9)  // AVG Samples Bit 256  - See table 3 spec
#define INA3221_CONFIG_AVG_512                  (0x0006<<9)  // AVG Samples Bit 512  - See table 3 spec
#define INA3221_CONFIG_AVG_1024	                (0x0007<<9)  // AVG Samples Bit 1024 - See table 3 spec

#define INA3221_CONFIG_VBUS_CT1                 (0x0000<<6)  // VBUS bit 1 Conversion time - See table 4 spec  140us
#define INA3221_CONFIG_VBUS_CT2                 (0x0001<<6)  // VBUS bit 0 Conversion time - See table 4 spec  204us
#define INA3221_CONFIG_VBUS_CT3                 (0x0002<<6)  // VBUS bit 2 Conversion time - See table 4 spec  332us
#define INA3221_CONFIG_VBUS_CT4                 (0x0003<<6)  // VBUS bit 1 Conversion time - See table 4 spec  588us
#define INA3221_CONFIG_VBUS_CT5                 (0x0004<<6)  // VBUS bit 0 Conversion time - See table 4 spec 1100us
#define INA3221_CONFIG_VBUS_CT6                 (0x0005<<6)  // VBUS bit 2 Conversion time - See table 4 spec 2116us
#define INA3221_CONFIG_VBUS_CT7                 (0x0006<<6)  // VBUS bit 1 Conversion time - See table 4 spec 4156us
#define INA3221_CONFIG_VBUS_CT8                 (0x0007<<6)  // VBUS bit 0 Conversion time - See table 4 spec 8244us

#define INA3221_CONFIG_VSH_CT2                  (0x0001<<3)  // VSHUNT bit 0 Conversion time - See table 5 spec  140us
#define INA3221_CONFIG_VSH_CT1                  (0x0000<<3)  // VSHUNT bit 1 Conversion time - See table 5 spec  204us
#define INA3221_CONFIG_VSH_CT3                  (0x0002<<3)  // VSHUNT bit 2 Conversion time - See table 5 spec  332us
#define INA3221_CONFIG_VSH_CT4                  (0x0003<<3)  // VSHUNT bit 1 Conversion time - See table 5 spec  588us
#define INA3221_CONFIG_VSH_CT5                  (0x0004<<3)  // VSHUNT bit 0 Conversion time - See table 5 spec 1100us
#define INA3221_CONFIG_VSH_CT6                  (0x0005<<3)  // VSHUNT bit 2 Conversion time - See table 5 spec 2116us
#define INA3221_CONFIG_VSH_CT7                  (0x0006<<3)  // VSHUNT bit 1 Conversion time - See table 5 spec 4156us
#define INA3221_CONFIG_VSH_CT8                  (0x0007<<3)  // VSHUNT bit 0 Conversion time - See table 5 spec 8244us

#define INA3221_CONFIG_MODE_2                   (0x0004)  // Operating Mode bit 2 - See table 6 spec
#define INA3221_CONFIG_MODE_1                   (0x0002)  // Operating Mode bit 1 - See table 6 spec
#define INA3221_CONFIG_MODE_0                   (0x0001)  // Operating Mode bit 0 - See table 6 spec

/*=========================================================================*/

/*=========================================================================*/
#define INA3221_REG_VSH_1			            (0x01)
#define INA3221_REG_VBUS_1          		    (0x02)

#define INA3221_REG_SHUNT_CRIT_LIM_1			(0x07)

#define INA3221_REG_SHUNT_WARN_LIM_1			(0x08)

#define INA3221_REG_SHUNT_SUM					(0x0D)
#define INA3221_REG_SHUNT_SUM_LIM				(0x0E)

#define INA3221_REG_MASK_ENABLE					(0x0F)
/*=========================================================================*/

/*=========================================================================*/
#define INA3221_BIT_CRIT_FLAG_CHAN1				(0x0200)

#define INA3221_BIT_WARN_FLAG_CHAN1				(0x0020)

#define INA3221_BIT_SUMM_FLAG					(0x0040)
/*=========================================================================*/
#define SHUNT_100m  (0.1)   // default shunt resistor value of 0.1 Ohm
#define SHUNT_10m  (0.01)   // default shunt resistor value of 0.01 Ohm
typedef struct {
    uint8_t ADDR;
    float SHUNTR[3];
    float CRIT_CURR[3];
    float WARN_CURR[3];


	uint16_t CHANNEL1_ENABLE;
	uint16_t CHANNEL2_ENABLE;
	uint16_t CHANNEL3_ENABLE;
    uint16_t AVG;
	uint16_t VBUS_CT;
	uint16_t VSH_CT;

    int16_t VBUS_raw[3];
    int16_t VSHUNT_raw[3];
    uint8_t CRIT_FLAG[3];
    uint8_t WARN_FLAG[3];
    uint8_t SUMM_FLAG;

    float VBUS[3];
    float VSHUNT[3];
    float CURR[3];
    float POW[3];

} INA3221;

void INA3221_wireWriteRegister(INA3221 *ina3221, uint8_t reg, uint16_t value);
void INA3221_wireReadRegister(INA3221 *ina3221, uint8_t reg, uint16_t *value);
void INA3221_Begin(I2C_HandleTypeDef *_td, INA3221 *ina3221, uint8_t DEFAULT);
int16_t INA3221_getBusVoltage_raw(INA3221 *ina3221, uint8_t channel);
int16_t INA3221_getShuntVoltage_raw(INA3221 *ina3221, uint8_t channel);
void INA3221_read(INA3221 *ina3221);

void INA3221_setCritCurrent(INA3221 *ina3221, uint8_t channel, uint16_t value);
void INA3221_setWarnCurrent(INA3221 *ina3221, uint8_t channel, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* _INA3221_H_ */
