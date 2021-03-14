#ifndef INA226_H_
#define INA226_H_

#include "stdint.h"
#include <esp_system.h>
#include "esp_err.h"
#include "i2cdev.h"
#include "esp_err.h"

#define PIN_CONFIG_TO_ADDR(a1, a0) ((0x40) | (a1 << 2) | (a0))
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define DEFAULT_CALIBRATION 256//LSB = 1mA @ .02R shunt
#define INA266_MSB 32768//2^15
#define INA226_CAL 0.00512

#define I2C_FREQ_HZ 1000000

/* *///Resolution///* */
#define RES_9b  0
#define RES_10b 1
#define RES_11b 2
#define RES_12b 3
#define RES_13b 4
#define RES_14b 5
#define RES_15b 6
#define RES_16b 7

#define INA226_DEFAULT_RESOLUTION  RES_12b

#define INA226_CONF_REG     0x00
#define INA226_V_REG        0x02
#define INA226_I_REG        0x04
#define INA226_CAL_REG      0x05

/* *///Configuration//* */
#define INA226_CONFIG_RESET                 0x8000

//Averaging
#define INA226_CONFIG_AVERAGING_1           0x0000
#define INA226_CONFIG_AVERAGING_4           0x0200
#define INA226_CONFIG_AVERAGING_16          0x0400
#define INA226_CONFIG_AVERAGING_64          0x0600
#define INA226_CONFIG_AVERAGING_129         0x0800
#define INA226_CONFIG_AVERAGING_256         0x0A00
#define INA226_CONFIG_AVERAGING_512         0x0C00
#define INA226_CONFIG_AVERAGING_1024        0x0E00

#define INA226_DEFAULT_AVERAGING INA226_CONFIG_AVERAGING_129

//Bus voltage conversion time
#define INA226_CONFIG_VBUS_140              0x0000//9b ADC
#define INA226_CONFIG_VBUS_204              0x0040//10b ADC
#define INA226_CONFIG_VBUS_332              0x0080//11b ADC
#define INA226_CONFIG_VBUS_588              0x00C0//12b ADC
#define INA226_CONFIG_VBUS_1100             0x0100//13b ADC
#define INA226_CONFIG_VBUS_2116             0x0140//14b ADC
#define INA226_CONFIG_VBUS_4156             0x0180//15b ADC
#define INA226_CONFIG_VBUS_8244             0x01C0//16b ADC

//Shunt voltage conversion time
#define INA226_CONFIG_SHUNT_140             0x0000//9b ADC
#define INA226_CONFIG_SHUNT_204             0x0008//10b ADC
#define INA226_CONFIG_SHUNT_332             0x0010//11b ADC
#define INA226_CONFIG_SHUNT_588             0x0018//12b ADC
#define INA226_CONFIG_SHUNT_1100            0x0020//13b ADC
#define INA226_CONFIG_SHUNT_2116            0x0028//14b ADC
#define INA226_CONFIG_SHUNT_4156            0x0030//15b ADC
#define INA226_CONFIG_SHUNT_8244            0x0038//16b ADC

//Mode
#define INA226_CONFIG_MODE_P_DOWN           0x0000
#define INA226_CONFIG_MODE_SH_V_TRIG        0x0001
#define INA226_CONFIG_MODE_BUS_V_TRIG       0x0002
#define INA226_CONFIG_MODE_SH_N_BUS_TRIG    0x0003
#define INA226_CONFIG_MODE_P_DOWN2          0x0004
#define INA226_CONFIG_MODE_SH_V_CONT        0x0005
#define INA226_CONFIG_MODE_BUS_V_CONT       0x0006
#define INA226_CONFIG_MODE_SH_N_BUS_V_CONT  0x0007

#define INA266_DEFAULT_MODE INA226_CONFIG_MODE_SH_N_BUS_V_CONT

typedef struct ina226_config{
    uint8_t resolution;
    uint16_t averaging;
    uint8_t mode;
    float shunt;
    float maxI;
}ina226_config_t;

typedef struct ina226_calibaration{
    uint16_t cal;
    float lsb;
}ina226_calibration_t;

typedef struct ina226{
    i2c_dev_t i2c;
    ina226_config_t config;
    ina226_calibration_t calibration;
}ina226_t;

typedef struct ina_data_raw{
    union{
        int rawCurrent;
        char rawCurrent_c[2];
    };
    union{
        int rawVoltage;
        char rawVoltage_c[2];
    };
}ina_data_raw_t;

typedef struct ina_data{
    float current;
    float voltage;
}ina_data_t;

esp_err_t initDesc(ina226_t* ina, uint8_t addr, i2c_port_t port, gpio_num_t sda, gpio_num_t scl);
esp_err_t initSensor(ina226_t* ina, ina226_config_t config);
esp_err_t readI(ina226_t* ina, ina_data_raw_t* data);
esp_err_t getResults(ina226_t* ina, ina_data_t* data);
esp_err_t u16write(i2c_dev_t* i2c, uint8_t reg, uint16_t data);

#endif//INA226_H_