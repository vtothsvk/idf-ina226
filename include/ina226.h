#ifndef INA226_H_
#define INA226_H_

#include "stdint.h"
#include <esp_system.h>
#include "esp_err.h"
#include "i2cdev.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define _DEBUG

#define PIN_CONFIG_TO_ADDR(a1, a0) ((0x40) | (a1 << 2) | (a0))
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define INA266_MSB 32768//2^15
#define INA226_CAL 0.00512

#define INA226_DEFAULT_I2C_FREQ 1000000

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

#define INA226_DEFAULT_MODE INA226_CONFIG_MODE_SH_N_BUS_V_CONT

/**
 * INA226 configuration parameters structure
 */
typedef struct ina226_config{
    uint8_t resolution;
    uint16_t averaging;
    uint8_t mode;
    float shunt;
    float maxI;
}ina226_config_t;

/**
 * INA226 calibration parameters structure
 */
typedef struct ina226_calibaration{
    uint16_t cal;
    float lsb;
}ina226_calibration_t;

/**
 * Raw sensor values
 */
typedef struct ina226_data_raw{
    union{
        int rawCurrent;
        char rawCurrent_c[2];
    };
    union{
        int rawVoltage;
        char rawVoltage_c[2];
    };
}ina226_data_raw_t;

/**
 * Floating point sensor values
 */
typedef struct ina226_data{
    float current;
    float voltage;
}ina226_data_t;

/**
 * INA226 sensor device data structure
 */
typedef struct ina226{
    i2c_dev_t i2c;
    ina226_calibration_t calibration;
    ina226_data_raw_t lastData;
}ina226_t;

/**
 * @brief Initialize device descriptor
 * 
 * @param ina Device descriptor
 * @param addr I2C address
 * @param port I2C port number
 * @param sda I2C SDA GPIO pin
 * @param scl I2C SCL GPIO pin
 * 
 * @returns
 *      ESP_OK if successfull
 *      ESP_ERR othervise 
 */
esp_err_t i226InitDesc(ina226_t* ina, uint8_t addr, i2c_port_t port, gpio_num_t sda, gpio_num_t scl);

/**
 * @brief Initialize INA226 sensor
 * 
 * @param ina Device descriptor
 * @param config INA226 configuration
 * 
 * @returns
 *      ESP_OK if successfull
 *      ESP_ERR othervise 
 */
esp_err_t i226InitSensor(ina226_t* ina, ina226_config_t config);

/**
 * @brief Pulls for a measurement and get its result in floating point representation
 * 
 * @param ina Device descriptor
 * @param data pointer to a data structure to be filled with the result
 * 
 * @returns
 *      ESP_OK if successfull
 *      ESP_ERR othervise 
 */
esp_err_t i226GetMeasurement(ina226_t* ina, ina226_data_t* data);

/**
 * @brief Reads data measured by the INA226 sensor
 * 
 * @param ina Device descriptor
 * 
 * @returns
 *      ESP_OK if successfull
 *      ESP_ERR othervise 
 */
esp_err_t i226ReadI(ina226_t* ina);

/**
 * @brief Gets result of a measurement in floating point representation
 * 
 * @param ina Device descriptor
 * @param data pointer to a data structure to be filled with the result
 * 
 * @returns
 *      ESP_OK if successfull
 *      ESP_ERR othervise 
 */
esp_err_t i226GetResults(ina226_t* ina, ina226_data_t* data);

/**
 * @brief 2 byte I2C write operation to a given 8 register
 * 
 * @param i2c I2C device descriptor
 * @param reg Register address
 * @param data data to be written
 * 
 * @returns
 *      ESP_OK if successfull
 *      ESP_ERR othervise 
 */
esp_err_t i226u16write(i2c_dev_t* i2c, uint8_t reg, uint16_t data);

#ifdef __cplusplus
}
#endif

#endif//INA226_H_