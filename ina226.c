#include "ina226.h"

esp_err_t i226InitDesc(ina226_t* ina, uint8_t addr, i2c_port_t port, gpio_num_t sda, gpio_num_t scl) {
    ina -> i2c.port = port;
    ina -> i2c.addr = addr;
    ina -> i2c.cfg.sda_io_num = sda;
    ina -> i2c.cfg.scl_io_num = scl;
    ina -> i2c.cfg.master.clk_speed = INA226_DEFAULT_I2C_FREQ;

    return i2c_dev_create_mutex(&ina -> i2c);
}//initDesc

esp_err_t i226InitSensor(ina226_t* ina, ina226_config_t config) {
    esp_err_t ret = ESP_OK;

    ina -> calibration.lsb = config.maxI / INA266_MSB;
    ina -> calibration.cal = (uint16_t)(INA226_CAL / (ina -> calibration.lsb * config.shunt));

    #ifdef DEBUG
    printf("cal: %d\r\n", ina -> calibration.cal);
    #endif
  
    ret = i226u16write(&ina -> i2c, INA226_CAL_REG, ina -> calibration.cal);
    if (ret) {
        printf("cal ret: %d\r\n", ret);
        return ret; 
    }

    uint16_t conf = config.averaging | config.mode;

    switch (config.resolution) {
        case RES_9b:
            conf |= (INA226_CONFIG_VBUS_140 | INA226_CONFIG_SHUNT_140);
        break;
        
        case RES_10b:
            conf |= (INA226_CONFIG_VBUS_204 | INA226_CONFIG_SHUNT_204);
        break;

        case RES_11b:
            conf |= (INA226_CONFIG_VBUS_332 | INA226_CONFIG_SHUNT_332);
        break;

        case RES_12b:
            conf |= (INA226_CONFIG_VBUS_588 | INA226_CONFIG_SHUNT_588);
        break;

        case RES_13b:
            conf |= (INA226_CONFIG_VBUS_1100 | INA226_CONFIG_SHUNT_1100);
        break;

        case RES_14b:
            conf |= (INA226_CONFIG_VBUS_2116 | INA226_CONFIG_SHUNT_2116);
        break;

        case RES_15b:
            conf |= (INA226_CONFIG_VBUS_4156 | INA226_CONFIG_SHUNT_4156);
        break;

        case RES_16b:
            conf |= (INA226_CONFIG_VBUS_8244 | INA226_CONFIG_SHUNT_8244);
        break;
    }//switch (config.resolution)

    #ifdef _DEBUG
    printf("config: %d\n", conf);
    #endif
    ret = i226u16write(&ina -> i2c, INA226_CONF_REG, conf);

    #ifdef DEBUG
    printf("conf ret: %d\r\n", ret);
    #endif

    return ret;
}//initSensor

esp_err_t i226ReadI(ina226_t* ina) {
    uint8_t buffer = INA226_I_REG;
    
    esp_err_t ret = i2c_dev_write(&ina -> i2c, NULL, 0, &buffer, 1);
    if (ret) return ret;

    ret = i2c_dev_read(&ina -> i2c, NULL, 0, ina -> lastData.rawCurrent_c, sizeof(ina -> lastData.rawCurrent_c));
    if (ret) return ret;

    #ifdef DEBUG
    printf("raw data:\r\nraw[0]: %d\r\nraw[1]: %d\r\n", ina -> lastData.rawCurrent_c[0], ina -> lastData.rawCurrent_c[1]);
    //printf("raw data:\r\nraw[0]: %d\r\nraw[1]: %d\r\n", buffer[0], buffer[1]);
    #endif

    ina -> lastData.rawCurrent = (ina -> lastData.rawCurrent_c[0] << 8) | (ina -> lastData.rawCurrent_c[1]);

    #ifdef DEBUG
    printf("raw u16: %d", ina -> lastData.rawCurrent);
    #endif

    return ret;
}//readI

esp_err_t i226GetResults(ina226_t* ina, ina226_data_t* data) {
    data -> current = (float)ina -> lastData.rawCurrent * ina -> calibration.lsb;
    return ESP_OK;
}

esp_err_t i226GetMeasurement(ina226_t* ina, ina226_data_t* data) {
    esp_err_t ret = i226ReadI(ina);
    if (ret) return ret;

    i226GetResults(ina, data);

    return ret;
}//getResults

esp_err_t i226u16write(i2c_dev_t* i2c, uint8_t reg, uint16_t data) {
    uint8_t buffer[3];
    buffer[0] = reg;
    buffer[1] = (data >> 8) & 0xff;
    buffer[2] = data & 0xff;
    
    #ifdef DEBUG
    printf("data: %d\r\n0x%x 0x%x\r\n", data, buffer[1], buffer[2]);
    #endif

    esp_err_t ret = i2c_dev_write(i2c, NULL, 0, &buffer[0], sizeof(buffer));
    
    return ret;
}//u16write