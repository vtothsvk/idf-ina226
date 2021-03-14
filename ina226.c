#include "ina226.h"

esp_err_t initDesc(ina226_t* ina, uint8_t addr, i2c_port_t port, gpio_num_t sda, gpio_num_t scl) {
    ina -> i2c.port = port;
    ina -> i2c.addr = addr;
    ina -> i2c.cfg.sda_io_num = sda;
    ina -> i2c.cfg.scl_io_num = scl;
    ina -> i2c.cfg.master.clk_speed = I2C_FREQ_HZ;

    return i2c_dev_create_mutex(&ina -> i2c);
}//initDesc

esp_err_t initSensor(ina226_t* ina, ina226_config_t config) {
    esp_err_t ret = ESP_OK;

    ina -> calibration.lsb = ina -> config.maxI / INA266_MSB;
    ina -> calibration.cal = (uint16_t)(INA226_CAL / (ina -> calibration.lsb * ina -> config.shunt));
  
    ret = u16write(&ina -> i2c, INA226_CAL_REG, ina -> calibration.cal);
    if (ret) return ret;

    uint16_t conf = ina -> config.averaging | ina -> config.mode;

    switch (ina -> config.resolution) {
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
    }//switch (ina -> config.resolution)

    ret = u16write(&ina -> i2c, INA226_CONF_REG, conf);

    return ret;
}//initSensor

esp_err_t readI(ina226_t* ina, ina_data_raw_t* data) {
    esp_err_t ret = ESP_OK;

    ret = i2c_dev_read_reg(&ina -> i2c, INA226_I_REG, data -> rawCurrent_c, sizeof(data -> rawCurrent_c));
    if (ret) return ret;

    data -> rawCurrent = (data -> rawCurrent_c[0] << 8) | (data -> rawCurrent_c[1]);

    #ifdef _DEBUG
    cout << "raw uint16 data: " << this -> data.rawCurrent + 0 << endl;
    #endif

    return ret;
}//readI

esp_err_t getResults(ina226_t* ina, ina_data_t* data) {
    esp_err_t ret = ESP_OK;

    ina_data_raw_t raw;

    ret = readI(ina, &raw);
    if (ret) return ret;

    data -> current = (float)raw.rawCurrent * ina -> calibration.lsb;

    return ret;
}//getResults

esp_err_t u16write(i2c_dev_t* i2c, uint8_t reg, uint16_t data) {
    esp_err_t ret = ESP_OK;
    uint8_t buffer[2];
    buffer[1] = (data >> 8) & 0xff;
    buffer[2] = data & 0xff;

    ret = i2c_dev_write(i2c, &reg, sizeof(reg), buffer, sizeof(buffer));
    
    return ret;
}//u16write