#ifndef __BMP280_BW_H__
#define __BMP280_BW_H__

#include <bme280.h>

typedef void (*delay_callback)(uint32_t msec, void *dev_addr);

class Bme280BoschWrapper
{
  public:
    //true: uses forced mode, sensore measures values on demand
    //false: uses continuous measuring mode
    Bme280BoschWrapper(bool forcedMode, delay_callback = nullptr);

    bool beginI2C(uint8_t dev_addr = 0x77,
                  uint8_t osr_t = BME280_OVERSAMPLING_16X,
                  uint8_t osr_h = BME280_OVERSAMPLING_16X,
                  uint8_t osr_p = BME280_OVERSAMPLING_16X,
                  uint8_t filter = BME280_FILTER_COEFF_OFF);

    bool beginSPI(int8_t cspin,
                  uint8_t osr_t = BME280_OVERSAMPLING_16X,
                  uint8_t osr_h = BME280_OVERSAMPLING_16X,
                  uint8_t osr_p = BME280_OVERSAMPLING_16X,
                  uint8_t filter = BME280_FILTER_COEFF_OFF);

    //this method performs measurement
    //be sure to call it before reading values
    bool measure();

    //Temperature in degrees of Celsius * 100
    int32_t getTemperature();

    //Relative humidity in % * 1024
    uint32_t getHumidity();

    //Air pressure in Pa
    uint32_t getPressure();

    //Set a custom delay callback
    void     setDelayCallback(delay_callback delayCallback);

    //Set the oversample and filtering
    bool     setSampleFilterSettings(uint8_t osr_t = BME280_OVERSAMPLING_16X,
                                  uint8_t osr_h = BME280_OVERSAMPLING_16X,
                                  uint8_t osr_p = BME280_OVERSAMPLING_16X,
                                  uint8_t filter = BME280_FILTER_COEFF_OFF);

  private:
    void I2CInit();
    void SPIInit();
    int8_t setSensorSettings();

    static int8_t I2CRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, void *dev_addr);
    static int8_t I2CWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t cnt, void *dev_addr);
    static int8_t SPIRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, void *dev_addr);
    static int8_t SPIWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t cnt, void *dev_addr);
    static void delayusec(uint32_t usec, void *dev);

    static int _cs;

    struct bme280_dev bme280;
    struct bme280_data comp_data;

    bool forced;
    bool error = false;
    uint32_t _req_delay;
    uint8_t  _dev_addr;
    delay_callback m_delayCallback;
};

#endif

