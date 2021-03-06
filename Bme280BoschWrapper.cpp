#include <Arduino.h>

#include <limits.h>
#include <SPI.h>
#include <Wire.h>

#include "Bme280BoschWrapper.h"

#define DOUBLE_NOT_CALCULATED -1000.0

int Bme280BoschWrapper::_cs = -1;

Bme280BoschWrapper::Bme280BoschWrapper(bool forced, delay_callback callback)
{
  this->forced = forced;
  m_delayCallback = callback;
}

void Bme280BoschWrapper::setDelayCallback(delay_callback delayCallback) {
    m_delayCallback = delayCallback;
}

bool Bme280BoschWrapper::beginI2C(uint8_t dev_addr, uint8_t osr_t, uint8_t osr_h, uint8_t osr_p, uint8_t filter)
{
  I2CInit();
  _dev_addr = dev_addr;
  bme280.intf_ptr = this;
  bme280.settings.osr_h = osr_h;
  bme280.settings.osr_p = osr_p;
  bme280.settings.osr_t = osr_t;
  bme280.settings.filter = filter;

  int8_t ret = bme280_init(&bme280);

  if(ret == BME280_OK){
    ret += setSensorSettings();
  }

  return (ret == BME280_OK);
}

bool Bme280BoschWrapper::beginSPI(int8_t cspin, uint8_t osr_t, uint8_t osr_h, uint8_t osr_p, uint8_t filter)
{
  Bme280BoschWrapper::_cs = cspin;

  SPIInit();
  pinMode(_cs, OUTPUT);

  bme280.settings.osr_h = osr_h;
  bme280.settings.osr_p = osr_p;
  bme280.settings.osr_t = osr_t;
  bme280.settings.filter = filter;

  int8_t ret = bme280_init(&bme280);

  setSensorSettings();

  return (ret == BME280_OK);
}

bool Bme280BoschWrapper::measure()
{
  int8_t ret = BME280_OK;
  if(forced)
  {
    ret += bme280_set_sensor_mode(BME280_FORCED_MODE, &bme280);
    bme280.delay_us(_req_delay, this);

    ret += bme280_get_sensor_data(BME280_PRESS | BME280_HUM | BME280_TEMP, &comp_data, &bme280);
  }
  else
  {
    ret += bme280_get_sensor_data(BME280_PRESS | BME280_HUM | BME280_TEMP, &comp_data, &bme280);
  }

  if(ret != BME280_OK) {
    error = true;
  }

  return (ret == BME280_OK);
}

int32_t Bme280BoschWrapper::getTemperature()
{
  return comp_data.temperature;
}

uint32_t Bme280BoschWrapper::getHumidity()
{
  return comp_data.humidity;
}

uint32_t Bme280BoschWrapper::getPressure()
{
  return comp_data.pressure;
}

/**
 * Wrapper functions for Bosch BME280 driver.
 */
#define SPI_READ  0x80
#define SPI_WRITE 0x7F

SPISettings bme280SpiSettings = SPISettings(2000000, MSBFIRST, SPI_MODE0);

void Bme280BoschWrapper::I2CInit()
{
  bme280.intf = BME280_I2C_INTF;
  bme280.write = Bme280BoschWrapper::I2CWrite;
  bme280.read = Bme280BoschWrapper::I2CRead;
  bme280.delay_us = Bme280BoschWrapper::delayusec;

  Wire.begin();
}

void Bme280BoschWrapper::SPIInit()
{
  bme280.intf_ptr = nullptr;
  bme280.intf = BME280_SPI_INTF;
  bme280.write = Bme280BoschWrapper::SPIWrite;
  bme280.read = Bme280BoschWrapper::SPIRead;
  bme280.delay_us = Bme280BoschWrapper::delayusec;

  SPI.begin();
}

int8_t Bme280BoschWrapper::I2CRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, void *dev)
{
//  Serial.println("I2C_bus_read");
  int8_t ret = BME280_OK;
  Bme280BoschWrapper *pDev = (Bme280BoschWrapper*)dev;

//  Serial.println(dev_addr, HEX);
  Wire.beginTransmission(pDev->_dev_addr);

//  Serial.println(reg_addr, HEX);
  Wire.write(reg_addr);

  if(Wire.endTransmission() != 0){
    return BME280_E_COMM_FAIL;
  }

  if(Wire.requestFrom(pDev->_dev_addr, cnt) != cnt){
    return BME280_E_COMM_FAIL;
  }

  uint8_t available = Wire.available();
  if(available != cnt)
  {
    ret = BME280_E_COMM_FAIL;
  }

  for(uint8_t i = 0; i < available; i++)
  {
    if(i < cnt)
    {
      *(reg_data + i) = Wire.read();
//      Serial.print(*(reg_data + i), HEX);
//      Serial.print(" ");
    }
    else
      Wire.read();
  }

//  Serial.println();

  return ret;
}

int8_t Bme280BoschWrapper::I2CWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t cnt, void *dev)
{
//  Serial.println("I2C_bus_write");
  int8_t ret = BME280_OK;
  Bme280BoschWrapper *pDev = (Bme280BoschWrapper*)dev;

//  Serial.println(dev_addr, HEX);
  Wire.beginTransmission(pDev->_dev_addr);

//  Serial.println(reg_addr, HEX);
  Wire.write(reg_addr);
  Wire.write(reg_data, cnt);
  Wire.endTransmission();

  return ret;
}

int8_t Bme280BoschWrapper::SPIRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, void *dev)
{
//  Serial.println("SPI_bus_read");
  int32_t ret = BME280_OK;

  SPI.beginTransaction(bme280SpiSettings);
  digitalWrite(_cs, LOW);

//  Serial.println(reg_addr | SPI_READ, HEX);

  SPI.transfer(reg_addr | SPI_READ);
  for (uint8_t i = 0; i < cnt; i++) {
    *(reg_data + i) = SPI.transfer(0);
//    Serial.print(*(reg_data + i), HEX);
//    Serial.print(" ");
  }

//  Serial.println();

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  return ret;
}

int8_t Bme280BoschWrapper::SPIWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t cnt, void *dev)
{
//  Serial.println("SPI_bus_write");
  int8_t ret = BME280_OK;

  SPI.beginTransaction(bme280SpiSettings);
  digitalWrite(_cs, LOW);
  for (uint8_t i = 0; i < cnt; i++)
  {
    uint8_t addr = (reg_addr++) & SPI_WRITE;
    uint8_t data = *(reg_data + i);

//    Serial.print(addr, HEX);
//    Serial.print(" ");
//    Serial.print(data, HEX);

    SPI.transfer(addr);
    SPI.transfer(data);
  }
//  Serial.println();

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  return ret;
}

void Bme280BoschWrapper::delayusec(uint32_t usec, void *dev)
{
  Bme280BoschWrapper *pDev = (Bme280BoschWrapper*)dev;
  if(pDev->m_delayCallback != nullptr) {
    pDev->m_delayCallback(usec, dev);
  } else {
    delayMicroseconds(usec);
  }
}

bool Bme280BoschWrapper::setSampleFilterSettings(uint8_t osr_t, uint8_t osr_h, uint8_t osr_p, uint8_t filter)
{
  bme280.settings.osr_h = osr_h;
  bme280.settings.osr_p = osr_p;
  bme280.settings.osr_t = osr_t;
  bme280.settings.filter = filter;

  return (setSensorSettings() == BME280_OK);
}

int8_t Bme280BoschWrapper::setSensorSettings()
{
  int8_t ret = BME280_OK;

  uint8_t settings_sel;

  settings_sel = BME280_OSR_PRESS_SEL|BME280_OSR_TEMP_SEL|BME280_OSR_HUM_SEL;

  ret += bme280_set_sensor_settings(settings_sel, &bme280);

  if(forced)
    ret += bme280_set_sensor_mode(BME280_FORCED_MODE, &bme280);
  else
    ret += bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme280);

  return ret;
}
