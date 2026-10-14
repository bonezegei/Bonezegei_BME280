/*
  Bonezegei_BME280.h - Arduino Library for BME280 Sensor
  Author: Jofel Batutay (Bonezegei)
  Date: October 14, 2026
*/

#ifndef BONEZEGEI_BME280_H
#define BONEZEGEI_BME280_H

#include <Arduino.h>
#include <Wire.h>

#define BME280_I2C_ADDR_PRIMARY   0x76
#define BME280_I2C_ADDR_SECONDARY 0x77

#define BME280_REG_DIG_T1         0x88
#define BME280_REG_DIG_H1         0xA1
#define BME280_REG_CHIP_ID        0xD0
#define BME280_REG_RESET          0xE0
#define BME280_REG_DIG_H2         0xE1
#define BME280_REG_CTRL_HUM       0xF2
#define BME280_REG_STATUS         0xF3
#define BME280_REG_CTRL_MEAS      0xF4
#define BME280_REG_CONFIG         0xF5
#define BME280_REG_PRESSURE_DATA  0xF7
#define BME280_REG_TEMP_DATA      0xFA
#define BME280_REG_HUMIDITY_DATA  0xFD

struct BME280_CalibData {
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;

  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;

  uint8_t  dig_H1;
  int16_t  dig_H2;
  uint8_t  dig_H3;
  int16_t  dig_H4;
  int16_t  dig_H5;
  int8_t   dig_H6;
};

class Bonezegei_BME280 {
public:
  Bonezegei_BME280(uint8_t addr = BME280_I2C_ADDR_PRIMARY);
  
  bool begin(TwoWire &wirePort = Wire);
  bool read();
  
  float getTemperature();  // Returns Temperature in Celsius
  float getPressure();     // Returns Pressure in Pascals (Pa)
  float getHumidity();     // Returns Relative Humidity in %
  float getAltitude(float seaLevelhPa = 1013.25); // Returns Altitude in meters

private:
  uint8_t _addr;
  TwoWire *_wire;
  BME280_CalibData _calib;
  int32_t _tFine;

  float _temperature;
  float _pressure;
  float _humidity;

  void readCalibrationData();
  void writeRegister(uint8_t reg, uint8_t value);
  uint8_t readRegister8(uint8_t reg);
  uint16_t readRegister16_LE(uint8_t reg);
  int16_t readRegister16_S_LE(uint8_t reg);

  int32_t compensateTemp(int32_t adc_T);
  uint32_t compensatePressure(int32_t adc_P);
  uint32_t compensateHumidity(int32_t adc_H);
};

#endif