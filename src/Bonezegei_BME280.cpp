/*
  Bonezegei_BME280.h - Arduino Library for BME280 Sensor
  Author: Jofel Batutay (Bonezegei)
  Date: October 14, 2026
*/

#include "Bonezegei_BME280.h"

Bonezegei_BME280::Bonezegei_BME280(uint8_t addr) {
  _addr = addr;
  _tFine = 0;
  _temperature = 0.0;
  _pressure = 0.0;
  _humidity = 0.0;
}

bool Bonezegei_BME280::begin(TwoWire &wirePort) {
  _wire = &wirePort;
  _wire->begin();

  // Check Chip ID (BME280 chip ID is 0x60)
  if (readRegister8(BME280_REG_CHIP_ID) != 0x60) {
    return false;
  }

  // Soft Reset
  writeRegister(BME280_REG_RESET, 0xB6);
  delay(100);

  readCalibrationData();

  // Set Humidity Oversampling to 1x (ctrl_hum = 0x01)
  writeRegister(BME280_REG_CTRL_HUM, 0x01);

  // Set Config: Standby 0.5ms, Filter Off (config = 0x00)
  writeRegister(BME280_REG_CONFIG, 0x00);

  // Set ctrl_meas: Pressure 1x, Temp 1x, Normal Mode (0x27)
  writeRegister(BME280_REG_CTRL_MEAS, 0x27);

  return true;
}

void Bonezegei_BME280::readCalibrationData() {
  _calib.dig_T1 = readRegister16_LE(BME280_REG_DIG_T1);
  _calib.dig_T2 = readRegister16_S_LE(0x8A);
  _calib.dig_T3 = readRegister16_S_LE(0x8C);

  _calib.dig_P1 = readRegister16_LE(0x8E);
  _calib.dig_P2 = readRegister16_S_LE(0x90);
  _calib.dig_P3 = readRegister16_S_LE(0x92);
  _calib.dig_P4 = readRegister16_S_LE(0x94);
  _calib.dig_P5 = readRegister16_S_LE(0x96);
  _calib.dig_P6 = readRegister16_S_LE(0x98);
  _calib.dig_P7 = readRegister16_S_LE(0x9A);
  _calib.dig_P8 = readRegister16_S_LE(0x9C);
  _calib.dig_P9 = readRegister16_S_LE(0x9E);

  _calib.dig_H1 = readRegister8(BME280_REG_DIG_H1);
  _calib.dig_H2 = readRegister16_S_LE(BME280_REG_DIG_H2);
  _calib.dig_H3 = readRegister8(0xE3);
  
  uint8_t e4 = readRegister8(0xE4);
  uint8_t e5 = readRegister8(0xE5);
  uint8_t e6 = readRegister8(0xE6);

  _calib.dig_H4 = (int16_t)((e4 << 4) | (e5 & 0x0F));
  _calib.dig_H5 = (int16_t)((e6 << 4) | (e5 >> 4));
  _calib.dig_H6 = (int8_t)readRegister8(0xE7);
}

bool Bonezegei_BME280::read() {
  _wire->beginTransmission(_addr);
  _wire->write(BME280_REG_PRESSURE_DATA);
  if (_wire->endTransmission() != 0) return false;

  _wire->requestFrom(_addr, (uint8_t)8);
  if (_wire->available() < 8) return false;

  uint32_t p_msb  = _wire->read();
  uint32_t p_lsb  = _wire->read();
  uint32_t p_xlsb = _wire->read();

  uint32_t t_msb  = _wire->read();
  uint32_t t_lsb  = _wire->read();
  uint32_t t_xlsb = _wire->read();

  uint32_t h_msb  = _wire->read();
  uint32_t h_lsb  = _wire->read();

  int32_t adc_P = (p_msb << 12) | (p_lsb << 4) | (p_xlsb >> 4);
  int32_t adc_T = (t_msb << 12) | (t_lsb << 4) | (t_xlsb >> 4);
  int32_t adc_H = (h_msb << 8) | h_lsb;

  int32_t temp_raw = compensateTemp(adc_T);
  _temperature = temp_raw / 100.0f;

  uint32_t press_raw = compensatePressure(adc_P);
  _pressure = press_raw / 256.0f;

  uint32_t hum_raw = compensateHumidity(adc_H);
  _humidity = hum_raw / 1024.0f;

  return true;
}

float Bonezegei_BME280::getTemperature() { return _temperature; }
float Bonezegei_BME280::getPressure() { return _pressure; }
float Bonezegei_BME280::getHumidity() { return _humidity; }

float Bonezegei_BME280::getAltitude(float seaLevelhPa) {
  float atmospheric = _pressure / 100.0f; // Pa to hPa
  return 44330.0f * (1.0f - pow(atmospheric / seaLevelhPa, 0.1903f));
}

int32_t Bonezegei_BME280::compensateTemp(int32_t adc_T) {
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)_calib.dig_T1 << 1))) * ((int32_t)_calib.dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)_calib.dig_T1))) >> 12) * ((int32_t)_calib.dig_T3)) >> 14;
  _tFine = var1 + var2;
  return (_tFine * 5 + 128) >> 8;
}

uint32_t Bonezegei_BME280::compensatePressure(int32_t adc_P) {
  int64_t var1 = ((int64_t)_tFine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_calib.dig_P3) >> 8) + ((var1 * (int64_t)_calib.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_calib.dig_P1) >> 33;

  if (var1 == 0) return 0; // Avoid divide by zero

  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_calib.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)_calib.dig_P7) << 4);

  return (uint32_t)p;
}

uint32_t Bonezegei_BME280::compensateHumidity(int32_t adc_H) {
  int32_t v_x1_u32r = (_tFine - ((int32_t)76800));
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)_calib.dig_H4) << 20) - (((int32_t)_calib.dig_H5) * v_x1_u32r)) + 
               ((int32_t)16384)) >> 15) * 
               (((((((v_x1_u32r * ((int32_t)_calib.dig_H6)) >> 10) * 
               (((v_x1_u32r * ((int32_t)_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * 
               ((int32_t)_calib.dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)_calib.dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  return (uint32_t)(v_x1_u32r >> 12);
}

void Bonezegei_BME280::writeRegister(uint8_t reg, uint8_t value) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write(value);
  _wire->endTransmission();
}

uint8_t Bonezegei_BME280::readRegister8(uint8_t reg) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->endTransmission();
  _wire->requestFrom(_addr, (uint8_t)1);
  return _wire->read();
}

uint16_t Bonezegei_BME280::readRegister16_LE(uint8_t reg) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->endTransmission();
  _wire->requestFrom(_addr, (uint8_t)2);
  uint8_t lsb = _wire->read();
  uint8_t msb = _wire->read();
  return (msb << 8) | lsb;
}

int16_t Bonezegei_BME280::readRegister16_S_LE(uint8_t reg) {
  return (int16_t)readRegister16_LE(reg);
}