/*!
 *  @file mpl115a2.h
 */

#pragma once

#include <i2cdev.h>

/*!
 *  @brief  Class that stores state and functions for interacting with
 *          MPL115A2 barometric pressure sensor
 */
class MPL115A2 {
 public:
  /**< I2C address **/
  static constexpr uint8_t DEFAULT_ADDRESS{0x60};
  /**< 10-bit Pressure ADC output value MSB **/
  static constexpr uint8_t REGISTER_PRESSURE_MSB{0x00};
  /**< 10-bit Pressure ADC output value LSB **/
  static constexpr uint8_t REGISTER_PRESSURE_LSB{0x01};
  /**< 10-bit Temperature ADC output value MSB **/
  static constexpr uint8_t REGISTER_TEMP_MSB{0x02};
  /**< 10-bit Temperature ADC output value LSB **/
  static constexpr uint8_t REGISTER_TEMP_LSB{0x03};
  /**< a0 coefficient MSB **/
  static constexpr uint8_t REGISTER_A0_COEFF_MSB{0x04};
  /**< a0 coefficient LSB **/
  static constexpr uint8_t REGISTER_A0_COEFF_LSB{0x05};
  /**< b1 coefficient MSB **/
  static constexpr uint8_t REGISTER_B1_COEFF_MSB{0x06};
  /**< b1 coefficient LSB **/
  static constexpr uint8_t REGISTER_B1_COEFF_LSB{0x07};
  /**< b2 coefficient MSB **/
  static constexpr uint8_t REGISTER_B2_COEFF_MSB{0x08};
  /**< b2 coefficient LSB **/
  static constexpr uint8_t REGISTER_B2_COEFF_LSB{0x09};
  /**< c12 coefficient MSB **/
  static constexpr uint8_t REGISTER_C12_COEFF_MSB{0x0A};
  /**< c12 coefficient LSB **/
  static constexpr uint8_t REGISTER_C12_COEFF_LSB{0x0B};
  /**< Start Pressure and Temperature Conversion **/
  static constexpr uint8_t REGISTER_STARTCONVERSION{0x12};

  MPL115A2(i2c_dev_t *i2c_dev);
  bool begin();

  float getPressure();
  float getTemperature();

  struct PressureTemperatureSample {
    float pressure;
    float temperature;
  };
  PressureTemperatureSample getPressureTemperature();

 private:
  struct Coefficients {
    float a0;
    float b1;
    float b2;
    float c12;
  };

  bool readCoefficients();

  i2c_dev_t *i2c_dev_;
  Coefficients calibration_;
};
