/*!
 *  @file mpl115a2.cpp
 *
 *  @mainpage Driver for the Adafruit MPL115A2 barometric pressure sensor
 *
 *  @section intro_sec Introduction
 *
 *  Driver for the MPL115A2 barometric pressure sensor
 *
 *  This is a library for the Adafruit MPL115A2 breakout
 *  ----> https://www.adafruit.com/products/992
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *
 *  @section author Author
 *
 *  K.Townsend (Adafruit Industries)
 *
 *  @section license License
 *
 *  BSD (see license.txt)
 *
 *  @section history
 *
 *  v1.0 - First release
 *  v1.1 - Rick Sellens added casts to make bit shifts work below 22.6C
 *       - get both P and T with a single call to getPressureTemperature
 */

#include <array>
#include <thread>
#include <chrono>

#include <esp_log.h>

#include "mpl115a2.h"

constexpr const char* TAG = "MPL115A2";

static inline void delay(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds{ms});
}

#define MPL_CHECK(tag, result, return_val)                     \
  do {                                                         \
    if (const auto status = result; status != ESP_OK) {        \
      ESP_LOGE(tag, "%u %s: 0x%x", __LINE__, #result, status); \
      return return_val;                                       \
    }                                                          \
  } while (false)

/*!
 *  @brief  Instantiates a new MPL115A2 class
 */
MPL115A2::MPL115A2(i2c_dev_t* i2c_dev) : i2c_dev_{i2c_dev}, calibration_{} {}

/*!
 *  @brief  Gets the factory-set coefficients for this particular sensor
 */
bool MPL115A2::readCoefficients() {
  const uint8_t cmd = REGISTER_A0_COEFF_MSB;
  std::array<uint8_t, 8> buffer{};

  MPL_CHECK(
      TAG,
      i2c_dev_read(i2c_dev_, &cmd, sizeof(cmd), buffer.data(), buffer.size()),
      false);

  int16_t a0coeff = ((static_cast<uint16_t>(buffer[0]) << 8) | buffer[1]);
  int16_t b1coeff = ((static_cast<uint16_t>(buffer[2]) << 8) | buffer[3]);
  int16_t b2coeff = ((static_cast<uint16_t>(buffer[4]) << 8) | buffer[5]);
  int16_t c12coeff = ((static_cast<uint16_t>(buffer[6]) << 8) | buffer[7]) >> 2;

  /*
  Serial.print("A0 = "); Serial.println(a0coeff, HEX);
  Serial.print("B1 = "); Serial.println(b1coeff, HEX);
  Serial.print("B2 = "); Serial.println(b2coeff, HEX);
  Serial.print("C12 = "); Serial.println(c12coeff, HEX);
  */

  calibration_ = Coefficients{
      .a0 = static_cast<float>(a0coeff) / 8,
      .b1 = static_cast<float>(b1coeff) / 8192,
      .b2 = static_cast<float>(b2coeff) / 16384,
      .c12 = static_cast<float>(c12coeff) / 4194304.f,
  };

  /*
  Serial.print("a0 = "); Serial.println(_mpl115a2_a0);
  Serial.print("b1 = "); Serial.println(_mpl115a2_b1);
  Serial.print("b2 = "); Serial.println(_mpl115a2_b2);
  Serial.print("c12 = "); Serial.println(_mpl115a2_c12);
  */
  return true;
}

/*!
 *  @brief  Setups the HW (reads coefficients values, etc.)
 *  @return Returns true if the device was found
 */
bool MPL115A2::begin() { return readCoefficients(); }

/*!
 *  @brief  Gets the floating-point pressure level in kPa
 *  @return Pressure in kPa
 */
float MPL115A2::getPressure() {
  const auto& [pressureComp, centigrade] = getPressureTemperature();
  return pressureComp;
}

/*!
 *  @brief  Gets the floating-point temperature in Centigrade
 *  @return Temperature in Centigrade
 */
float MPL115A2::getTemperature() {
  const auto& [pressureComp, centigrade] = getPressureTemperature();
  return centigrade;
}

/*!
 *  @brief  Gets both at once and saves a little time
 */
MPL115A2::PressureTemperatureSample MPL115A2::getPressureTemperature() {
  const std::array<uint8_t, 2> cmd{REGISTER_STARTCONVERSION, 0};

  MPL_CHECK(TAG, i2c_dev_write(i2c_dev_, cmd.data(), cmd.size(), nullptr, 0),
            {});

  // Wait a bit for the conversion to complete (3ms max)
  delay(5);

  std::array<uint8_t, 4> buffer{};
  MPL_CHECK(TAG,
            i2c_dev_read_reg(i2c_dev_, REGISTER_PRESSURE_MSB, buffer.data(),
                             buffer.size()),
            {});

  const uint16_t pressure = (((uint16_t)buffer[0] << 8) | buffer[1]) >> 6;
  const uint16_t temp = (((uint16_t)buffer[2] << 8) | buffer[3]) >> 6;

  // See datasheet p.6 for evaluation sequence
  const float pressureComp =
      calibration_.a0 + (calibration_.b1 + calibration_.c12 * temp) * pressure +
      calibration_.b2 * temp;

  // Return pressure and temperature as floating point values
  const PressureTemperatureSample result{
      .pressure = ((65.0F / 1023.0F) * pressureComp) + 50.0F,  // kPa
      .temperature = ((float)temp - 498.0F) / -5.35F + 25.0F,  // C
  };
  return result;
}
