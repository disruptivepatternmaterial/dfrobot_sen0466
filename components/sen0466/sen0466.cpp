#include "sen0466.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cmath>

namespace esphome {
  namespace sen0466_sensor {

    static const char *const TAG = "sen0466_sensor.sensor";

    void Sen0466Sensor::setup() {
      ESP_LOGCONFIG(TAG, "Setting up sen0466...");
      // Sensor may power up in INITIATIVE mode; set PASSIVITY so it responds to our read requests
      this->set_timeout(500, [this]() {
        if (this->set_acquire_mode(0x04)) {  // 0x04 = PASSIVITY (host polls)
          ESP_LOGI(TAG, "Acquire mode set to PASSIVITY");
        } else {
          ESP_LOGW(TAG, "Change acquire mode failed (sensor may already be in passivity)");
        }
      });
    }

    bool Sen0466Sensor::set_acquire_mode(uint8_t mode) {
      uint8_t protocol_data[6] = {0};
      protocol_data[0] = CMD_CHANGE_GET_METHOD;
      protocol_data[1] = mode;
      sProtocol_t writeData = pack_output_buffer(protocol_data, sizeof(protocol_data));
      if (!this->write_bytes(CMD_I2C_REGISTER, (uint8_t*)&writeData, sizeof(writeData)))
        return false;
      delay(25);
      uint8_t result[9] = {0};
      if (!this->read_bytes(CMD_I2C_REGISTER, result, 9))
        return false;
      return (result[2] == 1);
    }

    void Sen0466Sensor::update() {
      ESP_LOGV(TAG, "update start");
      float temperature = read_temperature_C();

      if (std::isnan(temperature)) {
        temperature_sensor_->publish_state(NAN);
        carbon_monoxide_sensor_->publish_state(NAN);
        ESP_LOGV(TAG, "update end");
        return;
      }

      ESP_LOGD(TAG, "update temperature: %f", temperature);

      float offset = temperature_offset_;
      ESP_LOGD(TAG, "update offset: %f", offset);

      float calc_temperature = temperature + offset;
      ESP_LOGD(TAG, "update calc temperature: %f", calc_temperature);

      temperature_sensor_->publish_state(calc_temperature);

      float gas = read_gas_ppm(calc_temperature);
      carbon_monoxide_sensor_->publish_state(gas);

      ESP_LOGV(TAG, "update end");
    }

    void Sen0466Sensor::dump_config() {
      ESP_LOGCONFIG(TAG, "DF Robot gas sen0466:");
      LOG_I2C_DEVICE(this);
      if (skip_checksum_) {
        ESP_LOGCONFIG(TAG, "  Skip checksum: YES (use if device at 0x36 uses different checksum)");
      }
      if (this->is_failed()) {
        ESP_LOGE(TAG, "Communication with sen0466 failed!");
      }
      LOG_UPDATE_INTERVAL(this);
    }

    void Sen0466Sensor::call_sensor(uint8_t command, uint8_t* result)
    {
      uint8_t protocol_data[6] = {0};
      protocol_data[0] = command;
      sProtocol_t writeData = pack_output_buffer(protocol_data, sizeof(protocol_data));
      ESP_LOGD(TAG, "call_sensor setup output buffer");

      const int max_tries = 3;
      const int delay_ms = 25;
      bool read_ok = false;

      for (int try_ = 0; try_ < max_tries && !read_ok; try_++) {
        if (try_ > 0) {
          delay(20);
        }
        if (!this->write_bytes(CMD_I2C_REGISTER, (uint8_t*)&writeData, sizeof(writeData))) {
          ESP_LOGW(TAG, "call_sensor write failed (try %d)", try_ + 1);
          continue;
        }
        delay(delay_ms);
        read_ok = this->read_bytes(CMD_I2C_REGISTER, result, (uint8_t) 9);
        if (!read_ok) {
          ESP_LOGW(TAG, "call_sensor read failed (try %d)", try_ + 1);
        }
      }
      ESP_LOGD(TAG, "call_sensor send/read: %s", read_ok ? "ok" : "fail");

      ESP_LOGV(TAG, "call_sensor result: %02x %02x %02x %02x %02x %02x %02x %02x %02x",
        result[0], result[1], result[2], result[3], result[4], result[5],
        result[6], result[7], result[8]);
    }

    float Sen0466Sensor::read_temperature_C(void)
    {
      ESP_LOGV(TAG, "read_temperature_C start");
      uint8_t result[9] = {0};
      call_sensor(CMD_GET_TEMP, (uint8_t*)&result);

      uint8_t expected_csum = calculate_data_checksum(result, 8);
      if (result[8] != expected_csum)
      {
        if (!skip_checksum_) {
          ESP_LOGE(TAG, "read_temperature_C checksum doesn't match! got 0x%02X expected 0x%02X",
                   result[8], expected_csum);
          ESP_LOGD(TAG, "read_temperature_C raw: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                   result[0], result[1], result[2], result[3], result[4], result[5],
                   result[6], result[7], result[8]);
          return NAN;
        }
        ESP_LOGW(TAG, "read_temperature_C checksum mismatch (got 0x%02X expected 0x%02X), parsing anyway (skip_checksum)",
                 result[8], expected_csum);
      }

      ESP_LOGD(TAG, "read_temperature_C calculate actual");
      uint16_t temp_ADC = (result[2] << 8) + result[3];

      ESP_LOGD(TAG, "read_temperature_C temp_ADC: %u", temp_ADC);

      if (temp_ADC == 0 || temp_ADC >= 1023)
      {
        ESP_LOGW(TAG, "read_temperature_C invalid temp_ADC %u", temp_ADC);
        return NAN;
      }

      float vPd3 = 3*(float)temp_ADC/1024;
      ESP_LOGD(TAG, "read_temperature_C vPd3: %f", vPd3);

      float rTh = vPd3*10000/(3-vPd3);
      ESP_LOGD(TAG, "read_temperature_C rTh: %f", rTh);

      float tBeta = 1/(1/(273.15+25)+1/3380.13*log(rTh/10000))-273.15;
      ESP_LOGD(TAG, "read_temperature_C tBeta: %f", tBeta);

      ESP_LOGV(TAG, "read_temperature_C end");
      return tBeta;
    }

    float Sen0466Sensor::read_gas_ppm(float temperature)
    {
      ESP_LOGV(TAG, "read_gas_ppm start");
      ESP_LOGD(TAG, "read_gas_ppm read data from sensor");
      uint8_t result[9] = {0xff};

      call_sensor(CMD_GET_GAS_CONCENTRATION, result);

      ESP_LOGD(TAG, "read_gas_ppm got data from sensor, correct for temperature");

      // these calculations from DFRobot sources
      uint8_t expected_csum = calculate_data_checksum(result, 8);
      if (expected_csum != result[8])
      {
        if (!skip_checksum_) {
          ESP_LOGE(TAG, "read_gas_ppm checksum invalid (got 0x%02X expected 0x%02X)",
                   result[8], expected_csum);
          ESP_LOGD(TAG, "read_gas_ppm raw: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                   result[0], result[1], result[2], result[3], result[4], result[5],
                   result[6], result[7], result[8]);
          return -2.0;
        }
        ESP_LOGW(TAG, "read_gas_ppm checksum mismatch (got 0x%02X expected 0x%02X), parsing anyway (skip_checksum)",
                 result[8], expected_csum);
      }

      float concentration = ((result[2]<<8) + result[3])*1.0;
      ESP_LOGD(TAG, "read_gas_ppm initial concentration: %f", concentration);

      ESP_LOGD(TAG, "read_gas_ppm multiplier: %d", result[5]);
      switch (result[5]) // always 0 with sen0466?
      {
        case 1:
          concentration *= 0.1;
          break;
        case 2:
          concentration *= 0.01;
          break;
        default:
          break;
      }

      ESP_LOGD(TAG, "read_gas_ppm temperature correction");
      if (((temperature) > -20) && ((temperature) <= 20))
      {
        ESP_LOGD(TAG, "read_gas_ppm correction for -20<t<=20 C");
        concentration = (concentration / (0.005 * (temperature) + 0.9));
      }
      else if (((temperature) > 20) && ((temperature) <= 40))
      {
        ESP_LOGD(TAG, "read_gas_ppm correction for 20<t<=40 C");
        concentration = (concentration / (0.005 * (temperature) + 0.9) - (0.3 * (temperature)-6));
      }
      else
      {
        ESP_LOGE(TAG, "read_gas_ppm temperature out of operating range");
        return -1.0;
      }

      if (concentration < 0)
      {
        ESP_LOGW(TAG, "read_gas_ppm clamping concentration to 0 (was %f)", concentration);
        concentration = 0.0;
      }
      
      return concentration;
    }

    /// Pack data buffer for i2c messages. _protocol.data contains command to execute
    sProtocol_t Sen0466Sensor::pack_output_buffer(uint8_t *pBuf, uint8_t len)
    {
      sProtocol_t _protocol;
      _protocol.head = 0xff;
      _protocol.addr = 0x01;
      memcpy(_protocol.data, pBuf, len);
      _protocol.check = calculate_data_checksum((uint8_t *)&_protocol, 8);
      return _protocol;
    }

    /// Called CRC in DFRobot sources, unsure if exactly CRC or what
    uint8_t Sen0466Sensor::calculate_data_checksum(uint8_t* i, uint8_t ln)
    {
      uint8_t j,tempq=0;
      i+=1;
      for(j=0;j<(ln-2);j++)
      {
        tempq+=*i;i++;
      }
      tempq=(~tempq)+1;
      return(tempq);
    }

  }  // namespace sen0466_sensor
}  // namespace esphome
