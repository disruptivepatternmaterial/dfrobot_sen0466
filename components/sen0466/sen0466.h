#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/i2c/i2c.h"

// ref:
// https://github.com/DFRobot/DFRobot_OzoneSensor

namespace esphome {
  namespace sen0466_sensor {
    typedef struct
    {
      uint8_t head;
      uint8_t addr;
      uint8_t data[6];
      uint8_t check;
    } sProtocol_t;

    class Sen0466Sensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
      public:
        void setup() override;
        void update() override;
        void dump_config() override;

        void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor;}
        void set_carbon_monoxide_sensor(sensor::Sensor *carbon_monoxide_sensor) { carbon_monoxide_sensor_ = carbon_monoxide_sensor;}
        void set_temperature_offset(float temperature_offset) { temperature_offset_ = temperature_offset; }

      protected:
        float read_temperature_C();
        float read_gas_ppm(float temperature);
        sProtocol_t pack_output_buffer(uint8_t*, uint8_t);
        uint8_t calculate_data_checksum(uint8_t* i,uint8_t ln);
        void call_sensor(uint8_t command, uint8_t* result);
        bool set_acquire_mode(uint8_t mode);

        sensor::Sensor *temperature_sensor_{nullptr};
        sensor::Sensor *carbon_monoxide_sensor_{nullptr};
        float temperature_offset_;

        // not a special value, just a random 0 hanging out in read/write commands
        // that took a bit to understand meaning
        static const uint8_t CMD_I2C_REGISTER           = 0x00;
        static const uint8_t CMD_CHANGE_GET_METHOD      = 0X78;
        static const uint8_t CMD_GET_GAS_CONCENTRATION  = 0X86;
        static const uint8_t CMD_GET_TEMP               = 0X87;
        static const uint8_t CMD_GET_ALL_DTTA           = 0X88;
        static const uint8_t CMD_SET_THRESHOLD_ALARMS   = 0X89;
        static const uint8_t CMD_IIC_AVAILABLE          = 0X90;
        static const uint8_t CMD_SENSOR_VOLTAGE         = 0X91;
        static const uint8_t CMD_CHANGE_IIC_ADDR        = 0X92;
    };

  }  // namespace sen0466_sensor
}  // namespace esphome