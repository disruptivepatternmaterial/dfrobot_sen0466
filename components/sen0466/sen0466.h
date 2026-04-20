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
        void set_skip_checksum(bool skip) { skip_checksum_ = skip; }
        void set_warm_up_seconds(uint32_t s) { warm_up_seconds_ = s; }
        void set_reinit_interval(uint16_t cycles) { reinit_interval_ = cycles; }
        void set_stale_threshold(uint8_t count) { stale_threshold_ = count; }
        void set_reference_temperature_sensor(sensor::Sensor *s) { reference_temp_sensor_ = s; }
        void set_temp_divergence_limit(float limit) { temp_divergence_limit_ = limit; }

      protected:
        float read_temperature_C();
        float read_gas_ppm(float temperature);
        sProtocol_t pack_output_buffer(uint8_t*, uint8_t);
        uint8_t calculate_data_checksum(uint8_t* i,uint8_t ln);
        void call_sensor(uint8_t command, uint8_t* result);
        bool set_acquire_mode(uint8_t mode);
        void reinit_sensor_();

        sensor::Sensor *temperature_sensor_{nullptr};
        sensor::Sensor *carbon_monoxide_sensor_{nullptr};
        sensor::Sensor *reference_temp_sensor_{nullptr};
        float temperature_offset_;
        bool skip_checksum_{false};
        uint32_t warm_up_seconds_{0};
        bool warm_up_done_{false};
        uint8_t consecutive_invalid_count_{0};

        uint16_t reinit_interval_{30};
        uint16_t update_count_{0};

        uint8_t stale_threshold_{10};
        float last_co_value_{NAN};
        uint8_t stale_co_count_{0};

        float temp_divergence_limit_{8.0};

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