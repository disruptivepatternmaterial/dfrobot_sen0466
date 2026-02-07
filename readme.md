DFRobot multi gas sensor, currently only for CO sensor (SEN0466), others should be easy to add. 
Uses I2C communications mode, no interrupt / threshold supported.

Always does temperature compensation (using algorithm in Arduino library).

Large parts blatently copied from DFRobot MultiGas Arduino library
https://github.com/DFRobot/DFRobot_MultiGasSensor

https://www.dfrobot.com/product-2508.html
https://wiki.dfrobot.com/SKU_SEN0465toSEN0476_Gravity_Gas_Sensor_Calibrated_I2C_UART

Configuration stanza:
```
external_components:
  - source:
      type: git
      url: https://github.com/jfurtner/dfrobot_sen0466.git 
    components: [sen0466]

sensor:
  - platform: sen0466
    temperature:
	  name: Temperature
	  # .. other settings from sensor apply
	carbon_monoxide:
	  name: Carbon Monoxide
	  # .. other settings from sensor apply
    address: 0x74
	  # Default address, see table below
    skip_checksum: false
      # Set true if your device (e.g. at 0x36) uses a different checksum; responses are still parsed and invalid ADC values yield NAN.
```

Address dip switches (from wiki https://wiki.dfrobot.com/SKU_SEN0465toSEN0476_Gravity_Gas_Sensor_Calibrated_I2C_UART)
| Address | A0 | A1 |
|-|-|-|
| 0x74 | 0 | 0 |
| 0x75 | 0 | 1 |
| 0x76 | 1 | 0 |
| 0x77 | 1 | 1 |

Ensure SEL dip switch set to 0

**Troubleshooting "write failed" / temp_ADC 0**: Check the I2C bus scan in the ESPHome boot log. The CO sensor must appear at the address you set (default 0x74). If you see a device at **0x36** but not 0x74, try `address: 0x36` in your config. Some boards or DIP settings use different addresses.

**Troubleshooting "checksum doesn't match" at 0x36**: If the device at 0x36 responds but checksum fails every time, it may use a different checksum. Add `skip_checksum: true` so the component still parses the response. Invalid temperature ADC (0 or ≥1023) will still yield NAN; if the frame format differs, readings may remain NAN or incorrect until the device sends valid data.

## Changelog

### Unreleased
- **Setup: set acquire mode to PASSIVITY**: After power-up the sensor may be in INITIATIVE mode (it pushes data). We send Change Get Method (0x78) with mode 0x04 (PASSIVITY) once in `setup()` so it responds to GET_TEMP / GET_GAS requests. Required for reliable I2C readback.
- **Temperature read fix**: Add 10 ms delay between I2C write and read (matches DFRobot library) so the sensor has time to respond; previously the MCU could read before the sensor filled the response, yielding zeros and -273.15°C.
- **Invalid reading handling**: Validate raw temperature ADC (reject 0 and ≥1023) and return NAN instead of running thermistor math on invalid data. On checksum failure, return NAN instead of -100.0.
- **Update path**: When temperature is invalid (NAN), publish NAN for both temperature and CO sensors for that cycle instead of -273.15°C or -100, and skip gas read so invalid temp is not used for compensation.
