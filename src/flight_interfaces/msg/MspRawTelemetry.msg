# Raw MSP v1 fields as returned by a candidate legacy controller.
# No SI conversion, axis mapping, covariance, or sensor validity is implied.
builtin_interfaces/Time stamp
bool ident_available
uint8 firmware_version
uint8 vehicle_type
uint8 msp_version
uint32 capability
bool status_available
uint16 cycle_time_us
uint16 i2c_errors
uint16 sensor_flags
uint32 mode_flags
uint8 profile
bool analog_available
uint8 battery_raw
uint16 power_meter_raw
uint16 rssi_raw
uint16 current_raw
bool raw_imu_available
# Archive order: ax, ay, az, gx, gy, gz, mx, my, mz; native units unknown.
int16[9] raw_imu
