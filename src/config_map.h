#ifndef __CONFIG_MAP__
#define __CONFIG_MAP__

// This is autogenerated file, with EEPROM map & defaults.
// Use `npm run config` to regenerate.

// Every virtual cell is 4-byte data, float.

#define CFG_SHUNT_RESISTANCE_ADDR 1
#define CFG_SHUNT_RESISTANCE_DEFAULT 10.0

#define CFG_MOTOR_RESISTANCE_ADDR 2
#define CFG_MOTOR_RESISTANCE_DEFAULT 95.0

#define CFG_RPM_MAX_ADDR 3
#define CFG_RPM_MAX_DEFAULT 30000.0

#define CFG_POWER_MAX_ADDR 4
#define CFG_POWER_MAX_DEFAULT 230.0

#define CFG_RPM_MIN_LIMIT_ADDR 5
#define CFG_RPM_MIN_LIMIT_DEFAULT 3000.0

#define CFG_RPM_MAX_LIMIT_ADDR 6
#define CFG_RPM_MAX_LIMIT_DEFAULT 30000.0

#define CFG_PID_P_ADDR 7
#define CFG_PID_P_DEFAULT 1.0

#define CFG_PID_I_ADDR 8
#define CFG_PID_I_DEFAULT 1.0

#define CFG_DEAD_ZONE_WIDTH_ADDR 9
#define CFG_DEAD_ZONE_WIDTH_DEFAULT 2.0

#define CFG_MOTOR_INDUCTANCE_ADDR 10
#define CFG_MOTOR_INDUCTANCE_DEFAULT 0.17

#define CFG_REKV_TO_SPEED_FACTOR_ADDR 11
#define CFG_REKV_TO_SPEED_FACTOR_DEFAULT 1000.0


#endif
