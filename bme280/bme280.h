#ifndef _BME280_H_
#define _BME280_H_

#define BME280_I2C_ADDRESS 0x77
#define I2C_TIMEOUT 3

#define BME280_REG_DIG_T1 0x88
#define BME280_REG_DIG_H1 0xa1
#define BME280_REG_ID 0xd0
#define BME280_REG_RESET 0xe0
#define BME280_REG_DIG_H2 0xe1
#define BME280_REG_CTRL_HUM 0xf2
#define BME280_REG_CTRL_MEAS 0xf4
#define BME280_REG_CONFIG 0xf5

#define BME280_REG_TEMP_MSB 0xfa
#define BME280_REG_PRESS_MSB 0xf7
#define BME280_REG_HUM_MSB 0xfd

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64


#define APP_ERROR_SENSOR_FAILED 1
typedef enum app_error {
  E_NO_ERROR,
  E_PRESSURE_VAR2_ZERO, 
  E_SENSOR_IO_FAILED,
} app_error_t;

//----------------------------------------------------------------------------- ----------------------------------------
// A list of event IDs handled by this plugin
//
typedef enum event_type {
  EVID_NONE,
  EVID_UNKNOWN,
  
  // A full list of events can be found with:  `grep -r --color  "void.*set_.*_callback"  applications/gui/*`
  // ...A free gift to you from the makers of well written code that conforms to a good coding standard
  EVID_KEY,
} event_type_t;

typedef struct app_event {
  event_type_t type;
  InputEvent input;
} app_event_t;

// ...from the datasheet. note the signedness of different values.
typedef struct bme280_calib {
  uint16_t dig_T1;
  int16_t dig_T2, dig_T3;
  
  uint16_t dig_P1;
  int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
  
  uint8_t dig_H1;
  uint16_t dig_H2;
  uint8_t dig_H3;
  uint16_t dig_H4, dig_H5;
  int8_t dig_H6;
} bme280_calib_t;


typedef struct state {
  bme280_calib_t calibration_data;

  bool bme280_not_found;
  
  int32_t t_fine;
  float temperature, pressure, humidity;
} state_t;
#endif
