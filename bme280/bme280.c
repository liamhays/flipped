// Inspiration drawn from the bc_demo example FAP at https://github.com/csBlueChip/FlipperZero_plugin_howto

#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <input/input.h>

#include "bme280_icons.h"
#include "bme280.h"

#define TAG "bme280"
// I2C includes
#include <furi_hal_i2c.h>


// declare functions to keep code order usable
static app_error_t get_temperature(state_t* state) {
  // temperature data is 20 bits
  uint8_t tx_data[1] = {BME280_REG_TEMP_MSB};
  uint8_t rx_data[3] = {0}; // data will be returned as {msb, lsb, xlsb}
  if (!furi_hal_i2c_trx(&furi_hal_i2c_handle_external,
			BME280_I2C_ADDRESS << 1,
			tx_data, 1,
			rx_data, 3,
			I2C_TIMEOUT)) {
    FURI_LOG_E(TAG, "Failed to read raw temperature data! Sensor disconnected?");
    return E_SENSOR_IO_FAILED;
  }

  // from SparkFun's Arduino library
  int32_t adc_T = ((uint32_t)rx_data[0] << 12) | ((uint32_t)rx_data[1] << 4) | ((rx_data[2] >> 4) & 0x0f);
  
  // the following calculations are all from the BME280 datasheet.
  int32_t var1, var2, final_temp, t_fine_calc;
  var1 = ((((adc_T>>3) - ((int32_t)state->calibration_data.dig_T1<<1))) * ((int32_t)state->calibration_data.dig_T2)) >> 11;
  var2 = (((((adc_T>>4) - ((int32_t)state->calibration_data.dig_T1)) * ((adc_T>>4) - ((int32_t)state->calibration_data.dig_T1))) >> 12) * ((int32_t)state->calibration_data.dig_T3)) >> 14;
  t_fine_calc = var1 + var2;
  final_temp = (t_fine_calc * 5 + 128) >> 8;

  state->t_fine = t_fine_calc;
  state->temperature = (float)final_temp / 100;

  return E_NO_ERROR;
}

static app_error_t get_pressure(state_t* state) {
  uint8_t tx_data[1] = {BME280_REG_PRESS_MSB};
  uint8_t rx_data[3] = {0}; // data will be returned as {msb, lsb, xlsb}
  if (!furi_hal_i2c_trx(&furi_hal_i2c_handle_external,
			BME280_I2C_ADDRESS << 1,
			tx_data, 1,
			rx_data, 3,
			I2C_TIMEOUT)) {
    FURI_LOG_E(TAG, "Failed to read raw pressure data! Sensor disconnected?");
    return E_SENSOR_IO_FAILED;
  }
    
  int32_t adc_P = ((uint32_t)rx_data[0] << 12) | ((uint32_t)rx_data[1] << 4) | ((rx_data[2] >> 4) & 0x0F);
  
  // straight from the datasheet
  int64_t var1, var2, p_acc;
  var1 = ((int64_t)state->t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)state->calibration_data.dig_P6;
  var2 = var2 + ((var1 * (int64_t)state->calibration_data.dig_P5) << 17);
  var2 = var2 + (((int64_t)state->calibration_data.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)state->calibration_data.dig_P3) >> 8) + ((var1 * (int64_t)state->calibration_data.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)state->calibration_data.dig_P1) >> 33;
  if (var1 == 0) {
    state->pressure = 0; // avoid exception caused by division by zero
    return E_PRESSURE_VAR2_ZERO;
  }
  p_acc = 1048576 - adc_P;
  p_acc = (((p_acc << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)state->calibration_data.dig_P9) * (p_acc >> 13) * (p_acc >> 13)) >> 25;
  var2 = (((int64_t)state->calibration_data.dig_P8) * p_acc) >> 19;
  p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)state->calibration_data.dig_P7) << 4);
  
  state->pressure = (float)p_acc / 256.0;
  return E_NO_ERROR;
}

static app_error_t get_humidity(state_t* state) {
  uint16_t adc_H_raw;
  // read_reg_16 takes care of shifting and data management for us
  if (!furi_hal_i2c_read_reg_16(&furi_hal_i2c_handle_external,
				BME280_I2C_ADDRESS << 1,
				BME280_REG_HUM_MSB,
				&adc_H_raw,
				I2C_TIMEOUT)) {
    FURI_LOG_E(TAG, "Failed to read raw pressure data! Sensor disconnected?");
    return E_SENSOR_IO_FAILED;
  }
  
  int32_t adc_H = (int32_t)adc_H_raw;
  // from the datasheet
  // what awful arithmetic
  int32_t var1;
  var1 = (state->t_fine - ((int32_t)76800));
  var1 = (((((adc_H << 14) - (((int32_t)state->calibration_data.dig_H4) << 20) - (((int32_t)state->calibration_data.dig_H5) * var1)) +
	    ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)state->calibration_data.dig_H6)) >> 10) * (((var1 * ((int32_t)state->calibration_data.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
					 ((int32_t)state->calibration_data.dig_H2) + 8192) >> 14));
  var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)state->calibration_data.dig_H1)) >> 4));
  var1 = (var1 < 0 ? 0 : var1);
  var1 = (var1 > 419430400 ? 419430400 : var1);
  
  state->humidity = (float)(var1 >> 12) / 1024.0;

  return E_NO_ERROR;
}


// Called by the OS when a key is pressed
static void key_input_callback(InputEvent* event, FuriMessageQueue* queue) {
  furi_assert(queue);
  furi_assert(event);

  app_event_t message = {
    .type = EVID_KEY,
    .input = *event,
  };
  furi_message_queue_put(queue, &message, FuriWaitForever);
}


// Called by the OS to redraw the screen

// it appears that the screen is blanked every time this is called, so
// you can't use a "new data" variable to only draw if new data
static void canvas_redraw_callback(Canvas* canvas, void* ctx) {
  furi_assert(canvas);
  furi_assert(ctx);

  state_t* state;

  // attempt to get mutex with a 25ms timeout
  if (!(state = (state_t*)acquire_mutex((ValueMutex*)ctx, 25))) {
    return;
  }

  // draw border around the screen
  canvas_draw_frame(canvas, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);

  if (state->bme280_not_found) {
    // back key is always exit, no other conditions needed
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 17, 15, "BME280 not found!");
    
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 29, 30, "Check pinout:");
    
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 7, 46, "SCL=PC0      SDA=PC1");

    // TODO: enable this
    canvas_draw_icon(canvas, 48, 51, &I_Pin_back_arrow_10x8);
    
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 62, 59, "Exit");
  } else {
    // fonts are FontPrimary, FontSecondary, FontKeyboard, FontBigNumbers
    // FontPrimary is bold, Secondary is not.
    
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 5, 14, "Temp.");
    
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 5, 29, "Humidity");
    
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 5, 44, "Pressure");
    
    canvas_draw_icon(canvas, 8, 50, &I_Ok_btn_9x9);
    
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 21, 58, "Refresh");
    
    canvas_draw_icon(canvas, 69, 50, &I_Pin_back_arrow_10x8);
    
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 83, 58, "Exit");

    // print numbers
    
    // there's no printf() but there are FuriString equivalents
    FuriString* buf = furi_string_alloc();
    furi_string_reserve(buf, 100); // reserve 100 chars for this string

    // The BME280 goes to 85 C (185 F). This string does not go beyond
    // the screen at sensor max, or min, which is -40 C (-40 F).
    furi_string_printf(buf, "%.1f C/%.1f F", (double)(state->temperature), (double)(state->temperature * 9 / 5 + 32));
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 60, 14, furi_string_get_cstr(buf));

    furi_string_reset(buf);
    furi_string_printf(buf, "%.0f%%", (double)(state->humidity));
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 60, 29, furi_string_get_cstr(buf));

    furi_string_reset(buf);
    furi_string_printf(buf, "%.1f hPa", (double)(state->pressure / 100));
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 60, 44, furi_string_get_cstr(buf));
    
    furi_string_free(buf);
  }
  
  release_mutex((ValueMutex*)ctx, state);

}

static app_error_t update_values(state_t* state) {
  /***** Enable forced mode and pressure+temperature measurements *****/
  // enable forced mode (read all data once) in the sensor,
  // temperature oversampling x2, pressure oversampling x2

  if (!furi_hal_i2c_write_reg_8(&furi_hal_i2c_handle_external,
				BME280_I2C_ADDRESS << 1,
				BME280_REG_CTRL_MEAS,
				0b01001001, // temperature oversampling x2, pressure oversampling x2, forced mode
				I2C_TIMEOUT)) {
    FURI_LOG_E(TAG, "Failed to set forced mode! Sensor disconnected?");
    return E_SENSOR_IO_FAILED;
  }

  // sensor will go back to sleep after taking data
  
  // t_fine is used in pressure and humidity calculations, so we have to run that first.
  // these store values in `state`.
  app_error_t err;
  
  if ((err = get_temperature(state)) != E_NO_ERROR) {
    return err;
  }
  if ((err = get_pressure(state)) != E_NO_ERROR) {
    return err;
  }
  if ((err = get_humidity(state)) != E_NO_ERROR) {
    return err;
  }

  return E_NO_ERROR;
}


// FAP entry point
int32_t bme280_entry() {
  // The use of a goto in this function was not my idea. It comes from
  // the example application I used to help build this (though I
  // actually don't think it's that bad, as goto usage goes...)
  Gui* gui = NULL;
  ViewPort* vpp = NULL;
  state_t* state = NULL;
  ValueMutex mutex = {0};
  
  app_event_t event;
  
  FuriMessageQueue* queue = NULL;

  // make message queue
  
  // A program starts by allocating various resources. Most of these
  // we do not really need to check the output of.
  queue = furi_message_queue_alloc(8, sizeof(app_event_t));
  gui = furi_record_open("gui");
  state = malloc(sizeof(state_t));

  state->calibration_data.dig_T1 = 0;
  state->calibration_data.dig_T2 = 0;
  state->calibration_data.dig_T3 = 0;
  
  state->calibration_data.dig_P1 = 0;
  state->calibration_data.dig_P2 = 0;
  state->calibration_data.dig_P3 = 0;
  state->calibration_data.dig_P4 = 0;
  state->calibration_data.dig_P5 = 0;
  state->calibration_data.dig_P6 = 0;
  state->calibration_data.dig_P7 = 0;
  state->calibration_data.dig_P8 = 0;
  state->calibration_data.dig_P9 = 0;

  state->calibration_data.dig_H1 = 0;
  state->calibration_data.dig_H2 = 0;
  state->calibration_data.dig_H3 = 0;
  state->calibration_data.dig_H4 = 0;
  state->calibration_data.dig_H5 = 0;
  state->calibration_data.dig_H6 = 0;

  // only set when I2C search doesn't find the chip
  state->bme280_not_found = false;

  // create state variable mutex
  init_mutex(&mutex, state, sizeof(state));
  vpp = view_port_alloc();

  // register event callbacks
  view_port_input_callback_set(vpp, key_input_callback, queue);
  view_port_draw_callback_set(vpp, canvas_redraw_callback, &mutex);

  // attach viewport to gui
  gui_add_view_port(gui, vpp, GuiLayerFullscreen);

  // now set up I2C
  // acquire mutex on state variables

  // A mutex should always be verified because the data may be shared among threads.

  if (!(state = (state_t*)acquire_mutex_block(&mutex))) {
    FURI_LOG_E(TAG, "Failed to acquire mutex");
    goto bail;
  }
  // acquire a handle to i2c bus, which is a global variable
  furi_hal_i2c_acquire(&furi_hal_i2c_handle_external);
  FURI_LOG_I(TAG, "Initialized I2C");
  
  if (!furi_hal_i2c_is_device_ready(&furi_hal_i2c_handle_external, BME280_I2C_ADDRESS << 1, I2C_TIMEOUT)) {
    // if chip is not ready (not found), release mutex and go to loop
    state->bme280_not_found = true;
    // we don't really need to check if releasing a mutex failed or
    // not. (also, this app is very simple, the mutex isn't likely to
    // be shared anyway)
    release_mutex(&mutex, state);
    goto loop;
  }

  // If we're here, the BME280 is present and the application is
  // running. Reset the sensor, read calibration data, take a reading, and enter
  // mainloop. Drawing at the right time is taken care of by the OS.

  /***** Reset the sensor *****/
  // this is done by writing 0xb6 to the reset register
  // left shift the address because I2C addresses are 7 bits, where
  // the lowest bit indicates read or write
  if (!furi_hal_i2c_write_reg_8(&furi_hal_i2c_handle_external,
				BME280_I2C_ADDRESS << 1,
				BME280_REG_RESET,
				0xb6,
				I2C_TIMEOUT)) {
    state->bme280_not_found = true;
    release_mutex(&mutex, state);
    goto loop;
  }
  
  FURI_LOG_I(TAG, "Reset sensor");

  /***** Check that this is in fact a BME280 *****/
  uint8_t chip_id;
  if (!furi_hal_i2c_read_reg_8(&furi_hal_i2c_handle_external,
			       BME280_I2C_ADDRESS << 1,
			       BME280_REG_ID,
			       &chip_id,
			       I2C_TIMEOUT)) {
    state->bme280_not_found = true;
    release_mutex(&mutex, state);
    goto loop;
  }
  if (chip_id != 0x60) {
    // something is here but it's not a BME280, which is the only thing we know how to read
    FURI_LOG_I(TAG, "Chip found with id 0x%x (expected 0x60)", chip_id);
    state->bme280_not_found = true;
    release_mutex(&mutex, state);
    goto loop;
  }
  /***** Read temperature and pressure compensation values *****/
  // start at register 0x88 and read to 0x9F (24 temperature and pressure calibration values)
  // 
  uint8_t tx_data[1] = {BME280_REG_DIG_T1};
  uint8_t rx_data[24] = {0};

  // read dig_Tx values
  // There is a furi_hal_i2c_read_reg_16() but we have a lot of
  // registers to read and checking the output of 12 function calls is
  // difficult
  if (!furi_hal_i2c_trx(&furi_hal_i2c_handle_external, BME280_I2C_ADDRESS << 1, tx_data, 1, rx_data, 24, I2C_TIMEOUT)) {
    state->bme280_not_found = true;
    release_mutex(&mutex, state);
    goto loop;
  }


  state->calibration_data.dig_T1 = (rx_data[1] << 8) | rx_data[0];
  state->calibration_data.dig_T2 = (rx_data[3] << 8) | rx_data[2];
  state->calibration_data.dig_T3 = (rx_data[5] << 8) | rx_data[4];

  state->calibration_data.dig_P1 = (rx_data[7] << 8) | rx_data[6];
  state->calibration_data.dig_P2 = (rx_data[9] << 8) | rx_data[8];
  state->calibration_data.dig_P3 = (rx_data[11] << 8) | rx_data[10];
  state->calibration_data.dig_P4 = (rx_data[13] << 8) | rx_data[12];
  state->calibration_data.dig_P5 = (rx_data[15] << 8) | rx_data[14];
  state->calibration_data.dig_P6 = (rx_data[17] << 8) | rx_data[16];
  state->calibration_data.dig_P7 = (rx_data[19] << 8) | rx_data[18];
  state->calibration_data.dig_P8 = (rx_data[21] << 8) | rx_data[20];
  state->calibration_data.dig_P9 = (rx_data[23] << 8) | rx_data[22];

  /***** Read humidity compensation values *****/
  // these are all different sizes and they're spread across different registers
  tx_data[0] = BME280_REG_DIG_H1;
  
  if (!furi_hal_i2c_trx(&furi_hal_i2c_handle_external, BME280_I2C_ADDRESS << 1, tx_data, 1, rx_data, 1, I2C_TIMEOUT)) {
    state->bme280_not_found = true;
    release_mutex(&mutex, state);
    goto loop;
  }
  
  state->calibration_data.dig_H1 = rx_data[0];
  // jump ahead to the read of the DIG_Hx values
  tx_data[0] = BME280_REG_DIG_H2;
  if (!furi_hal_i2c_trx(&furi_hal_i2c_handle_external, BME280_I2C_ADDRESS << 1, tx_data, 1, rx_data, 8, I2C_TIMEOUT)) {
    state->bme280_not_found = true;
    release_mutex(&mutex, state);
    goto loop;
  }
  state->calibration_data.dig_H2 = (rx_data[1] << 8) | rx_data[0];
  state->calibration_data.dig_H3 = rx_data[2];
  // H4 is 12 bits
  state->calibration_data.dig_H4 = ((int16_t)rx_data[3] << 4) | ((int16_t)rx_data[4] & 0x0f);
  // H5 is 12 bits but in a different order, using data from the last read
  state->calibration_data.dig_H5 = ((int16_t)rx_data[5] << 4) | ((int16_t)rx_data[4] >> 4);
  state->calibration_data.dig_H6 = rx_data[6];



  /***** Enable humidity measurements *****/
  if (!furi_hal_i2c_write_reg_8(&furi_hal_i2c_handle_external,
				BME280_I2C_ADDRESS << 1,
				BME280_REG_CTRL_HUM,
				0b010, // x2 oversampling
			        I2C_TIMEOUT)) {
    state->bme280_not_found = true;
    release_mutex(&mutex, state);
    goto loop;
  }
  
  /***** Set inactive duration between samples to 250 ms and disable IIR filter *****/
  if (!furi_hal_i2c_write_reg_8(&furi_hal_i2c_handle_external,
				BME280_I2C_ADDRESS << 1,
				BME280_REG_CONFIG,
				0b01100000,
			        I2C_TIMEOUT)) {
    state->bme280_not_found = true;
    release_mutex(&mutex, state);
    goto loop;
  }
  
  // Read the sensor a couple times with a delay to settle the sensor
  if (update_values(state) != E_NO_ERROR) {
    state->bme280_not_found = true;
    release_mutex(&mutex, state);
    goto loop;
  }
  
  furi_delay_ms(100);
  
  if (update_values(state) != E_NO_ERROR) {
    state->bme280_not_found = true;
    release_mutex(&mutex, state);
    goto loop;
  }

  release_mutex(&mutex, state);

 loop:
  // The app is responsible for pushing stuff to the message queue (here, using `key_callback()`).
  while (furi_message_queue_get(queue, &event, FuriWaitForever) == FuriStatusOk) {
    if (event.type == EVID_KEY) {
      FURI_LOG_I(TAG, "InputTypePress");
      if (event.input.key == InputKeyBack && event.input.type == InputTypePress) {
	FURI_LOG_I(TAG, "User exit");
	break;
      } else if (event.input.key == InputKeyOk && event.input.type == InputTypePress) {
	if (!(state = (state_t*)acquire_mutex_block(&mutex))) {
	  FURI_LOG_E(TAG, "Failed to acquire mutex");
	  goto bail;
	}
	if (!state->bme280_not_found) {
	  if (update_values(state) != E_NO_ERROR) {
	    state->bme280_not_found = true;
	  }
	}
	release_mutex(&mutex, state);
      }
    }
    view_port_update(vpp);
  }


  // must release all resources (including mutex) before exit

 bail:
  FURI_LOG_I(TAG, "Bail!");
  // release I2C bus
  furi_hal_i2c_release(&furi_hal_i2c_handle_external);
    
  // detach viewport
  gui_remove_view_port(gui, vpp);
  // destroy viewport
  if (vpp) {
    view_port_enabled_set(vpp, false);
    view_port_free(vpp);
    vpp = NULL;
  }
  // free mutex
  if (mutex.mutex) {
    delete_mutex(&mutex);
    mutex.mutex = NULL;
  }
  // free memory
  if (state) {
    free(state);
    state = NULL;
  }
  // close GUI
  furi_record_close("gui");
  // destroy message queue
  if (queue) {
    furi_message_queue_free(queue);
    queue = NULL;
  }
  
  return 0;
}
  
  
