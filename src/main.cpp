#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>

extern "C" {
#include "usbdrv/oddebug.h"
#include "usbdrv/usbdrv.h"

#include "calibrate.h"
}

#define BIT_LED 2

#define LED_PORT_DDR DDRB
#define LED_PORT_OUTPUT PORTB
#define LED_BIT 1

#include "i2c_support.h"

extern "C" {

enum {
  COMMAND_DEVICE_RESTART = 0x01,

  COMMAND_BIT_WRITE =
      0x10, // value_low=bit_value; index_low=bit_index, [index_hi=device_id],

  COMMAND_VALUE_WRITE =
      0x20, // value_low=value; index_low=N/U, [index_hi=device_id],

  // COMMAND_BIT_READ       = 0x11,
  // COMMAND_PIN_CLEAR_ALL  = 0x1,
  // COMMAND_PIN_SELECT_DEVICE = 0x1,
  // COMMAND_PIN_DEVICE_WRITE    = 0x,
  // COMMAND_PIN_DEVICE_READ    = 0x,
//   COMMAND_DEVICE_GET_TAG = 0x20,

  COMMAND_I2C_START_SCAN = 0x30,
  COMMAND_I2C_GET_DEVICE_MAP = 0x31,

  COMMAND_DEVICE_WRITE_DEFAULT_VALUE = 0x40,

  COMMAND_DEVICE_WRITE_TAG = 0x42, 
  COMMAND_DEVICE_READ_TAG = 0x43,
};

/* ------------------------------------------------------------------------- */

const PROGMEM uint8_t crc_table[16] = {
    0x00, 0x1d, 0x3b, 0x26, 0x76, 0x6b, 0x4d, 0x50,
    0xed, 0xf0, 0xd6, 0xcb, 0x9b, 0x86, 0xa0, 0xbd,
};
uint8_t block_crc(const uint8_t *data, uint8_t len) {

  uint8_t crc = 0xff;

  for (uint8_t index = 0; index < len; ++index) {
    // uint8_t crc_byte = pgm_read_byte(crc_table + index);
    crc = pgm_read_byte(crc_table + ((crc ^ data[index]) & 0x0f)) ^ (crc >> 4);
    crc = pgm_read_byte(crc_table + ((crc ^ (data[index] >> 4)) & 0x0f)) ^
          (crc >> 4);
    crc = ~crc;
  }

  return crc;
}

#define CONFIG_EEPROM_ADDRESS 0x10u

struct Configuration {
  uint8_t checksum;
  uint8_t startup_value;
  union {
    uint32_t tag;
    uint16_t tag_words[2];
  };

  void set_default() { 
    startup_value = 0; 
    tag = 0;
  }

  void read() {
    eeprom_read_block(this, (uint8_t *)(CONFIG_EEPROM_ADDRESS), sizeof(Configuration));

    uint8_t current_checksum = checksum;
    checksum = 0;
    if (current_checksum !=
        block_crc((const uint8_t *)this, sizeof(Configuration))) {
      set_default();
    }
  }

  void write() {
    checksum = 0;
    checksum = block_crc((const uint8_t *)this, sizeof(Configuration));
    eeprom_update_block(this, (uint8_t *)(CONFIG_EEPROM_ADDRESS), sizeof(Configuration));
  }
};

Configuration current_configuration;
bool configuration_dirty = false;

/* ------------------------------------------------------------------------- */

PCF8574 current_i2c_device{0x38};

uint8_t i2c_scan_result[16];
uint8_t current_command = 0;

BitCommandData pin_update_data;

/* ------------------------------------------------------------------------- */

usbMsgLen_t dispatch_usb_command(usbRequest_t *message) {
  switch (message->bRequest) {
  case COMMAND_I2C_START_SCAN:
  case COMMAND_DEVICE_RESTART:
    current_command = message->bRequest;
    return 0;

  case COMMAND_BIT_WRITE:
    pin_update_data.operation = BIT_OPERATION_WRITE_BIT;
    // pin_update_data.device = &current_i2c_device;
    pin_update_data.bit = message->wIndex.bytes[0];
    pin_update_data.value = message->wValue.bytes[0];
    return 0;
  case COMMAND_VALUE_WRITE:
    pin_update_data.operation = BIT_OPERATION_WRITE_VALUE;
    // pin_update_data.device = &current_i2c_device;
    // pin_update_data.bit = message->wIndex.bytes[0]; // not used
    pin_update_data.value = message->wValue.bytes[0];
    return 0;
  case COMMAND_DEVICE_WRITE_DEFAULT_VALUE:
    configuration_dirty = true;
    current_configuration.startup_value = message->wValue.bytes[0];
    return 0;
  case COMMAND_I2C_GET_DEVICE_MAP:
    usbMsgPtr = i2c_scan_result;
    return sizeof(i2c_scan_result);
  case COMMAND_DEVICE_WRITE_TAG:
    configuration_dirty = true;
    current_configuration.tag_words[0] = message->wIndex.word;
    current_configuration.tag_words[1] = message->wValue.word;
    return 0;
  case COMMAND_DEVICE_READ_TAG:
    usbMsgPtr = (uint8_t*)&current_configuration.tag;
    return sizeof(current_configuration.tag);
  }
  return 0;
}

usbMsgLen_t usbFunctionSetup(uchar *data) {
  usbRequest_t *rq = (usbRequest_t *)data;
  return dispatch_usb_command(rq);
}

void usbEventResetReady() {
  /* Disable interrupts during oscillator calibration since
   * usbMeasureFrameLength() counts CPU cycles.
   */
  cli();
  calibrateOscillator();
  sei();
  eeprom_update_byte(0, OSCCAL); /* store the calibrated value in EEPROM */
}

void restart() {
  usbDeviceDisconnect();
  wdt_enable(WDTO_15MS);
  cli();
  while (true)
    ;
}

void setup() {
  i2c_start();
  i2c_scan(i2c_scan_result, sizeof(i2c_scan_result));
  wdt_enable(WDTO_1S);
  current_configuration.read();
  pin_update_data =
      BitCommandData{&current_i2c_device, BIT_OPERATION_WRITE_VALUE, 0,
                     current_configuration.startup_value};
}

int main() {
  setup();

  odDebugInit();
  DBG1(0x00, 0, 0); /* debug output: main starts */
  usbInit();
  usbDeviceDisconnect(); /* enforce re-enumeration, do this while interrupts are
                            disabled! */
  uchar i = 0;
  while (--i) { /* fake USB disconnect for > 250 ms */
    wdt_reset();
    _delay_ms(1);
  }
  usbDeviceConnect();
  LED_PORT_DDR |= _BV(LED_BIT); /* make the LED bit an output */
  sei();
  DBG1(0x01, 0, 0);   /* debug output: main loop starts */
  for (;;) {          /* main event loop */
    DBG1(0x02, 0, 0); /* debug output: main loop iterates */
    wdt_reset();
    usbPoll();

    if (current_command > 0) {
      switch (current_command) {
      case COMMAND_DEVICE_RESTART:
        restart();
        break;
      case COMMAND_I2C_START_SCAN:
        i2c_scan(i2c_scan_result, sizeof(i2c_scan_result));
        break;
      }
      current_command = 0;
      continue;
    }

    if (pin_update_data.operation != BIT_OPERATION_NOP) {
      i2c_handle_operation(&pin_update_data);
      pin_update_data.operation = BIT_OPERATION_NOP;
      continue;
    }

    if(configuration_dirty) {
        current_configuration.write();
        configuration_dirty = false;
    }
  }
}
}
