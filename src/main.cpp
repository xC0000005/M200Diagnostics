#include <Arduino.h>
#include <STM32ADC.h>
#include <stdarg.h>
#include <libmaple/iwdg.h>
#include <libmaple/gpio.h>
#include "pins_MALYAN_M200.h"

// include the SD library:
#include "SdFat.h"


typedef enum
{
  NO_INPUT = 0,
  TEST_ENDSTOPS = 1,
  TURN_E0_HEATER_ON = 2,
  TURN_E0_HEATER_OFF = 3,
  TURN_BED_HEATER_ON = 4,
  TURN_BED_HEATER_OFF = 5,
  TURN_E0_FAN_ON = 6,
  TURN_E0_FAN_OFF = 7,
  TURN_CONTROLLER_FAN_ON = 8,
  TURN_CONTROLLER_FAN_OFF = 9,
  READ_E0_TEMP = 10,
  READ_BED_TEMP = 11,
  TEST_STEPPERS = 12,
  SEND_TO_LCD = 13,
  TEST_SD_CARD = 14,
} MenuState;

MenuState menu_state = NO_INPUT;

//
// Set DISABLE_CHIP_SELECT to disable a second SPI device.
// For example, with the Ethernet shield, set DISABLE_CHIP_SELECT
// to 10 to disable the Ethernet controller.
const int8_t DISABLE_CHIP_SELECT = -1;
//
// Test with reduced SPI speed for breadboards.  SD_SCK_MHZ(4) will select
// the highest speed supported by the board that is not over 4 MHz.
// Change SPI_SPEED to SD_SCK_MHZ(50) for best performance.
#define SPI_SPEED SD_SCK_MHZ(4)
//------------------------------------------------------------------------------
// File system object.
SdFat sd(1);

// On the Malyan M200, this will be Serial1. On a RAMPS board,
// it might not be.
#define LCD_SERIAL Serial1
#define LONG_FILENAME_LENGTH 512

// This is based on longest sys command + a filename, plus some buffer
// in case we encounter some data we don't recognize
// There is no evidence a line will ever be this long, but better safe than sorry
#define MAX_CURLY_COMMAND (32 + LONG_FILENAME_LENGTH) * 2

// Track incoming command bytes from the LCD
int inbound_count;
bool lastUsbStatus = false;

void init_pins() {
  pinMode(X_MIN_PIN, INPUT);
  pinMode(Y_MIN_PIN, INPUT);
  pinMode(Z_MIN_PIN, INPUT);

  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);

  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);

  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);

  pinMode(E0_STEP_PIN, OUTPUT);
  pinMode(E0_DIR_PIN, OUTPUT);
  pinMode(E0_ENABLE_PIN, OUTPUT);

  pinMode(TEMP_0_PIN, INPUT_ANALOG);
  pinMode(TEMP_BED_PIN, INPUT_ANALOG);

  pinMode(HEATER_0_PIN, OUTPUT);
  pinMode(HEATER_BED_PIN, OUTPUT);

  pinMode(FAN_PIN, OUTPUT);
  pinMode(FAN1_PIN, OUTPUT);
}

/*
 * turn off all steppers, heaters, and Fans
 */
void disable_everything() {
  digitalWrite(X_ENABLE_PIN, HIGH);
  digitalWrite(Y_ENABLE_PIN, HIGH);
  digitalWrite(Z_ENABLE_PIN, HIGH);
  digitalWrite(E0_ENABLE_PIN, HIGH);

  digitalWrite(HEATER_0_PIN, LOW);
  digitalWrite(HEATER_BED_PIN, LOW);

  digitalWrite(FAN_PIN, LOW);
  digitalWrite(FAN1_PIN, LOW);
}

void setup() {
  Serial.begin();
  LCD_SERIAL.begin(500000);
  init_pins();
  disable_everything();
}

// Reset watchdog. MUST be called at least every 4 seconds after the
// first watchdog_init or STM32F1 will reset.
inline void watchdog_reset() { iwdg_feed(); }

void Log(const char* format, ...) {
    char dest[1024];
    va_list argptr;
    va_start(argptr, format);
    vsprintf(dest, format, argptr);
    va_end(argptr);
    Serial.write(dest);
}

void LogLn(const char* format, ...) {
    va_list argptr;
    Log(format, argptr);
    Log("\r\n");
}

// Everything written needs the high bit set.
void write_to_lcd(const char * const message) {
  char encoded_message[MAX_CURLY_COMMAND];
  uint8_t message_length = min(strlen(message), sizeof(encoded_message));

  for (uint8_t i = 0; i < message_length; i++)
    encoded_message[i] = (message[i] | 0x80);

  LCD_SERIAL.Print::write(encoded_message, message_length);
}

/*
 * Show the menu of options supported
 */
void show_welcome() {
  LogLn("M200 hardware diagnostics.");
  LogLn("Compiled: " __DATE__ );
  LogLn("Select Function:");
  LogLn("0 - main menu");
  LogLn("1 - test endstops");
  LogLn("2 - Turn E0 Heater On");
  LogLn("3 - Turn E0 Heater Off");
  LogLn("4 - Turn Bed Heater On");
  LogLn("5 - Turn Bed Heater Off");
  LogLn("6 - Turn E0 Fan On");
  LogLn("7 - Turn E0 Fan Off");
  LogLn("8 - Turn Controller Fan On");
  LogLn("9 - Turn Controller Fan Off");
  LogLn("10 - Read E0 Temp");
  LogLn("11 - Read Bed Temp");
  LogLn("12 - Test Steppers");
  LogLn("13 - Send string to LCD");
  LogLn("14 - Test SD Card");
}

void drain_and_echo_lcd() {
  static char inbound_buffer[MAX_CURLY_COMMAND];

  while (LCD_SERIAL.available()) {
    const byte b = (byte)LCD_SERIAL.read() & 0x7F;
    inbound_buffer[inbound_count++] = b;
    if (b == '}' || inbound_count == sizeof(inbound_buffer) - 1) {
      inbound_buffer[inbound_count] = '\0';
      Log("Received from LCD:%s\r\n", inbound_buffer);
      inbound_count = 0;
      inbound_buffer[0] = 0;
    }
  }
}

/*
 * reads an int from the serial INPUT
 */
MenuState get_menu_input() {
  while (!Serial.available()) {
    watchdog_reset();
    return (MenuState)Serial.parseInt();
  }
}

void test_sd_card() {
  LogLn("Initializing SD Card");
  // initialize the first card
  if (!sd.begin(PA4, SD_SCK_MHZ(18))) {
    LogLn("Failed to initialize SD Card");
    return;
  }

  LogLn("Listing files:");
  sd.ls();
  LogLn("SD Card test complete.");
}
/*
 * reads a line from the usb input and outputs it to the LCD
 */
void send_to_lcd() {
  char send_buffer[MAX_CURLY_COMMAND];
  int outbound_count = 0;
  bool done = false;
  LogLn("Enter Message - finish with }");

  do {
    watchdog_reset();
    while (Serial.available()) {
      const byte b = (byte)Serial.read();
      Serial.write(b);
      send_buffer[outbound_count++] = b;
      if (b == '}' || outbound_count == sizeof(send_buffer) - 1) {
        send_buffer[outbound_count] = '\0';
        write_to_lcd(send_buffer);
        done = true;
      }
    }
    /* code */
  } while(!done);

  // next line, please
  LogLn("");
}

/*
 * waits for an endstop to be triggered.
 */
void wait_for_endstop(int endstop) {
    bool endstopState = false;
    bool newEndstopState = false;

    endstopState = digitalRead(endstop);
    LogLn("Default stop state is %i - watching for change.", endstopState);

    for (int i = 0; i < 10; i++) {
      newEndstopState = digitalRead(endstop);

      if (endstopState != newEndstopState) {
        break;
      }
      Log(".");
      watchdog_reset();
      delay(1000);
    }

    Log("\r\n");
    Log("Finished. Original State: %i, New State %i", endstopState, newEndstopState);
    LogLn("\r\n");
}

/*
 * sets a pin.
 */
void set_pin(const char *message, int pin, int state) {
  LogLn(message);
  digitalWrite(pin, state);
}

void turn_on_e0_heater() {
  set_pin("Turning on fan.", FAN_PIN, HIGH);
  set_pin("Enabling E0 Heater and fan for 15 seconds.", HEATER_0_PIN, HIGH);
  for (int i = 0; i < 15; i++) {
    watchdog_reset();
    delay(1000);
  }
  set_pin("Disabling E0 Heater, leaving fan on", HEATER_0_PIN, LOW);
}

void turn_off_e0_heater() {
  set_pin("Disabling E0 Heater", HEATER_0_PIN, LOW);
}

void turn_on_bed_heater() {
  set_pin("Enabling bed heater for 30 seconds", HEATER_BED_PIN, HIGH);
  for (int i = 0; i < 30; i++) {
    watchdog_reset();
    delay(1000);
  }
  set_pin("Disabling bed heater.", HEATER_BED_PIN, LOW);
}

void turn_off_bed_heater() {
  set_pin("Disabling bed heater", HEATER_BED_PIN, LOW);
}

void turn_on_e0_fan() {
  set_pin("Enabling e0 fan", FAN_PIN, HIGH);
}

void turn_off_e0_fan() {
  set_pin("Disabling e0 fan", FAN_PIN, LOW);
}

void turn_on_controller_fan() {
  set_pin("Enabling controller fan", FAN1_PIN, HIGH);
}

void turn_off_controller_fan() {
  set_pin("Disabling controller fan", FAN1_PIN, LOW);
}

void read_temp_pin(int pin) {
  Log("Reading temp direct - reported values are raw analog:");
  Log("Value is '%i'", analogRead(pin));
  LogLn("");
}

void test_one_stepper(int enable_pin, int step_pin, int direction_pin)
{
    LogLn("Stepper will move back and forth, slowly.");

    // The "enable" pin is actually a disable one
    digitalWrite(enable_pin, LOW);
    digitalWrite(direction_pin, LOW);
    for (int i = 0; i < 400; i++)
    {
      digitalWrite(step_pin, HIGH);
      delay(4);
      digitalWrite(step_pin, LOW);
    }

    digitalWrite(direction_pin, HIGH);
    for (int i = 0; i < 400; i++)
    {
      digitalWrite(step_pin, HIGH);
      delay(4);
      digitalWrite(step_pin, LOW);
    }

    LogLn("Stepper move complete.");
}

/*
 * tests a stepper.
 */
void test_steppers() {
  byte b = 0;
  LogLn("Testing steppers.");
  LogLn("Enter an stepper to test (X, Y, Z, E).");
  while (!Serial.available()) {
    watchdog_reset();
  }

  b = Serial.read();

  switch (b)
  {
    case 'x':
    case 'X':
      test_one_stepper(X_ENABLE_PIN, X_STEP_PIN, X_DIR_PIN);
      break;
    case 'y':
    case 'Y':
      test_one_stepper(Y_ENABLE_PIN, Y_STEP_PIN, Y_DIR_PIN);
      break;
    case 'z':
    case 'Z':
      test_one_stepper(Z_ENABLE_PIN, Z_STEP_PIN, Z_DIR_PIN);
      break;
    case 'e':
    case 'E':
      test_one_stepper(E0_ENABLE_PIN, E0_STEP_PIN, E0_DIR_PIN);
      break;
    default:
      LogLn("Unknown stepper selected:%c", b);
  }
}

/*
 * waits for an endstop to be triggered.
 */
void test_endstops() {
  byte b = 0;
  LogLn("Testing endstops.");
  LogLn("Enter an endstop to test (X, Y, Z).");
  while (!Serial.available()) {
    watchdog_reset();
  }

  b = Serial.read();

  switch (b)
  {
    case 'x':
    case 'X':
      wait_for_endstop(X_MIN_PIN);
      break;
    case 'y':
    case 'Y':
      wait_for_endstop(Y_MIN_PIN);
      break;
    case 'z':
    case 'Z':
      wait_for_endstop(Z_MIN_PIN);
      break;
    default:
      LogLn("Unknown endstop selected:%c", b);
  }
}

void loop() {
  // Show the welcome if new connection
  if (Serial && Serial != lastUsbStatus)
  {
    show_welcome();
    lastUsbStatus = !lastUsbStatus;
  }

  switch (menu_state) {
    case NO_INPUT:
      break;
    case TEST_ENDSTOPS:
      test_endstops();
      break;
    case TURN_E0_HEATER_ON:
      turn_on_e0_heater();
      break;
    case TURN_E0_HEATER_OFF:
      turn_off_e0_heater();
      break;
    case TURN_BED_HEATER_ON:
      turn_on_bed_heater();
      break;
    case TURN_BED_HEATER_OFF:
      turn_off_bed_heater();
      break;
    case TURN_E0_FAN_ON:
      turn_on_e0_fan();
      break;
    case TURN_E0_FAN_OFF:
      turn_off_e0_fan();
      break;
    case TURN_CONTROLLER_FAN_ON:
      turn_on_controller_fan();
      break;
    case TURN_CONTROLLER_FAN_OFF:
      turn_off_controller_fan();
      break;
    case READ_E0_TEMP:
      read_temp_pin(TEMP_0_PIN);
      break;
    case READ_BED_TEMP:
      read_temp_pin(TEMP_BED_PIN);
      break;
    case TEST_STEPPERS:
      test_steppers();
      break;
    case SEND_TO_LCD:
      send_to_lcd();
      break;
    case TEST_SD_CARD:
      test_sd_card();
      break;
  }

  menu_state = get_menu_input();

  drain_and_echo_lcd();

  watchdog_reset();
}
