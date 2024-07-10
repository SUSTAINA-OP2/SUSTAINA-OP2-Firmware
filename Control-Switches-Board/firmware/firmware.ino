/** 
* @file firmware.ino
* @brief  Firmware for SUSTAINA-Control-Switches-Board to be installed in SUSTAINA-OP2™
* @author Satoshi Inoue, Masato Kubotera
* @date 2024/07/05
* @version 0x01
* @copyright SUSTAINA-OP2™ 2024
*/

#include "Arduino.h"
#include "src/config.h"
#include "src/sustaina_serial.h"
#include "src/Adafruit_NeoPixel/Adafruit_NeoPixel.h"
#include <vector>

// #define SERIAL_DEBUG

SUSTAINA_PMX_SERIAL SutainaSerial;

Adafruit_NeoPixel pixels(NUM_NEOPIXEL, NEOPIXEL_LED_PIN, NEO_GRB + NEO_KHZ800);

// USB Serial
constexpr uint32_t USB_SERIAL_BAUDRATE = 115200;

uint64_t lastNeopixelSetupTime = 0;  // Variable to store the last call time of setupNeopixels

// Packet structure to send to Arduino.
// 0~11 bits| LED state
//       0~2  | 1st LED
//       3~5  | 2nd LED
//       6~8  | 3rd LED
//       9~11 | 4th LED
// 12-bit | Brightness setting (1: bright / 0: dim)
// 13-bit | LED reset (1: reset / 0: no reset)
// 14-bit | not used
// 15-bit | not used
// LED state is arranged in RGB order from the least significant bit. Each RGB has only two states: ON/OFF.
// Data is in little endian format.

// Packet structure returned from Arduino.
// 1 byte Button State
//   'n': NOT_PUSHED - 0x6E
//   'r': RED_PUSHED - 0x72
//   'R': RED_PUSHED_OVER_3TIMES - 0x52
//   'g': GREEN_PUSHED - 0x67

enum class ButtonStateEnum {
  NOT_PUSHED = 'n',
  RED_PUSHED = 'r',
  RED_PUSHED_OVER_3TIMES = 'R',
  GREEN_PUSHED = 'g'
};

// Variables to manage the state of the buttons
volatile uint8_t redButtonPressCount = 0;
volatile uint8_t greenButtonPressCount = 0;
volatile uint64_t redFirstPressTime = 0;
volatile uint64_t redLastPressTime = 0;
volatile uint64_t greenLastPressTime = 0;
volatile bool redButtonProcessed = false;
volatile bool greenButtonProcessed = false;
volatile bool redOver3TimesProcessed = false;

ButtonStateEnum lastButtonState;

// Interrupt handler for the red button
void redButtonInterruptHandler() {
  const uint64_t nowPressTime = millis();
  if (nowPressTime - redLastPressTime > CHATTERING_PREVENTION_SECONDS) {
    if (redButtonPressCount == 0 || (nowPressTime - redFirstPressTime > CONSECUTIVE_STRIKE_JUDGMENT_SECONDS)) {
      redFirstPressTime = nowPressTime;
      redButtonPressCount = 0;         // Reset count if more than 1.5 seconds have passed
      redOver3TimesProcessed = false;  // Allow reprocessing of RED_PUSHED_OVER_3TIMES
    }
    redButtonPressCount++;
    redLastPressTime = nowPressTime;
    redButtonProcessed = false;
  }
}

// Interrupt handler for the green button
void greenButtonInterruptHandler() {
  const uint64_t nowPressTime = millis();
  if (nowPressTime - greenLastPressTime > CHATTERING_PREVENTION_SECONDS) {
    redButtonPressCount = 0;         // Invalidate red button count
    redOver3TimesProcessed = false;  // Allow reprocessing of RED_PUSHED_OVER_3TIMES

    greenButtonPressCount++;
    greenLastPressTime = nowPressTime;
    greenButtonProcessed = false;
  }
}

// Function to read the state of the buttons and process the LED state
ButtonStateEnum readButtonState() {
  ButtonStateEnum returnState = ButtonStateEnum::NOT_PUSHED;

  // Process red button
  if (redButtonPressCount > 0 && !redButtonProcessed) {
    greenButtonPressCount = 0;
    if (millis() - redFirstPressTime <= CONSECUTIVE_STRIKE_JUDGMENT_SECONDS) {
      if (redButtonPressCount >= STRIKE_COUNT && !redOver3TimesProcessed) {
        returnState = ButtonStateEnum::RED_PUSHED_OVER_3TIMES;
        redOver3TimesProcessed = true;
      } else if (redButtonPressCount < STRIKE_COUNT) {
        returnState = ButtonStateEnum::RED_PUSHED;
      }
    } else {
      returnState = ButtonStateEnum::RED_PUSHED;
      redButtonPressCount = 1;               // Count the last press
      redFirstPressTime = redLastPressTime;  // Reset timer
      redOver3TimesProcessed = false;
    }
    redButtonProcessed = true;
  }

  // Invalidate red button count if green button is pressed
  if (greenButtonPressCount > 0 && !greenButtonProcessed) {
    redButtonPressCount = 0;
    greenButtonPressCount = 0;
    returnState = ButtonStateEnum::GREEN_PUSHED;
    greenButtonProcessed = true;
  }

  if (returnState != ButtonStateEnum::NOT_PUSHED) {
    lastButtonState = returnState;
  }

  handleLedState(returnState);

  return returnState;
}

// Update button state in the main loop cycle
ButtonStateEnum getLastButtonState() {
  ButtonStateEnum tempState = lastButtonState;
  lastButtonState = ButtonStateEnum::NOT_PUSHED;
  return tempState;
}

// Function to handle the state of the LEDs
void handleLedState(ButtonStateEnum currentState) {
  if (currentState == ButtonStateEnum::RED_PUSHED) {
#ifdef SERIAL_DEBUG
    Serial.println("Red button pressed");
#endif  // SERIAL_DEBUG
    digitalWrite(R_LED_PIN, LOW);
    digitalWrite(G_LED_PIN, HIGH);
    digitalWrite(B_LED_PIN, HIGH);
  } else if (currentState == ButtonStateEnum::GREEN_PUSHED) {
#ifdef SERIAL_DEBUG
    Serial.println("Green button pressed");
#endif  // SERIAL_DEBUG
    digitalWrite(R_LED_PIN, HIGH);
    digitalWrite(G_LED_PIN, LOW);
    digitalWrite(B_LED_PIN, HIGH);
  } else if (currentState == ButtonStateEnum::RED_PUSHED_OVER_3TIMES) {
#ifdef SERIAL_DEBUG
    Serial.println("Red button was hit repeatedly");
#endif  // SERIAL_DEBUG
    digitalWrite(R_LED_PIN, HIGH);
    digitalWrite(G_LED_PIN, HIGH);
    digitalWrite(B_LED_PIN, LOW);
  }
}

// Function to set the state of the LEDs
void setupNeopixels(const uint16_t targetState) {
  for (int i = 0; i < NUM_NEOPIXEL; i++) {
    uint8_t red = (targetState & (1 << (i * 3 + 0))) ? 125 : 0;
    uint8_t green = (targetState & (1 << (i * 3 + 1))) ? 125 : 0;
    uint8_t blue = (targetState & (1 << (i * 3 + 2))) ? 125 : 0;
    pixels.setPixelColor(i, pixels.Color(red, green, blue));
  }

  if ((targetState & (1 << 12)) == 0) {
    pixels.setBrightness(50);
  } else {
    pixels.setBrightness(255);
  }

  if ((targetState & (1 << 13)) != 0) {
    pixels.clear();
  }
  pixels.show();

  lastNeopixelSetupTime = millis();  // Update the last call time
}

void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(USB_SERIAL_BAUDRATE);
  while (!Serial) {
    ;  // Wait for USB serial port to connect. Needed for native USB
  }
#endif  // SERIAL_DEBUG

  SutainaSerial.setupSerial();

  pinMode(G_SWITCH_PIN, INPUT_PULLUP);
  pinMode(R_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(G_SWITCH_PIN), greenButtonInterruptHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(R_SWITCH_PIN), redButtonInterruptHandler, FALLING);

  pinMode(R_LED_PIN, OUTPUT);
  pinMode(G_LED_PIN, OUTPUT);
  pinMode(B_LED_PIN, OUTPUT);
  digitalWrite(R_LED_PIN, HIGH);
  digitalWrite(G_LED_PIN, HIGH);
  digitalWrite(B_LED_PIN, HIGH);

  pixels.begin();
  pixels.clear();

  for (int i = 0; i < NUM_NEOPIXEL; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
  }

  pixels.show();
}

void loop() {
  auto currentState = readButtonState();

  if (SutainaSerial.readPacket()) {
    if (SutainaSerial.checkCRCandID()) {
      if (!SutainaSerial.commandProcess()) {
        switch (SutainaSerial.rxCommand) {
          case GET_STATE_AND_SET_LED_CMD:
            {
              uint16_t receivedData = (SutainaSerial.rxData[1] << 8) | SutainaSerial.rxData[0];
              setupNeopixels(receivedData);

              uint8_t sendState = static_cast<uint8_t>(getLastButtonState());
              SutainaSerial.txData.push_back(sendState);
              break;
            }

          default:
            SutainaSerial.txError |= SutainaSerial.NOT_EXISTENT_CMD_ERROR;
        }
      }
      SutainaSerial.sendPacket();
    }
  }

  // Check if more than 1 second has passed since the last call to setupNeopixels
  if (millis() - lastNeopixelSetupTime > 1000) {
    for (int i = 0; i < NUM_NEOPIXEL; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    }
    pixels.show();
    lastNeopixelSetupTime = millis();  // Reset the timer
  }
}
