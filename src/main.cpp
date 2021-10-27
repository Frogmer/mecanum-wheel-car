#include <Arduino.h>
#include <SoftWire.h>
#include <Wire.h>

#include "PCA9685.h"

#define RPT_GEMEPAD_LEN 11     // game pad data length
#define USB_HOST_I2C_ADDR 0x8  // usb host i2c address
#define PCA9685_I2C_ADDR 0x40  // pca9685 i2c address
#define SOFT_WIRE_BUF_SIZE 16  // pca9685 soft wire buffer size
#define USB_HOST_SDA_PIN 2     // usb host i2c sda pin
#define USB_HOST_SCL_PIN 14    // usb host i2c scl pin

#define PCA9685_SDA_PIN 4  // pca9685 i2c sda pin
#define PCA9685_SCL_PIN 5  // pca9685 i2c scl pin

TwoWire _wireUsbHost;                                     // usb host i2c
SoftWire _wirePca9685(PCA9685_SDA_PIN, PCA9685_SCL_PIN);  // pca9685 i2c
PCA9685 _pca9685;

uint8_t _swTxBuffer[SOFT_WIRE_BUF_SIZE];  // pca9685 soft wire tx buffer
uint8_t _swRxBuffer[SOFT_WIRE_BUF_SIZE];  // pca9685 soft wire rx buffer

uint8_t _padDataIdx;  // index of pad data read from i2c buffer
uint8_t _lastValidPadData[RPT_GEMEPAD_LEN] = {0};  // last valid pad data
uint8_t _padData[RPT_GEMEPAD_LEN] = {0};           // new pad data

uint8_t _hMoveStrength = 0;     // horizontal(left right) move strength
uint8_t _vMoveStrength = 0;     // vertical(up down) move strength
uint8_t _hDutyCycles[8] = {0};  // horizontal(left right) duty cycles
uint8_t _vDutyCycles[8] = {0};  // vertical(up down) duty cycles

// analyze pad data
void analyzePadData(uint8_t* padData) {
  switch (padData[3]) {
    case 0 ... 0x7f:
      // left
      _hMoveStrength = map(padData[3], 0x7f, 0, 0, 100);
      _hDutyCycles[0] = 0;
      _hDutyCycles[1] = _hMoveStrength;
      _hDutyCycles[2] = _hMoveStrength;
      _hDutyCycles[3] = 0;
      _hDutyCycles[4] = 0;
      _hDutyCycles[5] = _hMoveStrength;
      _hDutyCycles[6] = _hMoveStrength;
      _hDutyCycles[7] = 0;
      break;
    case 0x80:
      // center
      _hMoveStrength = 0;
      for (uint8_t i = 0; i < 8; i++) {
        _hDutyCycles[i] = 0;
      }
      break;
    case 0x81 ... 0xff:
      // right
      _hMoveStrength = map(padData[3], 0x81, 0xff, 0, 100);
      _hDutyCycles[0] = _hMoveStrength;
      _hDutyCycles[1] = 0;
      _hDutyCycles[2] = 0;
      _hDutyCycles[3] = _hMoveStrength;
      _hDutyCycles[4] = _hMoveStrength;
      _hDutyCycles[5] = 0;
      _hDutyCycles[6] = 0;
      _hDutyCycles[7] = _hMoveStrength;
      break;
  }
  switch (padData[4]) {
    case 0 ... 0x7f:
      // up
      _vMoveStrength = map(padData[4], 0x7f, 0, 0, 100);
      _vDutyCycles[0] = 0;
      _vDutyCycles[1] = _vMoveStrength;
      _vDutyCycles[2] = 0;
      _vDutyCycles[3] = _vMoveStrength;
      _vDutyCycles[4] = _vMoveStrength;
      _vDutyCycles[5] = 0;
      _vDutyCycles[6] = _vMoveStrength;
      _vDutyCycles[7] = 0;
      break;
    case 0x80:
      // center
      _vMoveStrength = 0;
      for (uint8_t i = 0; i < 8; i++) {
        _vDutyCycles[i] = 0;
      }
      break;
    case 0x81 ... 0xff:
      // down
      _vMoveStrength = map(padData[4], 0x81, 0xff, 0, 100);
      _vDutyCycles[0] = _vMoveStrength;
      _vDutyCycles[1] = 0;
      _vDutyCycles[2] = _vMoveStrength;
      _vDutyCycles[3] = 0;
      _vDutyCycles[4] = 0;
      _vDutyCycles[5] = _vMoveStrength;
      _vDutyCycles[6] = 0;
      _vDutyCycles[7] = _vMoveStrength;
      break;
  }

  // combine all duty cycles
  for (uint8_t i = 0; i < 8; i++) {
    _pca9685.setChannelDutyCycle(i, max(_hDutyCycles[i], _vDutyCycles[i]), 0);
  }
}

void setup() {
  // init pca9685 I2C
  _wirePca9685.setTxBuffer(_swTxBuffer, SOFT_WIRE_BUF_SIZE);
  _wirePca9685.setRxBuffer(_swRxBuffer, SOFT_WIRE_BUF_SIZE);
  _wirePca9685.setDelay_us(5);
  _wirePca9685.setTimeout(1000);
  _wirePca9685.begin();
  _pca9685.setupSingleDevice(_wirePca9685, PCA9685_I2C_ADDR);
  _pca9685.setToFrequency(1600);
  // stop all wheels turning
  for (uint8_t i = 0; i < 8; i++) {
    _pca9685.setChannelDutyCycle(i, 0, 0);
  }

  // init usb host I2C
  _wireUsbHost.begin(USB_HOST_SDA_PIN, USB_HOST_SCL_PIN);
  // for compatibility with esp8266 I2C slave, frequency of master must be 25000
  _wireUsbHost.setClock(25000L);

  // init serial port
  Serial.begin(115200);
  Serial.println("Start");
}

void loop() {
  // read I2C data
  _wireUsbHost.requestFrom(USB_HOST_I2C_ADDR, RPT_GEMEPAD_LEN);

  _padDataIdx = 0;
  while (_wireUsbHost.available()) {
    char c = _wireUsbHost.read();
    _padData[_padDataIdx] = c;
    _padDataIdx++;
  }

  // check if the pad data has changed
  for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++) {
    if (_lastValidPadData[i] != _padData[i]) {
      // store the pad data for the next comparison
      memcpy(_lastValidPadData, _padData, RPT_GEMEPAD_LEN);
      // analze pad data when it changes
      analyzePadData(_padData);
      break;
    }
  }
}