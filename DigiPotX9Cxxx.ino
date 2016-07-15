/*
 * DigiPotX9Cxxx.cpp - Arduino library for managing digital potentiometers X9Cxxx (xxx = 102,103,104,503).
 * By Timo Fager, Jul 29, 2011.
 * Released to public domain.
 **/

#include "Arduino.h"
#include "DigiPotX9Cxxx.h"

void pot_setup(uint8_t incPin, uint8_t udPin, uint8_t csPin) {
  _incPin = incPin;
  _udPin = udPin;
  _csPin = csPin;  
  _currentValuePot = DIGIPOT_UNKNOWN;
  pinMode(_incPin, OUTPUT);
  pinMode(_udPin, OUTPUT);
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

}

void pot_reset() {
  // change down maximum number of times to ensure the value is 0
  pot_decrease(DIGIPOT_MAX_AMOUNT);
  _currentValuePot = 0;
}

void pot_set(uint8_t value) {
  value = constrain(value, 0, DIGIPOT_MAX_AMOUNT);
  if (_currentValuePot == DIGIPOT_UNKNOWN) pot_reset();
  if (_currentValuePot > value) {
    pot_change(DIGIPOT_DOWN, _currentValuePot-value);
  } else if (_currentValuePot < value) {
    pot_change(DIGIPOT_UP, value-_currentValuePot);
  }
}

uint8_t pot_get() {
  return _currentValuePot;
}


void pot_decrease(uint8_t amount) {
  amount = constrain(amount, 0, DIGIPOT_MAX_AMOUNT);
  pot_change(DIGIPOT_DOWN, amount);
}



void pot_change(uint8_t direction, uint8_t amount) {
  amount = constrain(amount, 0, DIGIPOT_MAX_AMOUNT);
  digitalWrite(_udPin, direction);
  digitalWrite(_incPin, HIGH);
  digitalWrite(_csPin, LOW);

  for (uint8_t i=0; i<amount; i++) {
    digitalWrite(_incPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_incPin, HIGH);
    delayMicroseconds(2);
    if (_currentValuePot != DIGIPOT_UNKNOWN) {
      _currentValuePot += (direction == DIGIPOT_UP ? 1 : -1);
      _currentValuePot = constrain(_currentValuePot, 0, DIGIPOT_MAX_AMOUNT);
    }
    
  }
  digitalWrite(_csPin, HIGH);
}

