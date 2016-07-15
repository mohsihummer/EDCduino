/*
 * DigiPotX9Cxxx.h - Arduino library for managing digital potentiometers X9Cxxx (xxx = 102,103,104,503).
 * By Timo Fager, Jul 29, 2011.
 * Released to public domain.
 **/

#ifndef DigiPotX9Cxxx_h
#define DigiPotX9Cxxx_h

#define DIGIPOT_UP   HIGH
#define DIGIPOT_DOWN LOW
#define DIGIPOT_MAX_AMOUNT 99
#define DIGIPOT_UNKNOWN 255

 void pot_setup(uint8_t incPin, uint8_t udPin, uint8_t csPin);
  void pot_change(uint8_t direction, uint8_t amount);
  void pot_set(uint8_t value);
  uint8_t pot_get();
  void pot_reset();
  uint8_t _incPin;
  uint8_t _udPin;
  uint8_t _csPin;
  uint8_t _currentValuePot;


#endif
