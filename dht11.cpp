#include "dht11.h"

int dht11::read(uint8_t pin) {
  // READ VALUES
  int rv = _readSensor(pin, DHTLIB_DHT11_WAKEUP);
  if (rv != DHTLIB_OK) {
    humidity = DHTLIB_INVALID_VALUE;     // invalid value, or is NaN prefered?
    temperature = DHTLIB_INVALID_VALUE;  // invalid value
    return rv;
  }

  // CONVERT AND STORE
  humidity = bits[0];     // bits[1] == 0;
  temperature = bits[2];  // bits[3] == 0;

  // TEST CHECKSUM
  // bits[1] && bits[3] both 0
  uint8_t sum = bits[0] + bits[2];
  if (bits[4] != sum) return DHTLIB_ERROR_CHECKSUM;

  return DHTLIB_OK;
}

int dht11::_readSensor(uint8_t pin, uint8_t wakeupDelay) {
  // INIT BUFFERVAR TO RECEIVE DATA
  uint8_t mask = 128;
  uint8_t idx = 0;

  // EMPTY BUFFER
  for (uint8_t i = 0; i < 5; i++) bits[i] = 0;

  // REQUEST SAMPLE
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(wakeupDelay);
  digitalWrite(pin, HIGH);
  delayMicroseconds(40);
  pinMode(pin, INPUT);

  // GET ACKNOWLEDGE or TIMEOUT
  uint16_t loopCnt = DHTLIB_TIMEOUT;
  while (digitalRead(pin) == LOW) {
    if (--loopCnt == 0) return DHTLIB_ERROR_TIMEOUT;
  }

  loopCnt = DHTLIB_TIMEOUT;
  while (digitalRead(pin) == HIGH) {
    if (--loopCnt == 0) return DHTLIB_ERROR_TIMEOUT;
  }

  // READ THE OUTPUT - 40 BITS => 5 BYTES
  for (uint8_t i = 40; i != 0; i--) {
    loopCnt = DHTLIB_TIMEOUT;
    while (digitalRead(pin) == LOW) {
      if (--loopCnt == 0) return DHTLIB_ERROR_TIMEOUT;
    }

    uint32_t t = micros();

    loopCnt = DHTLIB_TIMEOUT;
    while (digitalRead(pin) == HIGH) {
      if (--loopCnt == 0) return DHTLIB_ERROR_TIMEOUT;
    }

    if ((micros() - t) > 40) {
      bits[idx] |= mask;
    }
    mask >>= 1;
    if (mask == 0)  // next byte?
    {
      mask = 128;
      idx++;
    }
  }
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);

  return DHTLIB_OK;
}