#ifndef __GUARD_PCA9554_H__
#define __GUARD_PCA9554_H__

#include <Arduino.h>
#include <Wire.h>

#define NORMAL                          0
#define INVERTED                        1

class Pca9554Class
{
protected:
 public:
  Pca9554Class();
  void    begin(uint8_t address);
  boolean pinMode(uint8_t address, uint8_t pin, uint8_t mode);
  boolean pinPolarity(uint8_t address, uint8_t pin, uint8_t polarity);
  boolean digitalWrite(uint8_t address, uint8_t pin, boolean val);
  boolean digitalRead(uint8_t address, uint8_t pin);

 private:
  uint8_t m_inp;
  uint8_t m_out;
  uint8_t m_pol;
  uint8_t m_ctrl;
};

extern Pca9554Class Pca9554;

#endif //ifndef __GUARD_PCA9554_H__

// EOF
