#include <Arduino.h>
#include <Wire.h>
#include "Pca9554.h"



#define PCA9554_REG_INP                 0
#define PCA9554_REG_OUT                 1
#define PCA9554_REG_POL                 2
#define PCA9554_REG_CTRL                3

uint8_t pinNum2bitNum[] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };

/***************************************************************************
 *
 * Constructor for the Pca9554Class class, not much here yet
 *
 **************************************************************************/
Pca9554Class::Pca9554Class(void) { }

/***************************************************************************
 *
 *  Writes 8-bits to the specified destination register
 *
 **************************************************************************/
static void writeRegister(uint8_t i2cAddress, uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(i2cAddress);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}

/***************************************************************************
 *
 * Reads 8-bits from the specified source register
 *
 **************************************************************************/
static uint16_t readRegister(uint8_t i2cAddress, uint8_t reg)
{
  Wire.beginTransmission(i2cAddress);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(i2cAddress, (uint8_t)1);
  return Wire.read();
}

/***************************************************************************
 *
 * Sets the desired pin mode
 *
 **************************************************************************/
boolean Pca9554Class::pinMode(uint8_t address, uint8_t pin, uint8_t mode)
{
  // Make sure the pin number is OK
  if (pin >= sizeof pinNum2bitNum) {
    return false;
  }

  // Calculate the new control register value
  if (mode == OUTPUT) {
    m_ctrl &= ~pinNum2bitNum[pin];
  } else if (mode == INPUT) {
    m_ctrl |= pinNum2bitNum[pin];
  } else {
    return false;
  }

  writeRegister(address, PCA9554_REG_CTRL, m_ctrl);

  return true;
}

/***************************************************************************
 *
 * Write digital value to pin
 *
 **************************************************************************/
boolean Pca9554Class::digitalWrite(uint8_t address, uint8_t pin, boolean val)
{
  // Make sure pin number is OK
  if (pin >= sizeof pinNum2bitNum) {
    return false;
  }

  if (val == HIGH) {
    m_out |= pinNum2bitNum[pin];
  } else {
    m_out &= ~pinNum2bitNum[pin];
  }

  writeRegister(address, PCA9554_REG_OUT, m_out);
}

/***************************************************************************
 *
 * Read digital value from pin.
 * Note, so far this function will fail silently if the pin parameter is
 * incorrectly specified.
 *
 **************************************************************************/
boolean Pca9554Class::digitalRead(uint8_t address, uint8_t pin)
{
  //Serial.print("Debug: "); Serial.println(readRegister(address, PCA9554_REG_INP));
  //Serial.print("Pin numbrt"); Serial.println(pinNum2bitNum[pin]);
  return (readRegister(address, PCA9554_REG_INP) & (pinNum2bitNum[pin]) );
}





/***************************************************************************
 *
 * Begin method. This method must be called before using this library
 * either directly if the class is initializing the Wire library or by
 * calling this library's function begin(sda, scl) in which case that
 * function will call this one.
 *
 **************************************************************************/
void Pca9554Class::begin(uint8_t address)
{
  // Read out default values from the registers to the shadow variables.
  m_inp = readRegister(address, PCA9554_REG_INP);
  m_out = readRegister(address, PCA9554_REG_OUT);
  m_pol = readRegister(address, PCA9554_REG_POL);
  m_ctrl = readRegister(address, PCA9554_REG_CTRL);
}





/***************************************************************************
 *
 * Sets the desired pin polarity. This can be used to invert inverse
 * hardware logic.
 *
 **************************************************************************/
boolean Pca9554Class::pinPolarity(uint8_t address, uint8_t pin, uint8_t polarity)
{
  // Make sure pin number is OK
  if (pin >= sizeof pinNum2bitNum) {
    return false;
  }

  if (polarity == INVERTED) {
    m_pol |= pinNum2bitNum[pin];
  } else if (polarity == NORMAL) {
    m_pol &= ~pinNum2bitNum[pin];
  } else {
    return false;
  }

  writeRegister(address, PCA9554_REG_POL, m_pol);

  return true;
}



Pca9554Class Pca9554;

