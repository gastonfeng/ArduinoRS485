/*
  This file is part of the ArduinoRS485 library.
  Copyright (c) 2018 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "RS485.h"
#include <libmaple/usart.h>

void usart_disable_rx(usart_dev *dev)
{
  usart_reg_map *regs = dev->regs;
  // while (!rb_is_empty(dev->wb))
  //     ; // wait for TX completed
  // /* TC bit must be high before disabling the USART */
  // while ((regs->CR1 & USART_CR1_UE) && !(regs->SR & USART_SR_TC))
  //     ;

  regs->CR1 &= ~USART_CR1_RE; // don't change the word length etc, and 'or' in the patten not overwrite |USART_CR1_M_8N1);
  usart_reset_rx(dev);
}

void usart_enable_rx(usart_dev *dev)
{
  usart_reg_map *regs = dev->regs;
  while ((regs->CR1 & USART_CR1_UE) && !(regs->SR & USART_SR_TC))
    ;
  regs->CR1 |= (USART_CR1_TE | USART_CR1_RE |
                USART_CR1_RXNEIE); // don't change the word length etc, and 'or' in the patten not overwrite |USART_CR1_M_8N1);
  regs->CR1 |= USART_CR1_UE;
}

RS485Class::RS485Class(usart_dev *usart_device, int txPin, uint8 rx_pin, int dePin, int rePin) : HardwareSerial(
                                                                                                     usart_device, txPin, rx_pin),
                                                                                                 _txPin(txPin),
                                                                                                 _dePin(dePin),
                                                                                                 _rePin(rePin),
                                                                                                 _transmisionBegun(
                                                                                                     false)
{
}

void RS485Class::begin(unsigned long baudrate)
{
  begin(baudrate, SERIAL_8N1);
}

void RS485Class::begin(unsigned long baudrate, uint16_t config)
{
  _baudrate = baudrate;
  _config = config;

  if (_dePin > -1)
  {
    pinMode(_dePin, OUTPUT);
    digitalWrite(_dePin, LOW);
  }

  if (_rePin > -1)
  {
    pinMode(_rePin, OUTPUT);
    digitalWrite(_rePin, HIGH);
  }

  _transmisionBegun = false;

  HardwareSerial::begin(baudrate, config);
}

void RS485Class::end()
{
  this->end();

  if (_rePin > -1)
  {
    digitalWrite(_rePin, LOW);
    pinMode(_dePin, INPUT);
  }

  if (_dePin > -1)
  {
    digitalWrite(_dePin, LOW);
    pinMode(_rePin, INPUT);
  }
}

int RS485Class::available()
{
  return HardwareSerial::available();
}

int RS485Class::peek()
{
  return HardwareSerial::peek();
}

int RS485Class::read(void)
{
  return HardwareSerial::read();
}

void RS485Class::flush()
{
  return HardwareSerial::flush();
}

size_t RS485Class::write(uint8_t b)
{
  if (!_transmisionBegun)
  {
    setWriteError();
    return 0;
  }

  return HardwareSerial::write(b);
}

RS485Class::operator bool()
{
  return true;
}

void RS485Class::beginTransmission()
{
  if (_dePin > -1)
  {
    digitalWrite(_dePin, HIGH);
    delayMicroseconds(50);
  }
  noReceive();
  _transmisionBegun = true;
}

void RS485Class::endTransmission()
{
  this->flush();

  if (_dePin > -1)
  {
    delayMicroseconds(50);
    digitalWrite(_dePin, LOW);
  }
  receive();
  _transmisionBegun = false;
}

void RS485Class::receive()
{
  if (_rePin > -1)
  {
    digitalWrite(_rePin, LOW);
  }
  usart_enable_rx(this->c_dev());
}

void RS485Class::noReceive()
{
  if (_rePin > -1)
  {
    digitalWrite(_rePin, HIGH);
  }
  usart_disable_rx(this->c_dev());
}

void RS485Class::sendBreak(unsigned int duration)
{
  this->flush();
  this->end();
  pinMode(_txPin, OUTPUT);
  digitalWrite(_txPin, LOW);
  delay(duration);
  this->begin(_baudrate, _config);
}

void RS485Class::sendBreakMicroseconds(unsigned int duration)
{
  this->flush();
  this->end();
  pinMode(_txPin, OUTPUT);
  digitalWrite(_txPin, LOW);
  delayMicroseconds(duration);
  this->begin(_baudrate, _config);
}

void RS485Class::setPins(int txPin, int dePin, int rePin)
{
  _txPin = txPin;
  _dePin = dePin;
  _rePin = rePin;
}

//RS485Class RS485(SERIAL_PORT_HARDWARE, RS485_DEFAULT_TX_PIN, RS845_DEFAULT_DE_PIN, RS845_DEFAULT_RE_PIN);
