#pragma once

#include "esphome/components/uart/uart.h"
#include "Stream.h"

namespace esphome {
namespace esphome_serial {

class EsphomeSerial : public Stream {
 public:
  EsphomeSerial(uart::UARTDevice *uart_device);
  ~EsphomeSerial();

  virtual size_t write(uint8_t byte);
  virtual int read();
  virtual int available();
  virtual int peek();
  virtual void flush();
  operator bool() { return true; }

 private:
  uart::UARTDevice *uart_device_{nullptr};
};

}  // namespace esphome_serial
}  // namespace esphome
