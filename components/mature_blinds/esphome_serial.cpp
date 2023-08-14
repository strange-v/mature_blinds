#include "esphome_serial.h"

namespace esphome {
namespace esphome_serial {

EsphomeSerial::EsphomeSerial(uart::UARTDevice *uart_device) { uart_device_ = uart_device; }
EsphomeSerial::~EsphomeSerial() {}

size_t EsphomeSerial::write(uint8_t byte) { return uart_device_->write(byte); }

int EsphomeSerial::read() { return uart_device_->read(); }

int EsphomeSerial::available() { return uart_device_->available(); }

int EsphomeSerial::peek() { return uart_device_->peek(); }

void EsphomeSerial::flush() { uart_device_->flush(); }

}  // namespace esphome_serial
}  // namespace esphome
