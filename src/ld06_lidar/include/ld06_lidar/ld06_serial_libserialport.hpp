/*
 * Альтернативна реалізація для libserialport
 * Якщо ти хочеш використовувати libserialport замість ROS serial пакету
 */

#pragma once

// #include <cstdint>
#include <libserialport.h>
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include "ld06_node.hpp"

namespace ld06 {

class SerialReader {
 public:
  SerialReader(const std::string& port, uint32_t baudrate,
               uint32_t timeout_ms = 5);
  ~SerialReader();

  bool is_open() const { return port_ != nullptr; }
  bool read_packet(std::vector<uint8_t>& packet);

 private:
  struct sp_port* port_ = nullptr;
  std::vector<uint8_t> buffer_;

  bool sync_to_header();
  void throw_error(const std::string& msg, int error_code);
};

}  // namespace ld06
