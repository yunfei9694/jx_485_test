/***************************************************************
 * Copyright (C) 2025 RoboForce, Inc. All Rights Reserved.
 * Proprietary and confidential.
 * Unauthorized copying of this file is strictly prohibited.
 ***************************************************************/
#include "rs485_controller.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <thread>

namespace roboforce::driver {
RS485Controller::RS485Controller(const std::string& port_name,
                                 const int baud_rate, MobileBaseVersion version)
    : port_name_(port_name),
      baud_rate_(static_cast<LibSerial::BaudRate>(baud_rate)),
      serial_port_(nullptr),
      version_(version) {
  if (!this->open()) {
    throw std::runtime_error("Failed to open RS485 port: " + port_name);
  }
}
RS485Controller::~RS485Controller() { this->close(); }

bool RS485Controller::open() {
  try {
    serial_port_ = std::make_unique<LibSerial::SerialPort>();
    serial_port_->Open(port_name_);
    serial_port_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_port_->SetCharacterSize(CHARACTER_SIZE);
    serial_port_->SetParity(PARITY);
    serial_port_->SetStopBits(STOP_BITS);

    return serial_port_->IsOpen();
  } catch (const LibSerial::OpenFailed& e) {
    return false;
  } catch (const std::exception& e) {
    return false;
  }
}

bool RS485Controller::close() {
  if (serial_port_ && serial_port_->IsOpen()) {
    serial_port_->Close();
  }
  return !serial_port_ || !serial_port_->IsOpen();
}

// CRC functions
uint16_t RS485Controller::calculateCRC(uint8_t* ptr, uint16_t len) {
  uint16_t crc = 0xffff;
  uint8_t ch;

  for (uint16_t i = 0; i < len; i++) {
    ch = *ptr++;
    crc = CRC_TABLE[(ch ^ crc) & 15] ^ (crc >> 4);
    crc = CRC_TABLE[((ch >> 4) ^ crc) & 15] ^ (crc >> 4);
  }

  return crc;
}

bool RS485Controller::validateFrame(const std::vector<uint8_t>& frame) {
  if (frame.size() < status_frame_size_) {
    return false;
  }

  // Check header
  if (frame[FrameAddressBase::HEADER] != STATUS_HEADER) {
    return false;
  }

  // Validate CRC (big-endian: HIGH byte first, LOW byte second)
  size_t crc_offset = status_frame_size_ - 2;
  uint16_t received_crc = (frame[crc_offset] << 8) | frame[crc_offset + 1];
  uint16_t calculated_crc =
      calculateCRC(const_cast<uint8_t*>(frame.data()), crc_offset);

  return received_crc == calculated_crc;
}

bool RS485Controller::writeAndRead(
    [[maybe_unused]] const std::vector<uint8_t>& command,
    std::vector<uint8_t>& response) {
  if (!serial_port_ || !serial_port_->IsOpen()) {
    if (debug_mode_) {
      std::cout << "[DEBUG] Serial port not open!" << std::endl;
    }
    return false;
  }

  try {
    if (debug_mode_) {
      std::cout << "[DEBUG] Writing " << command.size() << " bytes to serial port..." << std::endl;
    }

    // Write command to serial port
    serial_port_->Write(command);
    serial_port_->DrainWriteBuffer();

    // Add small delay to allow device to process command and prepare response
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (debug_mode_) {
      std::cout << "[DEBUG] Write complete. Attempting to read " << status_frame_size_ << " bytes from serial port (timeout: " << RS485_TIMEOUT << "s)..." << std::endl;
      
      // Check if there's data available to read
      if (serial_port_->IsDataAvailable()) {
        std::cout << "[DEBUG] Data is available for reading" << std::endl;
      } else {
        std::cout << "[DEBUG] No data immediately available, will wait for timeout..." << std::endl;
      }
    }

    // Pre-size response buffer to expected frame size
    response.clear();
    response.reserve(status_frame_size_);

    auto start_time = std::chrono::steady_clock::now();
    
    // Try to read the complete frame with timeout
    try {
      std::vector<uint8_t> temp_buffer(status_frame_size_);
      serial_port_->Read(temp_buffer, status_frame_size_, RS485_TIMEOUT * 1000);
      
      // The Read function modifies the buffer size to reflect actual bytes read
      if (temp_buffer.size() > 0) {
        response = temp_buffer;
      }
    } catch (const std::exception& read_exception) {
      if (debug_mode_) {
        std::cout << "[DEBUG] Read exception: " << read_exception.what() << std::endl;
      }
    }
    
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    if (debug_mode_) {
      std::cout << "[DEBUG] Read completed in " << duration.count() << "ms" << std::endl;
      std::cout << "[DEBUG] Read " << response.size() << " bytes from serial port (expected " << status_frame_size_ << ")" << std::endl;
      if (response.size() != status_frame_size_) {
        std::cout << "[DEBUG] WARNING: Incomplete read! Got " << response.size() << " bytes, expected " << status_frame_size_ << std::endl;
      }
      
      // Print what we actually received
      if (response.size() > 0) {
        std::cout << "[DEBUG] Raw response data: ";
        for (size_t i = 0; i < response.size(); i++) {
          std::cout << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<int>(response[i]);
          if (i < response.size() - 1) {
            std::cout << " ";
          }
        }
        std::cout << std::dec << std::endl;
      } else {
        std::cout << "[DEBUG] No response data received" << std::endl;
      }
    }

    return response.size() == status_frame_size_;
  } catch (const std::exception& e) {
    if (debug_mode_) {
      std::cout << "[DEBUG] Exception in writeAndRead: " << e.what() << std::endl;
    }
    return false;
  }
}

bool RS485Controller::sendCommand() {
  std::vector<uint8_t> command_buffer;
  serializeCommand(command_buffer);

  if (debug_mode_) {
    std::cout << "[DEBUG] Preparing to send command..." << std::endl;
    std::cout << "[DEBUG] Command queue size before push: " << command_queue_.size() << std::endl;
    std::cout << "[DEBUG] Command: ";
    for (size_t i = 0; i < command_buffer.size(); i++) {
      std::cout << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(command_buffer[i]);
      if (i < command_buffer.size() - 1) {
        std::cout << " ";
      }
    }
    std::cout << std::dec << std::endl;
  }

  // Enqueue to command queue for Thread 1
  bool pushed = command_queue_.push(command_buffer,
                                    std::chrono::milliseconds(QUEUE_TIMEOUT_MS));

  if (debug_mode_) {
    if (pushed) {
      std::cout << "[DEBUG] Command successfully pushed to queue" << std::endl;
      std::cout << "[DEBUG] Command queue size after push: " << command_queue_.size() << std::endl;
    } else {
      std::cout << "[DEBUG] Failed to push command to queue (timeout)" << std::endl;
    }
  }

  return pushed;
}

// Thread 1: RS485 communication thread
std::vector<uint8_t> RS485Controller::rs485Thread() {
  if (debug_mode_) {
    std::cout << "[DEBUG] RS485 communication thread started" << std::endl;
  }

  while (thread_running_) {
    if (debug_mode_) {
      std::cout << "[DEBUG] Waiting for command from queue (size: " << command_queue_.size() << ")..." << std::endl;
    }

    // Pop command from queue
    auto command_opt =
        command_queue_.pop(std::chrono::milliseconds(QUEUE_TIMEOUT_MS));

    if (!command_opt.has_value()) {
      if (debug_mode_) {
        std::cout << "[DEBUG] Queue pop timeout (no command available)" << std::endl;
      }
      continue;  // Timeout, try again
    }

    if (debug_mode_) {
      std::cout << "[DEBUG] Command popped from queue successfully" << std::endl;
    }

    std::vector<uint8_t> command = command_opt.value();
    std::vector<uint8_t> response;

    // Send command and read 
    if (writeAndRead(command, response)) {
      // Print response hex even before validation
      if (debug_mode_) {
        std::cout << "[DEBUG] Response received [" << response.size() << " bytes]: ";
        for (size_t i = 0; i < response.size(); i++) {
          std::cout << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<int>(response[i]);
          if (i < response.size() - 1) {
            std::cout << " ";
          }
        }
        std::cout << std::dec << std::endl;
      }

      // Validate response
      if (validateFrame(response)) {
        if (debug_mode_) {
          std::cout << "[DEBUG] Response frame validated successfully" << std::endl;
        }

        // Deserialize and update internal status
        {
          // std::lock_guard<std::mutex> lock(status_mutex_);
          deserializeStatus(response);
          if (debug_mode_) {
            std::cout << "[DEBUG] Status updated successfully" << std::endl;
          }
        }
      } else {
        if (debug_mode_) {
          std::cout << "[DEBUG] Invalid response frame!" << std::endl;
          if (response.size() >= 1) {
            std::cout << "[DEBUG]   Header: 0x" << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(response[0]) << " (expected 0x" << static_cast<int>(STATUS_HEADER) << ")" << std::dec << std::endl;
          }
          if (response.size() >= status_frame_size_) {
            size_t crc_offset = status_frame_size_ - 2;
            uint16_t received_crc = (response[crc_offset] << 8) | response[crc_offset + 1];
            uint16_t calculated_crc = calculateCRC(const_cast<uint8_t*>(response.data()), crc_offset);
            std::cout << "[DEBUG]   Received CRC: 0x" << std::hex << std::setw(4) << std::setfill('0')
                      << received_crc << std::dec << std::endl;
            std::cout << "[DEBUG]   Calculated CRC: 0x" << std::hex << std::setw(4) << std::setfill('0')
                      << calculated_crc << std::dec << std::endl;
          }
        }
      }
    } else {
      if (debug_mode_) {
        std::cout << "[DEBUG] Failed to write/read from serial port" << std::endl;
        std::cout << "[DEBUG] Response buffer size: " << response.size() << " bytes (expected " << status_frame_size_ << ")" << std::endl;
        if (response.size() > 0) {
          std::cout << "[DEBUG] Partial response received: ";
          for (size_t i = 0; i < response.size(); i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(response[i]);
            if (i < response.size() - 1) {
              std::cout << " ";
            }
          }
          std::cout << std::dec << std::endl;
        } else {
          std::cout << "[DEBUG] No response data received (likely read timeout)" << std::endl;
        }
      }
    }
  }

  if (debug_mode_) {
    std::cout << "[DEBUG] RS485 communication thread stopped" << std::endl;
  }
  return std::vector<uint8_t>();  // Return empty vector when thread stops
}

void RS485Controller::startRS485Thread() {
  if (!thread_running_) {
    thread_running_ = true;
    rs485_thread_ = std::thread(&RS485Controller::rs485Thread, this);
  }
}

void RS485Controller::stopRS485Thread() {
  if (thread_running_) {
    thread_running_ = false;
    if (rs485_thread_.joinable()) {
      rs485_thread_.join();
    }
  }
}

bool RS485Controller::setChassisVelocity(float linear_x, float linear_y,
                                         float angular_z) {
  // Clamp values
  linear_x =
      std::clamp(linear_x, CHASSIS_MIN_VELOCITY_RAW, CHASSIS_MAX_VELOCITY_RAW);
  linear_y =
      std::clamp(linear_y, CHASSIS_MIN_VELOCITY_RAW, CHASSIS_MAX_VELOCITY_RAW);
  angular_z =
      std::clamp(angular_z, CHASSIS_MIN_VELOCITY_RAW, CHASSIS_MAX_VELOCITY_RAW);

  // Convert to protocol units: m/s -> mm/s, rad/s -> mrad/s
  command_payload_.chassis_vx = static_cast<int16_t>(linear_x);
  command_payload_.chassis_vy = static_cast<int16_t>(linear_y);
  command_payload_.chassis_vz = static_cast<int16_t>(angular_z);

  return true;
}

bool RS485Controller::setLiftVelocity(float lift_velocity) {
  // Clamp and convert to protocol units: m/s -> mm/s
  lift_velocity =
      std::clamp(lift_velocity, LIFT_MIN_VELOCITY_RAW, LIFT_MAX_VELOCITY_RAW);
  command_payload_.lift_speed = static_cast<int16_t>(lift_velocity);

  return true;
}

bool RS485Controller::serializeCommand(std::vector<uint8_t>& buffer) {
  buffer.resize(COMMAND_FRAME_SIZE);

  buffer[CommandAddress::HEADER] = command_payload_.header;
  buffer[CommandAddress::FUNCTION_CODE] = command_payload_.function_code;

  buffer[CommandAddress::VX_HIGH] = (command_payload_.chassis_vx >> 8) & 0xFF;
  buffer[CommandAddress::VX_LOW] = command_payload_.chassis_vx & 0xFF;

  buffer[CommandAddress::VY_HIGH] = (command_payload_.chassis_vy >> 8) & 0xFF;
  buffer[CommandAddress::VY_LOW] = command_payload_.chassis_vy & 0xFF;

  buffer[CommandAddress::VZ_HIGH] = (command_payload_.chassis_vz >> 8) & 0xFF;
  buffer[CommandAddress::VZ_LOW] = command_payload_.chassis_vz & 0xFF;

  buffer[CommandAddress::LIFT_SPEED_HIGH] =
      (command_payload_.lift_speed >> 8) & 0xFF;
  buffer[CommandAddress::LIFT_SPEED_LOW] = command_payload_.lift_speed & 0xFF;

  buffer[CommandAddress::LIGHT_POWER] = command_payload_.light_power;
  buffer[CommandAddress::SUSPENSION] = command_payload_.suspension_control;

  // Calculate and append CRC (big-endian: HIGH byte first, LOW byte second)
  uint16_t crc = calculateCRC(buffer.data(), COMMAND_FRAME_SIZE - 2);
  buffer[CommandAddress::CRC_LOW] = (crc >> 8) & 0xFF;
  buffer[CommandAddress::CRC_HIGH] = crc & 0xFF;

  return true;
}

/*
V 2.5
*/
RS485ControllerV25::RS485ControllerV25(const std::string& port_name,
                                       const int baud_rate)
    : RS485Controller(port_name, baud_rate, MobileBaseVersion::VERSION_2_5) {
  command_frame_size_ = COMMAND_FRAME_SIZE;
  status_frame_size_ = STATUS_FRAME_SIZE;

  // Explicitly initialize command payload to safe stop state
  command_payload_.header = COMMAND_HEADER;
  command_payload_.function_code = FUNCTION_CODE;
  command_payload_.chassis_vx = 0;
  command_payload_.chassis_vy = 0;
  command_payload_.chassis_vz = 0;
  command_payload_.lift_speed = 0;
  command_payload_.light_power = static_cast<uint8_t>(PowerStatus::ON);
  command_payload_.suspension_control =
      static_cast<uint8_t>(SuspensionStatus::LOCKED);
  command_payload_.crc_low = 0;
  command_payload_.crc_high = 0;
}

bool RS485ControllerV25::deserializeStatus(const std::vector<uint8_t>& buffer) {
  if (buffer.size() < STATUS_FRAME_SIZE) {
    return false;
  }

  status_payload_.chassis_vx =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::VX_HIGH] << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::VX_LOW];
  status_payload_.chassis_vy =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::VY_HIGH] << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::VY_LOW];
  status_payload_.chassis_vz =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::VZ_HIGH] << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::VZ_LOW];
  status_payload_.lift_speed =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::LIFT_SPEED_HIGH]
       << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::LIFT_SPEED_LOW];
  status_payload_.lift_position =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::LIFT_POSITION_HIGH]
       << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::LIFT_POSITION_LOW];
  status_payload_.voltage_48v =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::VOLTAGE_48V_HIGH]
       << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::VOLTAGE_48V_LOW];
  status_payload_.current_48v =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::CURRENT_48V_HIGH]
       << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::CURRENT_48V_LOW];
  status_payload_.voltage_24v =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::VOLTAGE_24V_HIGH]
       << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::VOLTAGE_24V_LOW];
  status_payload_.current_24v =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::CURRENT_24V_HIGH]
       << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::CURRENT_24V_LOW];
  status_payload_.battery_level =
      buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::BATTERY_LEVEL];
  status_payload_.suspension_status =
      buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::SUSPENSION];
  status_payload_.mobile_base_status =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::BASE_STATUS_HIGH]
       << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_2_5>::BASE_STATUS_LOW];

  return true;
}

bool RS485ControllerV25::getChassisVelocity(float& linear_x, float& linear_y,
                                            float& angular_z) {
  // Convert from protocol units: mm/s -> m/s, mrad/s -> rad/s
  // std::lock_guard<std::mutex> lock(status_mutex_);
  linear_x = static_cast<float>(status_payload_.chassis_vx) / 1000.0f;
  linear_y = static_cast<float>(status_payload_.chassis_vy) / 1000.0f;
  angular_z = static_cast<float>(status_payload_.chassis_vz) / 1000.0f;

  return true;
}

bool RS485ControllerV25::getLiftVelocity(float& lift_velocity) {
  // std::lock_guard<std::mutex> lock(status_mutex_);

  // Convert from protocol units: mm/s -> m/s
  lift_velocity = static_cast<float>(status_payload_.lift_speed) / 1000.0f;
  return true;
}

bool RS485ControllerV25::getLiftPosition(float& lift_position) {
  std::lock_guard<std::mutex> lock(status_mutex_);

  // Convert from protocol units: 0.01 mm -> m
  lift_position = static_cast<float>(status_payload_.lift_position) / 1000.0f;
  return true;
}

bool RS485ControllerV25::getBatteryLevel(int& battery) {
  // std::lock_guard<std::mutex> lock(status_mutex_);
  battery = static_cast<int>(status_payload_.battery_level);
  return true;
}

/*
V 3.0
*/
RS485ControllerV30::RS485ControllerV30(const std::string& port_name,
                                       const int baud_rate)
    : RS485Controller(port_name, baud_rate, MobileBaseVersion::VERSION_3_0) {
  command_frame_size_ = COMMAND_FRAME_SIZE;
  status_frame_size_ = STATUS_FRAME_SIZE;

  // Explicitly initialize command payload to safe stop state
  command_payload_.header = COMMAND_HEADER;
  command_payload_.function_code = FUNCTION_CODE;
  command_payload_.chassis_vx = 0;
  command_payload_.chassis_vy = 0;
  command_payload_.chassis_vz = 0;
  command_payload_.lift_speed = 0;
  command_payload_.light_power = static_cast<uint8_t>(LightStatus::OFF);
  command_payload_.suspension_control =
      static_cast<uint8_t>(SuspensionStatus::LOCKED);
  command_payload_.crc_low = 0;
  command_payload_.crc_high = 0;
}

bool RS485ControllerV30::deserializeStatus(const std::vector<uint8_t>& buffer) {
  if (buffer.size() < STATUS_FRAME_SIZE) {
    return false;
  }

  status_payload_.chassis_vx =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::VX_HIGH] << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::VX_LOW];
  status_payload_.chassis_vy =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::VY_HIGH] << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::VY_LOW];
  status_payload_.chassis_vz =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::VZ_HIGH] << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::VZ_LOW];
  status_payload_.lift_speed =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::LIFT_SPEED_HIGH]
       << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::LIFT_SPEED_LOW];
  status_payload_.lift_position =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::LIFT_POSITION_HIGH]
       << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::LIFT_POSITION_LOW];
  status_payload_.voltage_48v =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::VOLTAGE_48V_HIGH]
       << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::VOLTAGE_48V_LOW];
  status_payload_.current_48v =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::CURRENT_48V_HIGH]
       << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::CURRENT_48V_LOW];
  status_payload_.voltage_24v =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::VOLTAGE_24V_HIGH]
       << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::VOLTAGE_24V_LOW];
  status_payload_.current_24v =
      (buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::CURRENT_24V_HIGH]
       << 8) |
      buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::CURRENT_24V_LOW];
  status_payload_.battery_level =
      buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::BATTERY_LEVEL];
  status_payload_.suspension_status =
      buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::SUSPENSION];
  status_payload_.mobile_base_status =
      buffer[StatusAddress<MobileBaseVersion::VERSION_3_0>::BASE_STATUS];

  return true;
}

bool RS485ControllerV30::getChassisVelocity(float& linear_x, float& linear_y,
                                            float& angular_z) {
  // std::lock_guard<std::mutex> lock(status_mutex_);

  linear_x = static_cast<float>(status_payload_.chassis_vx) / 1000.0f;
  linear_y = static_cast<float>(status_payload_.chassis_vy) / 1000.0f;
  angular_z = static_cast<float>(status_payload_.chassis_vz) / 1000.0f;

  return true;
}

bool RS485ControllerV30::getLiftVelocity(float& lift_velocity) {
  // std::lock_guard<std::mutex> lock(status_mutex_);

  lift_velocity = static_cast<float>(status_payload_.lift_speed) / 1000.0f;
  return true;
}

bool RS485ControllerV30::getLiftPosition(float& lift_position) {
  // std::lock_guard<std::mutex> lock(status_mutex_);

  lift_position = static_cast<float>(status_payload_.lift_position) / 1000.0f;
  return true;
}

bool RS485ControllerV30::getBatteryLevel(int& battery) {
  // std::lock_guard<std::mutex> lock(status_mutex_);
  battery = static_cast<int>(status_payload_.battery_level);
  return true;
}
}  // namespace roboforce::driver
