/***************************************************************
 * Copyright (C) 2025 RoboForce, Inc. All Rights Reserved.
 * Proprietary and confidential.
 * Unauthorized copying of this file is strictly prohibited.
 ***************************************************************/
#pragma once

#include <libserial/SerialPort.h>

#include <atomic>
#include <map>
#include <mutex>
#include <thread>

#include "sync_queue.h"

namespace roboforce::driver {
static constexpr float CHASSIS_MAX_VELOCITY_RAW = 2000;  // mm/s and 0.001 rad/s
static constexpr float CHASSIS_MIN_VELOCITY_RAW =
    -2000;                                            // mm/s and 0.001 rad/s
static constexpr float LIFT_MAX_VELOCITY_RAW = 300;   // 0.01 m/s
static constexpr float LIFT_MIN_VELOCITY_RAW = -300;  // 0.01 m/s
static constexpr int RS485_TIMEOUT = 5;  // Increased timeout to 5 seconds

// RS485 format
static constexpr auto CHARACTER_SIZE = LibSerial::CharacterSize::CHAR_SIZE_8;
static constexpr auto PARITY = LibSerial::Parity::PARITY_NONE;
static constexpr auto STOP_BITS = LibSerial::StopBits::STOP_BITS_1;
static constexpr uint8_t COMMAND_HEADER = 0xA5;
static constexpr uint8_t STATUS_HEADER = 0xA8;
static constexpr uint8_t FUNCTION_CODE = 0x01;
static constexpr uint8_t LIGHT_POWER_STATUS = 0x00;

// CRC verification
static constexpr uint16_t CRC_TABLE[] = {
    0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
    0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400};
static const std::map<uint16_t, std::string> CHASSIS_2_5_LOOKUP = {
    {0x0000, "All good"},
    {0x0001, "String sensor comm overtime"},
    {0x0002, "Lift driver comm overtime"},
    {0x0004, "Forward driver comm overtime"},
    {0x0008, "Turn driver comm overtime"},
    {0x0010, "Encoder comm overtime"},
    {0x0020, "Battery comm overtime"},
    {0x0040, "Remote comm overtime"},
    {0x0080, "Forward motor error"},
    {0x0100, "Turn motor error"},
    {0x0200, "Lift motor error"},
    {0x0400, "Lift encoder zero position not set"},
    {0x0800, "E-Stop"}};

enum class MobileBaseVersion { VERSION_2_5, VERSION_3_0 };

// Data frame abstraction
struct FrameAddressBase {
  static constexpr uint8_t HEADER = 0;
  static constexpr uint8_t FUNCTION_CODE = 1;
  static constexpr uint8_t VX_HIGH = 2;
  static constexpr uint8_t VX_LOW = 3;
  static constexpr uint8_t VY_HIGH = 4;
  static constexpr uint8_t VY_LOW = 5;
  static constexpr uint8_t VZ_HIGH = 6;
  static constexpr uint8_t VZ_LOW = 7;
  static constexpr uint8_t LIFT_SPEED_HIGH = 8;
  static constexpr uint8_t LIFT_SPEED_LOW = 9;
  static constexpr uint8_t LIGHT_POWER = 10;
};

// command/status specific dataframe
struct CommandAddress : FrameAddressBase {
  static constexpr uint8_t SUSPENSION = 11;
  static constexpr uint8_t CRC_LOW = 13;
  static constexpr uint8_t CRC_HIGH = 12;
};

struct StatusAddressBase : FrameAddressBase {
  static constexpr uint8_t LIFT_POSITION_HIGH = 10;
  static constexpr uint8_t LIFT_POSITION_LOW = 11;
  static constexpr uint8_t VOLTAGE_48V_HIGH = 12;
  static constexpr uint8_t VOLTAGE_48V_LOW = 13;
  static constexpr uint8_t CURRENT_48V_HIGH = 14;
  static constexpr uint8_t CURRENT_48V_LOW = 15;
  static constexpr uint8_t VOLTAGE_24V_HIGH = 16;
  static constexpr uint8_t VOLTAGE_24V_LOW = 17;
  static constexpr uint8_t CURRENT_24V_HIGH = 18;
  static constexpr uint8_t CURRENT_24V_LOW = 19;
  static constexpr uint8_t BATTERY_LEVEL = 20;
  static constexpr uint8_t SUSPENSION = 21;
};

// dataframe templates
template <MobileBaseVersion V>
struct StatusAddress;

template <>
struct StatusAddress<MobileBaseVersion::VERSION_2_5> : StatusAddressBase {
  static constexpr uint8_t BASE_STATUS_HIGH = 22;
  static constexpr uint8_t BASE_STATUS_LOW = 23;
  static constexpr uint8_t CRC_LOW = 24;
  static constexpr uint8_t CRC_HIGH = 25;
};

template <>
struct StatusAddress<MobileBaseVersion::VERSION_3_0> : StatusAddressBase {
  static constexpr uint8_t BASE_STATUS = 22;
  static constexpr uint8_t CRC_LOW = 23;
  static constexpr uint8_t CRC_HIGH = 24;
};

// Payload abstraction
struct CommandPayload {
  uint8_t header;              // 0xA5
  uint8_t function_code;       // 0x01
  int16_t chassis_vx;          // -2000 ~ 2000 mm/s
  int16_t chassis_vy;          // -2000 ~ 2000 mm/s
  int16_t chassis_vz;          // -2 ~ 2 rad/s
  int16_t lift_speed;          // -30 ~ 30 mm/s
  uint8_t light_power;         // 0x00 off/ 0x01 on
  uint8_t suspension_control;  // 0x00 locked/ 0x01 unlocked
  uint8_t crc_low;
  uint8_t crc_high;
};

struct StatusPayloadBase : CommandPayload {
  int16_t lift_position;      // 0 ~ 600 mm
  uint16_t voltage_48v;       // 10 mV
  int16_t current_48v;        // 10 mA
  uint16_t voltage_24v;       // 10 mV
  int16_t current_24v;        // 10 mA
  uint8_t battery_level;      // 0x00 off/ 0x01 on
  uint8_t suspension_status;  // 0x00 locked/ 0x01 unlocked
};

// payload templates
template <MobileBaseVersion V>
struct StatusPayload;

template <>
struct StatusPayload<MobileBaseVersion::VERSION_2_5> : StatusPayloadBase {
  uint16_t mobile_base_status;
};

template <>
struct StatusPayload<MobileBaseVersion::VERSION_3_0> : StatusPayloadBase {
  uint8_t mobile_base_status;
};

enum class LightStatus { OFF, ON };
enum class PowerStatus { OFF, ON };

enum class SuspensionStatus : uint8_t {
  UNLOCKED = 0x00,
  LOCKED = 0x01,
};

struct MobileBaseStatus {
  uint16_t status_code;
  std::string status_message;
  bool is_error;
};

// Base RS485 controller
class RS485Controller {
 public:
  RS485Controller(const std::string& port_name, const int baud_rate_,
                  MobileBaseVersion version);
  virtual ~RS485Controller();

  // Chassis functions
  bool setChassisVelocity(float linear_x, float linear_y, float angular_z);
  virtual bool getChassisVelocity(float& linear_x, float& linear_y,
                                  float& angular_z) = 0;

  // Lift functions
  bool setLiftVelocity(float lift_velocity);
  virtual bool getLiftVelocity(float& lift_velocity) = 0;
  virtual bool getLiftPosition(float& lift_position) = 0;

  // Power domain functions
  virtual bool getBatteryLevel(int& battery) = 0;

  // Send command to Thread 1
  bool sendCommand();

  // Start/Stop Thread 1
  void startRS485Thread();
  void stopRS485Thread();

  // Debug functions
  void setDebugMode(bool enable) { debug_mode_ = enable; }
  bool getDebugMode() const { return debug_mode_; }

 protected:
  std::mutex status_mutex_;

  bool open();
  bool close();

  // CRC functions
  uint16_t calculateCRC(uint8_t* ptr, uint16_t len);
  bool validateFrame(const std::vector<uint8_t>& frame);

  bool serializeCommand(std::vector<uint8_t>& buffer);
  virtual bool deserializeStatus(const std::vector<uint8_t>& buffer) = 0;

  bool writeAndRead(const std::vector<uint8_t>& command,
                    std::vector<uint8_t>& response);

  std::vector<uint8_t> rs485Thread();

  std::string port_name_;
  LibSerial::BaudRate baud_rate_;
  std::unique_ptr<LibSerial::SerialPort> serial_port_;
  MobileBaseVersion version_;

  std::thread rs485_thread_;
  std::atomic<bool> thread_running_;

  static constexpr size_t QUEUE_SIZE = 100;
  static constexpr int QUEUE_TIMEOUT_MS = 200;

  // Command queue: Thread 0 -> Thread 1
  SyncQueue<std::vector<uint8_t>> command_queue_{QUEUE_SIZE};

  size_t command_frame_size_;
  size_t status_frame_size_;

  static constexpr size_t COMMAND_FRAME_SIZE = 14;

  CommandPayload command_payload_;

  bool debug_mode_ = false;
};

// Version 2.5 controller
class RS485ControllerV25 : public RS485Controller {
 public:
  static constexpr size_t STATUS_FRAME_SIZE = 26;

  RS485ControllerV25(const std::string& port_name, const int baud_rate);
  ~RS485ControllerV25() override = default;

  // Override base class methods

  bool getChassisVelocity(float& linear_x, float& linear_y,
                          float& angular_z) override;

  bool getLiftVelocity(float& lift_velocity) override;
  bool getLiftPosition(float& lift_position) override;

  bool getBatteryLevel(int& battery) override;

 protected:
  bool deserializeStatus(const std::vector<uint8_t>& buffer) override;

 private:
  StatusPayload<MobileBaseVersion::VERSION_2_5> status_payload_;
};

// Version 3.0 controller
class RS485ControllerV30 : public RS485Controller {
 public:
  static constexpr size_t STATUS_FRAME_SIZE = 25;

  RS485ControllerV30(const std::string& port_name, const int baud_rate);
  ~RS485ControllerV30() override = default;

  // Override base class methods
  bool getChassisVelocity(float& linear_x, float& linear_y,
                          float& angular_z) override;

  bool getLiftVelocity(float& lift_velocity) override;
  bool getLiftPosition(float& lift_position) override;

  bool getBatteryLevel(int& battery) override;

 protected:
  bool deserializeStatus(const std::vector<uint8_t>& buffer) override;

 private:
  StatusPayload<MobileBaseVersion::VERSION_3_0> status_payload_;
};
}  // namespace roboforce::driver
