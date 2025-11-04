/***************************************************************
 * Copyright (C) 2025 RoboForce, Inc. All Rights Reserved.
 * Proprietary and confidential.
 * Unauthorized copying of this file is strictly prohibited.
 ***************************************************************/
#include "rs485_controller.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

using namespace roboforce::driver;

void printHex(const std::string& label, const std::vector<uint8_t>& data) {
    std::cout << label << " [" << data.size() << " bytes]: ";
    for (size_t i = 0; i < data.size(); i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(data[i]);
        if (i < data.size() - 1) {
            std::cout << " ";
        }
    }
    std::cout << std::dec << std::endl;
}

void printStatus(RS485Controller* controller) {
    float linear_x, linear_y, angular_z;
    float lift_velocity, lift_position;
    int battery_level;

    // Get chassis velocity
    if (controller->getChassisVelocity(linear_x, linear_y, angular_z)) {
        std::cout << "\n=== Chassis Status ===" << std::endl;
        std::cout << "Linear X:  " << std::fixed << std::setprecision(3)
                  << linear_x << " m/s" << std::endl;
        std::cout << "Linear Y:  " << std::fixed << std::setprecision(3)
                  << linear_y << " m/s" << std::endl;
        std::cout << "Angular Z: " << std::fixed << std::setprecision(3)
                  << angular_z << " rad/s" << std::endl;
    } else {
        std::cout << "Failed to get chassis velocity" << std::endl;
    }

    // Get lift status
    if (controller->getLiftVelocity(lift_velocity)) {
        std::cout << "\n=== Lift Status ===" << std::endl;
        std::cout << "Lift Velocity: " << std::fixed << std::setprecision(3)
                  << lift_velocity << " m/s" << std::endl;
    } else {
        std::cout << "Failed to get lift velocity" << std::endl;
    }

    if (controller->getLiftPosition(lift_position)) {
        std::cout << "Lift Position: " << std::fixed << std::setprecision(3)
                  << lift_position << " m" << std::endl;
    } else {
        std::cout << "Failed to get lift position" << std::endl;
    }

    // Get battery level
    if (controller->getBatteryLevel(battery_level)) {
        std::cout << "\n=== Battery Status ===" << std::endl;
        std::cout << "Battery Level: " << battery_level << "%" << std::endl;
    } else {
        std::cout << "Failed to get battery level" << std::endl;
    }

    std::cout << "\n=====================\n" << std::endl;
}

int main(int argc, char** argv) {
    // Default parameters
    std::string port_name = "/dev/ttyUSB0";
    int baud_rate = 115200;
    std::string version = "2.5";

    // Parse command line arguments
    if (argc > 1) {
        port_name = argv[1];
    }
    if (argc > 2) {
        baud_rate = std::stoi(argv[2]);
    }
    if (argc > 3) {
        version = argv[3];
    }

    std::cout << "RS485 Test Script" << std::endl;
    std::cout << "=================" << std::endl;
    std::cout << "Port: " << port_name << std::endl;
    std::cout << "Baud Rate: " << baud_rate << std::endl;
    std::cout << "Version: " << version << std::endl;
    std::cout << std::endl;

    try {
        // Create controller based on version
        RS485Controller* controller = nullptr;
        if (version == "2.5") {
            controller = new RS485ControllerV25(port_name, baud_rate);
            std::cout << "Created RS485 Controller V2.5" << std::endl;
            std::cout << "Successfully initialized port " << port_name << " at " << baud_rate << " baud" << std::endl;
        } else if (version == "3.0") {
            controller = new RS485ControllerV30(port_name, baud_rate);
            std::cout << "Created RS485 Controller V3.0" << std::endl;
            std::cout << "Successfully initialized port " << port_name << " at " << baud_rate << " baud" << std::endl;
        } else {
            std::cerr << "Unknown version: " << version << std::endl;
            std::cerr << "Supported versions: 2.5, 3.0" << std::endl;
            return 1;
        }

        // Start RS485 communication thread
        std::cout << "Starting RS485 communication thread..." << std::endl;

        // Enable debug mode
        controller->setDebugMode(true);
        std::cout << "Debug mode enabled - will print hex data\n" << std::endl;

        controller->startRS485Thread();

        // Set all velocities to zero
        std::cout << "Setting chassis velocity to (0, 0, 0)..." << std::endl;
        if (!controller->setChassisVelocity(100.0f, 0.0f, 0.0f)) {
            std::cerr << "Failed to set chassis velocity" << std::endl;
        }

        std::cout << "Setting lift velocity to 0..." << std::endl;
        if (!controller->setLiftVelocity(0.0f)) {
            std::cerr << "Failed to set lift velocity" << std::endl;
        }

        // Send the command
        std::cout << "Sending command..." << std::endl;
        if (!controller->sendCommand()) {
            std::cerr << "Failed to send command" << std::endl;
        }

        // Wait for response to be received and processed
        std::cout << "Waiting for response..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // Print status
        printStatus(controller);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        printStatus(controller);

        // Keep running and printing status every 2 seconds
        std::cout << "Monitoring status (press Ctrl+C to exit)..." << std::endl;
        for (int i = 0; i < 10; i++) {
            std::this_thread::sleep_for(std::chrono::seconds(2));

            // Send command to get updated status
            controller->sendCommand();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            std::cout << "Status update #" << (i + 1) << ":" << std::endl;
            printStatus(controller);
        }

        // Stop RS485 thread
        std::cout << "Stopping RS485 communication thread..." << std::endl;
        controller->stopRS485Thread();

        // Clean up
        delete controller;
        std::cout << "Test completed successfully!" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
