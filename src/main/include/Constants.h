#pragma once

//Sets declaration (can be multiple as long as resolve to one definition) 

#include <units/velocity.h>
#include <units/angular_velocity.h>

namespace Drive {

    namespace ControllerPorts {
        //TBD USB ID for joystick or controller
        constexpr int kDriverControllerPort = 0;
    }

    namespace TeleopOperator {
        //TBD Max Speed, Deadband, Precision during Teleop
        constexpr double kDriveDeadband = 0.05;

        constexpr double kDriveAngleDeadband = 0.4;

        constexpr units::meters_per_second_t kDriveMoveSpeedMax = 3.0_mps;

        constexpr units::meters_per_second_t kDriveMoveSpeedLow = 1.5_mps;

        constexpr double kPercentDriveLow = kDriveMoveSpeedLow/kDriveMoveSpeedMax;

        constexpr units::meters_per_second_t kDrivePrecision = 0.6_mps;

        constexpr double kPercentDrivePrecision = kDrivePrecision/kDriveMoveSpeedMax;

        constexpr double kPrecisionThrottleThreshold = -0.6;

        constexpr units::radians_per_second_t kDriveAngleSpeedMax = 3.0_rad_per_s;
    }

    namespace DeviceIdentifier {
        //TBD Real World CANBus IDs
        constexpr int kGyroId = 1;
       
        constexpr int kFLDriveMotorId = 2;

        constexpr int kFLAngleMotorId = 3; 

        constexpr int kFRDriveMotorId = 4;

        constexpr int kFRAngleMotorId = 5;

        constexpr int kBLDriveMotorId = 6;

        constexpr int kBLAngleMotorId = 7;

        constexpr int kBRDriveMotorId = 8;

        constexpr int kBRAngleMotorId = 9;
    }

    namespace DeviceProperties {
        //TBD Default Settings for Software Classes
    }

    namespace Mechanism {
        //TBD Gear ratios for Real Robot
    }

    namespace SystemControl {
        //TBD Real World PID values and measurements
    }

    namespace AutoSettings {
        //TBD Max Speed value during auto
    }
}