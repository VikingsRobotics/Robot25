#pragma once

//Sets declaration (can be multiple as long as resolve to one definition) 

#include <units/velocity.h>
#include <units/angular_velocity.h>

#include <units/acceleration.h>
#include <units/angular_acceleration.h>

#include <units/length.h>

#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include <rev/SparkMax.h>
#include <rev/config/ClosedLoopConfig.h>
#include <rev/AbsoluteEncoder.h>

#include <pathplanner/lib/config/PIDConstants.h>

#include <frc/kinematics/SwerveDriveKinematics.h>

#include <math.h>
#include <numbers>

namespace Drive {

namespace ControllerPorts {
//USB ID for joystick for driver
constexpr int kDriverControllerPort = 0;
}

namespace TeleopOperator {
//Minimum percent of joystick distance before robot response (move)
constexpr double kDriveDeadband = 0.05;
//Minimum percent of joystick twist distance before robot response (angle)
constexpr double kDriveAngleDeadband = 0.05;
//Maximum speed that the robot will move (limited by physical design)
constexpr units::meters_per_second_t kDriveMoveSpeedMax = 3.0_mps;
//Minimum speed that the robot will move (limited by physical design)
constexpr units::meters_per_second_t kDriveMoveSpeedLow = 1.5_mps;
//Percentage that the mimimum speed is compared to maximum speed
constexpr double kPercentDriveMoveLow = kDriveMoveSpeedLow / kDriveMoveSpeedMax;
//Speed when speed throttle is less that precision throttle threshold
constexpr units::meters_per_second_t kDrivePrecision = 0.6_mps;
//Percentage that the percision speed is compared to maximum speed
constexpr double kPercentDriveMovePrecision = kDrivePrecision
		/ kDriveMoveSpeedMax;
//[-1 ... 1] when throttle transforms to precision
constexpr double kPrecisionThrottleThreshold = -0.6;
//Maximum turning speed that the robot will move (limited by physical design)
constexpr units::radians_per_second_t kDriveAngleSpeedMax = 3.0_rad_per_s;
//Maximum turning speed that the robot will move (limited by physical design)
constexpr units::radians_per_second_t kDriveAngleSpeedLow = 1.0_rad_per_s;
//Maximum turning speed that the robot will move (limited by physical design)
constexpr units::radians_per_second_t kDriveAngleSpeedPrecision = 0.5_rad_per_s;
//Percentage that the mimimum turning speed is compared to maximum turning speed
constexpr double kPercentDriveAngleLow = kDriveAngleSpeedLow
		/ kDriveAngleSpeedMax;
//Percentage that the mimimum turning speed is compared to maximum turning speed
constexpr double kPercentDriveAnglePrecision = kDriveAngleSpeedPrecision
		/ kDriveAngleSpeedMax;
}

namespace DeviceIdentifier {
//CTRE: CANBus Name for contructors of CRTE software classes
constexpr ctre::phoenix6::CANBus kCANBus { "" };
//CTRE: CANBus Pigeon2 ID
constexpr int kGyroId = 1;
//CTRE: Falcon 500 Front Left Motor ID
constexpr int kFLDriveMotorId = 2;
//REV: Neo 550 Front Left Angle Motor ID
constexpr int kFLAngleMotorId = 3;
//CTRE: Falcon 500 Front Right Motor ID
constexpr int kFRDriveMotorId = 4;
//REV: Neo 550 Front Right Angle Motor ID
constexpr int kFRAngleMotorId = 5;
//CTRE: Falcon 500 Back Left Motor ID
constexpr int kBLDriveMotorId = 6;
//REV: Neo 550 Back Left Angle Motor ID
constexpr int kBLAngleMotorId = 7;
//CTRE: Falcon 500 Back Right Motor ID
constexpr int kBRDriveMotorId = 8;
//REV: Neo 550 Back Right Angle Motor ID
constexpr int kBRAngleMotorId = 9;
}

namespace DeviceProperties {
// Default motor type used for REV spark max motors
constexpr rev::spark::SparkLowLevel::MotorType kSparkMotorType =
		rev::spark::SparkLowLevel::MotorType::kBrushless;
// Default voltage configs (nominal)
extern ctre::phoenix6::configs::VoltageConfigs kDriveMotorVoltageConfig;
// Default drive motor output configs
extern ctre::phoenix6::configs::MotorOutputConfigs kDriveMotorOutputConfig;
// Invert absolute encoder to match direction of motor movement
constexpr bool kInvertEncoder = true;
}

namespace Mechanism {
// How "tall" the base of the robot is
constexpr units::inch_t kWheelBase = 30_in;
// How wide the base of the robot is
constexpr units::inch_t kTrackWidth = 30_in;
// 990 motor teeth to 195 wheel teeth, converts motor rotations to wheel rotations
constexpr units::turn_t kDriveGearRatio { 990.0 / 195.0 };
// 1 rot to 2 pi radians, converts motor rotations to radians
constexpr units::turn_t kAngleGearRatio { 2 * std::numbers::pi };
// Max 108 rotations per sec from driving motor, without gear ratios
constexpr units::turns_per_second_t kDriveRps { 108 };
// Wheel diameter in inches, 3
constexpr units::inch_t kWheelDiameter { 3 };
// Wheel circumference in meters, ~0.24
constexpr units::meter_t kWheelCircumference { kWheelDiameter * std::numbers::pi };
// Converts meters to rotations, mps to rps, and mps^2 to rps^2, rotations = (meterTarget * gearRatio) / WheelCircumferenceMeters
constexpr units::unit_t<
		units::compound_unit<units::length::meters,
				units::inverse<units::angle::turns>>> kWheelTurnsToMetersDistance {
		kWheelCircumference / kDriveGearRatio };
// Min voltage required for driving motor to begin moving
constexpr units::volt_t kStaticVoltage { 0.15 };
// kV for feedforward, target rotation is multipled kV and added to velocity control
constexpr units::volt_t kVelocityVoltage { 12 / kDriveRps.value() };
// Max speed the wheel move, used to normialize swerve modules speeds to maintain control
constexpr units::meters_per_second_t kPhysicalMoveMax { kDriveRps
		* kWheelCircumference / kDriveGearRatio };
}

namespace SystemControl {
//Velocity PIDF Values (For CTRE Velocity Motor)
extern ctre::phoenix6::configs::Slot0Configs kVelocityPID;
//Angle PID Values
constexpr double kAngleP = 1.0;
constexpr double kAngleI = 0.0;
constexpr double kAngleD = 0.0;
constexpr double kAngleF = 0.0;
//extern rev::spark::ClosedLoopConfig kAnglePID;
//Swerve Kinematics (in cpp)
extern frc::SwerveDriveKinematics<4> kDriveKinematics;
}

namespace AutoSettings {
//Translation PID Values (PathplannerLib)
constexpr pathplanner::PIDConstants kTranslationPID { 1.0, 0.0, 0.0 };
//Rotation PID Values (PathplannerLib)
constexpr pathplanner::PIDConstants kRotationalPID { 1.0, 0.0, 0.0 };
//Max speed that PathplannerLib can use on the robot (limited by physical design)
constexpr units::meters_per_second_t kMaxSpeed = 5.0_mps;
//Max acceleration that Pathplanner can use on the robot (limited by physical design)
constexpr units::meters_per_second_squared_t kMaxAcceleration = 3.0_mps_sq;
//Max angular speed that PathplannerLib can use on the robot (limited by physical design)
constexpr units::radians_per_second_t kMaxAngularSpeed = 3.0_rad_per_s;
//Max angular acceleration that PathplannerLib can use on the robot (limited by physical design)
constexpr units::radians_per_second_squared_t kMaxAngularAcceleration =
		5.0_rad_per_s_sq;
}
}
