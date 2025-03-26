#pragma once

//Sets declaration (can be multiple as long as resolve to one definition) 

#include <units/velocity.h>
#include <units/angular_velocity.h>

#include <units/acceleration.h>
#include <units/angular_acceleration.h>

#include <units/time.h>
#include <units/frequency.h>

#include <units/length.h>

#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>

#include <frc/geometry/Transform3d.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/config/ClosedLoopConfig.h>
#include <rev/AbsoluteEncoder.h>

#include <pathplanner/lib/config/PIDConstants.h>

#include <frc/kinematics/SwerveDriveKinematics.h>

#include <frc/PneumaticsModuleType.h>

#include <math.h>
#include <numbers>

namespace Drive {

namespace ControllerPorts {
//USB ID for joystick for driver
constexpr int kDriverControllerPort = 0;
}

namespace TeleopOperator {
//Slew rate limiter for input controller
constexpr units::hertz_t kLimiter = units::scalar_t { 1 } / 1_s;
//Minimum percent of joystick distance before robot response (move)
constexpr double kDriveDeadband = 0.15;
//Minimum percent of joystick twist distance before robot response (angle)
constexpr double kDriveAngleDeadband = 0.15;
//Maximum speed that the robot will move (limited by physical design)
constexpr units::meters_per_second_t kDriveMoveSpeedMax = 3.0_mps;
//Speed when speed throttle is less that precision throttle threshold
constexpr units::meters_per_second_t kDrivePrecision = 0.6_mps;
//What the throttle value will be on startup
constexpr double kDefaultThrottleXbox = 0.7;
//Maximum turning speed that the robot will move (limited by physical design)
constexpr units::radians_per_second_t kDriveAngleSpeedMax = 3.0_rad_per_s;
//Maximum turning speed that the robot will move (limited by physical design)
constexpr units::radians_per_second_t kDriveAngleSpeedPrecision = 0.5_rad_per_s;
}

namespace DeviceProperties {
// Default motor type used for REV spark max motors
extern rev::spark::SparkMaxConfig& GetSparkMaxConfig();
// Default motor type used for TalonFX motors
extern ctre::phoenix6::configs::TalonFXConfiguration GetTalonFXConfig();
// Default motor type enum for REV spark max motors
constexpr rev::spark::SparkLowLevel::MotorType kSparkMotorType =
		rev::spark::SparkLowLevel::MotorType::kBrushless;
// Invert absolute encoder to match direction of motor movement
constexpr bool kInvertEncoder = true;
namespace SystemControl {
//Swerve Kinematics (in cpp)
extern frc::SwerveDriveKinematics<4> kDriveKinematics;
}
}

namespace Mechanism {
// How "tall" the base of the robot is
constexpr units::inch_t kWheelBase = 30_in;
// How wide the base of the robot is
constexpr units::inch_t kTrackWidth = 30_in;
// 990 motor teeth to 195 wheel teeth, converts motor rotations to wheel rotations
constexpr units::scalar_t kDriveGearRatio { 990.0 / 195.0 };
// 1 rot to 2 pi radians, converts motor rotations to radians
constexpr units::scalar_t kAngleGearRatio { 2 * std::numbers::pi };
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
		kWheelCircumference / (units::turn_t { kDriveGearRatio.value() }) };
// Min voltage required for driving motor to begin moving
constexpr units::volt_t kStaticVoltage { 0.15 };
// kV for feedforward, target rotation is multipled kV and added to velocity control
constexpr units::volt_t kVelocityVoltage { 12 / kDriveRps.value() };
// Max speed the wheel move, used to normialize swerve modules speeds to maintain control
constexpr units::meters_per_second_t kPhysicalMoveMax { kDriveRps
		* kWheelCircumference / (units::turn_t { kDriveGearRatio.value() }) };
// Nm divided by N resulting in meters of the newton force of the wheels
constexpr units::meter_t kDriveMotorNewtonForce = (kWheelDiameter / 2)
		/ kDriveGearRatio;
}

namespace SysId {
using ramp_rate_t = units::unit_t<
units::compound_unit<units::volt, units::inverse<units::second>>>;
namespace Translation {
constexpr ramp_rate_t kRampRate { 1.0 };
constexpr units::volt_t kStepVoltage { 7.0 };
constexpr units::second_t kTimeout { 10.0 };
}
namespace Rotation {
constexpr units::radians_per_second_squared_t kRotationRate { std::numbers::pi
		/ 6.0 };
constexpr ramp_rate_t kRampRate = kRotationRate * (1_V * 1_s / 1_rad);
constexpr units::radians_per_second_t kStepRotation { std::numbers::pi };
constexpr units::volt_t kStepVoltage = kStepRotation * (1_V * 1_s / 1_rad);
constexpr units::second_t kTimeout { 10.0 };
}
namespace Steer {
constexpr ramp_rate_t kRampRate { 1.0 };
constexpr units::volt_t kStepVoltage { 7.0 };
constexpr units::second_t kTimeout { 10.0 };
}

}

namespace Vision {
constexpr units::second_t kProcessDataEvery = 0.5_s;
constexpr uint8_t kProcessDataOnNth = 8;
constexpr float kRequiredConfidence = 0.0;
constexpr units::meter_t kRequiredDeltaDistance { 0.5 };
constexpr size_t kNumAprilTags = 22;
constexpr frc::Transform3d kCameraMountingPosition { frc::Translation3d { 0_m,
		15_in, 3_in }, frc::Rotation3d { 0_rad, 0_rad, 0_rad } };
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

// Converting java to C++
// Source: https://github.com/ElectroBunny/BetaBot2025/blob/develop/src/main/java/frc/robot/commands/AlignToReefTagRelative.java
namespace Align {
// Source: https://github.com/ElectroBunny/BetaBot2025/blob/develop/src/main/java/frc/robot/Constants.java
namespace System {
constexpr double kXTranslationP = 0.33;
constexpr double kYTranslationP = 0.33;
constexpr double kRotationP = 0.33;
}
namespace Location {
constexpr units::meter_t kXTranslationEndpoint = -0.34_m;
constexpr units::meter_t kXTranslationThreshold = 0.02_m;
constexpr units::meter_t kYTranslationEndpoint = 0.16_m;
constexpr units::meter_t kYTranslationThreshold = 0.02_m;
constexpr units::radian_t kRotationEndpoint = 0_rad;
constexpr units::radian_t kRotationThreshold = 1_deg;
}
namespace Time {
constexpr units::second_t kTagOutOfViewTime = 1_s;
constexpr units::second_t kPoseValidation = 0.3_s;
}
}
}

namespace Arm {

namespace TeleopOperator {
//Debounce on different positions
constexpr units::second_t kDebounce { 1.0 };
//Minimum percent of controller distance before robot response
constexpr double kArmDeadband = 0.15;
}

namespace Destination {
constexpr units::turn_t kAllowableError { 0.1 };
constexpr units::second_t kAllowableSwitchTime { 0.5 };
constexpr units::turn_t kMinTurn { 0.0 };
constexpr units::turn_t kMaxTurn { 0.5 };
constexpr units::turn_t kBottomTurn { 0.15 };
constexpr units::turn_t kMiddleTurn { 0.25 };
constexpr units::turn_t kTopTurn { 0.4 };
}

namespace DeviceProperties {
// Default motor type used for REV spark max motors
extern rev::spark::SparkMaxConfig& GetSparkMaxConfig();
// Default motor type enum for REV spark max motors
constexpr rev::spark::SparkLowLevel::MotorType kSparkMotorType =
		rev::spark::SparkLowLevel::MotorType::kBrushless;
// Invert absolute encoder to match direction of motor movement
constexpr bool kInvertEncoder = false;
}

namespace Mechanism {
constexpr units::volt_t kStaticVoltage { 0.0 };
constexpr units::turn_t kRotationalOffset { 0.0 };
constexpr units::turns_per_second_t kMaxAngularSpeed { 1.0 };
constexpr units::turns_per_second_squared_t kMaxAngularAcceleration { 1.0 };
constexpr units::scalar_t kGearRatio { 1.0 / 45.0 };
}
}

namespace Elevator {

namespace ControllerPorts {
//USB ID for xbox controller for driver
constexpr int kDriverControllerPort = 1;
}

namespace TeleopOperator {
//Debounce on different positions
constexpr units::second_t kDebounce { 0.1 };
//Minimum percent of controller distance before robot response
constexpr double kDriveDeadband = 0.15;
}

namespace Destination {
constexpr units::meter_t kAllowableError { 0.1 };
constexpr units::second_t kAllowableSwitchTime { 0.5 };
constexpr units::meter_t kMaxHeight = 26_in;
constexpr units::meter_t kCollectionHeight = 7_in;
constexpr units::meter_t kFourthGoal = 23_in;
constexpr units::meter_t kThirdGoal = 20_in;
constexpr units::meter_t kSecondGoal = 14_in;
constexpr units::meter_t kFirstGoal = 10_in;
constexpr units::meter_t kMinHeight = 0_in;
}

namespace Mechanism {
constexpr units::volt_t kStaticVoltage { 0.0 };
constexpr units::scalar_t kGearRatio { 1 / 20.0 };
constexpr units::meter_t kGearDiameter = units::inch_t { 2 };
constexpr units::unit_t<
		units::compound_unit<units::turn, units::inverse<units::meter>>> kDistanceToRotation =
		(1_tr) / (kGearDiameter * std::numbers::pi * kGearRatio);
// 2 feet per second
constexpr units::meters_per_second_t kMaxSpeedInMeters = 10000_fps;
// 2 feet per second squared
constexpr units::meters_per_second_squared_t kMaxAccelerationInMeters =
		10000_fps_sq;
}

namespace DeviceProperties {
// Default motor type used for REV spark max motors
extern rev::spark::SparkMaxConfig& GetElevatorConfig();
// Default motor type used for REV spark max motors
extern rev::spark::SparkMaxConfig& GetFollowerConfig();
// Default motor type enum for REV spark max motors
constexpr rev::spark::SparkLowLevel::MotorType kSparkMotorType =
		rev::spark::SparkLowLevel::MotorType::kBrushless;
}
}

namespace Roller {
namespace DeviceProperties {
// Pneumatics Hub Type
constexpr frc::PneumaticsModuleType kModuleType =
		frc::PneumaticsModuleType::CTREPCM;
}
namespace SolenoidId {
constexpr int kForwardChannelId = 0;
constexpr int kReverseChannelId = 1;
}

}

namespace DeviceIdentifier {
//CTRE: CANBus Name for contructors of CRTE software classes
constexpr ctre::phoenix6::CANBus kCANBus { "" };
//CTRE: PDH
constexpr int kPDHId = 1;
//REV: Pneumatic Hub
constexpr int kPneumaticHubId = 0;
//CTRE: CANBus Pigeon2 ID
constexpr int kGyroId = 3;
//CTRE: Falcon 500 Front Left Motor ID
constexpr int kFLDriveMotorId = 4;
//REV: Neo 550 Front Left Angle Motor ID
constexpr int kFLAngleMotorId = 5;
//CTRE: Falcon 500 Front Right Motor ID
constexpr int kFRDriveMotorId = 6;
//REV: Neo 550 Front Right Angle Motor ID
constexpr int kFRAngleMotorId = 7;
//CTRE: Falcon 500 Back Left Motor ID
constexpr int kBLDriveMotorId = 8;
//REV: Neo 550 Back Left Angle Motor ID
constexpr int kBLAngleMotorId = 9;
//CTRE: Falcon 500 Back Right Motor ID
constexpr int kBRDriveMotorId = 10;
//REV: Neo 550 Back Right Angle Motor ID
constexpr int kBRAngleMotorId = 11;
//REV: Neo 500 opposite of light
constexpr int kElevatorDriverId = 12;
//REV: Neo 500 by light
constexpr int kElevatorFollowId = 13;
//REV: Neo 500
constexpr int kDirectionMotorId = 14;
//VictorSPX
constexpr int kRepellerWheelId = 15;
//TalonSRX
constexpr int kRollerWheelId = 16;
}
