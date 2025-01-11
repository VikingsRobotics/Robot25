#pragma once

#include "subsystems/SwerveModule.h"
#include "Constants.h"

#include <ctre/phoenix6/Pigeon2.hpp>

#include <frc/smartdashboard/Field2d.h>

#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/kinematics/SwerveDriveOdometry.h>

#include <frc2/command/SubsystemBase.h>

class SwerveSubsystem: public frc2::SubsystemBase {
public:
	SwerveSubsystem();
	SwerveSubsystem(SwerveSubsystem &rhs) = delete;
	SwerveSubsystem& operator=(SwerveSubsystem &rhs) = delete;
	SwerveSubsystem(SwerveSubsystem &&rhs) = delete;
	SwerveSubsystem& operator=(SwerveSubsystem &&rhs) = delete;

	void Periodic() override;

	void ZeroHeading();

	units::degree_t GetHeading();

	frc::Rotation2d GetRotation2d();

	frc::Pose2d GetPose2d();

	void ResetOdometry(frc::Pose2d pose);

	void StopModules();

	void SetModulesState(wpi::array<frc::SwerveModuleState, 4> states);

	frc::ChassisSpeeds GetCurrentSpeeds();

	void Drive(frc::ChassisSpeeds speed);

	void Brake();

	void X();
private:
	// Gryo used for odometry and for field centric control
	ctre::phoenix6::hardware::Pigeon2 m_gryo { Drive::DeviceIdentifier::kGyroId,
			Drive::DeviceIdentifier::kCANBus };
	std::function<units::angle::degree_t()> m_getGyroYaw;
	// Front Left module
	SwerveModule m_frontLeft { Drive::DeviceIdentifier::kFLDriveMotorId,
			Drive::DeviceIdentifier::kFLAngleMotorId, units::radian_t {
					-std::numbers::pi / 2.0 } };
	// Front Right module
	SwerveModule m_frontRight { Drive::DeviceIdentifier::kFRDriveMotorId,
			Drive::DeviceIdentifier::kFRAngleMotorId, units::radian_t { 0 } };
	// Back Left module
	SwerveModule m_backLeft { Drive::DeviceIdentifier::kBLDriveMotorId,
			Drive::DeviceIdentifier::kBLAngleMotorId, units::radian_t {
					std::numbers::pi } };
	// Back Right module
	SwerveModule m_backRight { Drive::DeviceIdentifier::kBRDriveMotorId,
			Drive::DeviceIdentifier::kBRAngleMotorId, units::radian_t {
					std::numbers::pi / 2.0 } };
	// Track the position of the robot using wheel position and gryo rotation
	frc::SwerveDriveOdometry<4> m_odometry;

	frc::Field2d m_field { };
};
