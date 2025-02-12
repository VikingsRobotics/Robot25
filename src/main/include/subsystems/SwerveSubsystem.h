#pragma once

#include "subsystems/SwerveModule.h"
#include "Constants.h"

#include <vector>

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
	void SetModulesState(wpi::array<frc::SwerveModuleState, 4> states,
			const std::vector<units::newton_t> &feedforwardX,
			const std::vector<units::newton_t> &feedforwardY);

	frc::ChassisSpeeds GetCurrentSpeeds();

	void Drive(frc::ChassisSpeeds speed);
	void Drive(frc::ChassisSpeeds speed,
			const std::vector<units::newton_t> &feedforwardX,
			const std::vector<units::newton_t> &feedforwardY);

	void Brake();

	void X();

	friend struct SwerveSysIdRoutine;
private:
	// Gryo used for odometry and for field centric control
	ctre::phoenix6::hardware::Pigeon2 m_gryo { DeviceIdentifier::kGyroId,
			DeviceIdentifier::kCANBus };
	std::function<units::angle::degree_t()> m_getGyroYaw;
	// Front Left module
	SwerveModule m_frontLeft { DeviceIdentifier::kFLDriveMotorId,
			DeviceIdentifier::kFLAngleMotorId, units::radian_t {
					-std::numbers::pi / 2.0 } };
	// Front Right module
	SwerveModule m_frontRight { DeviceIdentifier::kFRDriveMotorId,
			DeviceIdentifier::kFRAngleMotorId, units::radian_t { 0 } };
	// Back Left module
	SwerveModule m_backLeft { DeviceIdentifier::kBLDriveMotorId,
			DeviceIdentifier::kBLAngleMotorId, units::radian_t {
					std::numbers::pi } };
	// Back Right module
	SwerveModule m_backRight { DeviceIdentifier::kBRDriveMotorId,
			DeviceIdentifier::kBRAngleMotorId, units::radian_t {
					std::numbers::pi / 2.0 } };
	// Track the position of the robot using wheel position and gryo rotation
	frc::SwerveDriveOdometry<4> m_odometry;

	frc::Field2d m_field { };
};
