#include "subsystems/SwerveSubsystem.h"

#include <frc/smartdashboard/smartdashboard.h>
#include <frc/DriverStation.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/FlippingUtil.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/config/RobotConfig.h>

SwerveSubsystem::SwerveSubsystem() : m_odometry {
		Drive::SystemControl::kDriveKinematics, frc::Rotation2d {
				units::radian_t { 0 } }, { m_frontLeft.GetPosition(),
				m_frontRight.GetPosition(), m_backLeft.GetPosition(),
				m_backRight.GetPosition() } } {
	m_gryo.Reset();
	SetName("Swerve Subsystem");

	pathplanner::RobotConfig config =
			pathplanner::RobotConfig::fromGUISettings();

	// @formatter:off 
    pathplanner::AutoBuilder::configure(
        [this](){ return GetPose2d(); },
        [this](frc::Pose2d pose){ ResetOdometry(pose); },
        [this](){ return GetCurrentSpeeds(); },
        [this](const frc::ChassisSpeeds speeds, const pathplanner::DriveFeedforwards dff){ Drive(speeds); },
        std::make_shared<pathplanner::PPHolonomicDriveController>( 
            Drive::AutoSettings::kTranslationPID,
            Drive::AutoSettings::kRotationalPID
        ),
        config,
        []() {
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this
    );
    // @formatter:on
	pathplanner::PathPlannerLogging::setLogActivePathCallback(
			[this](auto poses) {
				m_field.GetObject("path")->SetPoses(poses);
			});

	frc::SmartDashboard::PutData("Field Picture", &m_field);
	frc::SmartDashboard::PutData(this);
}

void SwerveSubsystem::Periodic() {
	// Tracks robot position using the position of swerve modules and gryo rotation
	m_odometry.Update(GetRotation2d(),
			{ m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
					m_backLeft.GetPosition(), m_backRight.GetPosition() });
}

void SwerveSubsystem::ZeroHeading() {
	m_gryo.Reset();
}

units::degree_t SwerveSubsystem::GetHeading() {
	double angle = m_gryo.GetAngle();
	double angleLeft = std::fmod(-angle, 360.0);
	double angleRight = 360.0 - std::fmod(angle, 360.0);
	return units::degree_t { angle > 0 ? angleLeft : angleRight };
}

frc::Rotation2d SwerveSubsystem::GetRotation2d() {
	return frc::Rotation2d { GetHeading() };
}

frc::Pose2d SwerveSubsystem::GetPose2d() {
	return m_odometry.GetPose();
}

void SwerveSubsystem::ResetOdometry(frc::Pose2d pose) {
	// Resets pose but still requires the current state of swerve module and gryo rotation
	m_odometry.ResetPosition(GetRotation2d(),
			{ m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
					m_backLeft.GetPosition(), m_backRight.GetPosition() },
			pose);
	m_field.SetRobotPose(GetPose2d());
}

void SwerveSubsystem::StopModules() {
	// Calls every swerve modules Stop function
	m_frontLeft.Stop();
	m_frontRight.Stop();
	m_backLeft.Stop();
	m_backRight.Stop();
}

void SwerveSubsystem::SetModulesState(
		wpi::array<frc::SwerveModuleState, 4> states) {
	// Make sure that we are under max speed
	Drive::SystemControl::kDriveKinematics.DesaturateWheelSpeeds(&states,
			Drive::Mechanism::kPhysicalMoveMax);
	// Calls every swerve modules SetStates function
	m_frontLeft.SetState(states[0]);
	m_frontRight.SetState(states[1]);
	m_backLeft.SetState(states[2]);
	m_backRight.SetState(states[3]);
}

frc::ChassisSpeeds SwerveSubsystem::GetCurrentSpeeds() {
	return Drive::SystemControl::kDriveKinematics.ToChassisSpeeds(
			{ m_frontLeft.GetState(), m_frontRight.GetState(),
					m_backLeft.GetState(), m_backRight.GetState() });
}

void SwerveSubsystem::Drive(frc::ChassisSpeeds speed) {
	wpi::array < frc::SwerveModuleState, 4 > states =
			Drive::SystemControl::kDriveKinematics.ToSwerveModuleStates(speed);
	SetModulesState (states);
}

void SwerveSubsystem::Brake() {
	m_frontLeft.SetState(frc::SwerveModuleState { 0_mps, frc::Rotation2d {
			45_deg } });
	m_frontRight.SetState(frc::SwerveModuleState { 0_mps, frc::Rotation2d {
			-45_deg } });
	m_backLeft.SetState(frc::SwerveModuleState { 0_mps, frc::Rotation2d {
			-45_deg } });
	m_backRight.SetState(frc::SwerveModuleState { 0_mps, frc::Rotation2d {
			45_deg } });
}
