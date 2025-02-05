#include "subsystems/SwerveSubsystem.h"
#ifndef NO_SWERVE

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/FlippingUtil.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/config/RobotConfig.h>

#include <units/math.h>

// @formatter:off 
SwerveSubsystem::SwerveSubsystem() : m_getGyroYaw { m_gryo.GetYaw().AsSupplier() }, m_odometry {
		Drive::DeviceProperties::SystemControl::kDriveKinematics, frc::Rotation2d {
				units::radian_t { 0 } }, { m_frontLeft.GetPosition(),
				m_frontRight.GetPosition(), m_backLeft.GetPosition(),
				m_backRight.GetPosition() } } {
	m_gryo.Reset();
	SetName("Swerve Subsystem");

	pathplanner::RobotConfig config =
			pathplanner::RobotConfig::fromGUISettings();

    pathplanner::AutoBuilder::configure(
        [this](){ return GetPose2d(); },
        [this](frc::Pose2d pose){ ResetOdometry(pose); },
        [this](){ return GetCurrentSpeeds(); },
        [this](const frc::ChassisSpeeds speeds, const pathplanner::DriveFeedforwards dff) { 
			Drive(speeds,dff.robotRelativeForcesX,dff.robotRelativeForcesY); 
		},
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
	frc::SmartDashboard::PutData("Field", &m_field);

    // Logging callback for current robot pose
    pathplanner::PathPlannerLogging::setLogCurrentPoseCallback([this](frc::Pose2d pose){
        // Do whatever you want with the pose here
        m_field.SetRobotPose(pose);
    });

    // Logging callback for target robot pose
    pathplanner::PathPlannerLogging::setLogTargetPoseCallback([this](frc::Pose2d pose){
        // Do whatever you want with the pose here
        m_field.GetObject("target pose")->SetPose(pose);
    });

    // Logging callback for the active path, this is sent as a vector of poses
    pathplanner::PathPlannerLogging::setLogActivePathCallback([this](std::vector<frc::Pose2d> poses){
        // Do whatever you want with the poses here
        m_field.GetObject("path")->SetPoses(poses);
    });

	frc::SmartDashboard::PutData(this);
}
// @formatter:on
void SwerveSubsystem::Periodic() {
	// Tracks robot position using the position of swerve modules and gryo rotation
	m_odometry.Update(GetRotation2d(),
			{ m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
					m_backLeft.GetPosition(), m_backRight.GetPosition() });
	frc::SmartDashboard::PutNumber("Gyro Angle (deg)", GetHeading().value());

}

void SwerveSubsystem::ZeroHeading() {
	m_gryo.Reset();
}

static units::degree_t WrapAngle(units::degree_t angle) {
	angle = units::math::fmod(angle + 180_deg, 360_deg);
	if (angle < 0_deg)
		angle += 360_deg;
	return angle - 180_deg;
}

units::degree_t SwerveSubsystem::GetHeading() {
	return WrapAngle(m_getGyroYaw());
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
	Drive::DeviceProperties::SystemControl::kDriveKinematics.DesaturateWheelSpeeds(
			&states, Drive::Mechanism::kPhysicalMoveMax);
	// Calls every swerve modules SetStates function
	m_frontLeft.SetState(states[0]);
	m_frontRight.SetState(states[1]);
	m_backLeft.SetState(states[2]);
	m_backRight.SetState(states[3]);
}

void SwerveSubsystem::SetModulesState(
		wpi::array<frc::SwerveModuleState, 4> states,
		const std::vector<units::newton_t> &feedforwardX,
		const std::vector<units::newton_t> &feedforwardY) {
	// Make sure that we are under max speed
	Drive::DeviceProperties::SystemControl::kDriveKinematics.DesaturateWheelSpeeds(
			&states, Drive::Mechanism::kPhysicalMoveMax);
	// Calls every swerve modules SetStates function with feedforward
	m_frontLeft.SetState(states[0], feedforwardX.at(0), feedforwardY.at(0));
	m_frontRight.SetState(states[1], feedforwardX.at(1), feedforwardY.at(1));
	m_backLeft.SetState(states[2], feedforwardX.at(2), feedforwardY.at(2));
	m_backRight.SetState(states[3], feedforwardX.at(3), feedforwardY.at(3));
}

frc::ChassisSpeeds SwerveSubsystem::GetCurrentSpeeds() {
	return Drive::DeviceProperties::SystemControl::kDriveKinematics.ToChassisSpeeds(
			{ m_frontLeft.GetState(), m_frontRight.GetState(),
					m_backLeft.GetState(), m_backRight.GetState() });
}

void SwerveSubsystem::Drive(frc::ChassisSpeeds speed) {
	wpi::array < frc::SwerveModuleState, 4 > states =
			Drive::DeviceProperties::SystemControl::kDriveKinematics.ToSwerveModuleStates(
					speed);
	SetModulesState (states);
}

void SwerveSubsystem::Drive(frc::ChassisSpeeds speed,
		const std::vector<units::newton_t> &feedforwardX,
		const std::vector<units::newton_t> &feedforwardY) {
	wpi::array < frc::SwerveModuleState, 4 > states =
			Drive::DeviceProperties::SystemControl::kDriveKinematics.ToSwerveModuleStates(
					speed);
	SetModulesState(states, feedforwardX, feedforwardY);
}

void SwerveSubsystem::Brake() {
	m_frontLeft.Brake();
	m_frontRight.Brake();
	m_backLeft.Brake();
	m_backRight.Brake();
}

void SwerveSubsystem::X() {
	m_frontLeft.SetState(frc::SwerveModuleState { 0_mps, frc::Rotation2d {
			45_deg } });
	m_frontRight.SetState(frc::SwerveModuleState { 0_mps, frc::Rotation2d {
			-45_deg } });
	m_backLeft.SetState(frc::SwerveModuleState { 0_mps, frc::Rotation2d {
			-45_deg } });
	m_backRight.SetState(frc::SwerveModuleState { 0_mps, frc::Rotation2d {
			45_deg } });
}

#endif
