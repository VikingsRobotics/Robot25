#include "commands/SwerveJoystickCommand.h"
#include "Constants.h"

#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/RunCommand.h>

#include <wpi/array.h>

SwerveJoystickCommand::SwerveJoystickCommand(SwerveSubsystem *const subsystem,
		frc2::CommandJoystick &joystick, bool enableFieldCentric) : m_subsystem {
		subsystem }, m_joystick { joystick.GetHID() }, m_fieldCentric {
		enableFieldCentric } {
	AddRequirements (m_subsystem);
	SetName("Swerve Joystick Command");

	joystick.Button(2).OnTrue(frc2::RunCommand([this]() {
		m_fieldCentric = !m_fieldCentric;
	}).ToPtr());
	joystick.Button(5).OnTrue(frc2::RunCommand([this]() {
		m_subsystem->ZeroHeading();
	}).ToPtr());
	joystick.Button(4).OnTrue(frc2::RunCommand([this]() {
		m_precision = !m_precision;
	}).ToPtr());
}

void SwerveJoystickCommand::Initialize() {
	m_fieldCentric = true;
	m_precision = false;
}

void SwerveJoystickCommand::Execute() {
	double throttle = -m_joystick.GetRawAxis(
			frc::Joystick::AxisType::kThrottleAxis) / 2;
	frc::SmartDashboard::PutNumber("Throttle", m_joystick.GetThrottle());
	frc::SmartDashboard::PutBoolean("Field Centric", m_fieldCentric);
	frc::SmartDashboard::PutBoolean("Stored Throttle", false);
	frc::SmartDashboard::PutBoolean("Precision Mode", m_precision);
	if (m_joystick.GetRawButton(1)) {
		m_subsystem->Brake();
		return;
	}

	double controllerX = m_joystick.GetRawAxis(frc::Joystick::AxisType::kXAxis);
	double controllerY = m_joystick.GetRawAxis(frc::Joystick::AxisType::kYAxis);
	double controllerRot = m_joystick.GetRawAxis(
			frc::Joystick::AxisType::kZAxis);

	double deadbandMove = Drive::TeleopOperator::kDriveDeadband;
	double deadbandRot = Drive::TeleopOperator::kDriveAngleDeadband;

	double renormalizedX = 0.0;
	double renormalizedY = 0.0;
	double renormalizedA = 0.0;
	// X^2 + Y^2 > D^2 
	// Easier process than sqrt(X^2 + Y^2) > Deadband
	if (controllerX * controllerX + controllerY * controllerY
			> deadbandMove * deadbandMove) {
		renormalizedX = -controllerX / (1 - deadbandMove);
		renormalizedY = -controllerY / (1 - deadbandMove);
	}
	if (std::abs(controllerRot) > deadbandRot) {
		renormalizedA = -controllerRot / (1 - deadbandRot);
	}

	units::meters_per_second_t vx = 0_mps;
	units::meters_per_second_t vy = 0_mps;
	units::radians_per_second_t va = 0_rad_per_s;

	if (!m_precision) {
		vx = renormalizedX * throttle
				* Drive::TeleopOperator::kDriveMoveSpeedMax;
		vy = renormalizedY * throttle
				* Drive::TeleopOperator::kDriveMoveSpeedMax;
		va = renormalizedA * throttle
				* Drive::TeleopOperator::kDriveAngleSpeedMax;
	} else {
		vx = renormalizedX * throttle * Drive::TeleopOperator::kDrivePrecision;
		vy = renormalizedY * throttle * Drive::TeleopOperator::kDrivePrecision;
		va = renormalizedA * throttle
				* Drive::TeleopOperator::kDriveAngleSpeedPrecision;
	}

	wpi::array < frc::SwerveModuleState, 4 > swerveModule { wpi::empty_array };

	if (m_fieldCentric) {
		swerveModule =
				Drive::DeviceProperties::SystemControl::kDriveKinematics.ToSwerveModuleStates(
						frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, va,
								m_subsystem->GetHeading()));
	} else {
		swerveModule =
				Drive::DeviceProperties::SystemControl::kDriveKinematics.ToSwerveModuleStates(
						frc::ChassisSpeeds { vx, vy, va });
	}

	m_subsystem->SetModulesState(swerveModule);
}

void SwerveJoystickCommand::End(bool interrupted) {
	m_subsystem->StopModules();
}

bool SwerveJoystickCommand::IsFinished() {
	return false;
}
