#include "commands/SwerveJoystickCommand.h"
#include "Constants.h"

#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <wpi/array.h>

SwerveJoystickCommand::SwerveJoystickCommand(SwerveSubsystem *const subsystem,
		frc2::CommandJoystick &joystick, bool enableFieldCentric) : m_subsystem {
		subsystem }, m_joystick { joystick.GetHID() }, m_fieldCentric {
		enableFieldCentric } {
	AddRequirements (m_subsystem);
	SetName("Swerve Joystick Command");
}

void SwerveJoystickCommand::Initialize() {
	// Nothing (for now >:])
}

void SwerveJoystickCommand::Execute() {
	frc::SmartDashboard::PutNumber("Throttle", m_joystick.GetThrottle());
	frc::SmartDashboard::PutBoolean("Field Centric", m_fieldCentric);
	if (m_joystick.GetRawButton(7)) {
		m_subsystem->Brake();
		return;
	}

	double renormalizedX = 0.0;
	double renormalizedY = 0.0;
	double renormalizedA = 0.0;
	// X^2 + Y^2 > D^2 
	// Easier process than sqrt(X^2 + Y^2) > Deadband
	if (m_joystick.GetX() * m_joystick.GetX()
			+ m_joystick.GetY() * m_joystick.GetY()
			> Drive::TeleopOperator::kDriveDeadband
					* Drive::TeleopOperator::kDriveDeadband) {
		renormalizedX = m_joystick.GetX()
				/ (1 - Drive::TeleopOperator::kDriveDeadband);
		renormalizedY = m_joystick.GetY()
				/ (1 - Drive::TeleopOperator::kDriveDeadband);
	}
	if (m_joystick.GetTwist() > Drive::TeleopOperator::kDriveAngleDeadband) {
		renormalizedA = m_joystick.GetTwist()
				/ (1 - Drive::TeleopOperator::kDriveAngleDeadband);
	}

	units::meters_per_second_t vx =
			m_joystick.GetThrottle()
					< Drive::TeleopOperator::kPrecisionThrottleThreshold ?
					Drive::TeleopOperator::kDrivePrecision * renormalizedX :
					Drive::TeleopOperator::kDriveMoveSpeedMax * renormalizedX;
	units::meters_per_second_t vy =
			m_joystick.GetThrottle()
					< Drive::TeleopOperator::kPrecisionThrottleThreshold ?
					Drive::TeleopOperator::kDrivePrecision * renormalizedY :
					Drive::TeleopOperator::kDriveMoveSpeedMax * renormalizedY;
	units::radians_per_second_t va =
			m_joystick.GetThrottle()
					< Drive::TeleopOperator::kPrecisionThrottleThreshold ?
					Drive::TeleopOperator::kDriveAngleSpeedPrecision
							* renormalizedA :
					Drive::TeleopOperator::kDriveAngleSpeedMax * renormalizedA;

	wpi::array < frc::SwerveModuleState, 4
			> swerveModule {
					Drive::SystemControl::kDriveKinematics.ToSwerveModuleStates(
							m_fieldCentric ?
									frc::ChassisSpeeds::FromFieldRelativeSpeeds(
											vx, vy, va,
											frc::Rotation2d {
													m_subsystem->GetHeading() }) :
									frc::ChassisSpeeds::FromRobotRelativeSpeeds(
											vx, vy, va,
											frc::Rotation2d {
													m_subsystem->GetHeading() })) };
	m_subsystem->SetModulesState(swerveModule);
}

void SwerveJoystickCommand::End(bool interrupted) {
	m_subsystem->StopModules();
}

bool SwerveJoystickCommand::IsFinished() {
	return false;
}

bool SwerveJoystickCommand::IsFieldCentric() {
	return m_fieldCentric;
}

void SwerveJoystickCommand::SetFieldCentric(bool enableFieldCentric) {
	m_fieldCentric = enableFieldCentric;
}
