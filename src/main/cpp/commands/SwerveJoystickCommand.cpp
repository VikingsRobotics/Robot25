#include "commands/SwerveJoystickCommand.h"
#ifndef NO_SWERVE_JOYSTICK_COMMAND

#include "Constants.h"

#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/shuffleboard/Shuffleboard.h>

#include <frc2/command/RunCommand.h>

SwerveJoystickCommand::SwerveJoystickCommand(SwerveSubsystem *const subsystem,
		frc2::CommandJoystick &joystick) : m_subsystem { subsystem }, m_joystick {
		joystick.GetHID() }, m_limiterX { Drive::TeleopOperator::kLimiter }, m_limiterY {
		Drive::TeleopOperator::kLimiter }, m_limiterA {
		Drive::TeleopOperator::kLimiter } {
	AddRequirements (m_subsystem);
	SetName("Swerve Joystick Command");

	frc::ShuffleboardTab &smart = frc::Shuffleboard::GetTab("SmartDashboard");
	frc::ShuffleboardLayout &layout = smart.GetLayout("Swerve",
			frc::BuiltInLayouts::kList);

	layout.AddNumber("Throttle", [&]() -> double {
		return (-m_joystick.GetRawAxis(3) + 1) / 2;
	});
	layout.AddBoolean("Field Centric", [&]() -> bool {
		return m_fieldCentric;
	});
	layout.AddBoolean("Stored Throttle", [&]() -> bool {
		return false;
	});
	layout.AddBoolean("Precision Mode", [&]() -> bool {
		return m_precision;
	});
}

void SwerveJoystickCommand::Initialize() {
	m_fieldCentric = true;
	m_precision = false;
	m_limiterX.Reset(0);
	m_limiterY.Reset(0);
	m_limiterA.Reset(0);
}

void SwerveJoystickCommand::Execute() {
	double throttle = (-m_joystick.GetRawAxis(3) + 1) / 2;
	if (m_joystick.GetRawButtonPressed(2)) {
		m_fieldCentric = !m_fieldCentric;
	}
	if (m_joystick.GetRawButtonPressed(4)) {
		m_precision = !m_precision;
	}
	if (m_joystick.GetRawButtonPressed(5)) {
		m_subsystem->ZeroHeading();
	}
	if (m_joystick.GetRawButton(1)) {
		m_subsystem->Brake();
		return;
	}

	double controllerX = m_limiterX.Calculate(m_joystick.GetRawAxis(1));
	double controllerY = m_limiterY.Calculate(m_joystick.GetRawAxis(0));
	double controllerRot = m_limiterA.Calculate(m_joystick.GetRawAxis(2));

	double deadbandMove = Drive::TeleopOperator::kDriveDeadband;
	double deadbandRot = Drive::TeleopOperator::kDriveAngleDeadband;

	double renormalizedX = 0.0;
	double renormalizedY = 0.0;
	double renormalizedA = 0.0;

	double normalizedDeadbandX = controllerX < 0 ? deadbandMove : -deadbandMove;
	double normalizedDeadbandY = controllerY < 0 ? deadbandMove : -deadbandMove;
	double normalizedDeadbandA =
			controllerRot < 0 ? deadbandMove : -deadbandMove;

	// X^2 + Y^2 > D^2 
	// Easier process than sqrt(X^2 + Y^2) > Deadband
	if (controllerX * controllerX + controllerY * controllerY
			> deadbandMove * deadbandMove) {
		renormalizedX = -(controllerX + normalizedDeadbandX)
				/ (1 - deadbandMove);
		renormalizedY = -(controllerY + normalizedDeadbandY)
				/ (1 - deadbandMove);
	}
	if (std::abs(controllerRot) > deadbandRot) {
		renormalizedA = -(controllerRot + normalizedDeadbandA)
				/ (1 - deadbandRot);
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

	frc::ChassisSpeeds speeds { };
	if (m_fieldCentric) {
		speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, va,
				m_subsystem->GetHeading());
	} else {
		speeds = frc::ChassisSpeeds { vx, vy, va };
	}

	wpi::array < frc::SwerveModuleState, 4 > swerveModule =
			Drive::DeviceProperties::SystemControl::kDriveKinematics.ToSwerveModuleStates(
					speeds);

	for (size_t i = 0; i < swerveModule.size(); ++i) {
		if (units::math::abs(swerveModule[i].speed) < 0.001_mps) {
			swerveModule[i].speed = 0_mps;
			swerveModule[i].angle = m_lastState[i];
		}
		m_lastState[i] = swerveModule[i].angle;
	}

	m_subsystem->SetModulesState(swerveModule);
}

void SwerveJoystickCommand::End(bool interrupted) {
	m_subsystem->StopModules();
}

bool SwerveJoystickCommand::IsFinished() {
	return false;
}

#endif
