#include "commands/SwerveControllerCommand.h"
#include "Constants.h"

#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/RunCommand.h>

SwerveControllerCommand::SwerveControllerCommand(
		SwerveSubsystem *const subsystem,
		frc2::CommandXboxController &controller) : m_subsystem { subsystem }, m_controller {
		controller.GetHID() }, m_limiterX { Drive::TeleopOperator::kLimiter }, m_limiterY {
		Drive::TeleopOperator::kLimiter }, m_limiterA {
		Drive::TeleopOperator::kLimiter } {
	AddRequirements (m_subsystem);
	SetName("Swerve Xbox Controller Command");
}

void SwerveControllerCommand::Initialize() {
	m_internalThrottle = Drive::TeleopOperator::kDefaultThrottleXbox;
	m_fieldCentric = true;
	m_storedThrottle = true;
	m_precision = false;
	m_limiterX.Reset(0);
	m_limiterY.Reset(0);
	m_limiterA.Reset(0);
}

void SwerveControllerCommand::Execute() {
	if (m_controller.GetRightBumperButtonPressed()) {
		m_fieldCentric = !m_fieldCentric;
	}
	if (m_controller.GetAButtonPressed()) {
		m_storedThrottle = !m_storedThrottle;
	}
	if (m_controller.GetStartButtonPressed()) {
		m_precision = !m_precision;
	}
	if (m_controller.GetBackButtonPressed()) {
		m_subsystem->ZeroHeading();
	}
	frc::SmartDashboard::PutNumber("Throttle", m_internalThrottle);
	frc::SmartDashboard::PutBoolean("Field Centric", m_fieldCentric);
	frc::SmartDashboard::PutBoolean("Stored Throttle", m_storedThrottle);
	frc::SmartDashboard::PutBoolean("Precision Mode", m_precision);
	if (m_controller.GetLeftBumperButton()) {
		m_subsystem->Brake();
		m_controller.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.2);
		return;
	}
	m_controller.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.2);

	double controllerX = m_limiterX.Calculate(
			m_controller.GetRawAxis(frc::XboxController::Axis::kLeftX));
	double controllerY = m_limiterX.Calculate(
			m_controller.GetRawAxis(frc::XboxController::Axis::kLeftY));
	double controllerRot = m_limiterA.Calculate(
			m_controller.GetRawAxis(frc::XboxController::Axis::kRightX));

	double deadbandMove = Drive::TeleopOperator::kDriveDeadband;
	double deadbandRot = Drive::TeleopOperator::kDriveAngleDeadband;

	double normalizedDeadbandX = controllerX < 0 ? deadbandMove : -deadbandMove;
	double normalizedDeadbandY = controllerY < 0 ? deadbandMove : -deadbandMove;
	double normalizedDeadbandA =
			controllerRot < 0 ? deadbandMove : -deadbandMove;

	double renormalizedX = 0.0;
	double renormalizedY = 0.0;
	double renormalizedA = 0.0;
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

	if (!m_storedThrottle) {
		m_internalThrottle = m_controller.GetRawAxis(
				frc::XboxController::Axis::kLeftTrigger);
	}

	units::meters_per_second_t vx = 0_mps;
	units::meters_per_second_t vy = 0_mps;
	units::radians_per_second_t va = 0_rad_per_s;

	if (!m_precision) {
		vx = renormalizedX * m_internalThrottle
				* Drive::TeleopOperator::kDriveMoveSpeedMax;
		vy = renormalizedY * m_internalThrottle
				* Drive::TeleopOperator::kDriveMoveSpeedMax;
		va = renormalizedA * m_internalThrottle
				* Drive::TeleopOperator::kDriveAngleSpeedMax;
	} else {
		vx = renormalizedX * m_internalThrottle
				* Drive::TeleopOperator::kDrivePrecision;
		vy = renormalizedY * m_internalThrottle
				* Drive::TeleopOperator::kDrivePrecision;
		va = renormalizedA * m_internalThrottle
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

void SwerveControllerCommand::End(bool interrupted) {
	m_controller.SetRumble(frc::GenericHID::kBothRumble, 0);
	m_subsystem->StopModules();
}

bool SwerveControllerCommand::IsFinished() {
	return false;
}
