#include "commands/SwerveControllerCommand.h"
#include "Constants.h"

#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/RunCommand.h>

#include <wpi/array.h>

SwerveControllerCommand::SwerveControllerCommand(
		SwerveSubsystem *const subsystem,
		frc2::CommandXboxController &controller, bool enableFieldCentric) : m_subsystem {
		subsystem }, m_controller { controller.GetHID() }, m_internalThrottle {
		Drive::TeleopOperator::kDefaultThrottleXbox }, m_fieldCentric {
		enableFieldCentric } {
	AddRequirements (m_subsystem);
	SetName("Swerve Xbox Controller Command");

	controller.RightBumper().OnTrue(frc2::RunCommand([this]() {
		m_fieldCentric = !m_fieldCentric;
	}).ToPtr());

	controller.A().OnTrue(frc2::RunCommand([this]() {
		m_storedThrottle = !m_storedThrottle;
	}).ToPtr());

	controller.Back().OnTrue(frc2::RunCommand([this]() {
		m_subsystem->ZeroHeading();
	}).ToPtr());

	controller.Start().OnTrue(frc2::RunCommand([this]() {
		m_precision = !m_precision;
	}).ToPtr());
}

void SwerveControllerCommand::Initialize() {
	// Nothing (for now >:])
}

void SwerveControllerCommand::Execute() {
	frc::SmartDashboard::PutNumber("Throttle", m_internalThrottle);
	frc::SmartDashboard::PutBoolean("Field Centric", m_fieldCentric);
	frc::SmartDashboard::PutBoolean("Stored Throttle", m_storedThrottle);
	frc::SmartDashboard::PutBoolean("Precision Mode", m_precision);
	if (m_controller.GetLeftBumper()) {
		m_subsystem->Brake();
		return;
	}

	double controllerX = m_controller.GetRawAxis(
			frc::XboxController::Axis::kLeftX);
	double controllerY = m_controller.GetRawAxis(
			frc::XboxController::Axis::kLeftY);
	double controllerRot = m_controller.GetRawAxis(
			frc::XboxController::Axis::kRightX);

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

void SwerveControllerCommand::End(bool interrupted) {
	m_subsystem->StopModules();
}

bool SwerveControllerCommand::IsFinished() {
	return false;
}
