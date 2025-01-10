#include "commands/SwerveControllerCommand.h"
#include "Constants.h"

#include <frc/kinematics/ChassisSpeeds.h>

#include <wpi/timestamp.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/RunCommand.h>

#include <wpi/array.h>

SwerveControllerCommand::SwerveControllerCommand(
		SwerveSubsystem *const subsystem,
		frc2::CommandXboxController &controller, bool enableFieldCentric) : m_subsystem {
		subsystem }, m_controller { controller.GetHID() }, m_throttleTimestamp {
		double(WPI_Now()) }, m_internalThrottle {
		Drive::TeleopOperator::kDefaultThrottleXbox }, m_fieldCentric {
		enableFieldCentric } {
	AddRequirements (m_subsystem);
	SetName("Swerve Xbox Controller Command");

	// Pressing the up on the little pad increases throttle
	controller.POVUp().WhileTrue(frc2::RunCommand([this]() {
		units::microsecond_t now { double(WPI_Now()) };
		units::second_t elapsedTime = now - m_throttleTimestamp;
		if (elapsedTime > 1_s)
			elapsedTime = 0_s;
		m_internalThrottle = std::clamp(m_internalThrottle + (Drive::TeleopOperator::kThrottleRateChange * elapsedTime).value(),-1.0,1.0);
		m_throttleTimestamp = now;
	}
	).ToPtr());
	// Pressing the down on the little pad descreases throttle
	controller.POVDown().WhileTrue(frc2::RunCommand([this]() {
		units::microsecond_t now { double(WPI_Now()) };
		units::second_t elapsedTime = now - m_throttleTimestamp;
		if (elapsedTime > 1_s)
			elapsedTime = 0_s;
		m_internalThrottle = std::clamp(m_internalThrottle - (Drive::TeleopOperator::kThrottleRateChange * elapsedTime).value(),-1.0,1.0);
		m_throttleTimestamp = now;
	}
	).ToPtr());
}

void SwerveControllerCommand::Initialize() {
	// Nothing (for now >:])
}

void SwerveControllerCommand::Execute() {
	frc::SmartDashboard::PutNumber("Throttle", m_internalThrottle);
	frc::SmartDashboard::PutBoolean("Field Centric", m_fieldCentric);
	if (m_controller.GetXButton()) {
		m_subsystem->Brake();
		return;
	}

	double renormalizedX = 0.0;
	double renormalizedY = 0.0;
	double renormalizedA = 0.0;
	// X^2 + Y^2 > D^2 
	// Easier process than sqrt(X^2 + Y^2) > Deadband
	if (m_controller.GetRawAxis(frc::XboxController::Axis::kLeftX)
			* m_controller.GetRawAxis(frc::XboxController::Axis::kLeftX)
			+ m_controller.GetRawAxis(frc::XboxController::Axis::kLeftY)
					* m_controller.GetRawAxis(frc::XboxController::Axis::kLeftY)
			> Drive::TeleopOperator::kDriveDeadband
					* Drive::TeleopOperator::kDriveDeadband) {
		renormalizedX = m_controller.GetRawAxis(
				frc::XboxController::Axis::kLeftX)
				/ (1 - Drive::TeleopOperator::kDriveDeadband);
		renormalizedY = m_controller.GetRawAxis(
				frc::XboxController::Axis::kLeftY)
				/ (1 - Drive::TeleopOperator::kDriveDeadband);
	}
	if (std::abs(m_controller.GetRawAxis(frc::XboxController::Axis::kRightX))
			> Drive::TeleopOperator::kDriveAngleDeadband) {
		renormalizedA = m_controller.GetRawAxis(
				frc::XboxController::Axis::kRightX)
				/ (1 - Drive::TeleopOperator::kDriveAngleDeadband);
	}

	units::meters_per_second_t vx =
			m_internalThrottle
					< Drive::TeleopOperator::kPrecisionThrottleThreshold ?
					Drive::TeleopOperator::kDrivePrecision * renormalizedX :
					Drive::TeleopOperator::kDriveMoveSpeedMax * renormalizedX;
	units::meters_per_second_t vy =
			m_internalThrottle
					< Drive::TeleopOperator::kPrecisionThrottleThreshold ?
					Drive::TeleopOperator::kDrivePrecision * renormalizedY :
					Drive::TeleopOperator::kDriveMoveSpeedMax * renormalizedY;
	units::radians_per_second_t va =
			m_internalThrottle
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

void SwerveControllerCommand::End(bool interrupted) {
	m_subsystem->StopModules();
}

bool SwerveControllerCommand::IsFinished() {
	return false;
}

bool SwerveControllerCommand::IsFieldCentric() {
	return m_fieldCentric;
}

void SwerveControllerCommand::SetFieldCentric(bool enableFieldCentric) {
	m_fieldCentric = enableFieldCentric;
}

double SwerveControllerCommand::GetThrottle() {
	return m_internalThrottle;
}

void SwerveControllerCommand::SetThrottle(double desired) {
	if (desired > 1)
		desired = 1;
	if (desired < -1)
		desired = -1;
	m_internalThrottle = desired;
}
