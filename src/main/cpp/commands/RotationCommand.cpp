#include "commands/RotationCommand.h"
#ifndef NO_ARM_ROTATION_COMMAND

#include "Constants.h"

#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>

RotationCommand::RotationCommand(ArmSubsystem *const subsystem,
		units::turn_t desiredRotation, units::second_t switchTime,
		units::turn_t allowedError = -1_tr) : m_subsystem { subsystem }, m_desiredRotation {
		desiredRotation }, m_stopOnLimitSeconds { switchTime }, m_allowedError {
		allowedError }, m_limitSwitch { subsystem->LimiterTriggered() } {
	AddRequirements(subsystem);
	SetName("Rotation Command");
}

void RotationCommand::Initialize() {
	// Nothing (for now >:])
	m_subsystem->RunRotation(m_desiredRotation,
			((m_desiredRotation - m_subsystem->GetRotation()) < 0_m ?
					-Arm::Mechanism::kStaticVoltage :
				(m_desiredRotation - m_subsystem->GetRotation()) > 0_m ?
						+Arm::Mechanism::kStaticVoltage : 0_V)
					+ (units::math::cos(
							(m_desiredRotation / Arm::Mechanism::kGearRatio)
									+ m_subsystem->GetRotationalOffset())
							* Arm::Mechanism::kGravity));
	frc::SmartDashboard::PutNumber("Desired Rotation (Deg)", units::degree_t {
			m_desiredRotation }.value());
}

void RotationCommand::Execute() {
	// Nothing (for now >:])
	units::degree_t error = m_desiredRotation - m_subsystem->GetRotation();
	frc::SmartDashboard::PutNumber("Error Rotation (Deg)", error.value());
	units::inch_t errorDistance = ((m_desiredRotation / 1_tr)
			* Arm::Mechanism::kArmLength) - m_subsystem->GetArcDistance();
	frc::SmartDashboard::PutNumber("Error Arc Distance (In)",
			errorDistance.value());
}

void RotationCommand::End(bool interrupted) {
	m_subsystem->RunVoltage(
			units::math::cos(
					(m_desiredRotation / Arm::Mechanism::kGearRatio)
							+ m_subsystem->GetRotationalOffset())
					* Arm::Mechanism::kGravity);
}

bool RotationCommand::IsFinished() {
	units::turn_t error = m_desiredRotation - m_subsystem->GetRotation();
	return m_limitSwitch.Debounce(m_stopOnLimitSeconds).Get()
			|| units::math::abs(error) < m_allowedError;
}

units::turn_t RotationCommand::GetDesiredRotation() {
	return m_desiredRotation;
}

units::second_t RotationCommand::GetLimitingTime() {
	return m_stopOnLimitSeconds;
}

units::turn_t RotationCommand::GetTolerance() {
	return m_allowedError;
}
#endif
