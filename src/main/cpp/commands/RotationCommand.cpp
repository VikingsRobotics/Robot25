#include "commands/RotationCommand.h"
#ifndef NO_ARM_ROTATION_COMMAND

#include "Constants.h"

#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>

RotationCommand::RotationCommand(ArmSubsystem *const subsystem,
		units::turn_t desiredRotation, units::second_t switchTime,
		units::turn_t allowedError) : m_subsystem { subsystem }, m_desiredRotation {
		desiredRotation }, m_stopOnLimitSeconds { switchTime }, m_allowedError {
		allowedError }, m_limitSwitch { subsystem->LimiterTriggered() } {
	AddRequirements(subsystem);
	SetName("Rotation Command");
}

static units::scalar_t sign(units::turn_t input) {
	return input > 0_tr ? +1 : input < 0_tr ? -1 : 0;
}

void RotationCommand::Initialize() {
	// Nothing (for now >:])
	m_subsystem->RunRotation(m_desiredRotation,
			(sign(m_desiredRotation - m_subsystem->GetRotation())
					* Arm::Mechanism::kStaticVoltage)
					+ (units::math::cos(m_desiredRotation)
							* Arm::Mechanism::kGravity));
#ifdef SMART_DEBUG
	frc::SmartDashboard::PutNumber("Desired Rotation (Deg)", units::degree_t {
			m_desiredRotation }.value());
	#endif
}

void RotationCommand::Execute() {
	// Nothing (for now >:])
#ifdef SMART_DEBUG
	units::degree_t error = m_subsystem->GetRotation() - m_desiredRotation;
	frc::SmartDashboard::PutNumber("Error Rotation (Deg)", error.value());
	units::inch_t errorDistance = m_subsystem->GetArcDistance(
			m_desiredRotation);
	frc::SmartDashboard::PutNumber("Error Arc Distance (In)",
			errorDistance.value());
	#endif
}

void RotationCommand::End(bool interrupted) {
	m_subsystem->RunVoltage(
			units::math::cos(m_desiredRotation) * Arm::Mechanism::kGravity);
}

bool RotationCommand::IsFinished() {
	units::turn_t error = m_subsystem->GetRotation() - m_desiredRotation;
	return units::math::abs(error) < m_allowedError
			|| m_limitSwitch.Debounce(m_stopOnLimitSeconds).Get();
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
