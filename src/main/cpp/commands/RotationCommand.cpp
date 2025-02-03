#include "commands/RotationCommand.h"
#include "Constants.h"

#include <units/math.h>

#include <frc/smartdashboard/SmartDashboard.h>

RotationCommand::RotationCommand(ArmSubsystem *const subsystem,
		units::turn_t desiredRotation, units::second_t switchTime,
		units::turn_t allowedError = -1_tr) : m_subsystem { subsystem }, m_desiredRotation {
		desiredRotation }, m_stopOnLimitSeconds { switchTime }, m_allowedError {
		allowedError }, m_limitSwitch { subsystem->LimiterTriggered() } {
	AddRequirements(subsystem);
}

void RotationCommand::Initialize() {
	// Nothing (for now >:])
	rev::REVLibError error =
			m_subsystem->m_directionPID.SetReference(
					(m_desiredRotation * Arm::Mechanism::kGearRatio).value(),
					rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl,
					rev::spark::kSlot0,
					(Arm::Mechanism::kStaticVoltage
							* units::math::cos(
									(m_desiredRotation
											* Arm::Mechanism::kGearRatio)
											+ m_subsystem->m_rotationalOffset)).value());
	if (error != rev::REVLibError::kOk) {
		this->Cancel();
	}
}

void RotationCommand::Execute() {
	// Nothing (for now >:])
}

void RotationCommand::End(bool interrupted) {
	m_subsystem->m_directionMotor.SetVoltage(
			Arm::Mechanism::kStaticVoltage
					* units::math::cos(
							(m_desiredRotation * Arm::Mechanism::kGearRatio)
									+ m_subsystem->m_rotationalOffset).value());
}

bool RotationCommand::IsFinished() {
	units::turn_t currentHeight = (units::turn_t {
			m_subsystem->m_directionEncoder.GetPosition() }
			* Arm::Mechanism::kGearRatio) + m_subsystem->m_rotationalOffset;
	units::turn_t error = m_desiredRotation - currentHeight;
	return m_limitSwitch.Debounce(m_stopOnLimitSeconds).Get()
			|| units::math::abs(error) < m_allowedError;
}

units::turn_t RotationCommand::GetDesiredRotation() {
	return m_desiredRotation;
}
