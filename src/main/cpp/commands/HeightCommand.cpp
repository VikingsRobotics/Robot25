#include "commands/HeightCommand.h"
#ifndef NO_ELEVATOR_HEIGHT_COMMAND

#include "Constants.h"

#include <units/math.h>

HeightCommand::HeightCommand(ElevatorSubsystem *const subsystem,
		units::meter_t height, units::second_t switchTime,
		units::meter_t allowedError = -1_m) : m_subsystem { subsystem }, m_desiredHeight {
		height }, m_stopOnLimitSeconds { switchTime }, m_allowedError {
		allowedError }, m_limitSwitch { subsystem->LimiterTriggered() } {
	AddRequirements(subsystem);
}

void HeightCommand::Initialize() {
	// Nothing (for now >:])
	rev::REVLibError error =
			m_subsystem->m_elevatorPID.SetReference(
					(m_desiredHeight * Elevator::Mechanism::kDistanceToRotation).value(),
					rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl,
					rev::spark::kSlot0,
					Elevator::Mechanism::kStaticVoltage.value());
	if (error != rev::REVLibError::kOk) {
		this->Cancel();
		return;
	}
}

void HeightCommand::Execute() {
	// Nothing (for now >:])
}

void HeightCommand::End(bool interrupted) {
	m_subsystem->m_elevatorDriver.SetVoltage(
			Elevator::Mechanism::kStaticVoltage);
}

bool HeightCommand::IsFinished() {
	units::meter_t currentHeight = units::turn_t {
			m_subsystem->m_driverEncoder.GetPosition() }
			/ Elevator::Mechanism::kDistanceToRotation;
	units::meter_t error = m_desiredHeight - currentHeight;
	return m_limitSwitch.Debounce(m_stopOnLimitSeconds).Get()
			|| units::math::abs(error) < m_allowedError;
}

units::meter_t HeightCommand::GetDesiredHeight() {
	return m_desiredHeight;
}

#endif
