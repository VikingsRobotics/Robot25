#include "commands/HeightCommand.h"
#ifndef NO_ELEVATOR_HEIGHT_COMMAND

#include "Constants.h"

#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>

HeightCommand::HeightCommand(ElevatorSubsystem *const subsystem,
		units::meter_t height, units::second_t switchTime,
		units::meter_t allowedError = -1_m) : m_subsystem { subsystem }, m_desiredHeight {
		height }, m_stopOnLimitSeconds { switchTime }, m_allowedError {
		allowedError }, m_limitSwitch { subsystem->LimiterTriggered() } {
	AddRequirements(subsystem);
	SetName("Height Command");
}

void HeightCommand::Initialize() {
	// Nothing (for now >:])
	m_subsystem->RunHeight(m_desiredHeight,
			((m_desiredHeight - m_subsystem->GetHeight()) < 0_m ?
					-Elevator::Mechanism::kStaticVoltage :
				(m_desiredHeight - m_subsystem->GetHeight()) > 0_m ?
						+Elevator::Mechanism::kStaticVoltage : 0_V)
					+ Elevator::Mechanism::kGravity);
	frc::SmartDashboard::PutNumber("Desired Height (Inch)", units::inch_t {
			m_desiredHeight }.value());
}

void HeightCommand::Execute() {
	// Nothing (for now >:])
	units::inch_t error = m_desiredHeight - m_subsystem->GetHeight();
	frc::SmartDashboard::PutNumber("Error Height (Inch)", error.value());
}

void HeightCommand::End(bool interrupted) {
	m_subsystem->RunVoltage(Elevator::Mechanism::kGravity);
}

bool HeightCommand::IsFinished() {
	units::inch_t error = m_desiredHeight - m_subsystem->GetHeight();
	return m_limitSwitch.Debounce(m_stopOnLimitSeconds).Get()
			|| units::math::abs(error) < m_allowedError;
}

units::meter_t HeightCommand::GetDesiredHeight() {
	return m_desiredHeight;
}

units::second_t HeightCommand::GetLimitingTime() {
	return m_stopOnLimitSeconds;
}

units::meter_t HeightCommand::GetTolerance() {
	return m_allowedError;
}

#endif
