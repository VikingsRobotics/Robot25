#include "commands/HeightCommand.h"
#ifndef NO_ELEVATOR_HEIGHT_COMMAND

#include "Constants.h"

#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>

HeightCommand::HeightCommand(ElevatorSubsystem *const subsystem,
		units::meter_t height, units::second_t switchTime,
		units::meter_t allowedError) : m_subsystem { subsystem }, m_desiredHeight {
		height }, m_stopOnLimitSeconds { switchTime }, m_allowedError {
		allowedError }, m_limitSwitch { subsystem->LimiterTriggered() } {
	AddRequirements(subsystem);
	SetName("Height Command");
}

static units::scalar_t sign(units::meter_t input) {
	return input > 0_m ? +1 : input < 0_m ? -1 : 0;
}

void HeightCommand::Initialize() {
	// Nothing (for now >:])
	m_subsystem->RunDistance(m_desiredHeight,
			(sign(m_desiredHeight - m_subsystem->GetDistance())
					* Elevator::Mechanism::kStaticVoltage)
					+ Elevator::Mechanism::kGravity);
#ifdef SMART_DEBUG
	frc::SmartDashboard::PutNumber("Desired Height (Inch)", units::inch_t {
			m_desiredHeight }.value());
	#endif
}

void HeightCommand::Execute() {
	// Nothing (for now >:])
#ifdef SMART_DEBUG
	units::inch_t error = m_subsystem->GetDistance() - m_desiredHeight;
	frc::SmartDashboard::PutNumber("Error Height (Inch)", error.value());
	units::turn_t errorRotation = m_subsystem->ConvertRawToRotation(
			m_subsystem->ConvertDistanceToRaw(m_desiredHeight))
			- m_subsystem->GetRotation();
	frc::SmartDashboard::PutNumber("Error Rotation (Rot)",
			errorRotation.value());
	#endif
}

void HeightCommand::End(bool interrupted) {
	m_subsystem->RunVoltage(Elevator::Mechanism::kGravity);
}

bool HeightCommand::IsFinished() {
	units::inch_t error = m_subsystem->GetDistance() - m_desiredHeight;
	return units::math::abs(error) < m_allowedError
			|| m_limitSwitch.Debounce(m_stopOnLimitSeconds).Get();
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
