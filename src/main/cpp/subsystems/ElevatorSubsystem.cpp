#include "subsystems/ElevatorSubsystem.h"
#ifndef NO_ELEVATOR

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

ElevatorSubsystem::ElevatorSubsystem() : m_elevatorDriver {
		DeviceIdentifier::kElevatorDriverId,
		Elevator::DeviceProperties::kSparkMotorType }, m_driverEncoder {
		m_elevatorDriver.GetEncoder() }, m_elevatorPID {
		m_elevatorDriver.GetClosedLoopController() }, m_elevatorFollow {
		DeviceIdentifier::kElevatorFollowId,
		Elevator::DeviceProperties::kSparkMotorType } {
	SetName("Elevator Subsystem");

	m_elevatorDriver.Configure(Elevator::DeviceProperties::GetElevatorConfig(),
			rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
			rev::spark::SparkBase::PersistMode::kPersistParameters);
	m_elevatorFollow.Configure(Elevator::DeviceProperties::GetFollowerConfig(),
			rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
			rev::spark::SparkBase::PersistMode::kPersistParameters);

	m_driverEncoder.SetPosition(0);

	frc::SmartDashboard::PutData(this);
}

void ElevatorSubsystem::Periodic() {
	frc::SmartDashboard::PutNumber("Chain Pos (Inch)",
			units::inch_t(
					units::turn_t { m_driverEncoder.GetPosition() }
							/ Elevator::Mechanism::kDistanceToRotation).value());
}

frc2::Trigger ElevatorSubsystem::LimiterTriggered() {
	return frc2::Trigger(
			[this]() -> bool {
				return m_elevatorDriver.GetForwardLimitSwitch().Get()
						|| m_elevatorDriver.GetReverseLimitSwitch().Get()
						|| m_elevatorFollow.GetForwardLimitSwitch().Get()
						|| m_elevatorFollow.GetReverseLimitSwitch().Get();
			});
}

#endif
