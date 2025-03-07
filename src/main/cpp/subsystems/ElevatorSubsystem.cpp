#include "subsystems/ElevatorSubsystem.h"
#ifndef NO_ELEVATOR

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include "Constants.h"

ElevatorSubsystem::ElevatorSubsystem() : m_elevatorDriver {
		DeviceIdentifier::kElevatorDriverId,
		Elevator::DeviceProperties::kSparkMotorType }, m_driverEncoder {
		m_elevatorDriver.GetEncoder() }, m_elevatorPID {
		m_elevatorDriver.GetClosedLoopController() }, m_elevatorFollow {
		DeviceIdentifier::kElevatorFollowId,
		Elevator::DeviceProperties::kSparkMotorType } {

	m_elevatorDriver.Configure(Elevator::DeviceProperties::GetElevatorConfig(),
			rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
			rev::spark::SparkBase::PersistMode::kPersistParameters);
	m_elevatorFollow.Configure(Elevator::DeviceProperties::GetFollowerConfig(),
			rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
			rev::spark::SparkBase::PersistMode::kPersistParameters);

	m_driverEncoder.SetPosition(0);

	frc::ShuffleboardTab &smart = frc::Shuffleboard::GetTab("SmartDashboard");
	frc::ShuffleboardLayout &layout = smart.GetLayout("Elevator",
			frc::BuiltInLayouts::kList);
	layout.AddNumber("Chain Pos (Inch)",
			[&]() -> double {
				return units::inch_t(
						units::turn_t { m_driverEncoder.GetPosition() }
								/ Elevator::Mechanism::kDistanceToRotation).value();
			});

	SetName("Elevator Subsystem");
	frc::SmartDashboard::PutData(this);
}

void ElevatorSubsystem::Periodic() {
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
