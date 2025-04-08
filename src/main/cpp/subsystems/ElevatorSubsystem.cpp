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

	m_elevatorDriver.Configure(Elevator::DeviceProperties::GetElevatorConfig(),
			rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
			rev::spark::SparkBase::PersistMode::kPersistParameters);
	m_elevatorFollow.Configure(Elevator::DeviceProperties::GetFollowerConfig(),
			rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
			rev::spark::SparkBase::PersistMode::kPersistParameters);

	m_driverEncoder.SetPosition(0);

	SetName("Elevator Subsystem");
	frc::SmartDashboard::PutData(this);
}

void ElevatorSubsystem::Periodic() {
	frc::SmartDashboard::PutNumber("Chain Height (Inch)", units::inch_t {
			GetDistance() }.value());
	frc::SmartDashboard::PutNumber("Chain Raw Pos (Rot)",
			GetRawRotation().value());
	frc::SmartDashboard::PutNumber("Chain Geared Pos (Rot)",
			GetRotation().value());
	frc::SmartDashboard::PutBoolean("Chain Limit", LimiterTriggered().Get());
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

void ElevatorSubsystem::RunDistance(units::meter_t distance,
		units::volt_t staticVolt) {
	RunRawRotation(ConvertDistanceToRaw(distance), staticVolt);
}

void ElevatorSubsystem::RunRotation(units::turn_t rotation,
		units::volt_t staticVolt) {
	RunRawRotation(ConvertRotationToRaw(rotation), staticVolt);
}

void ElevatorSubsystem::RunRawRotation(units::turn_t rotation,
		units::volt_t staticVolt) {
	m_elevatorPID.SetReference(rotation.value(),
			rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl,
			rev::spark::kSlot0, staticVolt.value(),
			rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
}

void ElevatorSubsystem::RunVoltage(units::volt_t voltage) {
	m_elevatorDriver.SetVoltage(voltage);
}

void ElevatorSubsystem::RunPercent(double speed) {
	m_elevatorDriver.Set(speed);
}

units::meter_t ElevatorSubsystem::GetHeight() {
	return GetDistance() / 2;
}

units::meter_t ElevatorSubsystem::GetDistance() {
	return ConvertRawToDistance(GetRawRotation());
}
units::turn_t ElevatorSubsystem::GetRotation() {
	return ConvertRawToRotation(GetRawRotation());
}

units::turn_t ElevatorSubsystem::GetRawRotation() {
	return units::turn_t { m_driverEncoder.GetPosition() };
}

double ElevatorSubsystem::GetPercent() {
	return m_elevatorDriver.Get();
}

units::turn_t ElevatorSubsystem::ConvertRawToRotation(units::turn_t raw) {
	return raw * Elevator::Mechanism::kGearRatio;
}

units::turn_t ElevatorSubsystem::ConvertRotationToRaw(units::turn_t rotation) {
	return rotation / Elevator::Mechanism::kGearRatio;
}

units::meter_t ElevatorSubsystem::ConvertRawToDistance(units::turn_t raw) {
	return raw * Elevator::Mechanism::kRotationToDistance;
}

units::turn_t ElevatorSubsystem::ConvertDistanceToRaw(units::meter_t distance) {
	return distance / Elevator::Mechanism::kRotationToDistance;
}

#endif
