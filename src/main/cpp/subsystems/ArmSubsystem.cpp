#include "subsystems/ArmSubsystem.h"
#ifndef NO_ARM

#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>

ArmSubsystem::ArmSubsystem() : m_directionMotor {
		DeviceIdentifier::kDirectionMotorId,
		Arm::DeviceProperties::kSparkMotorType }, m_directionEncoder {
		m_directionMotor.GetEncoder() }, m_directionPID {
		m_directionMotor.GetClosedLoopController() }, m_rotationalOffset {
		Arm::Mechanism::kRotationalOffset } {
	m_directionMotor.Configure(Arm::DeviceProperties::GetSparkMaxConfig(),
			rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
			rev::spark::SparkBase::PersistMode::kPersistParameters);

	frc::SmartDashboard::PutNumber("Offset Rot", m_rotationalOffset.value());

	SetName("Arm Subsystem");
	frc::SmartDashboard::PutData(this);
}

void ArmSubsystem::Periodic() {
	frc::SmartDashboard::PutNumber("Current Rot",
			(units::turn_t { m_directionEncoder.GetPosition() }
					* Arm::Mechanism::kGearRatio).value());
	frc::SmartDashboard::PutNumber("Full Rot",
			((units::turn_t { m_directionEncoder.GetPosition() }
					* Arm::Mechanism::kGearRatio) + m_rotationalOffset).value());
}

frc2::Trigger ArmSubsystem::LimiterTriggered() {
	return frc2::Trigger(
			[this]() -> bool {
				return m_directionMotor.GetForwardLimitSwitch().Get()
						|| m_directionMotor.GetReverseLimitSwitch().Get();
			});
}

void ArmSubsystem::RunRotation(units::turn_t rotation,
		units::volt_t staticVolt) {
	RunRawRotation(ConvertRotationToRaw(rotation), staticVolt);
}

void ArmSubsystem::RunRawRotation(units::turn_t rotation,
		units::volt_t staticVolt) {
	m_directionPID.SetReference(rotation.value(),
			rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl,
			rev::spark::kSlot0,
			(staticVolt * units::math::cos((rotation) + m_rotationalOffset)).value());
}

void ArmSubsystem::RunVoltage(units::volt_t voltage) {
	m_directionMotor.SetVoltage(voltage);
}

void ArmSubsystem::RunPercent(double speed) {
	m_directionMotor.Set(speed);
}

units::meter_t ArmSubsystem::GetArcDistance(units::turn_t from) {
	return ((GetRotation() - from) / 1_tr) * Arm::Mechanism::kArmLength;
}

units::turn_t ArmSubsystem::GetDeltaRotation() {
	return ConvertRawToRotation(GetDeltaRawRotation());
}

units::turn_t ArmSubsystem::GetRotation() {
	return GetDeltaRotation() + m_rotationalOffset;
}

units::turn_t ArmSubsystem::GetDeltaRawRotation() {
	return units::turn_t { m_directionEncoder.GetPosition() };
}

units::turn_t ArmSubsystem::GetRawRotation() {
	return GetDeltaRawRotation() + ConvertRotationToRaw(m_rotationalOffset);
}

double ArmSubsystem::GetPercent() {
	return m_directionMotor.Get();
}

units::turn_t ArmSubsystem::GetRotationalOffset() {
	return m_rotationalOffset;
}

units::turn_t ArmSubsystem::ConvertRawToRotation(units::turn_t raw) {
	return raw / Arm::Mechanism::kGearRatio;
}

units::turn_t ArmSubsystem::ConvertRotationToRaw(units::turn_t rotation) {
	return rotation * Arm::Mechanism::kGearRatio;
}

#endif
