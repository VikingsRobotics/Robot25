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

	SetName("Arm Subsystem");
	frc::SmartDashboard::PutNumber("Offset Pos (Rot)",
			m_rotationalOffset.value());
	frc::SmartDashboard::PutData(this);
}

void ArmSubsystem::Periodic() {
	frc::SmartDashboard::PutNumber("Full Pos (Rot)",
			((units::turn_t { m_directionEncoder.GetPosition() }
					* Arm::Mechanism::kGearRatio) + m_rotationalOffset).value());
	frc::SmartDashboard::PutNumber("Current Pos (Rot)",
			(units::turn_t { m_directionEncoder.GetPosition() }
					* Arm::Mechanism::kGearRatio).value());
}

frc2::Trigger ArmSubsystem::LimiterTriggered() {
	return frc2::Trigger(
			[this]() -> bool {
				return m_directionMotor.GetForwardLimitSwitch().Get()
						|| m_directionMotor.GetReverseLimitSwitch().Get();
			});
}

#endif
