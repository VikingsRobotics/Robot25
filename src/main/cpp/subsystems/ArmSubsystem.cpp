#include "subsystems/ArmSubsystem.h"

#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>

ArmSubsystem::ArmSubsystem() : m_directionMotor {
		DeviceIdentifier::kDirectionMotorId,
		Arm::DeviceProperties::kSparkMotorType }, m_directionEncoder {
		m_directionMotor.GetEncoder() }, m_directionPID {
		m_directionMotor.GetClosedLoopController() } {
	m_directionMotor.Configure(Arm::DeviceProperties::GetSparkMaxConfig(),
			rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
			rev::spark::SparkBase::PersistMode::kPersistParameters);

	SetName("Arm Subsystem");
	frc::SmartDashboard::PutData(this);
}

void ArmSubsystem::Periodic() {
	frc::SmartDashboard::PutNumber("Arm Position (Rotations)",
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
