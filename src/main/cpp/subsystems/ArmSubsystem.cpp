#include "subsystems/ArmSubsystem.h"
#ifndef NO_ARM

#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

ArmSubsystem::ArmSubsystem() : m_directionMotor {
		DeviceIdentifier::kDirectionMotorId,
		Arm::DeviceProperties::kSparkMotorType }, m_directionEncoder {
		m_directionMotor.GetEncoder() }, m_directionPID {
		m_directionMotor.GetClosedLoopController() }, m_rotationalOffset {
		Arm::Mechanism::kRotationalOffset } {
	m_directionMotor.Configure(Arm::DeviceProperties::GetSparkMaxConfig(),
			rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
			rev::spark::SparkBase::PersistMode::kPersistParameters);

	frc::ShuffleboardTab &smart = frc::Shuffleboard::GetTab("SmartDashboard");
	frc::ShuffleboardLayout &layout = smart.GetLayout("Arm",
			frc::BuiltInLayouts::kList);
	layout.AddNumber("Offset Pos (Rot)", [&]() -> double {
		return m_rotationalOffset.value();
	});
	layout.AddNumber("Full Pos (Rot)",
			[&]() -> double {
				return ((units::turn_t { m_directionEncoder.GetPosition() }
						* Arm::Mechanism::kGearRatio) + m_rotationalOffset).value();
			});
	layout.AddNumber("Current Pos (Rot)",
			[&]() -> double {
				return (units::turn_t { m_directionEncoder.GetPosition() }
						* Arm::Mechanism::kGearRatio).value();
			});

	SetName("Arm Subsystem");
	frc::SmartDashboard::PutData(this);
}

void ArmSubsystem::Periodic() {
}

frc2::Trigger ArmSubsystem::LimiterTriggered() {
	return frc2::Trigger(
			[this]() -> bool {
				return m_directionMotor.GetForwardLimitSwitch().Get()
						|| m_directionMotor.GetReverseLimitSwitch().Get();
			});
}

#endif
