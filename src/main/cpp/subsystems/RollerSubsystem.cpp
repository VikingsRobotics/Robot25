#include "subsystems/RollerSubsystem.h"

#ifndef NO_ROLLER

#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

RollerSubsystem::RollerSubsystem() : m_rollerWheel {
		DeviceIdentifier::kRollerWheelId }, m_solenoid {
		DeviceIdentifier::kPneumaticHubId,
		Roller::DeviceProperties::kModuleType,
		Roller::SolenoidId::kForwardChannelId,
		Roller::SolenoidId::kReverseChannelId } {
	SetRollerWheel(0.0);
	//PutRollerDown();

	frc::ShuffleboardTab &smart = frc::Shuffleboard::GetTab("SmartDashboard");
	frc::ShuffleboardLayout &layout = smart.GetLayout("Roller",
			frc::BuiltInLayouts::kList);

	layout.AddNumber("Wheel Spd", [&]() -> double {
		return m_rollerWheel.GetMotorOutputPercent();
	});
	layout.AddBoolean("Solenoid Down", [&]() -> double {
		return m_solenoid.Get() == frc::DoubleSolenoid::Value::kForward;
	});

	SetName("Roller Subsystem");
	frc::SmartDashboard::PutData(this);
}

void RollerSubsystem::Periodic() {

}

void RollerSubsystem::PutRollerDown() {
	m_solenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

void RollerSubsystem::PutRollerUp() {
	m_solenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

bool RollerSubsystem::IsRollerDown() {
	return m_solenoid.Get() == frc::DoubleSolenoid::Value::kForward;
}

bool RollerSubsystem::IsRollerUp() {
	return m_solenoid.Get() == frc::DoubleSolenoid::Value::kReverse;
}

void RollerSubsystem::SetRollerWheel(double speed) {
	m_rollerWheel.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
			speed);
}

double RollerSubsystem::GetRollerWheelSpeed() {
	return m_rollerWheel.GetMotorOutputPercent();
}

#endif
