#include "subsystems/RollerSubsystem.h"

#ifndef NO_ROLLER

#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>

RollerSubsystem::RollerSubsystem() : m_rollerWheel {
		DeviceIdentifier::kRollerWheelId }, m_solenoid {
		DeviceIdentifier::kPneumaticHubId,
		Roller::DeviceProperties::kModuleType,
		Roller::SolenoidId::kForwardChannelId,
		Roller::SolenoidId::kReverseChannelId } {
	SetRollerWheel(0.0);
	PutRollerUp();

	SetName("Roller Subsystem");
	frc::SmartDashboard::PutData(this);
}

void RollerSubsystem::Periodic() {
	frc::SmartDashboard::PutString("Roller Motor",
			GetRollerWheelSpeed() == 0 ? "Stalling" :
			GetRollerWheelSpeed() < 0 ? "Sucking" : "Throwing");
	frc::SmartDashboard::PutString("Roller Pos",
			IsRollerDown() ? "Down" : IsRollerUp() ? "Up" : "Unmoved");
	frc::SmartDashboard::PutNumber("Roller Speed", +GetRollerWheelSpeed());
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
