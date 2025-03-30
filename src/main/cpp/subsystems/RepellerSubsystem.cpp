#include "subsystems/RepellerSubsystem.h"

#ifndef NO_ELEVATOR
#include "Constants.h"
#include <frc2/command/RunCommand.h>

#include <frc/smartdashboard/SmartDashboard.h>

RepellerSubsystem::RepellerSubsystem() : m_wheel {
		DeviceIdentifier::kRepellerWheelId } {
	SetName("Elev Wheel Subsystem");
	frc::SmartDashboard::PutData(this);
}

void RepellerSubsystem::Periodic() {
	frc::SmartDashboard::PutString("Repeller State",
			GetRepellerWheelSpeed() == 0 ? "Stalling" :
			GetRepellerWheelSpeed() > 0 ? "Forward" : "Backward");
	frc::SmartDashboard::PutNumber("Repeller Speed", +GetRepellerWheelSpeed());
}

void RepellerSubsystem::SetRepellerWheel(double speed) {
	m_wheel.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

double RepellerSubsystem::GetRepellerWheelSpeed() {
	return m_wheel.GetMotorOutputPercent();
}

#endif
