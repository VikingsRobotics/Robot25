#include "subsystems/RepellerSubsystem.h"

#ifndef NO_ELEVATOR
#include "Constants.h"
#include <frc2/command/RunCommand.h>

RepellerSubsystem::RepellerSubsystem() : m_wheel {
		DeviceIdentifier::kRepellerWheelId } {

}

frc2::CommandPtr RepellerSubsystem::Forward() {
	return frc2::RunCommand { [&]() -> void {
		m_wheel.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
	}, { this } }.ToPtr();
}

frc2::CommandPtr RepellerSubsystem::Backward() {
	return frc2::RunCommand { [&]() -> void {
		m_wheel.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
				-1);
	}, { this } }.ToPtr();
}

frc2::CommandPtr RepellerSubsystem::Stall() {
	return frc2::RunCommand { [&]() -> void {
		m_wheel.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
	}, { this } }.ToPtr();
}

#endif
