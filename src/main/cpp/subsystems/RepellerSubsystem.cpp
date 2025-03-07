#include "subsystems/RepellerSubsystem.h"

#ifndef NO_ELEVATOR
#include "Constants.h"
#include <frc2/command/RunCommand.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

RepellerSubsystem::RepellerSubsystem() : m_wheel {
		DeviceIdentifier::kRepellerWheelId } {

	frc::ShuffleboardTab &smart = frc::Shuffleboard::GetTab("SmartDashboard");
	frc::ShuffleboardLayout &layout = smart.GetLayout("Arm",
			frc::BuiltInLayouts::kList);

	layout.AddNumber("Wheel Spd", [&]() -> double {
		return m_speed;
	});

	SetName("Elev Wheel Subsystem");
	frc::SmartDashboard::PutData(this);
}

frc2::CommandPtr RepellerSubsystem::Forward() {
	return frc2::RunCommand { [&]() -> void {
		m_wheel.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
		m_speed = 1.0;
	}, { this } }.ToPtr();
}

frc2::CommandPtr RepellerSubsystem::Backward() {
	return frc2::RunCommand { [&]() -> void {
		m_wheel.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
				-1);
		m_speed = -1.0;
	}, { this } }.ToPtr();
}

frc2::CommandPtr RepellerSubsystem::Stall() {
	return frc2::RunCommand { [&]() -> void {
		m_wheel.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		m_speed = 0.0;
	}, { this } }.ToPtr();
}

#endif
