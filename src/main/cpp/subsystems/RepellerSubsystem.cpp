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
		return m_wheel.GetMotorOutputPercent();
	});

	SetName("Elev Wheel Subsystem");
	frc::SmartDashboard::PutData(this);
}

void RepellerSubsystem::Periodic() {

}

void RepellerSubsystem::SetRepellerWheel(double speed) {
	m_wheel.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

double RepellerSubsystem::GetRepellerWheelSpeed() {
	return m_wheel.GetMotorOutputPercent();
}

#endif
