#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>

#include <rev/SparkMax.h>

#include <units/length.h>

class HeightCommand;

class ElevatorSubsystem: public frc2::SubsystemBase {
public:
	ElevatorSubsystem();
	ElevatorSubsystem(ElevatorSubsystem &rhs) = delete;
	ElevatorSubsystem& operator=(ElevatorSubsystem &rhs) = delete;
	ElevatorSubsystem(ElevatorSubsystem &&rhs) = delete;
	ElevatorSubsystem& operator=(ElevatorSubsystem &&rhs) = delete;

	void Periodic() override;

	frc2::Trigger LimiterTriggered();

	friend HeightCommand;
private:
	rev::spark::SparkMax m_elevatorDriver;
	rev::spark::SparkRelativeEncoder m_driverEncoder;
	rev::spark::SparkMax m_elevatorFollow;
	rev::spark::SparkClosedLoopController m_elevatorPID;
};
