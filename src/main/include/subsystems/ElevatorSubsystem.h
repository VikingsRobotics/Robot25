#pragma once

#include "Disable.h"
#ifndef NO_ELEVATOR

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>

#include <rev/SparkMax.h>

#include <units/length.h>
#include <units/angle.h>
#include <units/voltage.h>

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

	void RunHeight(units::meter_t height, units::volt_t staticVolt);
	void RunDistance(units::meter_t distance, units::volt_t staticVolt);
	void RunRotation(units::turn_t rotation, units::volt_t staticVolt);
	void RunRawRotation(units::turn_t rotation, units::volt_t staticVolt);
	void RunVoltage(units::volt_t voltage);
	void RunPercent(double speed);

	units::meter_t GetHeight();
	units::meter_t GetDistance();
	units::turn_t GetRotation();
	units::turn_t GetRawRotation();
	double GetPercent();

	units::turn_t ConvertRawToRotation(units::turn_t raw);
	units::turn_t ConvertRotationToRaw(units::turn_t rotation);
	units::meter_t ConvertRotationToDistance(units::turn_t rotation);
	units::turn_t ConvertDistanceToRotation(units::meter_t distance);
private:
	rev::spark::SparkMax m_elevatorDriver;
	rev::spark::SparkRelativeEncoder m_driverEncoder;
	rev::spark::SparkClosedLoopController &m_elevatorPID;
	rev::spark::SparkMax m_elevatorFollow;
};

#endif
