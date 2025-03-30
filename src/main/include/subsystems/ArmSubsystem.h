#pragma once

#include "Disable.h"
#ifndef NO_ARM

#include <units/angle.h>

#include <rev/SparkMax.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>

class RotationCommand;

class ArmSubsystem: public frc2::SubsystemBase {
public:
	ArmSubsystem();
	ArmSubsystem(ArmSubsystem &rhs) = delete;
	ArmSubsystem& operator=(ArmSubsystem &rhs) = delete;
	ArmSubsystem(ArmSubsystem &&rhs) = delete;
	ArmSubsystem& operator=(ArmSubsystem &&rhs) = delete;

	void Periodic() override;

	frc2::Trigger LimiterTriggered();

	void RunRotation(units::turn_t rotation, units::volt_t staticVolt);
	void RunRawRotation(units::turn_t rotation, units::volt_t staticVolt);
	void RunVoltage(units::volt_t voltage);
	void RunPercent(double speed);

	units::meter_t GetArcDistance();
	units::turn_t GetRotation();
	units::turn_t GetDeltaRotation();
	units::turn_t GetRawRotation();
	units::turn_t GetDeltaRawRotation();
	double GetPercent();

	units::turn_t GetRotationalOffset();
private:
	rev::spark::SparkMax m_directionMotor;
	rev::spark::SparkRelativeEncoder m_directionEncoder;
	rev::spark::SparkClosedLoopController &m_directionPID;
	units::turn_t m_rotationalOffset;
};

#endif
