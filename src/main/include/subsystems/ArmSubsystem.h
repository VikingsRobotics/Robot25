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

	friend class RotationCommand;
private:
	rev::spark::SparkMax m_directionMotor;
	rev::spark::SparkRelativeEncoder m_directionEncoder;
	rev::spark::SparkClosedLoopController &m_directionPID;
	units::turn_t m_rotationalOffset;
};

#endif
