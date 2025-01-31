#pragma once

#include <rev/SparkMax.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>

class ArmSubsystem : frc2::SubsystemBase {
public:
    ArmSubsystem();
	ArmSubsystem(ArmSubsystem &rhs) = delete;
	ArmSubsystem& operator=(ArmSubsystem &rhs) = delete;
	ArmSubsystem(ArmSubsystem &&rhs) = delete;
	ArmSubsystem& operator=(ArmSubsystem &&rhs) = delete;

	void Periodic() override;

    frc2::Trigger LimiterTriggered();
private:
    rev::spark::SparkMax m_directionMotor;
    rev::spark::SparkRelativeEncoder m_directionEncoder;
    rev::spark::SparkClosedLoopController m_directionPID;
};