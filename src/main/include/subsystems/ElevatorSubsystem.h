#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>

#include <rev/SparkMax.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
public:
    ElevatorSubsystem();
    ElevatorSubsystem(ElevatorSubsystem &rhs) = delete;
	ElevatorSubsystem& operator=(ElevatorSubsystem &rhs) = delete;
	ElevatorSubsystem(ElevatorSubsystem &&rhs) = delete;
	ElevatorSubsystem& operator=(ElevatorSubsystem &&rhs) = delete;

    frc2::Trigger LimiterTriggered();
    
private:
    rev::spark::SparkMax m_elevatorDriver;
    rev::spark::SparkMax m_elevatorFollow;
};