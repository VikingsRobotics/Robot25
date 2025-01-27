#pragma once

#include "subsystems/ElevatorSubsystem.h"

#include <units/time.h>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

class HeightCommand : public frc2::CommandHelper<frc2::Command, HeightCommand>
{
public:
	// Use this constructor for stopping after reaching destination (within error)
	HeightCommand(ElevatorSubsystem *const subsystem, units::meter_t height, units::second_t switchTime, units::meter_t allowedError);
	// Use this constructor for running continuely
	HeightCommand(ElevatorSubsystem *const subsystem, units::meter_t height, units::second_t switchTime);
	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;

    units::meter_t GetDesiredHeight();
private:
	ElevatorSubsystem *const m_subsystem;
    const units::meter_t m_desiredHeight = 0_m;
	const units::meter_t m_allowedError = 0.1_m;
    const units::second_t m_stopOnLimitSeconds = 0_s;
    frc2::Trigger m_limitSwitch;
};