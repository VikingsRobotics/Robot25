#pragma once

#include "subsystems/ElevatorSubsystem.h"

#include <units/time.h>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

class HeightCommand: public frc2::CommandHelper<frc2::Command, HeightCommand> {
public:
	// Use this constructor for stopping after reaching destination (within error), don't fill last for continous
	HeightCommand(ElevatorSubsystem *const subsystem, units::meter_t height,
			units::second_t switchTime, units::meter_t allowedError);

	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;

	units::meter_t GetDesiredHeight();
private:
	ElevatorSubsystem *const m_subsystem;
	const units::meter_t m_desiredHeight;
	const units::second_t m_stopOnLimitSeconds;
	const units::meter_t m_allowedError;
	frc2::Trigger m_limitSwitch;
};
