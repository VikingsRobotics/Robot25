#pragma once

#pragma once

#include "subsystems/ArmSubsystem.h"
#include "Disable.h"

#ifndef NO_ARM_ROTATION_COMMAND

#include <units/angle.h>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

class RotationCommand: public frc2::CommandHelper<frc2::Command, RotationCommand> {
public:
	// Use this constructor for stopping after reaching destination (within error), don't fill last for continous
	RotationCommand(ArmSubsystem *const subsystem, units::turn_t rotation,
			units::second_t switchTime, units::turn_t allowedError);

	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;

	units::turn_t GetDesiredRotation();
private:
	ArmSubsystem *const m_subsystem;
	const units::turn_t m_desiredRotation;
	const units::second_t m_stopOnLimitSeconds;
	const units::turn_t m_allowedError;
	frc2::Trigger m_limitSwitch;
};

#endif
