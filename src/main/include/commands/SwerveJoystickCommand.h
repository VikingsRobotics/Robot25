#pragma once

#include "subsystems/SwerveSubsystem.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include <frc2/command/button/CommandJoystick.h>

class SwerveJoystickCommand: public frc2::CommandHelper<frc2::Command,
		SwerveJoystickCommand> {
public:
	SwerveJoystickCommand(SwerveSubsystem *const subsystem,
			frc2::CommandJoystick &joystick, bool enableFieldCentric);

	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;
private:
	SwerveSubsystem *const m_subsystem;
	frc::Joystick &m_joystick;
	bool m_fieldCentric;
};
