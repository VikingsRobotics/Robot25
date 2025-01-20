#pragma once
#include "subsystems/SwerveSubsystem.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include <frc2/command/button/CommandXboxController.h>

class SwerveControllerCommand: public frc2::CommandHelper<frc2::Command,
		SwerveControllerCommand> {
public:
	SwerveControllerCommand(SwerveSubsystem *const subsystem,
			frc2::CommandXboxController &controller, bool enableFieldCentric);

	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;
private:
	SwerveSubsystem *const m_subsystem;
	frc::XboxController &m_controller;
	double m_internalThrottle;
	bool m_fieldCentric;
	bool m_precision = false;
	bool m_storedThrottle = true;
};
