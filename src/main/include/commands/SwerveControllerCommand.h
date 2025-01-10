#pragma once
#include "subsystems/SwerveSubsystem.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include <units/time.h>

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

	bool IsFieldCentric();

	void SetFieldCentric(bool enableFieldCentric);

	double GetThrottle();

	void SetThrottle(double desiredThrottle);
private:
	SwerveSubsystem *const m_subsystem;
	frc::XboxController &m_controller;
	units::microsecond_t m_throttleTimestamp;
	double m_internalThrottle;
	bool m_fieldCentric;
};
