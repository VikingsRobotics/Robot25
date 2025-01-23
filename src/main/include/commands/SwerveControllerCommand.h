#pragma once
#include "subsystems/SwerveSubsystem.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include <frc2/command/button/CommandXboxController.h>

#include <frc/filter/SlewRateLimiter.h>

#include <wpi/array.h>

class SwerveControllerCommand: public frc2::CommandHelper<frc2::Command,
		SwerveControllerCommand> {
public:
	SwerveControllerCommand(SwerveSubsystem *const subsystem,
			frc2::CommandXboxController &controller);

	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;
private:
	SwerveSubsystem *const m_subsystem;
	frc::XboxController &m_controller;
	wpi::array<frc::Rotation2d, 4> m_lastState { { frc::Rotation2d { },
			frc::Rotation2d { }, frc::Rotation2d { }, frc::Rotation2d { } } };
	double m_internalThrottle = 0.0;
	bool m_fieldCentric = true;
	bool m_precision = false;
	bool m_storedThrottle = true;
	frc::SlewRateLimiter<units::scalar> m_limiterX;
	frc::SlewRateLimiter<units::scalar> m_limiterY;
	frc::SlewRateLimiter<units::scalar> m_limiterA;
};
