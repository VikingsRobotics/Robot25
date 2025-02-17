// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>

#include "subsystems/SwerveSubsystem.h"

#include <frc2/command/button/CommandJoystick.h>
#include <frc/Alert.h>

class RobotContainer {
public:
	RobotContainer();

	frc2::Command* GetAutonomousCommand();

public:
	std::vector<frc2::CommandPtr> SwerveSysId { };

private:
	void ConfigureBindings();
	void ConfigureSwerveSysId();
	SwerveSubsystem swerveSubsystem { };
	frc2::CommandJoystick joystick;
	frc::Alert swerveSysIdTestMoveOnly {
			"Swerve System Identification Command can only be run in Test Mode",
			frc::Alert::AlertType::kWarning };
};
