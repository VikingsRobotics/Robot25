// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>

#include <vector>

#include "subsystems/SwerveSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "commands/HeightCommand.h"
#include "commands/RotationCommand.h"

#include <frc2/command/button/CommandJoystick.h>

class RobotContainer {
public:
	RobotContainer();

	frc2::Command* GetAutonomousCommand();

private:
	void ConfigureBindings();
	void ConfigureDestination();
	void ConfigureRotation();
	SwerveSubsystem swerveSubsystem { };
	ElevatorSubsystem elevatorSubsystem { };
	ArmSubsystem armSubsystem { };
	frc2::CommandJoystick joystick;
	std::vector<HeightCommand> destinationCommands;
	std::vector<RotationCommand> rotationCommands;
};
