// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Disable.h"

#include <frc2/command/Command.h>

#include <vector>

#include "subsystems/SwerveSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "commands/HeightCommand.h"
#include "commands/RotationCommand.h"

#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/CommandXboxController.h>

class RobotContainer {
public:
	RobotContainer();

	frc2::Command* GetAutonomousCommand();

private:
	void ConfigureBindings();
#ifndef NO_ELEVATOR_HEIGHT_COMMAND
	void ConfigureDestination();
	void BindElevatorCommand();
	std::vector<HeightCommand> destinationCommands { };
#endif
#ifndef NO_ARM_ROTATION_COMMAND
	void ConfigureRotation();
	void BindArmCommand();
	std::vector<RotationCommand> rotationCommands { };
#endif
#ifndef NO_SWERVE
	SwerveSubsystem swerveSubsystem { };
	frc2::CommandJoystick joystick;
#endif
#ifndef NO_ELEVATOR
	ElevatorSubsystem elevatorSubsystem { };
#endif
#ifndef NO_ARM
	ArmSubsystem armSubsystem { };
#endif
#if !defined(NO_ELEVATOR_ARM)  || !(defined(NO_ARM) && defined(NO_ELEVATOR)) 
	frc2::CommandXboxController xboxController;
#endif
};
