// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Disable.h"

#include <frc2/command/Command.h>

#include <vector>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "subsystems/SwerveSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/RepellerSubsystem.h"
#include "commands/HeightCommand.h"
#include "commands/RotationCommand.h"

#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/Alert.h>

class RobotContainer {
public:
	RobotContainer();

	frc2::Command* GetAutonomousCommand();

private:
	void ConfigureBindings();
#ifndef NO_SWERVE_SYSID_COMMAND
	void ConfigureSwerveSysId();
	frc::Alert swerveSysIdTestMoveOnly {
			"Swerve System Identification Command can only be run in Test Mode",
			frc::Alert::AlertType::kWarning };
	std::vector<frc2::CommandPtr> SwerveSysId { };
#endif
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
	frc::SendableChooser<frc2::Command*> autoChooser { };
#endif
#ifndef NO_ELEVATOR
	ElevatorSubsystem elevatorSubsystem { };
	RepellerSubsystem repellerSubsystem { };
#endif
#ifndef NO_ARM
	ArmSubsystem armSubsystem { };
#endif
	/* [[maybe_unused]] */
	frc2::CommandJoystick joystick;
	/* [[maybe_unused]] */
	frc2::CommandXboxController xboxController;
};
