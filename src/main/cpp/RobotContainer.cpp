// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "Constants.h"

#include "commands/SwerveControllerCommand.h"
#include "commands/SwerveJoystickCommand.h"

#include <frc2/command/RunCommand.h>

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() : joystick {
		Drive::ControllerPorts::kDriverControllerPort } {
	swerveSubsystem.SetDefaultCommand(SwerveJoystickCommand { &swerveSubsystem,
			joystick });

	ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
	ConfigureDestination();
}

void RobotContainer::ConfigureDestination() {
	std::vector < units::meter_t > destinations =
			{ Elevator::Destination::kMaxHeight,
					Elevator::Destination::kCollectionHeight,
					Elevator::Destination::kFourthGoal,
					Elevator::Destination::kThirdGoal,
					Elevator::Destination::kSecondGoal,
					Elevator::Destination::kFirstGoal,
					Elevator::Destination::kMinHeight };

	for (units::meter_t &height : destinations) {
		destinationCommands.emplace_back(&elevatorSubsystem, height,
				Elevator::Destination::kAllowableSwitchTime,
				Elevator::Destination::kAllowableError);
	}
}

void RobotContainer::ConfigureRotation() {
	std::vector < units::turn_t > rotations = { Arm::Destination::kMinTurn,
			Arm::Destination::kMaxTurn };

	for (units::turn_t &rotation : rotations) {
		rotationCommands.emplace_back(&armSubsystem, rotation,
				Arm::Destination::kAllowableSwitchTime,
				Arm::Destination::kAllowableError);
	}
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return nullptr;
}
