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
		Drive::ControllerPorts::kDriverControllerPort }, xboxController {
		Elevator::ControllerPorts::kDriverControllerPort } {
	swerveSubsystem.SetDefaultCommand(SwerveJoystickCommand { &swerveSubsystem,
			joystick });

	ConfigureBindings();
}

#define CommandBindIndex(controller,button,commandIndex) controller.##button##().Debounce(Elevator::TeleopOperator::kDebounce).OnTrue(HeightCommand{destinationCommands.at(commandIndex)}.ToPtr())

void RobotContainer::ConfigureBindings() {
	ConfigureDestination();
	ConfigureRotation();

	CommandBindIndex(xboxController, A, Elevator::Destination::kFirstGoalIndex);
	CommandBindIndex(xboxController, B,
			Elevator::Destination::kSecondGoalIndex);
	CommandBindIndex(xboxController, X, Elevator::Destination::kThirdGoalIndex);
	CommandBindIndex(xboxController, Y, Elevator::Destination::kForthGoalIndex);
	CommandBindIndex(xboxController, RightBumper,
			Elevator::Destination::kCollectionHeightIndex);

	CommandBindIndex(xboxController, Back,
			Elevator::Destination::kMaxHeightIndex);
	CommandBindIndex(xboxController, Start,
			Elevator::Destination::kMinHeightIndex);

	frc::EventLoop *loop =
			frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop();
	xboxController.GetHID().LeftTrigger(Arm::TeleopOperator::kArmDeadband, loop).Debounce(
			Arm::TeleopOperator::kDebounce).CastTo<frc2::Trigger>().OnTrue(
			RotationCommand { rotationCommands.at(
					Arm::Destination::kMaxTurnIndex) }.ToPtr());
	xboxController.GetHID().RightTrigger(Arm::TeleopOperator::kArmDeadband,
			loop).Debounce(Arm::TeleopOperator::kDebounce).CastTo<frc2::Trigger>().OnTrue(
			RotationCommand { rotationCommands.at(
					Arm::Destination::kMinTurnIndex) }.ToPtr());

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
			Arm::Destination::kMaxTurn, Arm::Destination::kBottomTurn,
			Arm::Destination::kMiddleTurn, Arm::Destination::kTopTurn };

	for (units::turn_t &rotation : rotations) {
		rotationCommands.emplace_back(&armSubsystem, rotation,
				Arm::Destination::kAllowableSwitchTime,
				Arm::Destination::kAllowableError);
	}
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return nullptr;
}
