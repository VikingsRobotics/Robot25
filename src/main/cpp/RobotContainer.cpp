// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "Constants.h"

#include "commands/SwerveControllerCommand.h"
#include "commands/SwerveJoystickCommand.h"

#include <frc2/command/RunCommand.h>

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() :
#ifndef NO_SWERVE
		joystick { Drive::ControllerPorts::kDriverControllerPort }
#endif
#if !defined(NO_SWERVE) && (!defined(NO_ELEVATOR_ARM)  || !(defined(NO_ARM) && defined(NO_ELEVATOR)))
		,
#endif
#if !defined(NO_ELEVATOR_ARM)  || !(defined(NO_ARM) && defined(NO_ELEVATOR)) 
		xboxController { Elevator::ControllerPorts::kDriverControllerPort }
#endif 
{
#ifndef NO_SWERVE
	swerveSubsystem.SetDefaultCommand(SwerveJoystickCommand { &swerveSubsystem,
			joystick });
#endif
#ifndef NO_ELEVATOR
	elevatorSubsystem.SetDefaultCommand(
			frc2::cmd::Idle(frc2::Requirements { &elevatorSubsystem }).WithInterruptBehavior(
					frc2::Command::InterruptionBehavior::kCancelSelf));
#endif
#ifndef NO_ARM
	armSubsystem.SetDefaultCommand(
			frc2::cmd::Idle(frc2::Requirements { &armSubsystem }).WithInterruptBehavior(
					frc2::Command::InterruptionBehavior::kCancelSelf));
#endif

	ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
#ifndef NO_ELEVATOR_HEIGHT_COMMAND
	ConfigureDestination();
	BindElevatorCommand();
#endif
#ifndef NO_ARM_ROTATION_COMMAND
	ConfigureRotation();
	BindArmCommand();
#endif
}

#ifndef NO_ELEVATOR_HEIGHT_COMMAND

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

void RobotContainer::BindElevatorCommand() {
	xboxController.A().Debounce(Elevator::TeleopOperator::kDebounce).OnTrue(
			HeightCommand { destinationCommands.at(
					Elevator::Destination::kFirstGoalIndex) }.ToPtr());
	xboxController.B().Debounce(Elevator::TeleopOperator::kDebounce).OnTrue(
			HeightCommand { destinationCommands.at(
					Elevator::Destination::kSecondGoalIndex) }.ToPtr());
	xboxController.X().Debounce(Elevator::TeleopOperator::kDebounce).OnTrue(
			HeightCommand { destinationCommands.at(
					Elevator::Destination::kThirdGoalIndex) }.ToPtr());
	xboxController.Y().Debounce(Elevator::TeleopOperator::kDebounce).OnTrue(
			HeightCommand { destinationCommands.at(
					Elevator::Destination::kForthGoalIndex) }.ToPtr());
	xboxController.RightBumper().Debounce(Elevator::TeleopOperator::kDebounce).OnTrue(
			HeightCommand { destinationCommands.at(
					Elevator::Destination::kCollectionHeightIndex) }.ToPtr());

	xboxController.Back().Debounce(Elevator::TeleopOperator::kDebounce).OnTrue(
			HeightCommand { destinationCommands.at(
					Elevator::Destination::kMaxHeightIndex) }.ToPtr());
	xboxController.Start().Debounce(Elevator::TeleopOperator::kDebounce).OnTrue(
			HeightCommand { destinationCommands.at(
					Elevator::Destination::kMinHeightIndex) }.ToPtr());
}

#endif
#ifndef NO_ARM_ROTATION_COMMAND

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

void RobotContainer::BindArmCommand() {
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

	xboxController.POVUp().Debounce(Arm::TeleopOperator::kDebounce).OnTrue(
			RotationCommand { rotationCommands.at(
					Arm::Destination::kTopTurnIndex) }.ToPtr());

	xboxController.POVLeft().Debounce(Arm::TeleopOperator::kDebounce).OnTrue(
			RotationCommand { rotationCommands.at(
					Arm::Destination::kMiddleTurnIndex) }.ToPtr());

	xboxController.POVDown().Debounce(Arm::TeleopOperator::kDebounce).OnTrue(
			RotationCommand { rotationCommands.at(
					Arm::Destination::kBottomTurnIndex) }.ToPtr());

}

#endif

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return nullptr;
}
