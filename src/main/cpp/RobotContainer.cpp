// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "Constants.h"

#include "commands/SwerveControllerCommand.h"
#include "commands/SwerveJoystickCommand.h"
#include "commands/SwerveSysIdRoutine.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/RobotState.h>

#include <frc2/command/RunCommand.h>

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() : joystick {
		Drive::ControllerPorts::kDriverControllerPort } {
	swerveSubsystem.SetDefaultCommand(SwerveJoystickCommand { &swerveSubsystem,
			joystick });

	ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
}

void RobotContainer::ConfigureSwerveSysId() {
	swerveSysIdTestMoveOnly.Set(false);

	SwerveSysIdRoutine commandHolder { &swerveSubsystem };

	/* Added all the translation SysId commands*/

	SwerveSysId.emplace_back(
			frc2::cmd::Either(
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(true);
					}).AndThen(
							commandHolder.sysIdRoutineTranslation.Quasistatic(
									frc2::sysid::kForward)),
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(false);
					}),
					[]() -> bool {
						return frc::RobotState::IsTest();
					}
			).WithName("Translation Quasistatic Forward"));
	SwerveSysId.emplace_back(
			frc2::cmd::Either(
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(true);
					}).AndThen(
							commandHolder.sysIdRoutineTranslation.Quasistatic(
									frc2::sysid::kReverse)),
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(false);
					}),
					[]() -> bool {
						return frc::RobotState::IsTest();
					}
			).WithName("Translation Quasistatic Reverse"));
	SwerveSysId.emplace_back(
			frc2::cmd::Either(
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(true);
					}).AndThen(
							commandHolder.sysIdRoutineTranslation.Dynamic(
									frc2::sysid::kForward)),
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(false);
					}),
					[]() -> bool {
						return frc::RobotState::IsTest();
					}
			).WithName("Translation Dyanamic Forward"));
	SwerveSysId.emplace_back(
			frc2::cmd::Either(
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(true);
					}).AndThen(
							commandHolder.sysIdRoutineTranslation.Dynamic(
									frc2::sysid::kReverse)),
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(false);
					}),
					[]() -> bool {
						return frc::RobotState::IsTest();
					}
			).WithName("Translation Dyanamic Reverse"));

	/* Add all the rotation SysId Commands */

	SwerveSysId.emplace_back(
			frc2::cmd::Either(
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(true);
					}).AndThen(
							commandHolder.sysIdRoutineRotation.Quasistatic(
									frc2::sysid::kForward)),
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(false);
					}),
					[]() -> bool {
						return frc::RobotState::IsTest();
					}
			).WithName("Rotation Quasistatic Forward"));
	SwerveSysId.emplace_back(
			frc2::cmd::Either(
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(true);
					}).AndThen(
							commandHolder.sysIdRoutineRotation.Quasistatic(
									frc2::sysid::kReverse)),
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(false);
					}),
					[]() -> bool {
						return frc::RobotState::IsTest();
					}
			).WithName("Rotation Quasistatic Reverse"));
	SwerveSysId.emplace_back(
			frc2::cmd::Either(
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(true);
					}).AndThen(
							commandHolder.sysIdRoutineRotation.Dynamic(
									frc2::sysid::kForward)),
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(false);
					}),
					[]() -> bool {
						return frc::RobotState::IsTest();
					}
			).WithName("Rotation Dynamic Forward"));
	SwerveSysId.emplace_back(
			frc2::cmd::Either(
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(true);
					}).AndThen(
							commandHolder.sysIdRoutineRotation.Dynamic(
									frc2::sysid::kReverse)),
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(false);
					}),
					[]() -> bool {
						return frc::RobotState::IsTest();
					}
			).WithName("Rotation Dynamic Reverse"));

	/* Adds all steer SysId commands */

	SwerveSysId.emplace_back(
			frc2::cmd::Either(
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(true);
					}).AndThen(
							commandHolder.sysIdRoutineSteer.Quasistatic(
									frc2::sysid::kForward)),
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(false);
					}),
					[]() -> bool {
						return frc::RobotState::IsTest();
					}
			).WithName("Steer Quasistatic Forward"));
	SwerveSysId.emplace_back(
			frc2::cmd::Either(
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(true);
					}).AndThen(
							commandHolder.sysIdRoutineSteer.Quasistatic(
									frc2::sysid::kReverse)),
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(false);
					}),
					[]() -> bool {
						return frc::RobotState::IsTest();
					}
			).WithName("Steer Quasistatic Reverse"));
	SwerveSysId.emplace_back(
			frc2::cmd::Either(
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(true);
					}).AndThen(
							commandHolder.sysIdRoutineSteer.Dynamic(
									frc2::sysid::kForward)),
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(false);
					}),
					[]() -> bool {
						return frc::RobotState::IsTest();
					}
			).WithName("Steer Dynamic Forward"));
	SwerveSysId.emplace_back(
			frc2::cmd::Either(
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(true);
					}).AndThen(
							commandHolder.sysIdRoutineSteer.Dynamic(
									frc2::sysid::kReverse)),
					frc2::cmd::RunOnce([&]() {
						swerveSysIdTestMoveOnly.Set(false);
					}),
					[]() -> bool {
						return frc::RobotState::IsTest();
					}
			).WithName("Steer Dynamic Reverse"));

	/* Publish Commands to Test Shuffleboard */

	for (frc2::CommandPtr &cmd : SwerveSysId) {
		frc::Shuffleboard::GetTab("Test").Add(*cmd.get());
	}
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return nullptr;
}
