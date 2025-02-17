#include "commands/SwerveSysIdRoutine.h"

#ifndef NO_SWERVE_SYSID_COMMAND

#include "Constants.h"
#include <frc2/command/Commands.h>

SwerveSysIdRoutine::SwerveSysIdRoutine(SwerveSubsystem *subsystem) : sysIdRoutineTranslation {
		frc2::sysid::Config { Drive::SysId::Translation::kRampRate,
				Drive::SysId::Translation::kStepVoltage,
				Drive::SysId::Translation::kTimeout, nullptr },
		frc2::sysid::Mechanism { [&](units::volt_t volt) {
			subsystem->m_frontLeft.GotoRotation(0_rad);
			subsystem->m_frontLeft.m_drivingTalonFx.SetVoltage(volt);

			subsystem->m_frontRight.GotoRotation(0_rad);
			subsystem->m_frontRight.m_drivingTalonFx.SetVoltage(volt);

			subsystem->m_backLeft.GotoRotation(0_rad);
			subsystem->m_backLeft.m_drivingTalonFx.SetVoltage(volt);

			subsystem->m_backRight.GotoRotation(0_rad);
			subsystem->m_backRight.m_drivingTalonFx.SetVoltage(volt);
		}
				,
				[&](frc::sysid::SysIdRoutineLog *logger) {
					logger->Motor("Front Left Steer Motor").position(
							units::turn_t {
									subsystem->m_frontLeft.GetPosition().angle.Degrees() }).velocity(
							units::turns_per_second_t {
									units::radians_per_second_t {
											subsystem->m_frontLeft.m_turningAbsoluteEncoder.GetVelocity() } }).voltage(
							units::volt_t {
									subsystem->m_frontLeft.m_turningSparkMax.GetBusVoltage() }).current(
							units::ampere_t {
									subsystem->m_frontLeft.m_turningSparkMax.GetOutputCurrent() }).value(
							"max-motion-enabled",
							subsystem->m_frontLeft.m_useSmartMotionSparkMax ?
									1 : 0, "bool");
					logger->Motor("Front Left Drive Motor").position(
							subsystem->m_frontLeft.GetPosition().distance).velocity(
							subsystem->m_frontLeft.GetState().speed).voltage(
							subsystem->m_frontLeft.m_drivingTalonFx.GetMotorVoltage().GetValue()).current(
							subsystem->m_frontLeft.m_drivingTalonFx.GetStatorCurrent().GetValue());

					logger->Motor("Front Right Steer Motor").position(
							units::turn_t {
									subsystem->m_frontRight.GetPosition().angle.Degrees() }).velocity(
							units::turns_per_second_t {
									units::radians_per_second_t {
											subsystem->m_frontRight.m_turningAbsoluteEncoder.GetVelocity() } }).voltage(
							units::volt_t {
									subsystem->m_frontRight.m_turningSparkMax.GetBusVoltage() }).current(
							units::ampere_t {
									subsystem->m_frontRight.m_turningSparkMax.GetOutputCurrent() }).value(
							"max-motion-enabled",
							subsystem->m_frontRight.m_useSmartMotionSparkMax ?
									1 : 0, "bool");
					logger->Motor("Front Right Drive Motor").position(
							subsystem->m_frontRight.GetPosition().distance).velocity(
							subsystem->m_frontRight.GetState().speed).voltage(
							subsystem->m_frontRight.m_drivingTalonFx.GetMotorVoltage().GetValue()).current(
							subsystem->m_frontRight.m_drivingTalonFx.GetStatorCurrent().GetValue());

					logger->Motor("Back Left Steer Motor").position(
							units::turn_t {
									subsystem->m_backLeft.GetPosition().angle.Degrees() }).velocity(
							units::turns_per_second_t {
									units::radians_per_second_t {
											subsystem->m_backLeft.m_turningAbsoluteEncoder.GetVelocity() } }).voltage(
							units::volt_t {
									subsystem->m_backLeft.m_turningSparkMax.GetBusVoltage() }).current(
							units::ampere_t {
									subsystem->m_backLeft.m_turningSparkMax.GetOutputCurrent() }).value(
							"max-motion-enabled",
							subsystem->m_backLeft.m_useSmartMotionSparkMax ?
									1 : 0, "bool");
					logger->Motor("Back Left Drive Motor").position(
							subsystem->m_backLeft.GetPosition().distance).velocity(
							subsystem->m_backLeft.GetState().speed).voltage(
							subsystem->m_backLeft.m_drivingTalonFx.GetMotorVoltage().GetValue()).current(
							subsystem->m_backLeft.m_drivingTalonFx.GetStatorCurrent().GetValue());

					logger->Motor("Back Right Steer Motor").position(
							units::turn_t {
									subsystem->m_backRight.GetPosition().angle.Degrees() }).velocity(
							units::turns_per_second_t {
									units::radians_per_second_t {
											subsystem->m_backRight.m_turningAbsoluteEncoder.GetVelocity() } }).voltage(
							units::volt_t {
									subsystem->m_backRight.m_turningSparkMax.GetBusVoltage() }).current(
							units::ampere_t {
									subsystem->m_backRight.m_turningSparkMax.GetOutputCurrent() }).value(
							"max-motion-enabled",
							subsystem->m_backRight.m_useSmartMotionSparkMax ?
									1 : 0, "bool");
					logger->Motor("Back Right Drive Motor").position(
							subsystem->m_backRight.GetPosition().distance).velocity(
							subsystem->m_backRight.GetState().speed).voltage(
							subsystem->m_backRight.m_drivingTalonFx.GetMotorVoltage().GetValue()).current(
							subsystem->m_backRight.m_drivingTalonFx.GetStatorCurrent().GetValue());
				},
				subsystem, "Swerve-Translation" } }, sysIdRoutineRotation {
		frc2::sysid::Config { Drive::SysId::Rotation::kRampRate,
				Drive::SysId::Rotation::kStepVoltage,
				Drive::SysId::Rotation::kTimeout, nullptr },
		frc2::sysid::Mechanism { [&](units::volt_t volt) {
			constexpr units::meters_per_second_t distanceFromCenter =
					frc::Translation2d { Drive::Mechanism::kWheelBase,
							Drive::Mechanism::kTrackWidth }.Norm() / 1_s;
			units::volt_t apply = ((volt.value() * distanceFromCenter)
					/ Drive::Mechanism::kPhysicalMoveMax) * 12_V;

			subsystem->m_frontLeft.GotoRotation(135_deg);
			subsystem->m_frontLeft.m_drivingTalonFx.SetVoltage(apply);

			subsystem->m_frontLeft.GotoRotation(45_deg);
			subsystem->m_frontLeft.m_drivingTalonFx.SetVoltage(apply);

			subsystem->m_backLeft.GotoRotation(225_deg);
			subsystem->m_frontLeft.m_drivingTalonFx.SetVoltage(apply);

			subsystem->m_frontLeft.GotoRotation(315_deg);
			subsystem->m_frontLeft.m_drivingTalonFx.SetVoltage(apply);
		}
				,
				[&](frc::sysid::SysIdRoutineLog *logger) {
					logger->Motor("Front Left Steer Motor").position(
							units::turn_t {
									subsystem->m_frontLeft.GetPosition().angle.Degrees() }).velocity(
							units::turns_per_second_t {
									units::radians_per_second_t {
											subsystem->m_frontLeft.m_turningAbsoluteEncoder.GetVelocity() } }).voltage(
							units::volt_t {
									subsystem->m_frontLeft.m_turningSparkMax.GetBusVoltage() }).current(
							units::ampere_t {
									subsystem->m_frontLeft.m_turningSparkMax.GetOutputCurrent() }).value(
							"max-motion-enabled",
							subsystem->m_frontLeft.m_useSmartMotionSparkMax ?
									1 : 0, "bool");
					logger->Motor("Front Left Drive Motor").position(
							subsystem->m_frontLeft.GetPosition().distance).velocity(
							subsystem->m_frontLeft.GetState().speed).voltage(
							subsystem->m_frontLeft.m_drivingTalonFx.GetMotorVoltage().GetValue()).current(
							subsystem->m_frontLeft.m_drivingTalonFx.GetStatorCurrent().GetValue());

					logger->Motor("Front Right Steer Motor").position(
							units::turn_t {
									subsystem->m_frontRight.GetPosition().angle.Degrees() }).velocity(
							units::turns_per_second_t {
									units::radians_per_second_t {
											subsystem->m_frontRight.m_turningAbsoluteEncoder.GetVelocity() } }).voltage(
							units::volt_t {
									subsystem->m_frontRight.m_turningSparkMax.GetBusVoltage() }).current(
							units::ampere_t {
									subsystem->m_frontRight.m_turningSparkMax.GetOutputCurrent() }).value(
							"max-motion-enabled",
							subsystem->m_frontRight.m_useSmartMotionSparkMax ?
									1 : 0, "bool");
					logger->Motor("Front Right Drive Motor").position(
							subsystem->m_frontRight.GetPosition().distance).velocity(
							subsystem->m_frontRight.GetState().speed).voltage(
							subsystem->m_frontRight.m_drivingTalonFx.GetMotorVoltage().GetValue()).current(
							subsystem->m_frontRight.m_drivingTalonFx.GetStatorCurrent().GetValue());

					logger->Motor("Back Left Steer Motor").position(
							units::turn_t {
									subsystem->m_backLeft.GetPosition().angle.Degrees() }).velocity(
							units::turns_per_second_t {
									units::radians_per_second_t {
											subsystem->m_backLeft.m_turningAbsoluteEncoder.GetVelocity() } }).voltage(
							units::volt_t {
									subsystem->m_backLeft.m_turningSparkMax.GetBusVoltage() }).current(
							units::ampere_t {
									subsystem->m_backLeft.m_turningSparkMax.GetOutputCurrent() }).value(
							"max-motion-enabled",
							subsystem->m_backLeft.m_useSmartMotionSparkMax ?
									1 : 0, "bool");
					logger->Motor("Back Left Drive Motor").position(
							subsystem->m_backLeft.GetPosition().distance).velocity(
							subsystem->m_backLeft.GetState().speed).voltage(
							subsystem->m_backLeft.m_drivingTalonFx.GetMotorVoltage().GetValue()).current(
							subsystem->m_backLeft.m_drivingTalonFx.GetStatorCurrent().GetValue());

					logger->Motor("Back Right Steer Motor").position(
							units::turn_t {
									subsystem->m_backRight.GetPosition().angle.Degrees() }).velocity(
							units::turns_per_second_t {
									units::radians_per_second_t {
											subsystem->m_backRight.m_turningAbsoluteEncoder.GetVelocity() } }).voltage(
							units::volt_t {
									subsystem->m_backRight.m_turningSparkMax.GetBusVoltage() }).current(
							units::ampere_t {
									subsystem->m_backRight.m_turningSparkMax.GetOutputCurrent() }).value(
							"max-motion-enabled",
							subsystem->m_backRight.m_useSmartMotionSparkMax ?
									1 : 0, "bool");
					logger->Motor("Back Right Drive Motor").position(
							subsystem->m_backRight.GetPosition().distance).velocity(
							subsystem->m_backRight.GetState().speed).voltage(
							subsystem->m_backRight.m_drivingTalonFx.GetMotorVoltage().GetValue()).current(
							subsystem->m_backRight.m_drivingTalonFx.GetStatorCurrent().GetValue());
				},
				subsystem, "Swerve-Rotation" } }, sysIdRoutineSteer {
		frc2::sysid::Config { Drive::SysId::Steer::kRampRate,
				Drive::SysId::Steer::kStepVoltage,
				Drive::SysId::Steer::kTimeout, nullptr },
		frc2::sysid::Mechanism { [&](units::volt_t volt) {
			subsystem->m_frontLeft.m_turningSparkMax.SetVoltage(volt);
			subsystem->m_frontLeft.m_drivingTalonFx.SetControl(
					ctre::phoenix6::controls::CoastOut { });

			subsystem->m_frontRight.m_turningSparkMax.SetVoltage(volt);
			subsystem->m_frontRight.m_drivingTalonFx.SetControl(
					ctre::phoenix6::controls::CoastOut { });

			subsystem->m_backLeft.m_turningSparkMax.SetVoltage(volt);
			subsystem->m_backLeft.m_drivingTalonFx.SetControl(
					ctre::phoenix6::controls::CoastOut { });

			subsystem->m_backRight.m_turningSparkMax.SetVoltage(volt);
			subsystem->m_backRight.m_drivingTalonFx.SetControl(
					ctre::phoenix6::controls::CoastOut { });

		}
				,
				[&](frc::sysid::SysIdRoutineLog *logger) {
					logger->Motor("Front Left Steer Motor").position(
							units::turn_t {
									subsystem->m_frontLeft.GetPosition().angle.Degrees() }).velocity(
							units::turns_per_second_t {
									units::radians_per_second_t {
											subsystem->m_frontLeft.m_turningAbsoluteEncoder.GetVelocity() } }).voltage(
							units::volt_t {
									subsystem->m_frontLeft.m_turningSparkMax.GetBusVoltage() }).current(
							units::ampere_t {
									subsystem->m_frontLeft.m_turningSparkMax.GetOutputCurrent() }).value(
							"max-motion-enabled",
							subsystem->m_frontLeft.m_useSmartMotionSparkMax ?
									1 : 0, "bool");
					logger->Motor("Front Left Drive Motor").position(
							subsystem->m_frontLeft.GetPosition().distance).velocity(
							subsystem->m_frontLeft.GetState().speed).voltage(
							subsystem->m_frontLeft.m_drivingTalonFx.GetMotorVoltage().GetValue()).current(
							subsystem->m_frontLeft.m_drivingTalonFx.GetStatorCurrent().GetValue());

					logger->Motor("Front Right Steer Motor").position(
							units::turn_t {
									subsystem->m_frontRight.GetPosition().angle.Degrees() }).velocity(
							units::turns_per_second_t {
									units::radians_per_second_t {
											subsystem->m_frontRight.m_turningAbsoluteEncoder.GetVelocity() } }).voltage(
							units::volt_t {
									subsystem->m_frontRight.m_turningSparkMax.GetBusVoltage() }).current(
							units::ampere_t {
									subsystem->m_frontRight.m_turningSparkMax.GetOutputCurrent() }).value(
							"max-motion-enabled",
							subsystem->m_frontRight.m_useSmartMotionSparkMax ?
									1 : 0, "bool");
					logger->Motor("Front Right Drive Motor").position(
							subsystem->m_frontRight.GetPosition().distance).velocity(
							subsystem->m_frontRight.GetState().speed).voltage(
							subsystem->m_frontRight.m_drivingTalonFx.GetMotorVoltage().GetValue()).current(
							subsystem->m_frontRight.m_drivingTalonFx.GetStatorCurrent().GetValue());

					logger->Motor("Back Left Steer Motor").position(
							units::turn_t {
									subsystem->m_backLeft.GetPosition().angle.Degrees() }).velocity(
							units::turns_per_second_t {
									units::radians_per_second_t {
											subsystem->m_backLeft.m_turningAbsoluteEncoder.GetVelocity() } }).voltage(
							units::volt_t {
									subsystem->m_backLeft.m_turningSparkMax.GetBusVoltage() }).current(
							units::ampere_t {
									subsystem->m_backLeft.m_turningSparkMax.GetOutputCurrent() }).value(
							"max-motion-enabled",
							subsystem->m_backLeft.m_useSmartMotionSparkMax ?
									1 : 0, "bool");
					logger->Motor("Back Left Drive Motor").position(
							subsystem->m_backLeft.GetPosition().distance).velocity(
							subsystem->m_backLeft.GetState().speed).voltage(
							subsystem->m_backLeft.m_drivingTalonFx.GetMotorVoltage().GetValue()).current(
							subsystem->m_backLeft.m_drivingTalonFx.GetStatorCurrent().GetValue());

					logger->Motor("Back Right Steer Motor").position(
							units::turn_t {
									subsystem->m_backRight.GetPosition().angle.Degrees() }).velocity(
							units::turns_per_second_t {
									units::radians_per_second_t {
											subsystem->m_backRight.m_turningAbsoluteEncoder.GetVelocity() } }).voltage(
							units::volt_t {
									subsystem->m_backRight.m_turningSparkMax.GetBusVoltage() }).current(
							units::ampere_t {
									subsystem->m_backRight.m_turningSparkMax.GetOutputCurrent() }).value(
							"max-motion-enabled",
							subsystem->m_backRight.m_useSmartMotionSparkMax ?
									1 : 0, "bool");
					logger->Motor("Back Right Drive Motor").position(
							subsystem->m_backRight.GetPosition().distance).velocity(
							subsystem->m_backRight.GetState().speed).voltage(
							subsystem->m_backRight.m_drivingTalonFx.GetMotorVoltage().GetValue()).current(
							subsystem->m_backRight.m_drivingTalonFx.GetStatorCurrent().GetValue());
				},
				subsystem, "Swerve-Steer" } } {

}
#endif
