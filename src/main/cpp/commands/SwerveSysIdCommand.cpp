#include "commands/SwerveSysIdCommand.h"

#include "Constants.h"

SwerveSysIdRoutine::SwerveSysIdRoutine(SwerveSubsystem* subsystem) :
    sysIdRoutineTranslation{
        frc2::sysid::Config {
            Drive::SysId::Translation::kRampRate,
            Drive::SysId::Translation::kStepVoltage,
            Drive::SysId::Translation::kTimeout,
            nullptr
        },
        frc2::sysid::Mechanism {
            [&](units::volt_t volt) {
                subsystem->m_frontLeft.GotoRotation(0_rad);
                subsystem->m_frontLeft.m_drivingTalonFx.SetVoltage(volt);

                subsystem->m_frontRight.GotoRotation(0_rad);
                subsystem->m_frontRight.m_drivingTalonFx.SetVoltage(volt);

                subsystem->m_backLeft.GotoRotation(0_rad);
                subsystem->m_backLeft.m_drivingTalonFx.SetVoltage(volt);

                subsystem->m_backRight.GotoRotation(0_rad);
                subsystem->m_backRight.m_drivingTalonFx.SetVoltage(volt);
            },
            [&](frc::sysid::SysIdRoutineLog* logger) {
                logger->Motor("Front Left Steer Motor").position(units::turn_t{subsystem->m_frontLeft.GetPosition().angle.Degrees()});
                logger->Motor("Front Left Drive Motor").position(subsystem->m_frontLeft.GetPosition().distance).velocity(subsystem->m_frontLeft.GetState().speed);

                logger->Motor("Front Right Steer Motor").position(units::turn_t{subsystem->m_frontRight.GetPosition().angle.Degrees()});;
                logger->Motor("Front Right Drive Motor").position(subsystem->m_frontRight.GetPosition().distance).velocity(subsystem->m_frontRight.GetState().speed);

                logger->Motor("Back Left Steer Motor").position(units::turn_t{subsystem->m_backLeft.GetPosition().angle.Degrees()});;
                logger->Motor("Back Left Drive Motor").position(subsystem->m_backLeft.GetPosition().distance).velocity(subsystem->m_backLeft.GetState().speed);

                logger->Motor("Back Right Steer Motor").position(units::turn_t{subsystem->m_backRight.GetPosition().angle.Degrees()});;
                logger->Motor("Back Right Drive Motor").position(subsystem->m_backRight.GetPosition().distance).velocity(subsystem->m_backRight.GetState().speed);
            },
            subsystem, "Swerve-Translation"
        }
    },
    sysIdRoutineRotation{
        frc2::sysid::Config {
            Drive::SysId::Rotation::kRampRate,
            Drive::SysId::Rotation::kStepVoltage,
            Drive::SysId::Rotation::kTimeout,
            nullptr
        },
        frc2::sysid::Mechanism {
            [](units::volt_t volt) {
                return;
            },
            [&](frc::sysid::SysIdRoutineLog* logger) {
                logger->Motor("Front Left Steer Motor").position(units::turn_t{subsystem->m_frontLeft.GetPosition().angle.Degrees()});
                logger->Motor("Front Left Drive Motor").position(subsystem->m_frontLeft.GetPosition().distance).velocity(subsystem->m_frontLeft.GetState().speed);

                logger->Motor("Front Right Steer Motor").position(units::turn_t{subsystem->m_frontRight.GetPosition().angle.Degrees()});;
                logger->Motor("Front Right Drive Motor").position(subsystem->m_frontRight.GetPosition().distance).velocity(subsystem->m_frontRight.GetState().speed);

                logger->Motor("Back Left Steer Motor").position(units::turn_t{subsystem->m_backLeft.GetPosition().angle.Degrees()});;
                logger->Motor("Back Left Drive Motor").position(subsystem->m_backLeft.GetPosition().distance).velocity(subsystem->m_backLeft.GetState().speed);

                logger->Motor("Back Right Steer Motor").position(units::turn_t{subsystem->m_backRight.GetPosition().angle.Degrees()});;
                logger->Motor("Back Right Drive Motor").position(subsystem->m_backRight.GetPosition().distance).velocity(subsystem->m_backRight.GetState().speed);
            },
            subsystem, "Swerve-Rotation"
        }
    },
    sysIdRoutineSteer{
        frc2::sysid::Config {
            Drive::SysId::Steer::kRampRate,
            Drive::SysId::Steer::kStepVoltage,
            Drive::SysId::Steer::kTimeout,
            nullptr
        },
        frc2::sysid::Mechanism {
            [&](units::volt_t volt) {
                subsystem->m_frontLeft.m_turningSparkMax.SetVoltage(volt);
                subsystem->m_frontLeft.m_drivingTalonFx.SetControl(ctre::phoenix6::controls::CoastOut{});
                
                subsystem->m_frontRight.m_turningSparkMax.SetVoltage(volt);
                subsystem->m_frontRight.m_drivingTalonFx.SetControl(ctre::phoenix6::controls::CoastOut{});

                subsystem->m_backLeft.m_turningSparkMax.SetVoltage(volt);
                subsystem->m_backLeft.m_drivingTalonFx.SetControl(ctre::phoenix6::controls::CoastOut{});

                subsystem->m_backRight.m_turningSparkMax.SetVoltage(volt);
                subsystem->m_backRight.m_drivingTalonFx.SetControl(ctre::phoenix6::controls::CoastOut{});

            },
            [&](frc::sysid::SysIdRoutineLog* logger) {
                logger->Motor("Front Left Steer Motor").position(units::turn_t{subsystem->m_frontLeft.GetPosition().angle.Degrees()}).velocity(units::turns_per_second_t{units::radians_per_second_t{subsystem->m_frontLeft.m_turningAbsoluteEncoder.GetVelocity()}}).voltage(units::volt_t{subsystem->m_frontLeft.m_turningSparkMax.GetBusVoltage()}).current(units::ampere_t{subsystem->m_frontLeft.m_turningSparkMax.GetOutputCurrent()});
                logger->Motor("Front Left Drive Motor").position(subsystem->m_frontLeft.GetPosition().distance).velocity(subsystem->m_frontLeft.GetState().speed).voltage(subsystem->m_frontLeft.m_drivingTalonFx.GetMotorVoltage().GetValue()).current(subsystem->m_frontLeft.m_drivingTalonFx.GetStatorCurrent().GetValue());

                logger->Motor("Front Right Steer Motor").position(units::turn_t{subsystem->m_frontRight.GetPosition().angle.Degrees()}).voltage(units::volt_t{subsystem->m_frontRight.m_turningSparkMax.GetBusVoltage()}).current(units::ampere_t{subsystem->m_frontRight.m_turningSparkMax.GetOutputCurrent()});
                logger->Motor("Front Right Drive Motor").position(subsystem->m_frontRight.GetPosition().distance).velocity(subsystem->m_frontRight.GetState().speed).voltage(subsystem->m_frontRight.m_drivingTalonFx.GetMotorVoltage().GetValue()).current(subsystem->m_frontRight.m_drivingTalonFx.GetStatorCurrent().GetValue());

                logger->Motor("Back Left Steer Motor").position(units::turn_t{subsystem->m_frontLeft.GetPosition().angle.Degrees()}).voltage(units::volt_t{subsystem->m_frontLeft.m_turningSparkMax.GetBusVoltage()}).current(units::ampere_t{subsystem->m_frontLeft.m_turningSparkMax.GetOutputCurrent()});
                logger->Motor("Back Left Drive Motor").position(subsystem->m_frontLeft.GetPosition().distance).velocity(subsystem->m_frontLeft.GetState().speed).voltage(subsystem->m_frontLeft.m_drivingTalonFx.GetMotorVoltage().GetValue()).current(subsystem->m_frontLeft.m_drivingTalonFx.GetStatorCurrent().GetValue());

                logger->Motor("Back Right Steer Motor").position(units::turn_t{subsystem->m_frontLeft.GetPosition().angle.Degrees()}).voltage(units::volt_t{subsystem->m_frontLeft.m_turningSparkMax.GetBusVoltage()}).current(units::ampere_t{subsystem->m_frontLeft.m_turningSparkMax.GetOutputCurrent()});
                logger->Motor("Back Right Drive Motor").position(subsystem->m_frontLeft.GetPosition().distance).velocity(subsystem->m_frontLeft.GetState().speed).voltage(subsystem->m_frontLeft.m_drivingTalonFx.GetMotorVoltage().GetValue()).current(subsystem->m_frontLeft.m_drivingTalonFx.GetStatorCurrent().GetValue());
            },
            subsystem, "Swerve-Steer"
        }
    }
{
    
}

SwerveSysIdCommand::SwerveSysIdCommand(SwerveSubsystem* subsystem, SysIdType type, SysIdAction defaultAction) : 
    m_subsystem{ subsystem }, m_type{ type }, m_currentAction{ defaultAction }, m_sysIdRoutines{subsystem} {
    
    AddRequirements(subsystem);

    switch(type)
    {
        case SysIdType::Translation:
        SetName("Translation SysId Command");
        break;
        case SysIdType::Rotation:
        SetName("Rotation SysId Command");
        break;
        case SysIdType::Steer:
        SetName("Steer SysId Command");
        break;
        default:
        SetName("Unknown SysId Command");
    }
    
}

void SwerveSysIdCommand::Initialize() {
    
}

void SwerveSysIdCommand::Execute() {

}

void SwerveSysIdCommand::End(bool interrupted) {

}

bool SwerveSysIdCommand::IsFinished() {
    
}

void SwerveSysIdCommand::SetAction(SysIdAction action) {

}

SysIdAction SwerveSysIdCommand::GetAction() {
    
}