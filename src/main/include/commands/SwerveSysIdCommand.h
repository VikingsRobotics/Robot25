#pragma once

#include "subsystems/SwerveSubsystem.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/sysid/SysIdRoutineLog.h>

struct SwerveSysIdRoutine {
    SwerveSysIdRoutine(SwerveSubsystem* subsystem);
    frc2::sysid::SysIdRoutine sysIdRoutineTranslation;
    frc2::sysid::SysIdRoutine sysIdRoutineRotation;
    frc2::sysid::SysIdRoutine sysIdRoutineSteer;
};

class SwerveSysIdCommand : frc2::CommandHelper<frc2::Command,SwerveSysIdCommand> {
public:

enum class SysIdType {
    Translation,
    Rotation,
    Steer
};

enum class SysIdAction : uint8_t {
    QuasistaticForward = 0b00,
    QuasistaticReverse = 0b01,
    DynamicForward = 0b10,
    DynamicReverse = 0b11
};
    SwerveSysIdCommand(SwerveSubsystem* subsystem, SysIdType type, SysIdAction defaultAction);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    void SetAction(SysIdAction action);

    SysIdAction GetAction();
private:
SwerveSubsystem* const m_subsystem;
SysIdType const m_type;
SysIdAction m_currentAction;
SwerveSysIdRoutine m_sysIdRoutines;
};
