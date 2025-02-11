#include "commands/SwerveSysIdCommand.h"

SwerveSysIdCommand::SwerveSysIdCommand(SwerveSubsystem* subsystem, SysIdType type) : 
    m_subsystem{ subsystem }, m_type{ type } {
    
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