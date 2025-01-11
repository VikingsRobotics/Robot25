#include "Constants.h"

#include <units/voltage.h>

// Sets definition (one definition rule)
namespace Drive {

namespace DeviceProperties {
const ctre::phoenix6::configs::VoltageConfigs kDriveMotorVoltageConfig =
		ctre::phoenix6::configs::VoltageConfigs { }.WithPeakForwardVoltage(12_V).WithPeakReverseVoltage(
				-12_V);
const ctre::phoenix6::configs::MotorOutputConfigs kDriveMotorOutputConfig =
		ctre::phoenix6::configs::MotorOutputConfigs { }.WithNeutralMode(
				ctre::phoenix6::signals::NeutralModeValue::Brake);
}

namespace SystemControl {
const ctre::phoenix6::configs::Slot0Configs kVelocityPID =
		ctre::phoenix6::configs::Slot0Configs { }.WithKP(1.0).WithKI(0.0).WithKD(
				0.0).WithKS(Mechanism::kStaticVoltage.value()).WithKV(
				Mechanism::kVelocityVoltage.value()).WithKA(0.0).WithGravityType(
				ctre::phoenix6::signals::GravityTypeValue::Elevator_Static).WithStaticFeedforwardSign(
				ctre::phoenix6::signals::StaticFeedforwardSignValue::UseVelocitySign);
const frc::SwerveDriveKinematics<4> kDriveKinematics { frc::Translation2d {
		+Mechanism::kWheelBase / 2, +Mechanism::kTrackWidth / 2 },
		frc::Translation2d { +Mechanism::kWheelBase / 2, -Mechanism::kTrackWidth
				/ 2 }, frc::Translation2d { -Mechanism::kWheelBase / 2,
				+Mechanism::kTrackWidth / 2 }, frc::Translation2d {
				-Mechanism::kWheelBase / 2, -Mechanism::kTrackWidth / 2 } };
//It not an nonaggregate class and no copy/move constructors (YAY not compile time!) >:(
//const rev::spark::ClosedLoopConfig kAnglePID; 
}

}
