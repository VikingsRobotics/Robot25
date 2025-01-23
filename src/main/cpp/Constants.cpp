#include "Constants.h"

#include <units/voltage.h>

// Sets definition (one definition rule)
namespace Drive {

namespace DeviceProperties {
static rev::spark::SparkMaxConfig storedConfig { };
rev::spark::SparkMaxConfig& GetSparkMaxConfig() {
	rev::spark::SparkMaxConfig &config = storedConfig;
	config.absoluteEncoder.PositionConversionFactor(
			Drive::Mechanism::kAngleGearRatio.value());
	config.absoluteEncoder.VelocityConversionFactor(
			Drive::Mechanism::kAngleGearRatio.value() / 60);

	config.absoluteEncoder.Inverted(kInvertEncoder);

	config.closedLoop.PositionWrappingEnabled(true);
	config.closedLoop.SetFeedbackSensor(
			rev::spark::ClosedLoopConfig::kAbsoluteEncoder);

	// * POSITION CLOSED-LOOP * //
	config.closedLoop.MinOutput(0);
	config.closedLoop.MaxOutput(std::numbers::pi * 2);
	config.closedLoop.OutputRange(-1, 1);
	config.closedLoop.Pidf(1.0, 0.0, 0.0, 0.0);
	// * END CLOSED-LOOP * //

	// * SMART MOTION CLOSED-LOOP * //
	config.closedLoop.MinOutput(0, rev::spark::kSlot1);
	config.closedLoop.MaxOutput(std::numbers::pi * 2, rev::spark::kSlot1);
	config.closedLoop.OutputRange(-1, 1, rev::spark::kSlot1);
	config.closedLoop.Pidf(1.0, 0.0, 0.0, 0.0, rev::spark::kSlot1);
	// SMART MOTION CONFIGS
	config.closedLoop.maxMotion.MaxVelocity(
			AutoSettings::kMaxAngularSpeed.value(), rev::spark::kSlot1);
	config.closedLoop.maxMotion.MaxAcceleration(
			AutoSettings::kMaxAngularAcceleration.value(), rev::spark::kSlot1);
	config.closedLoop.maxMotion.PositionMode(
			rev::spark::MAXMotionConfig::MAXMotionPositionMode::kMAXMotionTrapezoidal,
			rev::spark::kSlot1);
	config.closedLoop.maxMotion.AllowedClosedLoopError(0, rev::spark::kSlot1);
	// * END CLOSED-LOOP * //

	config.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);

	return config;
}

ctre::phoenix6::configs::TalonFXConfiguration GetTalonFXConfig() {
	ctre::phoenix6::configs::TalonFXConfiguration config;

	config.WithVoltage(
			ctre::phoenix6::configs::VoltageConfigs { }.WithPeakForwardVoltage(
					12_V).WithPeakReverseVoltage(-12_V));
	config.WithMotorOutput(
			ctre::phoenix6::configs::MotorOutputConfigs { }.WithNeutralMode(
					ctre::phoenix6::signals::NeutralModeValue::Brake));
	config.WithSlot0(
			ctre::phoenix6::configs::Slot0Configs { }.WithKP(0.1).WithKI(0.0).WithKD(
					0.0).WithKS(Mechanism::kStaticVoltage.value()).WithKV(
					Mechanism::kVelocityVoltage.value()).WithKA(0.0).WithGravityType(
					ctre::phoenix6::signals::GravityTypeValue::Elevator_Static).WithStaticFeedforwardSign(
					ctre::phoenix6::signals::StaticFeedforwardSignValue::UseVelocitySign));

	return config;
}

namespace SystemControl {
const frc::SwerveDriveKinematics<4> kDriveKinematics { frc::Translation2d {
		+Mechanism::kWheelBase / 2, +Mechanism::kTrackWidth / 2 },
		frc::Translation2d { +Mechanism::kWheelBase / 2, -Mechanism::kTrackWidth
				/ 2 }, frc::Translation2d { -Mechanism::kWheelBase / 2,
				+Mechanism::kTrackWidth / 2 }, frc::Translation2d {
				-Mechanism::kWheelBase / 2, -Mechanism::kTrackWidth / 2 } };
}

}
}
