#include "subsystems/SwerveModule.h"
#include "Constants.h"

SwerveModule::SwerveModule(const int drivingCANId, const int turningCANId,
		const units::radian_t chassisAngularOffest) : m_drivingTalonFx {
		drivingCANId, Drive::DeviceIdentifier::kCANBus }, m_turningSparkMax {
		turningCANId, Drive::DeviceProperties::kSparkMotorType }, m_turningAbsoluteEncoder {
		m_turningSparkMax.GetAbsoluteEncoder() }, m_chassisAngularOffest {
		chassisAngularOffest }, m_sparkLoopController {
		m_turningSparkMax.GetClosedLoopController() }, m_getTalonPosition {
		std::move(m_drivingTalonFx.GetPosition().AsSupplier()) }, m_getTalonVelocity {
		std::move(m_drivingTalonFx.GetVelocity().AsSupplier()) } {

	rev::spark::SparkBaseConfig sparkConfigurator { };
	sparkConfigurator.absoluteEncoder.PositionConversionFactor(
			Drive::Mechanism::kAngleGearRatio.value());
	sparkConfigurator.absoluteEncoder.VelocityConversionFactor(
			Drive::Mechanism::kAngleGearRatio.value() / 60);

	sparkConfigurator.absoluteEncoder.Inverted(
			Drive::DeviceProperties::kInvertEncoder);

	sparkConfigurator.closedLoop.PositionWrappingEnabled(true);
	sparkConfigurator.closedLoop.MinOutput(0);
	sparkConfigurator.closedLoop.MaxOutput(std::numbers::pi * 2);
	sparkConfigurator.closedLoop.SetFeedbackSensor(
			rev::spark::ClosedLoopConfig::kAbsoluteEncoder);
	sparkConfigurator.closedLoop.OutputRange(-1, 1);

	sparkConfigurator.closedLoop.Pidf(Drive::SystemControl::kAngleP,
			Drive::SystemControl::kAngleI, Drive::SystemControl::kAngleD,
			Drive::SystemControl::kAngleF);

	sparkConfigurator.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);

	m_turningSparkMax.Configure(sparkConfigurator,
			rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
			rev::spark::SparkBase::PersistMode::kPersistParameters);

	m_drivingTalonFx.GetConfigurator().Apply(
			Drive::DeviceProperties::kDriveMotorOutputConfig);
	m_drivingTalonFx.GetConfigurator().Apply(
			Drive::DeviceProperties::kDriveMotorVoltageConfig);
	m_drivingTalonFx.GetConfigurator().Apply(
			Drive::SystemControl::kVelocityPID);

	ResetEncoder();
	Stop();
}

frc::SwerveModuleState SwerveModule::GetState() {
	return frc::SwerveModuleState { .speed =
			units::meters_per_second_squared_t { m_getTalonVelocity().value()
					* Drive::Mechanism::kWheelTurnsToMetersDistance }, .angle =
			frc::Rotation2d { units::angle::radian_t {
					m_turningAbsoluteEncoder.GetPosition() }
					- m_chassisAngularOffest } };
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
	return frc::SwerveModulePosition { .distance = units::meter_t {
			m_getTalonPosition().value()
					* Drive::Mechanism::kWheelTurnsToMetersDistance }, .angle =
			frc::Rotation2d { units::angle::radian_t {
					m_turningAbsoluteEncoder.GetPosition() }
					- m_chassisAngularOffest } };
}

void SwerveModule::SetState(frc::SwerveModuleState desiredState) {
	if (std::abs(desiredState.speed.value()) < 0.00001) {
		Stop();
		return;
	}

	frc::SwerveModuleState optimizedDesiredState { };
	optimizedDesiredState.speed = desiredState.speed;
	optimizedDesiredState.angle = desiredState.angle + frc::Rotation2d {
			m_chassisAngularOffest };

	optimizedDesiredState.Optimize(frc::Rotation2d(units::radian_t {
			m_turningAbsoluteEncoder.GetPosition() }));

	m_drivingTalonFx.SetControl(
			ctre::phoenix6::controls::VelocityVoltage {
					optimizedDesiredState.speed
							/ Drive::Mechanism::kWheelTurnsToMetersDistance }.WithSlot(
					0));

	m_sparkLoopController.SetReference(
			optimizedDesiredState.angle.Radians().value(),
			rev::spark::SparkLowLevel::ControlType::kPosition);

}

void SwerveModule::ResetEncoder() {
	m_drivingTalonFx.SetPosition(0_tr);

}

void SwerveModule::Stop() {
	m_drivingTalonFx.SetControl(ctre::phoenix6::controls::StaticBrake { });
	m_turningSparkMax.Set(0);
}
