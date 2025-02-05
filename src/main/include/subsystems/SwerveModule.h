#pragma once

#include "Disable.h"
#ifndef NO_SWERVE

#include <units/angle.h>
#include <units/velocity.h>

#include <rev/SparkMax.h>
#include <rev/AbsoluteEncoder.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>

class SwerveModule {
public:
	SwerveModule(const int drivingCANId, const int turningCANId,
			const units::radian_t chassisAngularOffest);
	SwerveModule(SwerveModule &rhs) = delete;
	SwerveModule& operator=(SwerveModule &rhs) = delete;
	SwerveModule(SwerveModule &&rhs) = delete;
	SwerveModule& operator=(SwerveModule &&rhs) = delete;

	frc::SwerveModuleState GetState();
	frc::SwerveModulePosition GetPosition();

	void SetState(frc::SwerveModuleState desiredState);
	void SetState(frc::SwerveModuleState desiredState,
			units::newton_t feedforwardX, units::newton_t feedforwardY);

	void ResetEncoder();
	void Stop();
	void Brake();

	bool UsingMotionMagic();
	void UseMotionMagic(bool enable);

private:
	struct Feedforward {
		units::newton_meter_t torque;
		units::ampere_t current;
		units::volt_t voltage;
	};

	Feedforward CalculateFeedforward(units::newton_t feedforwardX,
			units::newton_t feedforwardY);

	bool m_useSmartMotionSparkMax = false;
	ctre::phoenix6::hardware::TalonFX m_drivingTalonFx;
	rev::spark::SparkMax m_turningSparkMax;

	rev::spark::SparkAbsoluteEncoder m_turningAbsoluteEncoder;
	units::radian_t m_chassisAngularOffest;

	rev::spark::SparkClosedLoopController m_sparkLoopController;

	std::function<units::angle::turn_t()> m_getTalonPosition;
	std::function<units::angular_velocity::turns_per_second_t()> m_getTalonVelocity;
};

#endif
