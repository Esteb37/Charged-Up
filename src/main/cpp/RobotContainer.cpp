// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{
	ConfigureSubsystems();
}

void RobotContainer::RobotInit()
{
	m_gyro.Calibrate();
}

void RobotContainer::RobotPeriodic()
{
	m_drivetrain.PrintEncoders();
	m_drivetrain.PrintPose();
}

void RobotContainer::ConfigureSubsystems()
{
	m_drivetrain.SetGyro(&m_gyro);
	m_drivetrain.SetPositionConversionFactor(DPR::DRIVETRAIN);

	m_drivetrain.ConfigureMovePID(PID::Move::P, PID::Move::I, PID::Move::D, PID::Move::TOLERANCE, true);

	m_drivetrain.ConfigureTurnPID(PID::Turn::P, PID::Turn::I, PID::Turn::D, PID::Turn::TOLERANCE);

	m_drivetrain.ConfigurePathFollower(Path::RAMSETE_B, Path::RAMSETE_ZETA, Path::KS, Path::KV, Path::KA, Path::KP, Path::KP);

	m_drivetrain.ResetPose();
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{

	DifferentialDriveKinematics kinematics{Wheel::TRACK_WIDTH};

	DifferentialDriveVoltageConstraint autoVoltageConstraint{
		SimpleMotorFeedforward<units::meters>{
			Path::KS, Path::KV, Path::KA},
		kinematics, 12_V};

	// Set up config for trajectory
	frc::TrajectoryConfig config{Path::MAX_SPEED,
								 Path::MAX_ACCELERATION};
	// Add kinematics to ensure max speed is actually obeyed
	config.SetKinematics(kinematics);
	// Apply the voltage constraint
	config.AddConstraint(autoVoltageConstraint);

	// An example trajectory to follow.  All units in meters.
	auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
		// Start at the origin facing the +X direction
		frc::Pose2d{0_m, 0_m, 0_deg},
		// Pass through these two interior waypoints, making an 's' curve path
		{frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}, frc::Translation2d{3_m, 0_m}},
		// End 3 meters straight ahead of where we started, facing forward
		frc::Pose2d{4_m, 0_m, 0_deg},
		// Pass the config
		config);

	RamseteCommand ramseteCommand{
		trajectory,
		[this]()
		{ return m_drivetrain.GetPose(); },
		RamseteController{Path::RAMSETE_B,
						  Path::RAMSETE_ZETA},
		SimpleMotorFeedforward<units::meters>{
			Path::KS,
			Path::KV,
			Path::KA},
		kinematics,
		[this]
		{ return m_drivetrain.GetWheelSpeeds(); },
		frc2::PIDController{Path::KP, 0, 0},
		frc2::PIDController{Path::KP, 0, 0},
		[this](auto left, auto right)
		{ m_drivetrain.TankDriveVolts(left, right); },
		{&m_drivetrain}};

	// Reset odometry to the starting pose of the trajectory.
	m_drivetrain.SetPose(trajectory.InitialPose());

	// no auto
	return new frc2::SequentialCommandGroup(
		std::move(ramseteCommand),
		frc2::InstantCommand([this]
							 { m_drivetrain.TankDriveVolts(0_V, 0_V); },
							 {}));
}

void RobotContainer::TeleopInit()
{
	m_gyro.Reset();
	m_drivetrain.ResetPose();
}
void RobotContainer::TeleopPeriodic()
{
	m_drivetrain.TankDriveVolts(5_V, 5_V);
}

void RobotContainer::AutonomousInit()
{
	m_gyro.Reset();
	m_drivetrain.ResetEncoders();
}

void RobotContainer::AutonomousPeriodic()
{
}

void RobotContainer::ConfigureControllerBindings() {}
