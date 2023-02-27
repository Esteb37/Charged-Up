// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer()
{
}

void RobotContainer::RobotInit()
{
	m_gyro.Calibrate();
	ConfigureControllerBindings();

	m_controller.SetLeftAxisThreshold(0.2, 1.0);
	m_controller.SetRightAxisThreshold(0.2, 1.0);

	ConfigureSubsystems();
}

void RobotContainer::RobotPeriodic()
{
}

void RobotContainer::ConfigureSubsystems()
{
	m_drivetrain.SetGyro(&m_gyro);

	m_drivetrain.SetPositionConversionFactor(DPR::DRIVETRAIN);

	m_drivetrain.ConfigureMovePID(PID::Move::P, PID::Move::I, PID::Move::D, PID::Move::TOLERANCE, true);

	m_drivetrain.ConfigureTurnPID(PID::Turn::P, PID::Turn::I, PID::Turn::D, PID::Turn::TOLERANCE);

	m_drivetrain.SetMaxSpeeds(Speed::DRIVETRAIN_MOVE, Speed::DRIVETRAIN_TURN);

	m_drivetrain.ConfigurePathFollower(Path::RAMSETE_B, Path::RAMSETE_ZETA, Path::KS, Path::KV, Path::KA, Path::KP, Path::KP);

	m_drivetrain.ResetPose();

	m_arm.Configure();

	m_intake.SetMaxSpeed(Speed::INTAKE);

	m_turret.SetMaxSpeed(Speed::TURRET);

	m_turret.SetPositionConversionFactor(DPR::TURRET);

	m_turret.ConfigurePositionPID(PID::TurretAngle::P, PID::TurretAngle::I, PID::TurretAngle::D, PID::TurretAngle::TOLERANCE);

	// m_turret.SetMinMaxPosition(-10,190);
	// m_turret.SetPositionSafety(true);

	m_turret.SetName("Turret");

	m_turret.SetSparkMaxIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

Command *RobotContainer::GetAutonomousCommand()
{

	DifferentialDriveKinematics kinematics{Wheel::TRACK_WIDTH};

	fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
	deployDirectory = deployDirectory / "output" / "auto.wpilib.json";
	auto trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

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
		PIDController{Path::KP, 0, 0},
		PIDController{Path::KP, 0, 0},
		[this](auto left, auto right)
		{ m_drivetrain.TankDriveVolts(left, right); },
		{&m_drivetrain}};

	// Reset odometry to the starting pose of the trajectory.
	m_drivetrain.SetPose(trajectory.InitialPose());

	// no auto
	return new SequentialCommandGroup(
		move(ramseteCommand),
		InstantCommand([this]
					   { m_drivetrain.TankDriveVolts(0_V, 0_V); },
					   {}));
}

void RobotContainer::TeleopInit()
{
}
void RobotContainer::TeleopPeriodic()
{
	double output = m_controller.GetTriggerDifference();
	double rotation = m_controller.GetRightX();

	m_drivetrain.Drive(output, rotation * (output == 0 ? 0.5 : 1.0));
}

void RobotContainer::AutonomousInit()
{
	Reset();
}

void RobotContainer::AutonomousPeriodic()
{
}

void RobotContainer::ConfigureControllerBindings()
{

	m_controller.RightBumper().
	operator&&(m_controller.A())
		.OnTrue(SetArmPose(Arm::Poses::kConeLow));

	m_controller.RightBumper().
	operator&&(m_controller.B())
		.OnTrue(SetArmPose(Arm::Poses::kConeMiddle));

	m_controller.RightBumper().
	operator&&(m_controller.Y())
		.OnTrue(SetArmPose(Arm::Poses::kConeHigh));

	m_controller.RightBumper().
	operator&&(m_controller.X())
		.WhileTrue(SetArmPose(Arm::Poses::kPickup))
		.OnFalse(SetArmPose(Arm::Poses::kTaxi));

	// Left Bumper
	m_controller.LeftBumper().
	operator&&(m_controller.A())
		.OnTrue(SetArmPose(Arm::Poses::kBoxLow));

	m_controller.LeftBumper().
	operator&&(m_controller.B())
		.OnTrue(SetArmPose(Arm::Poses::kBoxMiddle));

	m_controller.LeftBumper().
	operator&&(m_controller.Y())
		.OnTrue(SetArmPose(Arm::Poses::kBoxHigh));

	m_controller.LeftBumper().
	operator&&(m_controller.X())
		.WhileTrue(SetArmPose(Arm::Poses::kTray))
		.OnFalse(SetArmPose(Arm::Poses::kTaxi));

	m_controller.Start().OnTrue(SetArmPose(Arm::Poses::kTaxi));

	Trigger povUpTrigger([this]
						 { return m_controller.GetPOV() == 0; });

	Trigger povDownTrigger([this]
						   { return m_controller.GetPOV() == 180; });

	Trigger povRightTrigger([this]
							{ return m_controller.GetPOV() == 90; });

	povUpTrigger.OnTrue(SetTurretPose(Turret::Poses::kFront));

	povDownTrigger.OnTrue(SetTurretPose(Turret::Poses::kBack));

	povRightTrigger.OnTrue(SetTurretPose(Turret::Poses::kSide));
}

void RobotContainer::Reset()
{
	m_gyro.Reset();
	m_drivetrain.ResetEncoders();
	m_arm.ResetEncoders();
	m_turret.ResetEncoder();
}

CommandPtr RobotContainer::GetArmPoseCmd(Arm::Poses pose)
{
	// Cancel movement if any, start new movement towards pose

	bool isPlacing = pose >= Arm::Poses::kConeLow &&
					 pose <= Arm::Poses::kBoxHigh;

	bool isPickup = pose == Arm::Poses::kPickup ||
					pose == Arm::Poses::kTray;

	if (isPlacing)
	{
		double spitSpeed = pose <= Arm::Poses::kConeHigh ? Speed::CONE_SPIT : Speed::BOX_SPIT;

		return Sequence(
			// Move towards position
			m_arm.SetPose(pose),

			// Spit object
			m_intake.SpitCmd(spitSpeed),

			m_arm.SetPose(Arm::Poses::kTaxi));
	}
	else if (isPickup)
	{
		return Sequence(
			// Move towards position
			m_arm.SetPose(pose),
			// Intake object, it will run repeatedly until interrupted (when the button is released)
			m_intake.TakeCmd(Speed::INTAKE).Repeatedly());
	}

	return m_arm.SetPose(pose);
}

CommandPtr RobotContainer::SetArmPose(Arm::Poses pose)
{
	return frc2::InstantCommand([this, pose]
								{
		m_currentArmCommand.Cancel();
		m_currentArmCommand = GetArmPoseCmd(pose);
		m_currentArmCommand.Schedule(); })
		.ToPtr();
}

CommandPtr RobotContainer::SetTurretPose(Turret::Poses pose)
{
	return frc2::InstantCommand([this, pose]
								{
		m_currentTurretCommand.Cancel();
		m_currentTurretCommand = m_turret.SetPose(pose);
		m_currentTurretCommand.Schedule(); })
		.ToPtr();
}