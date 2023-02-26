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

frc2::Command *RobotContainer::GetAutonomousCommand()
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
}
void RobotContainer::TeleopPeriodic()
{
	double output = m_controller.GetTriggerDifference();
	double rotation = m_controller.GetRightX();

	m_drivetrain.Drive(output, rotation * (output == 0 ? 0.5 : 1.0));

	// arm.SetMotor(mc_controller.AxisYLeft()/5);
	// shoulder.SetMotor(mc_controller.AxisYRight()/5);
	// arm.PrintPosition();
	// shoulder.PrintPosition();
	// intake1.SetMotor(mc_controller.TriggerRight()-mc_controller.TriggerLeft());
	// intake2.SetMotor(-(mc_controller.TriggerRight()-mc_controller.TriggerLeft()));
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

	m_controller.Y().OnTrue(GetArmPoseCmd(Arm::Poses::kConeHigh));

	m_controller.A().OnTrue(GetArmPoseCmd(Arm::Poses::kConeLow));

	m_controller.X().OnTrue(GetArmPoseCmd(Arm::Poses::kHome));

	m_controller.B().OnTrue(GetArmPoseCmd(Arm::Poses::kConeMiddle));
}

void RobotContainer::Reset()
{
	m_gyro.Reset();
	m_drivetrain.ResetEncoders();
	m_arm.ResetEncoders();
	m_turret.ResetEncoder();
}

frc2::CommandPtr RobotContainer::GetArmPoseCmd(Arm::Poses pose)
{
	return frc2::InstantCommand([this, pose]
								{
		m_currentCommand.Cancel();
		m_currentCommand = std::move(m_arm.SetPose(pose));
		m_currentCommand.Schedule(); })
		.ToPtr();
}