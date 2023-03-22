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

	m_controller.SetLeftAxisThreshold(0.2, 1.0);
	m_controller.SetRightAxisThreshold(0.2, 1.0);
	m_controller.SetLeftTriggerThreshold(0.2, 1.0);
	m_controller.SetRightTriggerThreshold(0.2, 1.0);

	ConfigureControllerBindings();
	ConfigureSubsystems();
}

void RobotContainer::RobotPeriodic()
{
}

void RobotContainer::ConfigureSubsystems()
{
	// ------------------- Drivetrain -------------------
	m_drivetrain.SetGyro(&m_gyro);
	m_drivetrain.SetPositionConversionFactor(DPR::DRIVETRAIN);

	// TODO : Configure PID
	m_drivetrain.ConfigureMovePID(PID::Move::P, PID::Move::I, PID::Move::D, PID::Move::TOLERANCE, true);
	m_drivetrain.ConfigureTurnPID(PID::Turn::P, PID::Turn::I, PID::Turn::D, PID::Turn::TOLERANCE);

	m_drivetrain.SetMaxSpeeds(Speed::DRIVETRAIN_MOVE, Speed::DRIVETRAIN_TURN);
	m_drivetrain.ConfigurePathFollower(Path::RAMSETE_B, Path::RAMSETE_ZETA, Path::KS, Path::KV, Path::KA, Path::KP, Path::KP);
	m_drivetrain.ResetPose();

	// ------------------- Arm -------------------
	m_arm.Configure();

	// ------------------- Intake -------------------
	// TODO : Check Max Speed
	m_intake.SetMaxSpeed(1);
	// TODO : Check inversion
	m_intake.Invert(false);

	// ------------------- Turret -------------------
	// TODO : Check Max Speed, PID and PCF
	m_turret.SetSparkMaxIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	m_turret.SetMaxSpeed(Speed::TURRET);
	m_turret.SetPositionConversionFactor(DPR::TURRET);
	m_turret.ConfigurePositionPID(PID::TurretAngle::P, PID::TurretAngle::I, PID::TurretAngle::D, PID::TurretAngle::TOLERANCE);

	// TODO : Check inversions
	m_turret.InvertMotor(false);
	m_turret.InvertEncoder(false);

	m_drivetrain.ConfigureBalancePID(PID::Balance::P, PID::Balance::I, PID::Balance::D, PID::Balance::TOLERANCE);

	// TODO : If PID works, check this
	// m_turret.SetMinMaxPosition(-10,190);
	// m_turret.SetPositionSafety(true);
}

Command *RobotContainer::GetAutonomousCommand()
{
	// Get the selected path from the SmartDashboard
	auto selectedPath = m_chooser.GetSelected();

	auto placeConeLeave = Sequence(

		// Set initial position
		InstantCommand([this]
					   { m_drivetrain.SetInitialPosition(Positions::INITIAL.x.value(), Positions::INITIAL.y.value()); }),

		// Place cone
		move(m_arm.SetPose(Arm::Poses::kConeHigh)),

		// While moving towards the item backwards, retract arm, then turn turret
		Parallel(
			Sequence(move(m_arm.SetPose(Arm::Poses::kTaxi)),
					 move(m_turret.SetPose(Turret::Poses::kBack))),
			move(m_drivetrain.MoveToCmd(Positions::PREPARE::PICK_ITEM_1.x.value(), Positions::PREPARE::PICK_ITEM_1.y.value(), 1, 1, true))));

	auto placeConePickBoxPlaceBox = Sequence(
		move(placeConeLeave),
		// Setup pickup
		move(m_arm.SetPose(Arm::Poses::kPickup)),

		// Move backwards towards item while taking
		Run([this]
			{ m_intake.Take(); })
			.Repeatedly()
			.RaceWith(move(m_drivetrain.MoveToCmd(Positions::PICK::ITEM_1.x.value(), Positions::PICK::ITEM_1.y.value(), 1, 1, true))),

		// While moving back, retract arm and turn turret
		Parallel(
			Sequence(move(m_arm.SetPose(Arm::Poses::kTaxi)), move(m_turret.SetPose(Turret::Poses::kFront))),
			move(m_drivetrain.MoveToCmd(Positions::PLACE::BOX_L.x.value(), Positions::PLACE::BOX_L.y.value(), 1, 1))),

		// Move arm to high box pose
		move(m_arm.SetPose(Arm::Poses::kBoxHigh)),

		// Place box
		move(m_intake.SpitCmd(1)));

	auto placeConePickBoxPlaceBoxBalance = Sequence(
		move(placeConePickBoxPlaceBox),

		// While moving forward towards platform, retract arm
		Parallel(
			move(m_arm.SetPose(Arm::Poses::kTaxi)),
			move(m_drivetrain.MoveToCmd(Positions::PREPARE::BALANCE.x.value(), Positions::PREPARE::BALANCE.y.value(), 1, 1))),

		// Hop onto platform
		move(m_drivetrain.MoveToCmd(Positions::BALANCE.x.value(), Positions::BALANCE.y.value(), 1, 1)),

		// Balance
		move(m_drivetrain.BalanceRoll(1)));
		
	return nullptr;
}

Command *RobotContainer::GetPathFollowingCommand(string pathName)
{

	DifferentialDriveKinematics kinematics{Wheel::TRACK_WIDTH};

	fs::path deployDirectory = frc::filesystem::GetDeployDirectory();

	auto fileName = pathName + ".wpilib.json";

	deployDirectory = deployDirectory / "output" / fileName;

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
	double output = controller_a.GetLeftY();
	double rotation = controller_a.GetRightX();

	m_drivetrain.Drive(output, rotation * (output == 0 ? 0.5 : 1.0));

	m_turret.PrintPosition();

	m_arm.PrintAngles();
	m_arm.PrintPose();

	m_gyro.PrintAngles();
	

	if (controller_b.GetLeftStickButton()) {
		m_arm.m_elbow.SetMotor(-controller_b.GetLeftY() * 0.5);
	}

	if (controller_b.GetLeftStickButtonReleased()) {
		m_arm.m_elbow.SetMotor(0);
	}



	m_intake.SetMotor(controller_b.GetRightTriggerAxis() - controller_b.GetLeftTriggerAxis());

}

void RobotContainer::AutonomousInit()
{
	Reset();
	
	timer.Stop();
	timer.Reset();
	auto_flag1 = false;
	auto_flag2 = false;
	auto_flag3 = false;
	
}

void RobotContainer::AutonomousPeriodic()
{

	auto command1 = Sequence(
		SetArmPose(Arm::Poses::kBoxMiddle).WithTimeout(3_s)
	);

	auto command2 = Sequence(
		SetArmPose(Arm::Poses::kTaxi).WithTimeout(3_s)
	);

	auto command3 = Sequence(
		m_drivetrain.BalanceRoll(0.7)
	);

	
	if(!auto_flag1){
		auto_flag1 = true;
		timer.Reset();
		timer.Start();
		CommandScheduler::GetInstance().Schedule(move(command1));
	} else{

		if(timer.Get()>2.5_s && timer.Get()<3_s){
			m_intake.SetMotor(-0.5);
		} 
		
		if(timer.Get()>3_s && timer.Get()<5_s){
			m_intake.SetMotor(0);
			if(!auto_flag2){
				auto_flag2 = true;
				command1.Cancel();
				CommandScheduler::GetInstance().Schedule(move(command2));
			}
		}

		if(timer.Get()>5_s && timer.Get()<11_s){
			m_drivetrain.Drive(0.6,0.15);
			command2.Cancel();
		}


	}

}

void RobotContainer::ConfigureControllerBindings()
{

	/* ------------------- Right Bumper -------------------
	 * A - Cone Low
	 * B - Cone Middle
	 * Y - Cone High
	 * X - Pickup
	 */

	
	controller_b.RightBumper().
	operator&&(controller_b.A())
		.OnTrue(SetArmPose(Arm::Poses::kBoxLow));

	controller_b.RightBumper().
	operator&&(controller_b.B())
		.OnTrue(SetArmPose(Arm::Poses::kBoxMiddle));

	controller_b.RightBumper().
	operator&&(controller_b.Y())
		.OnTrue(SetArmPose(Arm::Poses::kBoxHigh));

	controller_b.RightBumper().
	operator&&(controller_b.X())
		.OnTrue(SetArmPose(Arm::Poses::kPickup));
		// .OnFalse(SetArmPose(Arm::Poses::kTaxi));


	/* ------------------- Left Bumper -------------------
	 * A - Box Low
	 * B - Box Middle
	 * Y - Box High
	 * X - Tray
	 */

	/*

	controller_b.LeftBumper().
	operator&&(controller_b.A())
		.OnTrue(SetArmPose(Arm::Poses::kConeLow));

	controller_b.LeftBumper().
	operator&&(controller_b.B())
		.OnTrue(SetArmPose(Arm::Poses::kConeMiddle));

	controller_b.LeftBumper().
	operator&&(controller_b.Y())
		.OnTrue(SetArmPose(Arm::Poses::kConeHigh));

	/*
	controller_b.LeftBumper().
	operator&&(controller_b.X())
		.OnTrue(SetArmPose(Arm::Poses::kTray))
		.OnFalse(SetArmPose(Arm::Poses::kTaxi));
	*/

	/* ------------------- POV -------------------
	 * Up - Front
	 * Down - Back
	 * Right - Side
	 */
	// m_controller.A().OnTrue(m_drivetrain.BalanceRoll(PID::Balance::SPEED));

	Trigger povUpTrigger([this]
						 { return controller_b.GetPOV() == 0; });

	Trigger povDownTrigger([this]
						   { return controller_b.GetPOV() == 180; });

	Trigger povRightTrigger([this]
							{ return controller_b.GetPOV() == 90; });

	povUpTrigger.OnTrue(SetTurretPose(Turret::Poses::kFront));

	povDownTrigger.OnTrue(SetTurretPose(Turret::Poses::kBack));

	povRightTrigger.OnTrue(SetTurretPose(Turret::Poses::kSide));

	// Return to Taxi with start
	controller_b.Start().OnTrue(SetArmPose(Arm::Poses::kTaxi));

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

	bool isPlacing = pose >= Arm::Poses::kConeLow &&
					 pose <= Arm::Poses::kBoxHigh;

	bool isPickup = pose == Arm::Poses::kPickup ||
					pose == Arm::Poses::kTray;

	if (isPlacing)
	{
		double spitSpeed = pose <= Arm::Poses::kConeHigh ? Speed::CONE_SPIT : Speed::BOX_SPIT;

		return Sequence(
			// Move towards position
			m_arm.SetPose(pose));
			
			/*,

			// Spit object
			m_intake.SpitCmd(spitSpeed),

			// Return to taxi
			m_arm.SetPose(Arm::Poses::kTaxi));*/
	}
	else if (isPickup)
	{
		return Sequence(
			// Move towards position
			m_arm.SetPose(pose));

			// Intake object, it will run repeatedly until interrupted (when the button is released)
			//m_intake.TakeCmd(Speed::INTAKE).Repeatedly());
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