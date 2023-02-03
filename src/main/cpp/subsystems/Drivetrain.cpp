/*------------------------------------------------------------
						&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& /&&&&,
					.&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& /&&&&&&&&%
				.&&&&/ &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& /&&&&&&&&&&&*
			.%&&&(      &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&% %&&&&&&&&&&&&&
		%&&&#     %#&&#           (&&&&&&&&&&&              %&&&&&&&&&&&&&
	*&&&#                          (&&&&&&&&&&&    /           %&&&&&&&&&&&
*&%  ,                           (&&&&&&&&&&&(&&&&(           &&&&&&&&&&&
& (,..                          (&&&&&&&&&&&&&&&&            %&&&&&&&&&&
	&*                             (&&&&&&&&&&&&&&&&            &&&&&&&&&&&
	&/                             (&&&&&&&&&&&&&&&&%          &&&&&&&&&&&(
	#&&    .&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&#**(&&&&&&&&&&&&&#
	&#  (&                        ......... &&&&&&&&&&&&&&&&&&&&&&&&&&
	/&   &                                   .&&&&&&&&&&&&&&&&&&&&&&
		%&&* &*                                   ,%&&&&&&&&&&&&&%*

	Author: Esteban Padilla Cerdio
	Email: esteban37padilla@gmail.com
	URL: github.com/esteb37
		 github.com/tecdroid-3354
	Date: 12/04/2022
	Language: cpp
	Copyright (c) TecDroid 3354 and Esteban Padilla Cerdio
	Open Source Software; you can modify and/or share it under the terms of
*/

#include "subsystems/Drivetrain.h"
#include <cmath>
#include <typeinfo>

namespace TD
{

	using namespace DrivetrainTypes;

	template <>
	double Drivetrain<NEO>::GetRightEncoders()
	{
		return m_rightEncodersDirection * (m_frontRightEncoder->GetPosition() + m_backRightEncoder->GetPosition()) / 2;
	}

	template <>
	double Drivetrain<CLASSIC>::GetRightEncoders()
	{
		if (m_rightEncoder == nullptr)
			return 0.0;

		return m_rightEncodersDirection * m_rightEncoder->GetDistance();
	}

	template <>
	double Drivetrain<NEO>::GetLeftEncoders()
	{
		return m_leftEncodersDirection * (m_frontLeftEncoder->GetPosition() + m_backLeftEncoder->GetPosition()) / 2;
	}

	template <>
	double Drivetrain<CLASSIC>::GetLeftEncoders()
	{
		if (m_leftEncoder == nullptr)
			return 0.0;

		return m_leftEncodersDirection * m_leftEncoder->GetDistance();
	}

	template <>
	Drivetrain<NEO>::Drivetrain(unsigned int frontRight, unsigned int frontLeft, unsigned int backRight, unsigned int backLeft)
	{
		SetName("Drivetrain");

		m_frontRight = new CANSparkMax(frontRight, CANSparkMaxLowLevel::MotorType::kBrushless);
		m_frontLeft = new CANSparkMax(frontLeft, CANSparkMaxLowLevel::MotorType::kBrushless);
		m_backRight = new CANSparkMax(backRight, CANSparkMaxLowLevel::MotorType::kBrushless);
		m_backLeft = new CANSparkMax(backLeft, CANSparkMaxLowLevel::MotorType::kBrushless);

		// initialize encoders
		m_frontRightEncoder = new SparkMaxRelativeEncoder(m_frontRight->GetEncoder());
		m_frontLeftEncoder = new SparkMaxRelativeEncoder(m_frontLeft->GetEncoder());
		m_backRightEncoder = new SparkMaxRelativeEncoder(m_backRight->GetEncoder());
		m_backLeftEncoder = new SparkMaxRelativeEncoder(m_backLeft->GetEncoder());

		// initialize motorcontrollergroups
		m_right = new MotorControllerGroup(*m_frontRight, *m_backRight);
		m_left = new MotorControllerGroup(*m_frontLeft, *m_backLeft);

		// initialize drivetrain
		m_drive = new DifferentialDrive(*m_left, *m_right);

		InvertLeft();
		InvertRightEncoders();
	}

	template <>
	Drivetrain<CLASSIC>::Drivetrain(unsigned int frontRight, unsigned int frontLeft, unsigned int backRight, unsigned int backLeft)
	{
		SetName("Drivetrain");

		m_frontRight = new VictorSP(frontRight);
		m_frontLeft = new VictorSP(frontLeft);
		m_backRight = new VictorSP(backRight);
		m_backLeft = new VictorSP(backLeft);

		// initialize motorcontrollergroups
		m_right = new MotorControllerGroup(*m_frontRight, *m_backRight);
		m_left = new MotorControllerGroup(*m_frontLeft, *m_backLeft);

		// initialize drivetrain
		m_drive = new DifferentialDrive(*m_left, *m_right);

		InvertLeft();
		InvertRightEncoders();
	}

	template <>
	Drivetrain<NEO>::Drivetrain(unsigned int frontRight, unsigned int frontLeft, unsigned int backRight, unsigned int backLeft, unsigned int encoderRightA, unsigned int encoderRightB, unsigned int encoderLeftA, unsigned int encoderLeftB)
	{
		assert(false && "NEO drivetrains cannot have FRC encoders");
	}

	template <>
	Drivetrain<CLASSIC>::Drivetrain(unsigned int frontRight, unsigned int frontLeft, unsigned int backRight, unsigned int backLeft, unsigned int encoderRightA, unsigned int encoderRightB, unsigned int encoderLeftA, unsigned int encoderLeftB)
	{
		SetName("Drivetrain");

		m_frontRight = new VictorSP(frontRight);
		m_frontLeft = new VictorSP(frontLeft);
		m_backRight = new VictorSP(backRight);
		m_backLeft = new VictorSP(backLeft);

		// initialize encoders
		m_rightEncoder = new Encoder(encoderRightA, encoderRightB);
		m_leftEncoder = new Encoder(encoderLeftA, encoderLeftB);

		// initialize motorcontrollergroups
		m_right = new MotorControllerGroup(*m_frontRight, *m_backRight);
		m_left = new MotorControllerGroup(*m_frontLeft, *m_backLeft);

		// initialize drivetrain
		m_drive = new DifferentialDrive(*m_left, *m_right);

		InvertLeft();
		InvertRightEncoders();
	}

	template <class T>
	void Drivetrain<T>::Periodic()
	{
		UpdatePose();
	}

	// --------------------- Control ----------------------
	template <class T>
	void Drivetrain<T>::Drive(double speed, double rotation)
	{
		m_drive->ArcadeDrive(speed * m_moveDirection * m_maxMoveSpeed, rotation * m_rotationDirection * m_maxTurnSpeed);
	}

	template <typename T>
	void Drivetrain<T>::AutoDrive()
	{
		m_drive->ArcadeDrive(m_autoMoveOutput, m_autoTurnOutput);
	}

	template <typename T>
	void Drivetrain<T>::ResetSensors()
	{
		m_gyro->Reset();
		ResetEncoders();
	}
	template <class T>
	void Drivetrain<T>::SetMaxSpeeds(double speed, double rotation)
	{
		m_maxMoveSpeed = speed;
		m_maxTurnSpeed = rotation;
	}
	template <class T>
	void Drivetrain<T>::InvertMove(bool invert)
	{
		m_moveDirection = invert ? -1 : 1;
	}
	template <class T>
	void Drivetrain<T>::InvertRotation(bool invert)
	{
		m_rotationDirection = invert ? -1 : 1;
	}
	template <class T>
	void Drivetrain<T>::SetSafetyEnabled(bool enabled)
	{
		m_drive->SetSafetyEnabled(enabled);
	}

	// ---------------------- Motors ----------------------
	template <>
	void Drivetrain<NEO>::ResetMotors()
	{
		m_frontRight->RestoreFactoryDefaults();
		m_frontLeft->RestoreFactoryDefaults();
		m_backRight->RestoreFactoryDefaults();
		m_backLeft->RestoreFactoryDefaults();
	}

	template <class T>
	void Drivetrain<T>::ResetMotors()
	{
	}

	template <class T>
	void Drivetrain<T>::InvertRight(bool invert)
	{
		m_right->SetInverted(invert);
	}

	template <class T>
	void Drivetrain<T>::InvertLeft(bool invert)
	{
		m_left->SetInverted(invert);
	}

	template <class T>
	void Drivetrain<T>::PrintMotors()
	{
		SmartDashboard::PutNumber(GetName() + " FR Motor", m_frontRight->Get());
		SmartDashboard::PutNumber(GetName() + " FL Motor", m_frontLeft->Get());
		SmartDashboard::PutNumber(GetName() + " BR Motor", m_backRight->Get());
		SmartDashboard::PutNumber(GetName() + " BL Motor", m_backLeft->Get());
	}

	// --------------------- Encoders ---------------------

	template <class T>
	double Drivetrain<T>::GetRightEncodersTotal()
	{
		return m_rightEncodersTotal + GetRightEncoders();
	}

	template <class T>
	double Drivetrain<T>::GetLeftEncodersTotal()
	{
		return m_leftEncodersTotal + GetLeftEncoders();
	}

	template <class T>
	double Drivetrain<T>::GetEncoderAverage()
	{
		return (GetRightEncoders() + GetLeftEncoders()) / 2;
	}

	template <>
	void Drivetrain<NEO>::ResetEncoders()
	{

		m_rightEncodersTotal = GetRightEncodersTotal();
		m_leftEncodersTotal = GetLeftEncodersTotal();

		m_frontRightEncoder->SetPosition(0);
		m_frontLeftEncoder->SetPosition(0);
		m_backRightEncoder->SetPosition(0);
		m_backLeftEncoder->SetPosition(0);
	}

	template <>
	void Drivetrain<CLASSIC>::ResetEncoders()
	{

		m_rightEncodersTotal = GetRightEncodersTotal();
		m_leftEncodersTotal = GetLeftEncodersTotal();

		m_rightEncoder->Reset();
		m_leftEncoder->Reset();
	}

	template <class T>
	void Drivetrain<T>::InvertRightEncoders(bool invert)
	{
		m_rightEncodersDirection = invert ? -1 : 1;
	}

	template <class T>
	void Drivetrain<T>::InvertLeftEncoders(bool invert)
	{
		m_leftEncodersDirection = invert ? -1 : 1;
	}

	template <class T>
	void Drivetrain<T>::PrintEncoders()
	{
		// Added this because SmartDashboard doesn't work with specified templates for some reason
		if (is_same<T, NEO>::value)
		{
			SmartDashboard::PutNumber(GetName() + " FR Encoder", m_frontRightEncoder->GetPosition() * m_rightEncodersDirection);
			SmartDashboard::PutNumber(GetName() + " FL Encoder", m_frontLeftEncoder->GetPosition() * m_leftEncodersDirection);
			SmartDashboard::PutNumber(GetName() + " BR Encoder", m_backRightEncoder->GetPosition() * m_rightEncodersDirection);
			SmartDashboard::PutNumber(GetName() + " BL Encoder", m_backLeftEncoder->GetPosition() * m_leftEncodersDirection);
			SmartDashboard::PutNumber(GetName() + " Encoder Average", GetEncoderAverage());
		}
		else
		{
			SmartDashboard::PutNumber(GetName() + " Right Encoder", m_rightEncoder->GetDistance() * m_rightEncodersDirection);
			SmartDashboard::PutNumber(GetName() + " Left Encoder", m_leftEncoder->GetDistance() * m_leftEncodersDirection);
			SmartDashboard::PutNumber(GetName() + " Encoder Average", GetEncoderAverage());
			SmartDashboard::PutNumber(GetName() + " Encoder Average", GetEncoderAverage());
		}
	}

	template <>
	void Drivetrain<NEO>::SetPositionConversionFactor(double pcf_meters)
	{
		m_backRightEncoder->SetPositionConversionFactor(pcf_meters);
		m_backLeftEncoder->SetPositionConversionFactor(pcf_meters);
		m_frontRightEncoder->SetPositionConversionFactor(pcf_meters);
		m_frontLeftEncoder->SetPositionConversionFactor(pcf_meters);
	}

	template <>
	void Drivetrain<CLASSIC>::SetPositionConversionFactor(double pcf_meters)
	{
		m_rightEncoder->SetDistancePerPulse(pcf_meters);
		m_leftEncoder->SetDistancePerPulse(pcf_meters);
	}

	// ----------------------- Gyro -----------------------
	template <class T>
	void Drivetrain<T>::SetGyro(CustomGyro<GyroTypes::NAVX> *gyro)
	{
		m_gyro = gyro;
	}

	template <class T>
	void Drivetrain<T>::SetGyro(CustomGyro<ADIS16448_IMU> *gyro)
	{
		m_gyro = gyro;
	}

	// ----------------------- Auto -----------------------

	template <class T>
	bool Drivetrain<T>::Move(double distance, double speed)
	{
		m_movePIDController.SetSetpoint(distance);
		double output = m_movePIDController.Calculate(GetEncoderAverage());
		output = clamp(output, -1.0, 1.0) * speed * m_movePIDDirection;
		m_autoMoveOutput = output;
		AutoDrive();
		return m_movePIDController.AtSetpoint();
	}

	template <class T>
	void Drivetrain<T>::ResetMovePIDController()
	{
		m_autoMoveOutput = 0;
		m_movePIDController.Reset();
	}

	template <class T>
	void Drivetrain<T>::ConfigureMovePID(double p, double i, double d, double tolerance, bool inverted)
	{
		m_movePIDController.SetPID(p, i, d);
		m_movePIDController.SetTolerance(tolerance);
		m_movePIDDirection = inverted ? -1 : 1;
	}
	template <class T>
	void Drivetrain<T>::PrintMoveError()
	{
		SmartDashboard::PutNumber(GetName() + " Move Error", m_movePIDController.GetPositionError());
	}

	template <class T>
	bool Drivetrain<T>::Turn(double angle, double speed)
	{
		m_turnPIDController.SetSetpoint(angle);
		double output = m_turnPIDController.Calculate(m_gyro->GetAngle().value());
		output = clamp(output, -1.0, 1.0) * speed * m_turnPIDDirection;
		m_autoTurnOutput = output;
		AutoDrive();
		return m_turnPIDController.AtSetpoint();
	}

	template <class T>
	void Drivetrain<T>::ResetTurnPIDController()
	{
		m_autoTurnOutput = 0;
		m_turnPIDController.Reset();
	}

	template <class T>
	void Drivetrain<T>::ConfigureTurnPID(double p, double i, double d, double tolerance, bool inverted)
	{
		m_turnPIDController.SetPID(p, i, d);
		m_turnPIDController.SetTolerance(tolerance);
		m_turnPIDDirection = inverted ? -1 : 1;
	}
	template <class T>
	void Drivetrain<T>::PrintTurnError()
	{
		SmartDashboard::PutNumber(GetName() + " Turn Error", m_turnPIDController.GetPositionError());
	}

	template <class T>
	bool Drivetrain<T>::MoveTo(double x, double y, double speed, double turnSpeed)
	{
		double targetX = x - m_currentX;
		double targetY = y - m_currentY;

		double angle = GetAbsoluteAngle(targetX, targetY);

		double distance = sqrt(pow(targetX, 2) + pow(targetY, 2));

		bool finished = false;

		if (!m_reachedAngle)
		{
			if (Turn(angle, turnSpeed))
			{
				m_reachedAngle = true;
				ResetEncoders();
			}
		}
		else
		{
			if (Move(distance, speed))
			{
				ResetEncoders();
				m_currentX = x;
				m_currentY = y;
				finished = true;
				m_reachedAngle = false;
			}
		}

		return finished;
	}

	template <class T>
	void Drivetrain<T>::PrintMoveToError()
	{
		PrintMoveError();
		PrintTurnError();
	}

	template <class T>
	void Drivetrain<T>::PrintCurrentPosition()
	{
		SmartDashboard::PutNumber(GetName() + " Current X", m_currentX);
		SmartDashboard::PutNumber(GetName() + " Current Y", m_currentY);
	}

	template <class T>
	bool Drivetrain<T>::SetAngleWithTarget(double angle, double speed)
	{
		m_alignPIDController.SetSetpoint(angle);
		double output = m_alignPIDController.Calculate(0); // m_limelight.GetHorizontalAngle()l* m_alignPIDDirection);
		output = clamp(output, -1.0, 1.0);
		Drive(0, output * speed);
		return m_alignPIDController.AtSetpoint();
	}

	template <class T>
	bool Drivetrain<T>::AlignWithTarget(double speed)
	{
		return SetAngleWithTarget(0, speed);
	}

	template <class T>
	void Drivetrain<T>::ResetAlignPIDController()
	{
		m_alignPIDController.Reset();
	}

	template <class T>
	void Drivetrain<T>::ConfigureAlignPID(double p, double i, double d, double tolerance, bool inverted)
	{
		m_alignPIDController.SetPID(p, i, d);
		m_alignPIDController.SetTolerance(tolerance);
		m_alignPIDDirection = inverted ? -1 : 1;
	}

	template <class T>
	void Drivetrain<T>::PrintAlignError()
	{
		SmartDashboard::PutNumber(GetName() + " Align Error", m_alignPIDController.GetPositionError());
	}

	template <class T>
	bool Drivetrain<T>::SetDistanceWithTarget(double objectiveHeight, double distance, double speed)
	{
		m_distancePIDController.SetSetpoint(distance);
		double output = m_distancePIDController.Calculate(0); // m_limelight.GetDistanceToTarget(objectiveHeight) * m_distancePIDDirection);
		output = clamp(output, -1.0, 1.0);
		Drive(output * speed, 0);
		return m_distancePIDController.AtSetpoint();
	}

	template <class T>
	void Drivetrain<T>::ResetDistancePIDController()
	{
		m_distancePIDController.Reset();
	}

	template <class T>
	void Drivetrain<T>::ConfigureDistancePID(double p, double i, double d, double tolerance, bool inverted)
	{
		m_distancePIDController.SetPID(p, i, d);
		m_distancePIDController.SetTolerance(tolerance);
		m_distancePIDDirection = inverted ? -1 : 1;
	}

	template <class T>
	void Drivetrain<T>::PrintSetDistanceError()
	{
		SmartDashboard::PutNumber(GetName() + " Set Distance Error", m_distancePIDController.GetPositionError());
	}

	template <class T>
	void Drivetrain<T>::ResetPIDControllers()
	{
		ResetMovePIDController();
		ResetTurnPIDController();
		ResetAlignPIDController();
		ResetDistancePIDController();
	}

	template <class T>
	double Drivetrain<T>::GetAbsoluteAngle(double x, double y)
	{
		double relAngle = atan(y / (x == 0 ? 0.01 : x));

		if (x < 0)
			relAngle += M_PI;

		else if (y < 0)
			relAngle += 2 * M_PI;

		return (relAngle)*180 / M_PI;
	}

	template <class T>
	void Drivetrain<T>::SetPose(Pose2d startingPosition)
	{
		ResetEncoders();
		m_odometry.ResetPosition(m_gyro->GetRotation2d(), units::meter_t(GetLeftEncodersTotal()), units::meter_t(GetRightEncodersTotal()), startingPosition);
		m_field.SetRobotPose(startingPosition);
	}

	template <class T>
	void Drivetrain<T>::ResetPose()
	{
		ResetEncoders();
		m_odometry.ResetPosition(m_gyro->GetRotation2d(), 0_m, 0_m, Pose2d{});
		m_field.SetRobotPose(Pose2d{});
	}

	template <class T>
	void Drivetrain<T>::UpdatePose()
	{
		m_odometry.Update(m_gyro->GetRotation2d(),
						  units::meter_t(GetLeftEncodersTotal()),
						  units::meter_t(GetRightEncodersTotal()));

		m_field.SetRobotPose(m_odometry.GetPose());
	}
	template <class T>
	Pose2d Drivetrain<T>::GetPose()
	{
		return m_odometry.GetPose();
	}

	template <class T>
	void Drivetrain<T>::PrintPose()
	{
		SmartDashboard::PutNumber(GetName() + " X", m_odometry.GetPose().Translation().X().value());
		SmartDashboard::PutNumber(GetName() + " Y", m_odometry.GetPose().Translation().Y().value());
		SmartDashboard::PutNumber(GetName() + " Angle", m_odometry.GetPose().Rotation().Degrees().value());
		SmartDashboard::PutData("Field", &m_field);
	}

	template <>
	DifferentialDriveWheelSpeeds Drivetrain<NEO>::GetWheelSpeeds()
	{
		return {m_leftEncodersDirection * units::meters_per_second_t(m_frontLeftEncoder->GetVelocity()),
				m_rightEncodersDirection * units::meters_per_second_t(m_frontRightEncoder->GetVelocity())};
	}

	template <>
	DifferentialDriveWheelSpeeds Drivetrain<CLASSIC>::GetWheelSpeeds()
	{
		return {m_leftEncodersDirection * units::meters_per_second_t(m_leftEncoder->GetRate()),
				m_rightEncodersDirection * units::meters_per_second_t(m_rightEncoder->GetRate())};
	}

	template <class T>
	void Drivetrain<T>::TankDriveVolts(units::volt_t left, units::volt_t right)
	{
		SmartDashboard::PutNumber("Right Speed", m_rightEncoder->GetRate());
		SmartDashboard::PutNumber("Left Speed", m_leftEncoder->GetRate());
		m_right->SetVoltage(-m_moveDirection * right);
		m_left->SetVoltage(-m_moveDirection * left);
		m_drive->Feed();
	}

	template <class T>
	void Drivetrain<T>::ConfigurePathFollower(units::unit_t<b_unit> b,
											  units::unit_t<zeta_unit> z,
											  units::volt_t ks,
											  units::unit_t<kv_unit> kv,
											  units::unit_t<ka_unit> ka, double leftP, double rightP)
	{
		m_pathB = b;
		m_pathZeta = z;
		m_pathKs = ks;
		m_pathKv = kv;
		m_pathKa = ka;
		m_pathLeftP = leftP;
		m_pathRightP = rightP;
	}

	template class Drivetrain<NEO>;
	template class Drivetrain<CLASSIC>;
}