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

#include "subsystems/EncoderSubsystemBase.h"

using namespace TD;

EncoderSubsystemBase::EncoderSubsystemBase()
{
	SetName("EncoderSubsystemBase");
}

EncoderSubsystemBase &EncoderSubsystemBase::GetInstance()
{
	static EncoderSubsystemBase instance;
	return instance;
}

void EncoderSubsystemBase::Initialize(MotorConfig motorConfig, unsigned int motorPort)
{
	MotorSubsystemBase::Initialize(motorConfig, motorPort);
	SetName("EncoderSubsystemBase");
}

void EncoderSubsystemBase::Initialize(MotorConfig motorConfig, vector<unsigned int> motorPorts)
{
	MotorSubsystemBase::Initialize(motorConfig, motorPorts);
	SetName("EncoderSubsystemBase");
}

void EncoderSubsystemBase::Initialize(MotorConfig motorConfig, EncoderConfig encoderConfig, unsigned int motorPort)
{
	MotorSubsystemBase::Initialize(motorConfig, motorPort);

	m_encoderConfig = encoderConfig;

	if (motorConfig == MotorConfig::NEO && encoderConfig == EncoderConfig::REV)
	{
		m_encoderSpark = new SparkMaxRelativeEncoder(m_motorSpark->GetEncoder());
	}
	else
	{
		throw std::invalid_argument("Encoder must be Spark and Motor must be Neo");
	}

	SetName("EncoderSubsystemBase");
}

void EncoderSubsystemBase::Initialize(MotorConfig motorConfig, EncoderConfig encoderConfig, vector<unsigned int> motorPorts)
{

	MotorSubsystemBase::Initialize(motorConfig, motorPorts);

	m_encoderConfig = encoderConfig;

	if (motorConfig == MotorConfig::NEO && encoderConfig == EncoderConfig::REV)
	{
		m_encoderSpark = new SparkMaxRelativeEncoder(m_motorSparkList[0]->GetEncoder());
	}
	else
	{
		throw std::invalid_argument("Encoder must be Spark and Motor must be Neo");
	}

	SetName("EncoderSubsystemBase");
}

void EncoderSubsystemBase::Initialize(MotorConfig motorConfig, EncoderConfig encoderConfig, unsigned int motorPort, unsigned int encoderA, unsigned int encoderB)
{

	MotorSubsystemBase::Initialize(motorConfig, motorPort);

	if (encoderConfig == EncoderConfig::REV)
	{
		throw std::invalid_argument("Encoder must be FRC");
	}

	m_encoderConfig = encoderConfig;

	m_encoder = new Encoder(encoderA, encoderB, false, Encoder::EncodingType::k4X);

	SetName("EncoderSubsystemBase");
}

void EncoderSubsystemBase::Initialize(MotorConfig motorConfig, EncoderConfig encoderConfig, vector<unsigned int> motorPorts, unsigned int encoderA, unsigned int encoderB)
{

	MotorSubsystemBase::Initialize(motorConfig, motorPorts);

	if (encoderConfig == EncoderConfig::REV)
	{
		throw std::invalid_argument("Encoder must be FRC");
	}

	m_encoderConfig = encoderConfig;

	m_encoder = new Encoder(encoderA, encoderB, false, Encoder::EncodingType::k4X);

	SetName("EncoderSubsystemBase");
}

void EncoderSubsystemBase::Periodic()
{
}

void EncoderSubsystemBase::Reset()
{
	ResetEncoder();
	ResetPositionPID();
	ResetRPMPID();
}

void EncoderSubsystemBase::SetMotor(double speed)
{

	if (m_positionSafetyActive)
	{
		if (GetPosition() >= m_maxPosition)
		{
			speed = fmin(speed, 0);
		}
		else if (GetPosition() <= m_minPosition)
		{
			speed = fmax(speed, 0);
		}
	}

	MotorSubsystemBase::SetMotor(speed);
}

void EncoderSubsystemBase::SetMotors(double speed)
{

	if (m_positionSafetyActive)
	{
		if (GetPosition() >= m_maxPosition)
		{
			speed = fmin(speed, 0);
		}
		else if (GetPosition() <= m_minPosition)
		{
			speed = fmax(speed, 0);
		}
	}

	MotorSubsystemBase::SetMotors(speed);
}

void EncoderSubsystemBase::SetMotors(vector<double> speeds)
{
	for (unsigned i = 0; i < speeds.size(); i++)
	{
		if (m_positionSafetyActive)
		{
			if (GetPosition() >= m_maxPosition)
			{
				speeds[i] = fmin(speeds[i], 0);
			}
			else if (GetPosition() <= m_minPosition)
			{
				speeds[i] = fmax(speeds[i], 0);
			}
		}
	}

	MotorSubsystemBase::SetMotors(speeds);
}

void EncoderSubsystemBase::ResetEncoder()
{
	switch (m_encoderConfig)
	{
	case EncoderConfig::REV:
		m_encoderSpark->SetPosition(0);
		break;
	case EncoderConfig::FRC:
	default:
		m_encoder->Reset();
		break;
	}
}

void EncoderSubsystemBase::InvertEncoder(bool invert)
{
	switch (m_encoderConfig)
	{
	case EncoderConfig::REV:
		if (m_motorSpark->GetMotorType() == CANSparkMaxLowLevel::MotorType::kBrushless)
		{
			m_encoderDirection = invert ? -1 : 1;
		}
		else
		{
			m_encoderDirection = 1;
			m_encoderSpark->SetInverted(invert);
		}

		break;
	case EncoderConfig::FRC:
	default:
		m_encoder->SetReverseDirection(invert);
		break;
	}
}

void EncoderSubsystemBase::SetEncoderPorts(unsigned int A, unsigned int B)
{
	switch (m_encoderConfig)
	{
	case EncoderConfig::REV:
		throw std::invalid_argument("This function cannot be used with REV encoder.");
		break;

	case EncoderConfig::FRC:
	default:
		m_encoder = new Encoder(A, B, false, Encoder::EncodingType::k4X);
		break;
	}
}

bool EncoderSubsystemBase::SetPosition(double position, double speed)
{

	m_positionPID.SetSetpoint(position);

	double output = m_positionPID.Calculate(GetPosition() * m_positionPIDDirection);

	SetMotor(output * speed);

	return m_positionPID.AtSetpoint();
}

double EncoderSubsystemBase::GetPosition()
{
	switch (m_encoderConfig)
	{
	case EncoderConfig::REV:
		return m_encoderSpark->GetPosition() * m_encoderDirection;
		break;

	case EncoderConfig::FRC:
	default:
		return m_encoder->Get();
		break;
	}
}

void EncoderSubsystemBase::ConfigurePositionPID(double p, double i, double d, double tolerance, bool inverted)
{
	m_positionPID.SetPID(p, i, d);
	m_positionPID.SetTolerance(tolerance);
	m_positionPIDDirection = inverted ? -1 : 1;
}

void EncoderSubsystemBase::SetPositionConversionFactor(double conversionFactor)
{
	switch (m_encoderConfig)
	{
	case EncoderConfig::REV:
		m_encoderSpark->SetPositionConversionFactor(conversionFactor);
		break;

	case EncoderConfig::FRC:
	default:
		m_encoder->SetDistancePerPulse(conversionFactor);
		break;
	}
}

void EncoderSubsystemBase::ResetPositionPID()
{
	m_positionPID.Reset();
}

void EncoderSubsystemBase::PrintPosition()
{
	SmartDashboard::PutNumber(GetName() + " Encoder Position", GetPosition());
}

void EncoderSubsystemBase::PrintPositionError()
{
	SmartDashboard::PutNumber(GetName() + " Encoder Position Error", m_positionPID.GetPositionError());
}

void EncoderSubsystemBase::SetPositionSafety(bool active)
{
	m_positionSafetyActive = active;
}

bool EncoderSubsystemBase::SetRPM(double speed, double acceleration)
{
	m_RPMPID.SetSetpoint(speed);

	double output = m_RPMPID.Calculate(GetPosition() * m_RPMPIDDirection);

	SetMotor(output * acceleration);

	return m_positionPID.AtSetpoint();
}

double EncoderSubsystemBase::GetRPM()
{
	switch (m_encoderConfig)
	{
	case EncoderConfig::REV:
		return m_encoderSpark->GetVelocity() * m_encoderDirection;
		break;

	case EncoderConfig::FRC:
	default:
		return m_encoder->GetRate() * m_RPMConversionFactor;
		break;
	}
}

void EncoderSubsystemBase::ConfigureRPMPID(double p, double i, double d, double tolerance, bool inverted)
{
	m_RPMPID.SetPID(p, i, d);
	m_RPMPID.SetTolerance(tolerance);
	m_RPMPIDDirection = inverted ? -1 : 1;
}

void EncoderSubsystemBase::SetRPMConversionFactor(double conversionFactor)
{
	switch (m_encoderConfig)
	{
	case EncoderConfig::REV:
		m_encoderSpark->SetVelocityConversionFactor(conversionFactor);
		break;

	case EncoderConfig::FRC:
	default:
		m_RPMConversionFactor = conversionFactor;
		break;
	}
}

void EncoderSubsystemBase::ResetRPMPID()
{
	m_RPMPID.Reset();
}

void EncoderSubsystemBase::PrintRPM()
{
	SmartDashboard::PutNumber(GetName() + " Encoder Speed", GetRPM());
}

void EncoderSubsystemBase::PrintRPMError()
{
	SmartDashboard::PutNumber(GetName() + " Encoder Speed Error", m_RPMPID.GetPositionError());
}

void EncoderSubsystemBase::SetMinMaxPosition(double min, double max)
{
	m_minPosition = min;
	m_maxPosition = max;
}