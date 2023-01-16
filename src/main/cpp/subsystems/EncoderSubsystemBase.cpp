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
using namespace EncoderTypes;

template <>
EncoderSubsystemBase<MotorTypes::SPARK, NEO>::EncoderSubsystemBase(unsigned int motorPort) : MotorSubsystemBase<MotorTypes::SPARK>(motorPort, true)
{
	m_encoder = new SparkMaxRelativeEncoder(m_motor->GetEncoder());
	SubsystemBase::SetName("EncoderSubsystem");
}

template <class MotorType, class EncoderType>
EncoderSubsystemBase<MotorType, EncoderType>::EncoderSubsystemBase(unsigned int motorPort, unsigned int encoderA, unsigned int encoderB) : MotorSubsystemBase<MotorType>(motorPort, false)
{
	static_assert(!std::is_same<EncoderType, NEO>::value, "NEO encoder has to be used with SPARK MAX motor");

	m_encoder = new Encoder(encoderA, encoderB, false, Encoder::EncodingType::k4X);

	SubsystemBase::SetName("EncoderSubsystem");
}

template <>
EncoderSubsystemBase<MotorTypes::SPARK, NEO>::EncoderSubsystemBase(vector<unsigned int> motorPorts) : MotorSubsystemBase<MotorTypes::SPARK>(motorPorts, true)
{
	m_encoder = new SparkMaxRelativeEncoder(m_motorList[0]->GetEncoder());
	SubsystemBase::SetName("EncoderSubsystem");
}

template <class MotorType, class EncoderType>
EncoderSubsystemBase<MotorType, EncoderType>::EncoderSubsystemBase(vector<unsigned int> motorPorts, unsigned int encoderA, unsigned int encoderB) : MotorSubsystemBase<MotorType>(motorPorts, false)
{

	static_assert(!std::is_same<EncoderType, NEO>::value, "NEO encoder has to be used with SPARK MAX motor");

	m_encoder = new Encoder(encoderA, encoderB, false, Encoder::EncodingType::k4X);

	SubsystemBase::SetName("EncoderSubsystem");
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::Periodic()
{
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::Reset()
{
	ResetEncoder();
	ResetPositionPID();
	ResetRPMPID();
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::SetMotor(double speed)
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

	MotorSubsystemBase<MotorType>::SetMotor(speed);
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::SetMotors(double speed)
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

	MotorSubsystemBase<MotorType>::SetMotors(speed);
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::SetMotors(vector<double> speeds)
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

	MotorSubsystemBase<MotorType>::SetMotors(speeds);
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::ResetEncoder()
{
	if (std::is_same<EncoderType, NEO>::value)
	{
		m_encoder->SetPosition(0);
	}
	else
	{
		m_encoder->Reset();
	}
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::InvertEncoder(bool invert)
{
	if (std::is_same<EncoderType, NEO>::value)
	{
		m_encoder->SetInverted(invert);
	}
	else
	{
		m_encoder->SetReverseDirection(invert);
	}
}

template <class MotorType, class EncoderType>
double EncoderSubsystemBase<MotorType, EncoderType>::GetPosition()
{
	if (std::is_same<EncoderType, NEO>::value)
	{
		return m_encoder->GetPosition();
	}
	else
	{
		return m_encoder->GetDistance();
	}
}

template <class MotorType, class EncoderType>
bool EncoderSubsystemBase<MotorType, EncoderType>::SetPosition(double position, double speed)
{
	m_positionPID.SetSetpoint(position);

	double output = m_positionPID.Calculate(GetPosition() * m_positionPIDDirection);

	MotorSubsystemBase<MotorType>::SetMotor(output * speed);

	return m_positionPID.AtSetpoint();
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::ConfigurePositionPID(double p, double i, double d, double tolerance, bool inverted)
{
	m_positionPID.SetPID(p, i, d);
	m_positionPID.SetTolerance(tolerance);
	m_positionPIDDirection = inverted ? -1 : 1;
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::SetPositionConversionFactor(double conversionFactor)
{
	if (std::is_same<EncoderType, NEO>::value)
	{
		m_encoder->SetPositionConversionFactor(conversionFactor);
	}
	else
	{
		m_encoder->SetDistancePerPulse(conversionFactor);
	}
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::ResetPositionPID()
{
	m_positionPID.Reset();
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::PrintPosition()
{
	SmartDashboard::PutNumber(SubsystemBase::GetName() + " Encoder Position", GetPosition());
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::PrintPositionError()
{
	SmartDashboard::PutNumber(SubsystemBase::GetName() + " Encoder Position Error", m_positionPID.GetPositionError());
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::SetPositionSafety(bool active)
{
	m_positionSafetyActive = active;
}

template <class MotorType, class EncoderType>
bool EncoderSubsystemBase<MotorType, EncoderType>::SetRPM(double speed, double acceleration)
{
	m_RPMPID.SetSetpoint(speed);

	double output = m_RPMPID.Calculate(GetPosition() * m_RPMPIDDirection);

	MotorSubsystemBase<MotorType>::SetMotor(output * acceleration);

	return m_positionPID.AtSetpoint();
}

template <class MotorType, class EncoderType>
double EncoderSubsystemBase<MotorType, EncoderType>::GetRPM()
{
	if (std::is_same<EncoderType, NEO>::value)
	{
		return m_encoder->GetVelocity();
	}
	else
	{
		return m_encoder->GetRate();
	}
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::ConfigureRPMPID(double p, double i, double d, double tolerance, bool inverted)
{
	m_RPMPID.SetPID(p, i, d);
	m_RPMPID.SetTolerance(tolerance);
	m_RPMPIDDirection = inverted ? -1 : 1;
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::SetRPMConversionFactor(double conversionFactor)
{
	if (std::is_same<EncoderType, NEO>::value)
	{
		m_encoder->SetVelocityConversionFactor(conversionFactor);
	}
	else
	{
		m_encoder->SetDistancePerPulse(conversionFactor);
	}
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::ResetRPMPID()
{
	m_RPMPID.Reset();
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::PrintRPM()
{
	SmartDashboard::PutNumber(SubsystemBase::GetName() + " Encoder Speed", GetRPM());
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::PrintRPMError()
{
	SmartDashboard::PutNumber(SubsystemBase::GetName() + " Encoder Speed Error", m_RPMPID.GetPositionError());
}

template <class MotorType, class EncoderType>
void EncoderSubsystemBase<MotorType, EncoderType>::SetMinMaxPosition(double min, double max)
{
	m_minPosition = min;
	m_maxPosition = max;
}