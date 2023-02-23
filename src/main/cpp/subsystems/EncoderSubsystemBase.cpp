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

namespace TD
{
	using namespace EncoderTypes;

	template <>
	EncoderSubsystemBase<MotorTypes::SPARK, NEO>::EncoderSubsystemBase(unsigned int motorPort, bool isBrushless) : MotorSubsystemBase<MotorTypes::SPARK>(motorPort, isBrushless)
	{
		m_encoder = new SparkMaxRelativeEncoder(m_motor->GetEncoder());
		SubsystemBase::SetName("EncoderSubsystem");
	}

	template <class MotorType, class EncoderType>
	EncoderSubsystemBase<MotorType, EncoderType>::EncoderSubsystemBase(unsigned int motorPort, bool isBrushless) : MotorSubsystemBase<MotorType>(motorPort)
	{
		throw invalid_argument("This constructor is reserved for NEO encoders.");
	}

	template <>
	EncoderSubsystemBase<MotorTypes::SPARK, NEO>::EncoderSubsystemBase(unsigned int motorPort, unsigned int encoderA, unsigned int encoderB) : MotorSubsystemBase<MotorTypes::SPARK>(motorPort)
	{
		throw invalid_argument("This constructor is reserved for CLASSIC encoders.");
	}

	template <class MotorType, class EncoderType>
	EncoderSubsystemBase<MotorType, EncoderType>::EncoderSubsystemBase(unsigned int motorPort, unsigned int encoderA, unsigned int encoderB) : MotorSubsystemBase<MotorType>(motorPort)
	{
		static_assert(!std::is_same<EncoderType, NEO>::value, "NEO encoder has to be used with SPARK MAX motor");

		m_encoder = new Encoder(encoderA, encoderB, false, Encoder::EncodingType::k4X);

		SubsystemBase::SetName("EncoderSubsystem");
	}

	template <>
	EncoderSubsystemBase<MotorTypes::SPARK, NEO>::EncoderSubsystemBase(vector<unsigned int> motorPorts, bool isBrushless) : MotorSubsystemBase<MotorTypes::SPARK>(motorPorts, isBrushless)
	{
		m_encoder = new SparkMaxRelativeEncoder(m_motorList[0]->GetEncoder());
		SubsystemBase::SetName("EncoderSubsystem");
	}

	template <class MotorType, class EncoderType>
	EncoderSubsystemBase<MotorType, EncoderType>::EncoderSubsystemBase(vector<unsigned int> motorPorts, bool isBrushless) : MotorSubsystemBase<MotorType>(motorPorts)
	{
		throw invalid_argument("This constructor is reserved for NEO encoders.");
	}

	template <>
	EncoderSubsystemBase<MotorTypes::SPARK, NEO>::EncoderSubsystemBase(vector<unsigned int> motorPorts, unsigned int encoderA, unsigned int encoderB) : MotorSubsystemBase<MotorTypes::SPARK>(motorPorts)
	{

		throw invalid_argument("This constructor is reserved for CLASSIC encoders.");
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
			((NEO *)m_encoder)->SetPosition(0);
		}
		else
		{
			((CLASSIC *)m_encoder)->Reset();
		}
	}

	template <class MotorType, class EncoderType>
	void EncoderSubsystemBase<MotorType, EncoderType>::InvertEncoder(bool invert)
	{
		if (std::is_same<EncoderType, NEO>::value)
		{
			((NEO *)m_encoder)->SetInverted(invert);
		}
		else
		{
			((CLASSIC *)m_encoder)->SetReverseDirection(invert);
		}
	}

	template <class MotorType, class EncoderType>
	double EncoderSubsystemBase<MotorType, EncoderType>::GetPosition()
	{
		if (std::is_same<EncoderType, NEO>::value)
		{
			return ((NEO *)m_encoder)->GetPosition();
		}
		else
		{
			return ((CLASSIC *)m_encoder)->GetDistance();
		}
	}

	template <class MotorType, class EncoderType>
	bool EncoderSubsystemBase<MotorType, EncoderType>::SetPosition(double position, double speed)
	{
		m_positionPID.SetSetpoint(position);

		double output = m_positionPID.Calculate(GetPosition() * m_positionPIDDirection);
		output = std::clamp(output, -1.0, 1.0);

		MotorSubsystemBase<MotorType>::SetMotor(output * speed);

		return m_positionPID.AtSetpoint();
	}

	template <class MotorType, class EncoderType>
	frc2::CommandPtr EncoderSubsystemBase<MotorType, EncoderType>::SetPositionCmd(double position, double speed) {
		return frc2::FunctionalCommand(
			[this, &position] { m_positionPID.SetSetpoint(position); },

			[this, &speed] {
				double output = m_positionPID.Calculate(GetPosition() * m_positionPIDDirection);
				SetMotor(output * speed);
			},

			[this] (bool wasInterrupted) { SetMotor(0.0); },

			[this] { return m_positionPID.AtSetpoint(); }
		).ToPtr();
	}

	template <class MotorType, class EncoderType>
	frc2::CommandPtr EncoderSubsystemBase<MotorType, EncoderType>::SetAngleCmd(units::angle::degree_t degrees, double speed) {
		return frc2::FunctionalCommand(
			[this, &degrees] { m_positionPID.SetSetpoint(std::fmod(degrees.value(), 360.0)); },

			[this, &speed] {
				double output = m_positionPID.Calculate(GetPosition() * m_positionPIDDirection);
				SetMotor(output * speed);
			},

			[this] (bool wasInterrupted) { SetMotor(0.0); },

			[this] { return m_positionPID.AtSetpoint(); }
		).ToPtr();
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
			((NEO *)m_encoder)->SetPositionConversionFactor(conversionFactor);
		}
		else
		{
			((CLASSIC *)m_encoder)->SetDistancePerPulse(conversionFactor);
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

	template <>
	void EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>::SetSparkMaxIdleMode(rev::CANSparkMax::IdleMode mode) {
		for (auto motor: m_motorList) {
			motor->SetIdleMode(mode);
		}
	}

	template <>
	void EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>::SetSparkSoftLimit(rev::CANSparkMax::SoftLimitDirection direction, double limit) {
		softLimitDirection = direction;

		for (auto motor: m_motorList) {
			motor->SetSoftLimit(softLimitDirection, limit);
		}
	}

	template <>
	void EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>::EnableSparkSoftLimit() {
		for (auto motor: m_motorList) {
			motor->EnableSoftLimit(softLimitDirection, true);
		}
	}

	template <>
	void EncoderSubsystemBase<MotorTypes::SPARK, EncoderTypes::NEO>::DisableSparkSoftLimit() {
		for (auto motor: m_motorList) {
			motor->EnableSoftLimit(softLimitDirection, false);
		}
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
			return ((NEO *)m_encoder)->GetVelocity();
		}
		else
		{
			return ((CLASSIC *)m_encoder)->GetRate();
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
			((NEO *)m_encoder)->SetVelocityConversionFactor(conversionFactor);
		}
		else
		{
			((CLASSIC *)m_encoder)->SetDistancePerPulse(conversionFactor);
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

	template class EncoderSubsystemBase<MotorTypes::SPARK, NEO>;
	template class EncoderSubsystemBase<MotorTypes::SPARK, CLASSIC>;
	template class EncoderSubsystemBase<MotorTypes::VICTOR_CAN, CLASSIC>;
	template class EncoderSubsystemBase<MotorTypes::VICTOR_PWM, CLASSIC>;
}