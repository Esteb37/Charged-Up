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

#include "subsystems/MotorSubsystemBase.h"

namespace TD
{

	MotorSubsystemBase::MotorSubsystemBase()
	{
		SetName("MotorSubsystemBase");
	}

	MotorSubsystemBase &MotorSubsystemBase::GetInstance()
	{
		static MotorSubsystemBase instance;
		return instance;
	}

	void MotorSubsystemBase::Initialize(MotorConfig config, unsigned int motorPort)
	{
		m_motorConfig = config;

		switch (config)
		{
		case MotorConfig::NEO:
			m_motorSpark = new CANSparkMax(motorPort, CANSparkMaxLowLevel::MotorType::kBrushless);
			break;
		case MotorConfig::SPARK:
			m_motorSpark = new CANSparkMax(motorPort, CANSparkMaxLowLevel::MotorType::kBrushed);
			break;
		case MotorConfig::VICTOR_CAN:
			m_motorVictorCAN = new VictorSPX(motorPort);
			break;
		case MotorConfig::VICTOR_PWM:
		default:
			m_motorVictorPWM = new VictorSP(motorPort);
			break;
		}

		SetName("MotorSubsystemBase");
	}

	void MotorSubsystemBase::Initialize(MotorConfig config, vector<unsigned int> motorPorts)
	{
		m_motorConfig = config;

		m_motorCount = motorPorts.size();

		if (m_motorCount <= 1)
		{
			throw std::invalid_argument("MotorSubsystemBase: Port list must have more than 1 motor, use single motor constructor instead.");
		}

		switch (config)
		{
		case MotorConfig::NEO:

			m_motorSparkList = vector<CANSparkMax *>(m_motorCount);

			for (unsigned int i = 0; i < m_motorCount; i++)
			{
				m_motorSparkList.at(i) = new CANSparkMax(motorPorts.at(i), CANSparkMaxLowLevel::MotorType::kBrushless);
			}
			break;
		case MotorConfig::SPARK:

			m_motorSparkList = vector<CANSparkMax *>(m_motorCount);

			for (unsigned int i = 0; i < m_motorCount; i++)
			{
				m_motorSparkList.at(i) = new CANSparkMax(motorPorts.at(i), CANSparkMaxLowLevel::MotorType::kBrushed);
			}
			break;
		case MotorConfig::VICTOR_CAN:

			m_motorVictorCANList = vector<VictorSPX *>(m_motorCount);

			for (unsigned int i = 0; i < m_motorCount; i++)
			{
				m_motorVictorCANList.at(i) = new VictorSPX(motorPorts.at(i));
			}
			break;
		case MotorConfig::VICTOR_PWM:
		default:
			m_motorVictorPWMList = vector<VictorSP *>(m_motorCount);

			for (unsigned int i = 0; i < m_motorCount; i++)
			{
				m_motorVictorPWMList.at(i) = new VictorSP(motorPorts.at(i));
			}
			break;
		}

		SetName("MotorSubsystemBase");
	}

	void MotorSubsystemBase::Periodic()
	{
	}

	void MotorSubsystemBase::SetMotor(double speed)
	{

		if (m_motorCount > 1)
		{
			throw std::invalid_argument("MotorSubsystemBase: SetMotor() is not supported for multiple motors.");
		}

		if (m_limitSafetyActive)
		{
			if (GetUpperLimit())
			{
				speed = fmin(speed, 0);
			}
			else if (GetLowerLimit())
			{
				speed = fmax(speed, 0);
			}
		}

		switch (m_motorConfig)
		{
		case MotorConfig::NEO:
		case MotorConfig::SPARK:
			m_motorSpark->Set(std::clamp(speed, -m_maxSpeed, m_maxSpeed));
			break;

		case MotorConfig::VICTOR_CAN:
			m_motorVictorCAN->Set(VictorSPXControlMode::PercentOutput, std::clamp(speed, -m_maxSpeed, m_maxSpeed));
			break;

		case MotorConfig::VICTOR_PWM:
		default:
			m_motorVictorPWM->Set(std::clamp(speed, -m_maxSpeed, m_maxSpeed));
			break;
		}
	}

	void MotorSubsystemBase::SetMotors(double speed)
	{
		if (m_motorCount <= 1)
		{
			throw std::invalid_argument("MotorSubsystemBase: This subsystem has only one motor. Use SetMotor instead.");
		}

		else
		{

			for (unsigned int i = 0; i < m_motorCount; i++)
			{

				if (m_limitSafetyActive)
				{
					if (GetUpperLimit())
					{
						speed = fmin(speed, 0);
					}
					else if (GetLowerLimit())
					{
						speed = fmax(speed, 0);
					}
				}

				switch (m_motorConfig)
				{
				case MotorConfig::NEO:
				case MotorConfig::SPARK:
					m_motorSparkList[i]->Set(std::clamp(speed, -m_maxSpeed, m_maxSpeed));
					break;

				case MotorConfig::VICTOR_CAN:
					m_motorVictorCANList[i]->Set(VictorSPXControlMode::PercentOutput, std::clamp(speed, -m_maxSpeed, m_maxSpeed));
					break;

				case MotorConfig::VICTOR_PWM:
				default:
					m_motorVictorPWMList[i]->Set(std::clamp(speed, -m_maxSpeed, m_maxSpeed));
					break;
				}
			}
		}
	}

	void MotorSubsystemBase::SetMotors(vector<double> speeds)
	{
		if (m_motorCount <= 1)
		{
			throw std::invalid_argument("MotorSubsystemBase: This subsystem has only one motor. Use SetMotor instead.");
		}
		else if (m_motorCount != speeds.size())
		{
			throw std::invalid_argument("MotorSubsystemBase: Speed vector must have same size as motor count.");
		}
		else
		{

			for (unsigned int i = 0; i < m_motorCount; i++)
			{

				double speed = speeds[i];

				if (m_limitSafetyActive)
				{
					if (GetUpperLimit())
					{
						speed = fmin(speeds[i], 0);
					}
					else if (GetLowerLimit())
					{
						speed = fmax(speeds[i], 0);
					}
				}

				switch (m_motorConfig)
				{
				case MotorConfig::NEO:
				case MotorConfig::SPARK:
					m_motorSparkList[i]->Set(std::clamp(speed, -m_maxSpeed, m_maxSpeed));
					break;

				case MotorConfig::VICTOR_CAN:
					m_motorVictorCANList[i]->Set(VictorSPXControlMode::PercentOutput, std::clamp(speed, -m_maxSpeed, m_maxSpeed));
					break;

				case MotorConfig::VICTOR_PWM:
				default:
					m_motorVictorPWMList[i]->Set(std::clamp(speed, -m_maxSpeed, m_maxSpeed));
					break;
				}
			}
		}
	}

	double MotorSubsystemBase::GetMotor()
	{

		if (m_motorCount > 1)
		{
			throw std::invalid_argument("MotorSubsystemBase: GetMotor() is not supported for multiple motors.");
		}

		switch (m_motorConfig)
		{
		case MotorConfig::NEO:
		case MotorConfig::SPARK:
			return m_motorSpark->Get();

		case MotorConfig::VICTOR_CAN:
			return m_motorVictorCAN->GetMotorOutputPercent();

		case MotorConfig::VICTOR_PWM:
		default:
			return m_motorVictorPWM->Get();
		}
	}

	vector<double> MotorSubsystemBase::GetMotors()
	{

		if (m_motorCount <= 1)
		{
			throw std::invalid_argument("MotorSubsystemBase: This subsystem has only one motor. Use GetMotor instead.");
		}

		vector<double> speeds(m_motorCount);

		for (unsigned int i = 0; i < m_motorCount; i++)
		{
			switch (m_motorConfig)
			{
			case MotorConfig::NEO:
			case MotorConfig::SPARK:
				speeds[i] = m_motorSparkList[i]->Get();
				break;

			case MotorConfig::VICTOR_CAN:
				speeds[i] = m_motorVictorCANList[i]->GetMotorOutputPercent();
				break;

			case MotorConfig::VICTOR_PWM:
			default:
				speeds[i] = m_motorVictorPWMList[i]->Get();
				break;
			}
		}

		return speeds;
	}

	void MotorSubsystemBase::SetPDPChannel(unsigned int channel)
	{
		m_pdpChannel = channel;
	}

	double MotorSubsystemBase::GetCurrent()
	{
		return m_pdp.GetCurrent(m_pdpChannel);
	}

	void MotorSubsystemBase::InvertMotor(bool inverted)
	{

		if (m_motorCount > 1)
		{
			throw std::invalid_argument("MotorSubsystemBase: InvertMotor() is not supported for multiple motors.");
		}

		switch (m_motorConfig)
		{
		case MotorConfig::NEO:
		case MotorConfig::SPARK:
			m_motorSpark->SetInverted(inverted);
			break;

		case MotorConfig::VICTOR_CAN:
			m_motorVictorCAN->SetInverted(inverted);
			break;

		case MotorConfig::VICTOR_PWM:
		default:
			m_motorVictorPWM->SetInverted(inverted);
			break;
		}
	}

	void MotorSubsystemBase::InvertMotors(vector<bool> invert)
	{

		if (m_motorCount <= 1)
		{
			throw std::invalid_argument("MotorSubsystemBase: This subsystem has only one motor. Use InvertMotor instead.");
		}
		else if (m_motorCount != invert.size())
		{
			throw std::invalid_argument("MotorSubsystemBase: Invert values do not match the amount of motors.");
		}
		else
		{

			for (unsigned int i = 0; i < m_motorCount; i++)
			{
				switch (m_motorConfig)
				{
				case MotorConfig::NEO:
				case MotorConfig::SPARK:
					m_motorSparkList[i]->SetInverted(invert[i]);
					break;

				case MotorConfig::VICTOR_CAN:
					m_motorVictorCANList[i]->SetInverted(invert[i]);
					break;

				case MotorConfig::VICTOR_PWM:
				default:
					m_motorVictorPWMList[i]->SetInverted(invert[i]);
					break;
				}
			}
		}
	}

	void MotorSubsystemBase::PrintMotor()
	{
		if (m_motorCount > 1)
		{
			throw std::invalid_argument("MotorSubsystemBase: PrintMotor() is not supported for multiple motors.");
		}
		SmartDashboard::PutNumber(GetName() + " Motor", GetMotor());
	}

	void MotorSubsystemBase::PrintMotors()
	{

		if (m_motorCount <= 1)
		{
			throw std::invalid_argument("MotorSubsystemBase: This subsystem has only one motor. Use PrintMotor instead.");
		}
		else
		{
			vector<double> speeds = GetMotors();

			for (unsigned int i = 0; i < m_motorCount; i++)
			{
				SmartDashboard::PutNumber(GetName() + " Motor " + std::to_string(i), speeds[i]);
			}
		}
	}

	bool MotorSubsystemBase::GetUpperLimit()
	{
		return m_upperLimit->Get();
	}

	bool MotorSubsystemBase::GetLowerLimit()
	{
		return m_lowerLimit->Get();
	}

	void MotorSubsystemBase::ConfigureLimitSwitches(unsigned int upperLimitPort, unsigned int lowerLimitPort)
	{
		m_upperLimit = new DigitalInput(upperLimitPort);
		m_lowerLimit = new DigitalInput(lowerLimitPort);
	}

	void MotorSubsystemBase::SetLimitSafety(bool active)
	{
		m_limitSafetyActive = active;
	}

	// print limits
	void MotorSubsystemBase::PrintLimits()
	{
		SmartDashboard::PutBoolean(GetName() + " Upper Limit", GetUpperLimit());
		SmartDashboard::PutBoolean(GetName() + " Lower Limit", GetLowerLimit());
	}

	void MotorSubsystemBase::SetMaxSpeed(double maxSpeed)
	{
		m_maxSpeed = maxSpeed;
	}
}