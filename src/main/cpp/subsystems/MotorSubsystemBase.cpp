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
	using namespace MotorTypes;

	template <class T>
	MotorSubsystemBase<T> &MotorSubsystemBase<T>::GetInstance()
	{
		static MotorSubsystemBase instance;
		return instance;
	}

	template <>
	MotorSubsystemBase<SPARK>::MotorSubsystemBase(unsigned int motorPort, bool isBrushless)
	{
		SetName("MotorSubsystem");

		m_motorCount = 1;

		m_motor = new CANSparkMax(motorPort,
								  isBrushless ? CANSparkMaxLowLevel::MotorType::kBrushless : CANSparkMaxLowLevel::MotorType::kBrushed);
	}

	template <>
	MotorSubsystemBase<SPARK>::MotorSubsystemBase(vector<unsigned int> motorPorts, bool isBrushless)
	{
		assert(motorPorts.size() > 1 && "MotorSubsystemBase: Port list must have more than 1 motor, use single motor constructor instead.");

		SetName("MotorSubsystem");

		m_motorCount = motorPorts.size();

		m_motorList = vector<CANSparkMax *>(m_motorCount);

		for (unsigned int i = 0; i < m_motorCount; i++)
		{
			m_motorList.at(i) = new CANSparkMax(motorPorts[i],
												isBrushless ? CANSparkMaxLowLevel::MotorType::kBrushless : CANSparkMaxLowLevel::MotorType::kBrushed);
		}
	}

	template <class T>
	MotorSubsystemBase<T>::MotorSubsystemBase(unsigned int motorPort)
	{
		SetName("MotorSubsystem");

		m_motorCount = 1;

		m_motor = new T(motorPort);
	}

	template <class T>
	MotorSubsystemBase<T>::MotorSubsystemBase(vector<unsigned int> motorPorts)
	{
		SetName("MotorSubsystemBase");

		m_motorCount = motorPorts.size();

		m_motorList = vector<MotorType *>(m_motorCount);

		for (unsigned int i = 0; i < m_motorCount; i++)
		{
			m_motorList.at(i) = new T(motorPorts.at(i));
		}
	}

	template <class T>
	void MotorSubsystemBase<T>::Periodic()
	{
	}

	template <class T>
	void MotorSubsystemBase<T>::SetMotor(double speed)
	{
		assert(m_motorCount != 0 && "MotorSubsystemBase: No motor has been configured");

		if (m_motorCount > 1)
		{
			return SetMotors(speed);
		}

		if (m_limitSafetyActive)
		{
			if (GetUpperLimit())
			{
				speed = fmin(speed, 0);
			}
			else
			{
				speed = fmax(speed, 0);
			}
		}

		if (std::is_same(T, VICTOR_CAN))
		{
			m_motor->Set(VictorSPXControlMode::PercentOutput, std::clamp(speed, -m_maxSpeed, m_maxSpeed));
		}
		else
		{
			m_motor->Set(std::clamp(speed, -m_maxSpeed, m_maxSpeed));
		}
	}

	template <class T>
	void MotorSubsystemBase<T>::SetMotors(double speed)
	{
		if (m_motorCount <= 1)
		{
			return SetMotor(speed);
		}

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

			if (std::is_same(T, VICTOR_CAN))
			{
				m_motorList[i]->Set(VictorSPXControlMode::PercentOutput, std::clamp(speed, -m_maxSpeed, m_maxSpeed));
			}
			else
			{
				m_motorList[i]->Set(std::clamp(speed, -m_maxSpeed, m_maxSpeed));
			}
		}
	}

	template <class T>
	void MotorSubsystemBase<T>::SetMotors(vector<double> speeds)
	{
		assert(speeds.size() > 1 && "MotorSubsystemBase: Speed vector must have more than 1 motor, use single motor constructor instead.");

		assert(speeds.size() == m_motorCount && "MotorSubsystemBase: Speed vector must have the same number of motors as the port list.");

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

			if (std::is_same(T, VICTOR_CAN))
			{
				m_motor->Set(VictorSPXControlMode::PercentOutput, std::clamp(speed, -m_maxSpeed, m_maxSpeed));
			}
			else
			{
				m_motor->Set(std::clamp(speed, -m_maxSpeed, m_maxSpeed));
			}
		}
	}

	template <class T>
	void MotorSubsystemBase<T>::SetVoltage(units::voltage::volt_t volts)
	{
		static_assert(!std::is_same(T, VICTOR_CAN), "MotorSubsystemBase: SetVoltage() is not supported for CAN VictorSPX.");

		m_motor->SetVoltage(volts);
	}

	template <class T>
	double MotorSubsystemBase<T>::GetMotor()
	{
		assert(m_motorCount != 0 && "MotorSubsystemBase: No motor has been configured");

		assert(m_motorCount == 1 && "MotorSubsystemBase: GetMotor() is not supported for multiple motors.");

		if (std::is_same(T, VICTOR_CAN))
		{
			return m_motor->GetMotorOutputPercent();
		}

		return m_motor->Get();
	}

	template <class T>
	vector<double> MotorSubsystemBase<T>::GetMotors()
	{

		if (m_motorCount <= 1)
		{
			return {GetMotor()};
		}

		vector<double> speeds;

		for (unsigned int i = 0; i < m_motorCount; i++)
		{
			if (std::is_same(T, VICTOR_CAN))
			{
				speeds.push_back(m_motorList[i]->GetMotorOutputPercent());
			}
			else
			{
				speeds.push_back(m_motorList[i]->Get());
			}
		}

		return speeds;
	}

	template <class T>
	void MotorSubsystemBase<T>::SetPDPChannel(unsigned int channel)
	{
		m_pdpChannel = channel;
	}

	template <class T>
	double MotorSubsystemBase<T>::GetCurrent()
	{
		return m_pdp.GetCurrent(m_pdpChannel);
	}

	template <class T>
	void MotorSubsystemBase<T>::InvertMotor(bool inverted)
	{
		assert(m_motorCount != 0 && "MotorSubsystemBase: No motor has been configured");

		if (m_motorCount > 1)
		{
			return InvertMotors(inverted);
		}

		m_motor->SetInverted(inverted);
	}

	template <class T>
	void MotorSubsystemBase<T>::InvertMotors(vector<bool> invert)
	{
		assert(invert.size() == m_motorCount && "MotorSubsystemBase: Invert vector must have the same number of motors as the port list.");

		assert(invert.size() > 1 && "MotorSubsystemBase: Invert vector must have more than 1 motor, use single motor constructor instead.");

		for (unsigned int i = 0; i < m_motorCount; i++)
		{
			m_motorList[i]->SetInverted(invert[i]);
		}
	}

	template <class T>
	void MotorSubsystemBase<T>::PrintMotor()
	{
		if (m_motorCount > 1)
		{
			return PrintMotors();
		}

		SmartDashboard::PutNumber(GetName() + " Motor", GetMotor());
	}

	template <class T>
	void MotorSubsystemBase<T>::PrintMotors()
	{

		if (m_motorCount <= 1)
		{
			return PrintMotor();
		}

		vector<double> speeds = GetMotors();

		for (unsigned int i = 0; i < m_motorCount; i++)
		{
			SmartDashboard::PutNumber(GetName() + " Motor " + to_string(i), speeds[i]);
		}
	}

	template <class T>
	bool MotorSubsystemBase<T>::GetUpperLimit()
	{
		if (m_upperLimit == nullptr)
		{
			return false;
		}
		return m_upperLimit->Get();
	}

	template <class T>
	bool MotorSubsystemBase<T>::GetLowerLimit()
	{
		if (m_lowerLimit == nullptr)
		{
			return false;
		}
		return m_lowerLimit->Get();
	}

	template <class T>
	void MotorSubsystemBase<T>::ConfigureLimitSwitches(unsigned int upperLimitPort, unsigned int lowerLimitPort)
	{
		ConfigureUpperLimitSwitch(upperLimitPort);
		ConfigureLowerLimitSwitch(lowerLimitPort);
	}

	template <class T>
	void MotorSubsystemBase<T>::ConfigureUpperLimitSwitch(unsigned int port)
	{
		m_upperLimit = new DigitalInput(port);
	}

	template <class T>
	void MotorSubsystemBase<T>::ConfigureLowerLimitSwitch(unsigned int port)
	{
		m_lowerLimit = new DigitalInput(port);
	}

	template <class T>
	void MotorSubsystemBase<T>::SetLimitSafety(bool active)
	{
		m_limitSafetyActive = active;
	}

	// print limits
	template <class T>
	void MotorSubsystemBase<T>::PrintLimits()
	{
		SmartDashboard::PutBoolean(GetName() + " Upper Limit", GetUpperLimit());
		SmartDashboard::PutBoolean(GetName() + " Lower Limit", GetLowerLimit());
	}

	template <class T>
	void MotorSubsystemBase<T>::SetMaxSpeed(double maxSpeed)
	{
		m_maxSpeed = maxSpeed;
	}

	template class MotorSubsystemBase<SPARK>;
	template class MotorSubsystemBase<VICTOR_PWM>;
	template class MotorSubsystemBase<VICTOR_CAN>;
}
