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

#pragma once

#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/PowerDistribution.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

using namespace frc;
using namespace frc2;
using namespace rev;
using namespace std;

namespace TD
{

	namespace MotorTypes
	{
		typedef CANSparkMax SPARK;
		typedef VictorSP VICTOR_PWM;
		typedef VictorSPX VICTOR_CAN;
	}

	template <typename T>
	class MotorSubsystemBase : virtual public SubsystemBase
	{

	public:
		MotorSubsystemBase(unsigned int, bool = false);

		MotorSubsystemBase(vector<unsigned int>, bool = false);

		void Periodic() override;

		// ---------- Motor -----------

		/**
		 * @brief Sets the Motor speed
		 * @param speed Speed and direction to turn
		 */
		void SetMotor(double);

		void SetMotors(double);

		void SetMotors(vector<double>);

		void SetMaxSpeed(double);

		void SetVoltage(units::voltage::volt_t volts);

		/**
		 * @brief Gets the Motor speed
		 */
		double GetMotor();

		vector<double> GetMotors();

		void SetPDPChannel(unsigned int);

		double GetCurrent();

		/**
		 * @brief Invert motor direction
		 * @param invert True to invert, false to not
		 */
		void InvertMotor(bool);

		void InvertMotors(vector<bool>);

		/**
		 * @brief Publishes the motor speed to the dashboard
		 */
		void PrintMotor();

		void PrintMotors();

		bool GetUpperLimit();

		bool GetLowerLimit();

		void ConfigureLimitSwitches(unsigned int, unsigned int);

		void ConfigureUpperLimitSwitch(unsigned int);

		void ConfigureLowerLimitSwitch(unsigned int);

		void SetLimitSafety(bool);

		void PrintLimits();

		// ---------- Components -----------

		T *m_motor;

		vector<T *>
			m_motorList;

		DigitalInput *m_upperLimit;

		DigitalInput *m_lowerLimit;

	protected:
		double m_maxSpeed;

		bool m_limitSafetyActive = false;

		unsigned int m_motorCount = 0;

		PowerDistribution m_pdp{0, PowerDistribution::ModuleType::kCTRE};

		const double kMotorStallThreshold = 10;

		double m_pdpChannel = 0;
	};


	template<>
	MotorSubsystemBase<MotorTypes::SPARK>::MotorSubsystemBase(unsigned int motorPort, bool isBrushless)
	{
		SetName("MotorSubsystem");

		m_motorCount = 1;

		m_motor = new CANSparkMax(motorPort,
								isBrushless ? CANSparkMaxLowLevel::MotorType::kBrushless : CANSparkMaxLowLevel::MotorType::kBrushed);
	}

	template<>
	MotorSubsystemBase<MotorTypes::SPARK>::MotorSubsystemBase(vector<unsigned int> motorPorts, bool isBrushless)
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

	using namespace MotorTypes;

	template <>
	void MotorSubsystemBase<VICTOR_CAN>::SetMotor(double speed)
	{
		assert(m_motorCount != 0 && "MotorSubsystemBase: No motor has been configured");

		if (m_motorCount > 1)
		{
			throw std::invalid_argument("More than one motor has been configured. Use MotorSubsystem::SetMotors instead.");
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

		m_motor->Set(VictorSPXControlMode::PercentOutput, std::clamp(speed, -m_maxSpeed, m_maxSpeed));
	}

	template<>
	void MotorSubsystemBase<MotorTypes::VICTOR_CAN>::SetMotors(double speed)
	{
		if (m_motorCount <= 1)
		{
			throw std::invalid_argument("Only one or less motors have been configued. Use MotorSubsystemBase::SetMotor instead");
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

			m_motorList[i]->Set(VictorSPXControlMode::PercentOutput, std::clamp(speed, -m_maxSpeed, m_maxSpeed));
		}
	}

	template<>
	void MotorSubsystemBase<MotorTypes::VICTOR_CAN>::SetMotors(vector<double> speeds)
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

			m_motor->Set(VictorSPXControlMode::PercentOutput, std::clamp(speed, -m_maxSpeed, m_maxSpeed));
		}
	}

	template<>
	void MotorSubsystemBase<MotorTypes::VICTOR_CAN>::SetVoltage(units::voltage::volt_t volts)
	{
		assert(false && "MotorSubsystemBase: SetVoltage() is not supported for VictorSPX");
	}

	template<>
	double MotorSubsystemBase<MotorTypes::VICTOR_CAN>::GetMotor()
	{
		assert(m_motorCount != 0 && "MotorSubsystemBase: No motor has been configured");

		assert(m_motorCount == 1 && "MotorSubsystemBase: GetMotor() is not supported for multiple motors.");

		return m_motor->GetMotorOutputPercent();
	}

	template<>
	vector<double> MotorSubsystemBase<MotorTypes::VICTOR_CAN>::GetMotors()
	{

		if (m_motorCount <= 1)
		{
			throw std::invalid_argument("Only one or less motors have been configued. Use MotorSubsystemBase::SetMotor instead");
		}

		vector<double> speeds;

		for (unsigned int i = 0; i < m_motorCount; i++)
		{
			speeds.push_back(m_motorList[i]->GetMotorOutputPercent());
		}

		return speeds;
	}

	extern template class MotorSubsystemBase<MotorTypes::SPARK>;
	extern template class MotorSubsystemBase<MotorTypes::VICTOR_CAN>;
	extern template class MotorSubsystemBase<MotorTypes::VICTOR_PWM>;

}