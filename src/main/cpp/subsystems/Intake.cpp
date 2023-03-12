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

#include "subsystems/Intake.h"

namespace TD
{
	Intake::Intake(unsigned int motorPort) : MotorSubsystemBase(motorPort)
	{
		SubsystemBase::SetName("Intake");
	}

	Intake::Intake(vector<unsigned int> motorPorts) : MotorSubsystemBase(motorPorts)
	{
		SubsystemBase::SetName("Intake");
		InvertMotors({false, true});

	}

	void Intake::Periodic()
	{
	}

	// ---------- Actions -----------

	void Intake::Take()
	{
		m_motorList[1]->Set(-1.0);
		m_motorList[0]->Set(-1.0);

		return;
		if (m_motorCount <= 1)
		{
			SetMotor(1);
		}
		else
		{
			SetMotors(1);
		}
	}

	void Intake::Spit(double speed)
	{
		m_motorList[1]->Set(1.0);
		m_motorList[0]->Set(1.0);

		return;

		if (m_motorCount <= 1)
		{
			SetMotor(-1 * speed);
		}
		else
		{
			SetMotors(-1 * speed);
		}
	}

	void Intake::Stop()
	{
		if (m_motorCount <= 1)
		{
			SetMotor(0);
		}
		else
		{
			SetMotors(0);
		}
	}

	frc2::CommandPtr Intake::SpitCmd(double speed)
	{
		// Command that runs for 1 second
		return frc2::FunctionalCommand(
				   [this]
				   {
					   m_spitTimer.Reset();
					   m_spitTimer.Start();
				   },
				   [this, speed]
				   {
					   Spit(speed);
				   },
				   [this](bool wasInterrupted)
				   {
					   Stop();
					   m_spitTimer.Stop();
				   },

				   [this]
				   { return m_spitTimer.HasElapsed(Times::SPIT); })
			.ToPtr();
	}

	frc2::CommandPtr Intake::TakeCmd(double speed)
	{
		return InstantCommand([this, speed]
							  { Take(); })
			.ToPtr();
	}

	void Intake::Invert(bool invert)
	{
		if (m_motorCount <= 1)
		{
			InvertMotor(invert);
		}
		else
		{
			InvertMotors({invert, !invert});
		}
	}
}