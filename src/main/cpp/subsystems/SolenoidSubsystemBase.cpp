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

#include "subsystems/SolenoidSubsystemBase.h"

using namespace TD;

SolenoidSubsystemBase::SolenoidSubsystemBase()
{
	SetName("SolenoidSubsystem");
}

void SolenoidSubsystemBase::Initialize(int forward, int reverse)
{

	m_rightSolenoid = new DoubleSolenoid(PneumaticsModuleType::REVPH, forward, reverse);

	m_isDoubleSolenoid = false;

	SetName("SingleSolenoidSubsystem");
}

void SolenoidSubsystemBase::Initialize(int forwardA, int reverseA, int forwardB, int reverseB)
{

	m_rightSolenoid = new DoubleSolenoid(PneumaticsModuleType::REVPH, forwardA, reverseA);
	m_leftSolenoid = new DoubleSolenoid{PneumaticsModuleType::REVPH, forwardB, reverseB};

	m_isDoubleSolenoid = true;

	SetName("DoubleSolenoidSubsystem");
}

void SolenoidSubsystemBase::Periodic()
{
}

void SolenoidSubsystemBase::OpenSolenoids()
{
	if (m_solenoidsInverted)
	{
		m_rightSolenoid->Set(DoubleSolenoid::Value::kForward);
		if (m_isDoubleSolenoid)
			m_leftSolenoid->Set(DoubleSolenoid::Value::kForward);
	}
	else
	{
		m_rightSolenoid->Set(DoubleSolenoid::Value::kReverse);
		if (m_isDoubleSolenoid)
			m_leftSolenoid->Set(DoubleSolenoid::Value::kReverse);
	}
}

void SolenoidSubsystemBase::CloseSolenoids()
{
	if (!m_solenoidsInverted)
	{
		m_rightSolenoid->Set(DoubleSolenoid::Value::kForward);
		if (m_isDoubleSolenoid)
			m_leftSolenoid->Set(DoubleSolenoid::Value::kForward);
	}
	else
	{
		m_rightSolenoid->Set(DoubleSolenoid::Value::kReverse);
		if (m_isDoubleSolenoid)
			m_leftSolenoid->Set(DoubleSolenoid::Value::kReverse);
	}
}

void SolenoidSubsystemBase::SolenoidsOff()
{
	m_rightSolenoid->Set(DoubleSolenoid::Value::kOff);
	if (m_isDoubleSolenoid)
		m_leftSolenoid->Set(DoubleSolenoid::Value::kOff);
}

unsigned int SolenoidSubsystemBase::GetRightSolenoid()
{
	return m_rightSolenoid->Get();
}

unsigned int SolenoidSubsystemBase::GetLeftSolenoid()
{
	return m_leftSolenoid->Get();
}

unsigned int SolenoidSubsystemBase::GetSolenoid()
{
	return GetRightSolenoid();
}

void SolenoidSubsystemBase::PrintSolenoids()
{

	if (m_isDoubleSolenoid)
	{
		SmartDashboard::PutNumber(GetName() + " Right Solenoid", GetRightSolenoid());
		SmartDashboard::PutNumber(GetName() + " Left Solenoid", GetLeftSolenoid());
	}
	else
	{
		SmartDashboard::PutNumber(GetName() + " Solenoid", GetSolenoid());
	}
}

void SolenoidSubsystemBase::ToggleSolenoids()
{
	m_rightSolenoid->Toggle();
	if (m_isDoubleSolenoid)
		m_leftSolenoid->Toggle();
}

void SolenoidSubsystemBase::InvertSolenoids(bool inverted)
{
	m_solenoidsInverted = inverted;
}