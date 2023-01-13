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

#include "subsystems/MotorClawBase.h"

using namespace TD;

MotorClawBase::MotorClawBase()
{
	SetName("Claw");
}

MotorClawBase &MotorClawBase::GetInstance()
{
	static MotorClawBase instance;
	return instance;
}

void MotorClawBase::Initialize(MotorConfig config, unsigned int motorPort)
{
	m_hand.SetName("Claw Hand");
	m_hand.Initialize(config, motorPort);
}

void MotorClawBase::Initialize(MotorConfig config, unsigned int motorPortClaw, unsigned int motorPortWrist)
{
	m_hand.SetName("Claw Hand");
	m_wrist.SetName("Claw Wrist");
	m_hand.Initialize(config, motorPortClaw);
	m_wrist.Initialize(config, motorPortWrist);
}

void MotorClawBase::Periodic()
{
}

void MotorClawBase::OpenHand()
{
	m_hand.SetMotor(1);
}

void MotorClawBase::CloseHand()
{
	m_hand.SetMotor(-1);
}

void MotorClawBase::ToggleHand()
{
	if (m_handOpen)
	{
		CloseHand();
	}
	else
	{
		OpenHand();
	}
}

void MotorClawBase::InvertHand(bool invert)
{
	m_hand.InvertMotor(invert);
}

unsigned int MotorClawBase::GetHand()
{
	return m_hand.GetMotor();
}

void MotorClawBase::PrintHand()
{
	m_hand.PrintMotor();
}

void MotorClawBase::RaiseWrist()
{
	m_wrist.SetMotor(1);
}

void MotorClawBase::LowerWrist()
{
	m_wrist.SetMotor(-1);
}

void MotorClawBase::ToggleWrist()
{
	if (m_wristLowered)
	{
		RaiseWrist();
	}
	else
	{
		LowerWrist();
	}
}

void MotorClawBase::InvertWrist(bool invert)
{
	m_wrist.InvertMotor(invert);
}

unsigned int MotorClawBase::GetWrist()
{
	return m_wrist.GetMotor();
}

void MotorClawBase::PrintWrist()
{
	m_wrist.PrintMotor();
}