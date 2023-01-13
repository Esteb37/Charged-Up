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

#include "subsystems/SolenoidClawBase.h"

using namespace TD;

SolenoidClawBase::SolenoidClawBase()
{
	SetName("Claw");
}

SolenoidClawBase &SolenoidClawBase::GetInstance()
{
	static SolenoidClawBase instance;
	return instance;
}

void SolenoidClawBase::Initialize(unsigned int solenoidForward, unsigned int solenoidReverse)
{
	m_hand.SetName("Claw Hand");
	m_hand.Initialize(solenoidForward, solenoidReverse);
}

void SolenoidClawBase::Initialize(unsigned int handForward, unsigned int handReverse, unsigned int wristForward, unsigned int wristReverse)
{
	m_hand.SetName("Claw Hand");
	m_wrist.SetName("Claw Wrist");
	m_hand.Initialize(handForward, handReverse);
	m_wrist.Initialize(wristForward, wristReverse);
}

void SolenoidClawBase::Periodic()
{
}

void SolenoidClawBase::OpenHand()
{
	m_hand.OpenSolenoids();
}

void SolenoidClawBase::CloseHand()
{
	m_hand.CloseSolenoids();
}

void SolenoidClawBase::ToggleHand()
{
	m_hand.ToggleSolenoids();
}

void SolenoidClawBase::InvertHand(bool invert)
{
	m_hand.InvertSolenoids(invert);
}

void SolenoidClawBase::HandOff()
{
	m_hand.SolenoidsOff();
}

unsigned int SolenoidClawBase::GetHand()
{
	return m_hand.GetSolenoid();
}

void SolenoidClawBase::PrintHand()
{
	m_hand.PrintSolenoids();
}

void SolenoidClawBase::RaiseWrist()
{
	m_wrist.CloseSolenoids();
}

void SolenoidClawBase::LowerWrist()
{
	m_wrist.OpenSolenoids();
}

void SolenoidClawBase::ToggleWrist()
{
	m_wrist.ToggleSolenoids();
}

void SolenoidClawBase::InvertWrist(bool invert)
{
	m_wrist.InvertSolenoids(invert);
}

void SolenoidClawBase::WristOff()
{
	m_wrist.SolenoidsOff();
}

unsigned int SolenoidClawBase::GetWrist()
{
	return m_wrist.GetSolenoid();
}

void SolenoidClawBase::PrintWrist()
{
	m_wrist.PrintSolenoids();
}