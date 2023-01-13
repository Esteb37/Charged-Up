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

#include "subsystems/ShooterBase.h"

using namespace TD;

ShooterBase::ShooterBase()
{
	SetName("Shooter");
}

ShooterBase &ShooterBase::GetInstance()
{
	static ShooterBase instance;
	return instance;
}

void ShooterBase::Periodic()
{
}

void ShooterBase::Reset()
{
	ResetEncoder();
	ResetRPMPID();
	ResetServos();
	SolenoidsOff();
}

double ShooterBase::CalculateRPM(double targetHeight)
{
	return m_rpmCalculationFunction(m_limelight.GetDistanceToTarget(targetHeight));
}

void ShooterBase::SetAngleServos(double angle)
{
	m_rightServo->Set(angle);
	m_leftServo->Set(angle);
}

void ShooterBase::ResetServos()
{
	m_rightServo->Set(0);
	m_leftServo->Set(0);
}

void ShooterBase::ConfigureServos(unsigned int rightPort, unsigned int leftPort)
{
	m_rightServo = new Servo(rightPort);
	m_leftServo = new Servo(leftPort);
}

double ShooterBase::GetRightServo()
{
	return m_rightServo->Get();
}

double ShooterBase::GetLeftServo()
{
	return m_leftServo->Get();
}

void ShooterBase::PrintServos()
{
	SmartDashboard::PutNumber(GetName() + " Right Servo", GetRightServo());
	SmartDashboard::PutNumber(GetName() + " Left Servo", GetLeftServo());
}

void ShooterBase::Shoot()
{
	SetMotor(1);
}

void ShooterBase::ConfigureSolenoid(unsigned int forward, unsigned int reverse)
{
	SolenoidSubsystemBase::Initialize(forward, reverse);
}

void ShooterBase::ConfigureSolenoids(unsigned int forwardA, unsigned int reverseA, unsigned int forwardB, unsigned int reverseB)
{
	SolenoidSubsystemBase::Initialize(forwardA, reverseA, forwardB, reverseB);
}