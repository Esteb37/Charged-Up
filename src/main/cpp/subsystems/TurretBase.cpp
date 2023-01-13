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
	Date: 12/04/2022	Language: cpp
	Copyright (c) TecDroid 3354 and Esteban Padilla Cerdio
	Open Source Software; you can modify and/or share it under the terms of
*/

#include "subsystems/TurretBase.h"

using namespace TD;

TurretBase::TurretBase()
{
	SetName("Turret");
}

TurretBase &TurretBase::GetInstance()
{
	static TurretBase instance;
	return instance;
}

void TurretBase::Periodic()
{

	if (!m_keepingStill)
	{
		m_angle = GetAngle();
	}

	m_keepingStill = false;
}

void TurretBase::Turn(double speed)
{
	SetMotor(speed);
}

void TurretBase::Reset()
{
	ResetEncoder();
	ResetPositionPID();
	ResetAlignPID();
}

void TurretBase::KeepStill(double speed)
{
	m_keepingStill = true;

	SetAngle(m_angle, speed);
}

bool TurretBase::Center(double speed)
{
	return SetAngle(0, speed);
}

bool TurretBase::SetAngle(double angle, double speed)
{

	if (angle > m_freedom / 2 && angle < -m_freedom / 2)
	{
		throw std::invalid_argument("Turret angle must be within " + to_string(m_freedom / 2) + " degrees of the center");
	}
	else
	{
		return SetPosition(angle, speed);
	}
}

double TurretBase::GetAngle()
{
	return 0;
}

void TurretBase::ConfigureAnglePID(double p, double i, double d, double tolerance, bool inverted)
{
	ConfigurePositionPID(p, i, d, tolerance, inverted);
}

void TurretBase::PrintAngle()
{
	SmartDashboard::PutNumber(GetName() + " Angle", GetAngle());
}

void TurretBase::PrintAnglePIDError()
{
	SmartDashboard::PutNumber(GetName() + " Angle PID Error", m_positionPID.GetPositionError());
}

bool TurretBase::Align(double speed)
{
	m_alignPID.SetSetpoint(0);

	double output = m_alignPID.Calculate(m_limelight.GetHorizontalAngle() * m_alignPIDDirection);

	SetMotor(output * speed);

	return m_alignPID.AtSetpoint();
}

void TurretBase::ConfigureAlignPID(double p, double i, double d, double tolerance, bool inverted)
{
	m_alignPID.SetPID(p, i, d);
	m_alignPID.SetTolerance(tolerance);
	m_alignPIDDirection = inverted ? -1 : 1;
}

void TurretBase::ResetAlignPID()
{
	m_alignPID.Reset();
}

void TurretBase::PrintAlignPIDError()
{
	SmartDashboard::PutNumber(GetName() + " Align PID Error", m_alignPID.GetPositionError());
}

void TurretBase::SetFreedom(double freedom)
{
	m_freedom = freedom;
	SetMinMaxPosition(-freedom / 2, freedom / 2);
}