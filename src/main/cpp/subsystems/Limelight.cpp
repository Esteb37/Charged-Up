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

#include "subsystems/Limelight.h"

using namespace TD;

Limelight::Limelight()
{
	limelight = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}

Limelight &Limelight::GetInstance()
{
	static Limelight instance;
	return instance;
}

void Limelight::Initialize(double angle, double height)
{
	SetMountAngle(angle);
	SetMountHeight(height);
}
double Limelight::GetArea()
{
	return limelight->GetEntry("ta").GetDouble(0);
}

double Limelight::GetHorizontalAngle()
{
	return limelight->GetEntry("tx").GetDouble(0);
}

double Limelight::GetVerticalAngle()
{
	return limelight->GetEntry("ty").GetDouble(0);
}

double Limelight::GetSkew()
{
	return limelight->GetEntry("ts").GetDouble(0);
}

double Limelight::GetPipeline()
{
	return limelight->GetEntry("getpipe").GetDouble(0);
}

double Limelight::GetValid()
{
	return limelight->GetEntry("tv").GetDouble(0);
}

double Limelight::GetTarget()
{
	return limelight->GetEntry("target").GetDouble(0);
}

double Limelight::GetDistanceToTarget(double objectiveHeight)
{
	return (objectiveHeight - m_mountHeight) / tan(m_mountHeight + GetVerticalAngle());
}

void Limelight::SetMountAngle(double angle)
{
	m_mountAngle = angle * M_PI / 180;
}

void Limelight::SetMountHeight(double height)
{
	m_mountHeight = height;
}

void Limelight::SetPipeline(int pipeline)
{
	limelight->GetEntry("pipeline").SetDouble(pipeline);
}

void Limelight::SetLED(int led)
{
	limelight->GetEntry("ledMode").SetDouble(led);
}

void Limelight::SetCamMode(int mode)
{
	limelight->GetEntry("camMode").SetDouble(mode);
}

void Limelight::SetStream(int stream)
{
	limelight->GetEntry("stream").SetDouble(stream);
}

void Limelight::SetSnapshot(int snapshot)
{
	limelight->GetEntry("snapshot").SetDouble(snapshot);
}

void Limelight::SetLight(int light)
{
	limelight->GetEntry("light").SetDouble(light);
}

void Limelight::SetLEDMode(int mode)
{
	limelight->GetEntry("ledMode").SetDouble(mode);
}

void Limelight::SetLEDFrequency(int freq)
{
	limelight->GetEntry("ledFrequency").SetDouble(freq);
}

void Limelight::SetLEDIntensity(int intensity)
{
	limelight->GetEntry("ledIntensity").SetDouble(intensity);
}

void Limelight::SetLEDState(int state)
{
	limelight->GetEntry("ledState").SetDouble(state);
}

void Limelight::SetLEDOff()
{
	limelight->GetEntry("ledMode").SetDouble(0);
}

void Limelight::SetLEDOn()
{
	limelight->GetEntry("ledMode").SetDouble(1);
}

void Limelight::PrintHorizontalAngle()
{
	SmartDashboard::PutNumber("Horizontal Angle", GetHorizontalAngle());
}

void Limelight::PrintVerticalAngle()
{
	SmartDashboard::PutNumber("Vertical Angle", GetVerticalAngle());
}

void Limelight::PrintArea()
{
	SmartDashboard::PutNumber("Area", GetArea());
}

void Limelight::PrintSkew()
{
	SmartDashboard::PutNumber("Skew", GetSkew());
}

void Limelight::PrintDistanceToTarget(double objectiveHeight)
{
	SmartDashboard::PutNumber("Distance to Target", GetDistanceToTarget(objectiveHeight));
}