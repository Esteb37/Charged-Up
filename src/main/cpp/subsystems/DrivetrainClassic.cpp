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

#include "subsystems/DrivetrainClassic.h"
#include <cmath>

using namespace TD;

DrivetrainClassic &DrivetrainClassic::GetInstance()
{
	static DrivetrainClassic instance;
	return instance;
}

void DrivetrainClassic::Initialize(unsigned int frontRight, unsigned int frontLeft, unsigned int backRight, unsigned int backLeft, unsigned int encoderRightA, unsigned int encoderRightB, unsigned int encoderLeftA, unsigned int encoderLeftB)
{
	m_frontRight = new VictorSP(frontRight);
	m_frontLeft = new VictorSP(frontLeft);
	m_backRight = new VictorSP(backRight);
	m_backLeft = new VictorSP(backLeft);

	// initialize encoders
	m_rightEncoder = new Encoder(encoderRightA, encoderRightB);
	m_leftEncoder = new Encoder(encoderLeftA, encoderLeftB);

	// initialize motorcontrollergroups
	m_right = new MotorControllerGroup(*m_frontRight, *m_backRight);
	m_left = new MotorControllerGroup(*m_frontLeft, *m_backLeft);

	// initialize drivetrain
	m_drive = new DifferentialDrive(*m_left, *m_right);
}

// --------------------- Control ----------------------

// --------------------- Encoders ---------------------

double DrivetrainClassic::GetRightEncoder()
{
	return m_rightEncoder->Get();
}

double DrivetrainClassic::GetRightEncoderTotal()
{
	return m_rightEncoderTotal + GetRightEncoder();
}

double DrivetrainClassic::GetLeftEncoder()
{
	return m_leftEncoder->Get();
}

double DrivetrainClassic::GetLeftEncoderTotal()
{
	return m_leftEncoderTotal + GetLeftEncoder();
}

void DrivetrainClassic::ResetEncoders()
{
	m_rightEncoderTotal = GetRightEncoderTotal();
	m_leftEncoderTotal = GetLeftEncoderTotal();
	m_rightEncoder->Reset();
	m_leftEncoder->Reset();
}

void DrivetrainClassic::InvertRightEncoder(bool invert)
{
	m_rightEncoder->SetReverseDirection(invert);
}

void DrivetrainClassic::InvertLeftEncoder(bool invert)
{
	m_leftEncoder->SetReverseDirection(invert);
}

void DrivetrainClassic::PrintEncoders()
{
	SmartDashboard::PutNumber(GetName() + " Right Encoder", GetRightEncoder());
	SmartDashboard::PutNumber(GetName() + " Left Encoder", GetLeftEncoder());
}

void DrivetrainClassic::SetPositionConversionFactor(double pcf_meters)
{
	m_leftEncoder->SetDistancePerPulse(pcf_meters);
	m_rightEncoder->SetDistancePerPulse(pcf_meters);
}
