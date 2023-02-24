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
        InvertMotors({false,true});
	}
	
	void Intake::Periodic()
	{
	}

	// ---------- Actions -----------

	
	void Intake::Take()
	{
        if(m_motorCount <= 1){
            SetMotor(1);
        } else {
		    SetMotors(1);
        }
	}

    
	void Intake::Spit()
	{
		if(m_motorCount <= 1){
            SetMotor(-1);
        } else {
		    SetMotors(-1);
        }
	}

    
	void Intake::Invert(bool invert)
	{
		if(m_motorCount <= 1){
            InvertMotor(invert);
        } else {
		    InvertMotors({invert,!invert});
        }
	}
}