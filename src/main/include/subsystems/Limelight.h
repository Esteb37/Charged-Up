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

#include <frc/smartdashboard/SmartDashboard.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace frc;

namespace TD
{
	class Limelight
	{

	public:
		Limelight();

		static Limelight &GetInstance();

		/**
		 * @brief Configure the limelight's mount properties
		 * @param angle The limelight's facing angle
		 * @param height Height from floor to center of camera
		 */
		void Initialize(double, double);

		// ---------- Actions ----------

		double GetDistanceToTarget(double);

		// ---------- Getters ----------

		double GetHorizontalAngle();

		double GetVerticalAngle();

		double GetArea();

		double GetSkew();

		double GetPipeline();

		double GetValid();

		double GetTarget();

		// ---------- Setters ----------

		void SetMountAngle(double);

		void SetMountHeight(double);

		void SetPipeline(int);

		void SetLED(int);

		void SetCamMode(int);

		void SetStream(int);

		void SetSnapshot(int);

		void SetLight(int);

		void SetLEDMode(int);

		void SetLEDFrequency(int);

		void SetLEDIntensity(int);

		void SetLEDState(int);

		void SetLEDOff();

		void SetLEDOn();

		// ---------- Printers ----------

		void PrintHorizontalAngle();

		void PrintVerticalAngle();

		void PrintDistanceToTarget(double);

		void PrintArea();

		void PrintSkew();

	protected:
		double m_mountAngle = 45;

		double m_mountHeight = 60;

		std::shared_ptr<nt::NetworkTable> *limelight;
	};

}