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
    Date: 17/01/2023
    Language: cpp
    Copyright (c) TecDroid 3354 and Esteban Padilla Cerdio
    Open Source Software; you can modify and/or share it under the terms of
*/

#pragma once

#include "Constants.h"
#include <AHRS.h>
#include <frc/ADIS16448_IMU.h>
#include <frc/geometry/Pose3D.h>
#include <frc/interfaces/Gyro.h>
#include <frc/smartdashboard/SmartDashboard.h>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace frc;
using namespace std;

namespace TD
{

    namespace GyroTypes
    {
        typedef ADIS16448_IMU CLASSIC;
        typedef AHRS NAVX;
    }

    class CustomGyroBase
    {
    public:
        CustomGyroBase() = default;
        virtual ~CustomGyroBase() = default;
        virtual units::angle::degree_t GetAngle() = 0;
        virtual units::angle::degree_t GetHeading() = 0;
        virtual void ResetAngle() = 0;
        virtual void ResetDisplacement() = 0;
        virtual void Reset() = 0;
        virtual void Invert(bool) = 0;
        virtual void Print() = 0;
        virtual void PrintAngles() = 0;
        virtual void PrintYaw() = 0;
        virtual void PrintPitch() = 0;
        virtual void PrintRoll() = 0;
        virtual frc::Rotation2d GetRotation2d() = 0;
        virtual units::angle::degree_t GetYaw() = 0;
        virtual units::angle::degree_t GetPitch() = 0;
        virtual units::angle::degree_t GetRoll() = 0;
        virtual void Calibrate() = 0;
    };

    template <class T>
    class CustomGyro : public CustomGyroBase
    {
    public:
        CustomGyro();

        // ----------------------- Gyro -----------------------

        /**
         * @brief Gets the current angle of the gyro in degrees
         * @return double angle
         */
        units::angle::degree_t GetAngle() override;

        /**
         * @brief Get the absolute angle to which the robot is heading
         * @return double the absolute angle
         */
        units::angle::degree_t GetHeading() override;

        /**
         * @brief Resets the angle to 0
         */
        void ResetAngle() override;

        void ResetDisplacement() override;

        void Reset() override;

        /**
         * @brief Invert the direction of the gyro
         * @param invert True to invert, false to not
         */
        void Invert(bool) override;

        /**
         * @brief Publish the value of the gyro to the dashboard
         */
        void Print() override;

        void PrintAngles() override;

        void PrintYaw() override;

        void PrintPitch() override;

        void PrintRoll() override;

        frc::Rotation2d GetRotation2d() override;

        /**
         * @brief Gets the current angle of the gyro in degrees
         * @return double angle
         */
        units::angle::degree_t GetYaw() override;

        /**
         * @brief Gets the current angle of the gyro in degrees
         * @return double angle
         */
        units::angle::degree_t GetPitch() override;

        /**
         * @brief Gets the current angle of the gyro in degrees
         * @return double angle
         */
        units::angle::degree_t GetRoll() override;

        void Calibrate() override;

    private:
        T *m_gyro;

        units::angle::degree_t m_heading;

        char m_direction = 1;
    };

}