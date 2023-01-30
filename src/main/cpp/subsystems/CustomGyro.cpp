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

#include "subsystems/CustomGyro.h"

using namespace frc;
using namespace std;

namespace TD
{

    using namespace GyroTypes;

    template <>
    CustomGyro<CLASSIC>::CustomGyro()
    {
        m_gyro = new CLASSIC();
    }

    template <>
    CustomGyro<NAVX>::CustomGyro()
    {
        m_gyro = new NAVX(SPI::Port::kMXP);
    }

    template <class T>
    units::angle::degree_t CustomGyro<T>::GetAngle()
    {
        return units::angle::degree_t{m_gyro->GetAngle()} * m_direction;
    }

    template <class T>
    units::angle::degree_t CustomGyro<T>::GetHeading()
    {
        return GetAngle() + m_heading;
    }

    template <>
    units::angle::degree_t CustomGyro<CLASSIC>::GetYaw()
    {
        return m_gyro->GetGyroAngleZ();
    }

    template <>
    units::angle::degree_t CustomGyro<NAVX>::GetYaw()
    {
        return units::angle::degree_t{m_gyro->GetYaw()};
    }

    template <>
    units::angle::degree_t CustomGyro<CLASSIC>::GetRoll()
    {
        return m_gyro->GetGyroAngleX();
    }

    template <>
    units::angle::degree_t CustomGyro<NAVX>::GetRoll()
    {
        return units::angle::degree_t{m_gyro->GetRoll()};
    }

    template <>
    units::angle::degree_t CustomGyro<CLASSIC>::GetPitch()
    {
        return m_gyro->GetGyroAngleY();
    }

    template <>
    units::angle::degree_t CustomGyro<NAVX>::GetPitch()
    {
        return units::angle::degree_t{m_gyro->GetPitch()};
    }

    template <class T>
    void CustomGyro<T>::Reset()
    {
        ResetAngle();
        ResetDisplacement();
    }

    template <class T>
    void CustomGyro<T>::ResetAngle()
    {
        m_gyro->Reset();
    }

    template <class T>
    void CustomGyro<T>::ResetDisplacement()
    {
        if (typeid(T) == typeid(NAVX))
        {
            ((NAVX *)m_gyro)->ResetDisplacement();
        }
    }

    template <class T>
    void CustomGyro<T>::Invert(bool inverted)
    {
        m_direction = inverted ? -1 : 1;
    }

    template <class T>
    void CustomGyro<T>::Print()
    {
        SmartDashboard::PutNumber("Gyro", GetAngle().value());
    }

    template <class T>
    void CustomGyro<T>::PrintYaw()
    {
        SmartDashboard::PutNumber("Gyro Yaw", GetAngle().value());
    }

    template <class T>
    void CustomGyro<T>::PrintPitch()
    {
        SmartDashboard::PutNumber("Gyro Pitch", GetPitch().value());
    }

    template <class T>
    void CustomGyro<T>::PrintRoll()
    {
        SmartDashboard::PutNumber("Gyro Roll", GetRoll().value());
    }

    template <class T>
    void CustomGyro<T>::PrintAngles()
    {
        PrintYaw();
        PrintPitch();
        PrintRoll();
    }

    template <class T>
    frc::Rotation2d CustomGyro<T>::GetRotation2d()
    {
        return Rotation2d{-GetHeading()};
    }

    template <class T>
    void CustomGyro<T>::Calibrate()
    {
        m_gyro->Calibrate();
    }

    template class CustomGyro<CLASSIC>;
    template class CustomGyro<NAVX>;
}