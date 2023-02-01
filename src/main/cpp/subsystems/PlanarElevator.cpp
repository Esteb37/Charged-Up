#include "subsystems/PlanarElevator.h"

#include "Constants.h"

namespace TD
{
    PlanarElevator::PlanarElevator(Elevator *xAxis, Elevator *yAxis): xAxis(xAxis), yAxis(yAxis)
    {
        yAxis->SetLimitSafety(true);
        xAxis->SetLimitSafety(true);
        yAxis->SetName("Y Axis");
        xAxis->SetName("X Axis");
    }

    void PlanarElevator::Periodic()
    {
    }

    double PlanarElevator::GetX()
    {
        return xAxis->GetPosition();
    }

    double PlanarElevator::GetY()
    {
        return yAxis->GetPosition();
    }

    utility::Point2D PlanarElevator::GetPosition()
    {
        return utility::Point2D(GetX(), GetY());
    }

    bool PlanarElevator::MoveTo(utility::Point2D &point, double speed)
    {
        return xAxis->SetPosition(point.GetX(), speed) &&
               yAxis->SetPosition(point.GetY(), speed);
    }

    bool PlanarElevator::MoveBy(utility::Vector2D &vec, double speed)
    {
        return xAxis->SetPosition(GetX() + vec.GetX(), speed) &&
               yAxis->SetPosition(GetY() + vec.GetY(), speed);
    }

    void PlanarElevator::MoveHorizontally(float amount)
    {
        xAxis->Move(amount);
    }

    void PlanarElevator::MoveVertically(float amount)
    {
        yAxis->Move(amount);
    }

    frc2::CommandPtr PlanarElevator::GotoPositiveX(double speed)
    {
        return frc2::cmd::Run([this] {}).Until([this, &speed] {
            utility::Point2D target = utility::Point2D(plane.axisLength, GetY());
            return MoveTo(target, speed); });
    }

    frc2::CommandPtr PlanarElevator::GotoPositiveY(double speed)
    {
        return frc2::cmd::Run([this] {}).Until([this, &speed] {
            utility::Point2D target = utility::Point2D(GetX(), plane.axisLength);
            return MoveTo(target, speed);
        });            
    }

    frc2::CommandPtr PlanarElevator::GotoNegativeX(double speed)
    {
        return frc2::cmd::Run([this] {}).Until([this, &speed] {
            utility::Point2D target = utility::Point2D(-plane.axisLength, GetY());
            return MoveTo(target, speed); });
    }

    frc2::CommandPtr PlanarElevator::GotoNegativeY(double speed)
    {
        return frc2::cmd::Run([this] {}).Until([this, &speed] {
            utility::Point2D target = utility::Point2D(GetX(), -plane.axisLength);
            return MoveTo(target, speed); });
    }

    frc2::CommandPtr PlanarElevator::CenterHorizontally(double speed)
    {
        return frc2::cmd::Run([this] {}).Until([this, &speed] {
            utility::Point2D target = utility::Point2D(0, GetY());
            return MoveTo(target, speed); });
    }

    frc2::CommandPtr PlanarElevator::CenterVertically(double speed)
    {
        return frc2::cmd::Run([this] {}).Until([this, &speed] {
            utility::Point2D target = utility::Point2D(GetX(), 0);
            return MoveTo(target, speed); });
    }

    frc2::CommandPtr PlanarElevator::GotoTopRight(double speed)
    {
        return GotoPositiveX(speed)
            .AlongWith(std::move(GotoPositiveY(speed)));
    }

    frc2::CommandPtr PlanarElevator::GotoTopLeft(double speed)
    {
        return GotoNegativeX(speed)
            .AlongWith(std::move(GotoPositiveY(speed)));
    }

    frc2::CommandPtr PlanarElevator::GotoBottomRight(double speed)
    {
        return GotoPositiveX(speed)
            .AlongWith(std::move(GotoNegativeY(speed)));
    }

    frc2::CommandPtr PlanarElevator::GotoBottomLeft(double speed)
    {
        return GotoNegativeX(speed)
            .AlongWith(std::move(GotoNegativeY(speed)));
    }

    frc2::CommandPtr PlanarElevator::GotoOrigin(double speed)
    {
        return CenterHorizontally(speed)
            .AlongWith(std::move(CenterVertically(speed)));
    }

}