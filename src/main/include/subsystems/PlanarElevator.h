#pragma once

#include <frc2/command/SubsystemBase.h>

#include "subsystems/ElevatorBase.h"
#include "../utility/Plane.h"

namespace TD {
  class PlanarElevator : frc2::SubsystemBase {
    public:

    PlanarElevator() = default;

    void Periodic() override;

    void MoveTo(utility::Point2D const &point);

    void MoveHorizontally(float amount);
    void MoveVertically(float amount);

    // TODO: Translate into commands.

    void GotoTopLeft();
    void GotoTopRight();
    void GotoBottomLeft();
    void GotoBottomRight();

    void GotoOrigin();

    void GotoPositiveX();
    void GotoNegativeX();
    void GotoPositiveY();
    void GotoNegativeY();

    private:

    ElevatorBase *xAxis;
    ElevatorBase *yAxis;

    utility::BoundPlane<1> plane{};
    
  };
}
