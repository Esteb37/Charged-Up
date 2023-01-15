#include "utility/Lines.h"

namespace TD { namespace utility {

    Line2D::Line2D(degree_t standardAngle):
    standardAngle(standardAngle) {}

    Ray2D::Ray2D(Point2D &startingPosition, degree_t standardAngle):
	Line2D(standardAngle), startingPosition(startingPosition) {}

    Segment2D::Segment2D(Point2D &startingPosition, Point2D &endingPosition):
	Ray2D(startingPosition, degree_t{0}), endingPosition(endingPosition) {}

    Line3D::Line3D(degree_t xAngle, degree_t zAngle):
	xAngle(xAngle), zAngle(zAngle) {} 

    Ray3D::Ray3D(Point3D &startingPosition, degree_t xAngle, degree_t zAngle):
	Line3D(xAngle, zAngle), startingPosition(startingPosition) {}

    Segment3D::Segment3D(Point3D &startingPosition, Point3D &endingPosition):
	Ray3D(startingPosition, degree_t{0}, degree_t{0}), endingPosition(endingPosition) {}

    auto Segment2D::squaredLength() -> double {
        return startingPosition.squaredDistanceTo(endingPosition);
    }

    auto Segment2D::length() -> double {
        return startingPosition.distanceTo(endingPosition);
    }

    auto Segment3D::squaredLength() -> double {
        return startingPosition.squaredDistanceTo(endingPosition);
    }

    auto Segment3D::length() -> double {
        return startingPosition.distanceTo(endingPosition);
    }

}}
