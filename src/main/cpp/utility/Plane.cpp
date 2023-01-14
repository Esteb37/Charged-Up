
#include "utility/Plane.h"

namespace TD { namespace utility {

    template<int T>
	auto BoundPlane<T>::origin() -> const Point2D {
        return Point2D(0, 0);
	}


    template<int T>
	auto BoundPlane<T>::topCenter() -> const Point2D {
        return Point2D(0, axisLength);
	}

    template<int T>
	auto BoundPlane<T>::bottomCenter() -> const Point2D {
        return Point2D(0, -axisLength);
	}

    template<int T>
	auto BoundPlane<T>::leftCenter() -> const Point2D {
        return Point2D(-axisLength, 0);
	}

    template<int T>
	auto BoundPlane<T>::rightCenter() -> const Point2D {
        return Point2D(axisLength, 0);
	}


    template<int T>
	auto BoundPlane<T>::bottomLeft() -> const Point2D {
        return Point2D(-axisLength, -axisLength);
	}

    template<int T>
	auto BoundPlane<T>::bottomRight() -> const Point2D {
        return Point2D(axisLength, -axisLength);
	}

    template<int T>
	auto BoundPlane<T>::topLeft() -> const Point2D {
        return Point2D(-axisLength, axisLength);
	}

    template<int T>
	auto BoundPlane<T>::topRight() -> const Point2D {
        return Point2D(axisLength, axisLength);
	}


}}