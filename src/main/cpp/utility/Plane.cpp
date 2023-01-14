
#include "utility/Plane.h"

namespace TD { namespace utility {

    template<int T>
	auto BoundPlane<T>::origin() -> Point2D {
        return Point2D(0, 0);
	}


    template<int T>
	auto BoundPlane<T>::topCenter() -> Point2D {
        return Point2D(0, axisLength);
	}

    template<int T>
	auto BoundPlane<T>::bottomCenter() -> Point2D {
        return Point2D(0, -axisLength);
	}

    template<int T>
	auto BoundPlane<T>::leftCenter() -> Point2D {
        return Point2D(-axisLength, 0);
	}

    template<int T>
	auto BoundPlane<T>::rightCenter() -> Point2D {
        return Point2D(axisLength, 0);
	}


    template<int T>
	auto BoundPlane<T>::bottomLeft() -> Point2D {
        return Point2D(-axisLength, -axisLength);
	}

    template<int T>
	auto BoundPlane<T>::bottomRight() -> Point2D {
        return Point2D(axisLength, -axisLength);
	}

    template<int T>
	auto BoundPlane<T>::topLeft() -> Point2D {
        return Point2D(-axisLength, axisLength);
	}

    template<int T>
	auto BoundPlane<T>::topRight() -> Point2D {
        return Point2D(axisLength, axisLength);
	}


}}