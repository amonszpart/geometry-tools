#ifndef __SMARTGEOMETRY_GEOMETRY_H__
#define __SMARTGEOMETRY_GEOMETRY_H__

#include <vector>

namespace smartgeometry
{
    namespace geometry
    {
        template <class PointsT>
        inline int fitLine( PointsT points
                            , std::vector<int> *p_indices );
    }

} // ns smartgeometry

#include "geometry/geometry.hpp"

#endif // __SMARTGEOMETRY_GEOMETRY_H__
