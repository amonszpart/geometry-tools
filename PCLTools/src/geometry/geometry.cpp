#if 0
#include "pcltools/util.hpp"

#include "Eigen/Dense"

namespace smartgeometry
{
    namespace geometry
    {
//        template<> float
//        pointPrimitiveDistance<float,6>(Eigen::Matrix<float,3,1> const& pnt, Eigen::Matrix<float,6,1> const& line )
//        {
//            return (line.template head<3>() - pnt).cross( line.template segment<3>(3) ).norm();
//        }

        template <> double
        pointPrimitiveDistance (Eigen::Matrix<double,3,1> const& pnt, Eigen::Matrix<double,6,1> const& line )
        {
            return (line.template head<3>() - pnt).cross( line.template segment<3>(3) ).norm();
        }

        template <> float
        pointPrimitiveDistance<float,4> (Eigen::Matrix<float,3,1> const& pnt, Eigen::Matrix<float,4,1> const& plane )
        {
            //coeff.dot (p) + model_coefficients[3]
            //return (line.template head<3>() - pnt).cross( line.template segment<3>(3) ).norm();
        }

        template <> double
        pointPrimitiveDistance (Eigen::Matrix<double,3,1> const& pnt, Eigen::Matrix<double,4,1> const& plane )
        {
            //return (line.template head<3>() - pnt).cross( line.template segment<3>(3) ).norm();
        }

    } // ... ns geometry
} // ... ns smartgeometry
#endif
