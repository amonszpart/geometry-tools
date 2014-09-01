#ifndef __SMARTGEOMETRY_GEOMETRY_HPP__
#define __SMARTGEOMETRY_GEOMETRY_HPP__

#include <vector>
#include <iostream>

namespace smartgeometry
{
    namespace geometry
    {
        template <class PointsT>
        inline int fitLine( PointsT points
                            , std::vector<int> *p_indices )
        {
            const int N = p_indices ? p_indices->size() : points.size();
            if ( N < 2 ) { std::cerr << "[" << __func__ << "]: " << "can't fit line to less then 2 points..." << std::endl; return EXIT_FAILURE; }

            // skip, if no neighbours found this point won't contribute a primitive for now
            if ( neighs[pid].size() < 2 )  continue;

            // compute neighbourhood cov matrix
            Eigen::Matrix<Scalar,3,3> cov;
            smartgeometry::computeCovarianceMatrix<pcl::PointCloud<PointT>,float>( cov, *cloud, &(neighs[pid]), NULL, NULL );
            // solve for neighbourhood biggest eigen value
            Eigen::SelfAdjointEigenSolver< Eigen::Matrix<Scalar, 3, 3> > es;
            es.compute( cov );
            // get eigen vector for biggest eigen value
            const int max_eig_val_id = std::distance( es.eigenvalues().data(), std::max_element( es.eigenvalues().data(), es.eigenvalues().data()+3 ) );
            Eigen::Matrix<Scalar,3,1> eig2 = es.eigenvectors().col( max_eig_val_id ).normalized();
            // compute line direction perpendicular to eigen vector
            Eigen::Matrix<Scalar,3,1> p0 = cloud->at(pid).getVector3fMap();                                      // point on line

        }
    }

} // ns smartgeometry

#endif // __SMARTGEOMETRY_GEOMETRY_HPP__
