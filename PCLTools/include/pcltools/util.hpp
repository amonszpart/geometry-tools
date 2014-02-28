#ifndef __SMARTGEOMETRY_UTIL_HPP__
#define __SMARTGEOMETRY_UTIL_HPP__

#include "pcl/point_cloud.h"
#include "pcl/PointIndices.h"

#include <pcl/features/normal_3d.h> // normalsegmentation

#include "pcltools/primitives.h"

namespace smartgeometry
{
    // untested
    template <typename PointT>
    int
    addGaussianNoise( boost::shared_ptr<pcl::PointCloud<PointT> >      & cloud
                      , std::vector<int>                          const* indices
                      , const Eigen::Vector3f                            stddevs = { 0.003f, 0.003f, 0.003f }
                      , const Eigen::Vector3f                            means   = Eigen::Vector3f::Zero() )
    {
        if ( !cloud )
        {
            std::cerr << "[" << __func__ << "]" << "input cloud empty...nothing to do" << std::endl;
            return EXIT_FAILURE;
        }

        std::random_device rd;
        std::mt19937 gen(rd());

        const int id_limit = indices ? indices->size() : cloud->size();
        for ( size_t id = 0; id != id_limit; ++id )
        {
            int point_index = indices ? (*indices)[id] : id;
            for ( int d = 0; d != 3; ++d )
            {
                std::normal_distribution<> nd( means[d] + cloud->at(point_index).data[d], stddevs[d] );
                cloud->at( id ).data[d] = nd( gen );
            }
        }

        return EXIT_SUCCESS;
    }

    // untested
    template <typename MyPointT>
    int
    calculateNormals(   pcl::PointCloud<pcl::Normal>::Ptr                   &cloud_normals_ptr
                        , boost::shared_ptr<const pcl::PointCloud<MyPointT> >  in_cloud_ptr
                        , pcl::PointIndices::ConstPtr                          in_indices_ptr
                        , bool                                                 normalize
                        , int                                                  normal_estimation_K )
    {
        int ret = EXIT_SUCCESS;

        if (!cloud_normals_ptr)     cloud_normals_ptr.reset( new pcl::PointCloud<pcl::Normal> );

        // Normals
        pcl::PointCloud<pcl::Normal>::Ptr roi_cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
        {
            pcl::NormalEstimation<MyPointT, pcl::Normal> normal_estimation;
            typename pcl::search::KdTree<MyPointT>::Ptr  tree( new pcl::search::KdTree<MyPointT> );

            normal_estimation.setSearchMethod( tree                );
            normal_estimation.setInputCloud  ( in_cloud_ptr        );
            normal_estimation.setKSearch     ( normal_estimation_K );
            if ( in_indices_ptr )
                normal_estimation.setIndices( in_indices_ptr );

            normal_estimation.compute        ( *roi_cloud_normals_ptr );
        }

        // resize normals cloud to match cloud_ptr, copy estimated ones to right positions, leave others as 0
        if ( in_indices_ptr )
        {
            ret += smartgeometry::Primitives::expandNormals( /*   out_cloud: */ cloud_normals_ptr,
                                                             /*   in_indice: */ in_indices_ptr,
                                                             /*  in_normals: */ roi_cloud_normals_ptr,
                                                             /* output_size: */ in_cloud_ptr->size() );
        }
        else
        {
            cloud_normals_ptr = roi_cloud_normals_ptr;
        }

        if ( normalize )
        {
            const size_t siz = cloud_normals_ptr->size();
            float norm = 1.f;
            for ( size_t pid = 0; pid != siz; ++pid )
            {
                norm = Eigen::Map<Eigen::Vector3f>( cloud_normals_ptr->at(pid).normal ).norm();
                cloud_normals_ptr->at(pid).normal[0] /= norm;
                cloud_normals_ptr->at(pid).normal[1] /= norm;
                cloud_normals_ptr->at(pid).normal[2] /= norm;
            }
        }

        return ret;
    }

    /**
     * @brief getNeighbourhoodIndices   Get's a list of neighbours for each point in pointcloud/indices_arg, indices untested
     * @param[out] neighbour_indices    List of list of neighbour indices. One list for each point in cloud.
     * @param[in ] cloud                3D point cloud.
     * @param[in ] indices_arg          Optional, if given, selects point from cloud (untested).
     * @param[out] p_distances          Optional, neighbourhood squared distances arranged as neighbour_indices.
     * @param[in ] K                    Maximum number of neighbours
     * @param[in ] radius               Optional, maximum radius to look for K neighbours in
     */
    template <typename MyPointT>
    static int
    getNeighbourhoodIndices( std::vector<std::vector<int> >                            & neighbour_indices
                            , boost::shared_ptr<pcl::PointCloud<MyPointT> >              cloud
                            , std::vector<int>                                    const* indices_arg        = NULL
                            , std::vector<std::vector<float> >                         * p_distances        = NULL
                            , int                                                        K                  = 15
                            , float                                                      radius             = -1.f )
    {
        // prepare output
        const int   N              = indices_arg ? indices_arg->size() : cloud->size();
        const bool  doRadiusSearch = radius > 0.f;

        neighbour_indices.resize( N, std::vector<int>(K) );
        if ( p_distances )    p_distances->resize( N );

        // create KdTree
        typename pcl::search::KdTree<MyPointT>::Ptr tree( new pcl::search::KdTree<MyPointT> );
        if ( indices_arg )
        {
            pcl::IndicesPtr indices_ptr( new std::vector<int>() );
            *indices_ptr = *indices_arg; // copy indices
            tree->setInputCloud( cloud, indices_ptr );
        }
        else
            tree->setInputCloud( cloud );

        MyPointT            searchPoint;
        std::vector<float>  sqr_dists( K );
        int                 found_points_count = 0;
        for ( size_t pid = 0; pid != N; ++pid )
        {
            // copy search point
            searchPoint = indices_arg ? cloud->at( (*indices_arg)[pid] )
                                      : cloud->at( pid                 );

            // calculate neighbourhood indices
            if ( doRadiusSearch ) found_points_count = tree->radiusSearch  ( searchPoint, radius, neighbour_indices[pid], sqr_dists, K );
            else                  found_points_count = tree->nearestKSearch( searchPoint,      K, neighbour_indices[pid], sqr_dists    );

            // output distances
            if ( found_points_count > 0 )
            {
                if ( p_distances )
                {
                    p_distances->at(pid) = sqr_dists;
                }
            }
            else
            {
                // clear results for this point
                neighbour_indices[pid].resize(0);
                if ( p_distances )
                    p_distances->at(pid).resize(0);
                // report error
                std::cerr << __func__ << ": no neighs found for point " << pid << std::endl;
            }
        }

        return EXIT_SUCCESS;
    }


    template <typename PointsT, typename Scalar> inline int
    computeCentroid( Eigen::Matrix<Scalar,4,1>       & centroid
                     , PointsT                  const& cloud
                     , std::vector<int>         const* indices_arg )
    {
        centroid.setZero();

        const int N = indices_arg ? indices_arg->size() : cloud.size();
        for (size_t pid = 0; pid != N; ++pid )
        {
            const int index = indices_arg ? (*indices_arg)[pid] : pid;
            centroid[0] += cloud[index].x;
            centroid[1] += cloud[index].y;
            centroid[2] += cloud[index].z;
        }
        centroid /= static_cast<Scalar>( N );

        return EXIT_SUCCESS;
    }

    // untested with indices
    template <typename PointsT, typename Scalar> inline int
    computeCovarianceMatrix( Eigen::Matrix<Scalar, 3, 3>         & covariance_matrix
                             , PointsT                      const& cloud
                             , std::vector<int>             const* indices_arg
                             , Eigen::Matrix<Scalar, 4, 1>  const* centroid_arg
                             , std::vector<Scalar>          const* weights_arg
                             )
    {
        // Initialize to 0
        covariance_matrix.setZero();

        const int N = indices_arg ? indices_arg->size() : cloud.size();

        // init centroid
        Eigen::Matrix<Scalar,4,1> centroid; centroid.setZero();
        if ( centroid_arg )
            centroid = *centroid_arg;
        else
            computeCentroid( centroid, cloud, indices_arg );

        // For each point in the cloud
        for ( size_t pid = 0; pid != N; ++pid )
        {
            const int    index  = indices_arg ? (*indices_arg)[pid  ] : pid;
            const Scalar weight = weights_arg ? (*weights_arg)[pid  ] : 1;

            Eigen::Matrix<Scalar, 4, 1> pt;
            pt[0] = cloud[index].x - centroid[0];
            pt[1] = cloud[index].y - centroid[1];
            pt[2] = cloud[index].z - centroid[2];

            covariance_matrix (1, 1) += weight * pt.y () * pt.y ();
            covariance_matrix (1, 2) += weight * pt.y () * pt.z ();
            covariance_matrix (2, 2) += weight * pt.z () * pt.z ();

            pt *= pt.x ();

            covariance_matrix (0, 0) += weight * pt.x ();
            covariance_matrix (0, 1) += weight * pt.y ();
            covariance_matrix (0, 2) += weight * pt.z ();
        }
        covariance_matrix (1, 0) = covariance_matrix( 0, 1);
        covariance_matrix (2, 0) = covariance_matrix( 0, 2);
        covariance_matrix (2, 1) = covariance_matrix( 1, 2);

        if ( weights_arg )
            covariance_matrix /= static_cast<Scalar>( std::accumulate(weights_arg->begin(), weights_arg->end(), 0.) );
        else
            covariance_matrix /= static_cast<Scalar>( N );

        return EXIT_SUCCESS;
    }

} // ns smartgeometry

#endif // __SMARTGEOMETRY_UTIL_HPP__
