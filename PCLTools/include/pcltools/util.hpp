#ifndef __SMARTGEOMETRY_UTIL_HPP__
#define __SMARTGEOMETRY_UTIL_HPP__

#include <random>

#include "pcl/point_cloud.h"
#include "pcl/PointIndices.h"
#include "pcl/features/normal_3d.h" // normalsegmentation
#include <pcl/common/transforms.h> // transformpointcloud
#include "pcltools/primitives.h"


#define DEBUG_FITLINE 0
#if DEBUG_FITLINE
    #include <pcl/visualization/pcl_visualizer.h>
#endif

#define SG_STATIC_ASSERT( condition, name )\
    typedef char assert_failed_ ## name [ (condition) ? 1 : -1 ];

namespace smartgeometry
{
    template <typename inCloudT> inline ::pcl::PointIndices::Ptr
    allIndicesOf( inCloudT cloud_ptr )
    {
        ::pcl::PointIndices::Ptr indices_ptr( new ::pcl::PointIndices() );
        indices_ptr->indices.reserve( cloud_ptr->size() );

        for ( int i = 0; i != cloud_ptr->size(); ++i )
            indices_ptr->indices.push_back( i );

        return indices_ptr;
    }

    inline ::pcl::PointXYZ
    toPointXYZ( Eigen::Vector3f const& vector3f )
    {
        return ::pcl::PointXYZ( vector3f.x(), vector3f.y(), vector3f.z() );
    }

    // untested
    template <typename PointT>
    int
    addGaussianNoise( boost::shared_ptr<pcl::PointCloud<PointT> >      & cloud
                      , std::vector<int>                          const* indices
                      , Eigen::Vector3f                           const  stddevs = { 0.003f, 0.003f, 0.003f }
                      , Eigen::Vector3f                           const  means   = Eigen::Vector3f::Zero()
                      , Eigen::Vector3f                           const* sensor_origin = NULL )
    {
        if ( !cloud )
        {
            std::cerr << "[" << __func__ << "]" << "input cloud empty...nothing to do" << std::endl;
            return EXIT_FAILURE;
        }

        std::random_device rd;
        std::mt19937 gen(rd());

        const int id_limit = indices ? indices->size() : cloud->size();
        std::vector<float> dists( id_limit, 0.f );
        float max_distance = 0.f;

        // get max distance
        if ( sensor_origin )
            for ( size_t id = 0; id != id_limit; ++id )
            {
                int point_index = indices ? (*indices)[id] : id;

                if ( (dists[id]=(*sensor_origin - cloud->at( point_index ).getVector3fMap()).norm()) > max_distance )
                    max_distance = dists[id];
            }

        // modify eacch point
        for ( size_t id = 0; id != id_limit; ++id )
        {
            int point_index = indices ? (*indices)[id] : id;

            for ( int d = 0; d != 3; ++d )
            {
                float dev = stddevs[d];
                if ( sensor_origin )
                {
                    float coeff = dists[id] / (max_distance);
                    dev *= std::max( 0.3f, coeff * coeff);
//                    std::cout << "dev is now " << dev << ", where dist is " << dists[id]
//                              << ", point is "
//                              << cloud->at(point_index).data[0] << ", "
//                              << cloud->at(point_index).data[1] << ", "
//                              << cloud->at(point_index).data[2] << ", "
//                              << ", max_distance is " << max_distance << std::endl;
                }

                std::normal_distribution<> nd( means[d] + cloud->at(point_index).data[d], dev );
                cloud->at( point_index ).data[d] = nd( gen );
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
     * @param[in ] soft_radius          Return K neighbours even if some outside radius
     */
    template <typename MyPointT>
    static int
    getNeighbourhoodIndices( std::vector<std::vector<int> >                            & neighbour_indices
                            , boost::shared_ptr<pcl::PointCloud<MyPointT> >              cloud
                            , std::vector<int>                                    const* indices_arg        = NULL
                            , std::vector<std::vector<float> >                         * p_distances        = NULL
                            , int                                                        K                  = 15
                            , float                                                      radius             = -1.f
                            , bool                                                       soft_radius        = false
                            )
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

            if ( (found_points_count <2 ) && (soft_radius) )
                found_points_count = tree->nearestKSearch( searchPoint,      K, neighbour_indices[pid], sqr_dists    );

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
        {
            Scalar sum_weight = std::accumulate(weights_arg->begin(), weights_arg->end(), 0.);
            if ( sum_weight > FLT_EPSILON ) covariance_matrix /= sum_weight;
        }
        else
        {
            if ( N == 0 ) std::cerr << "[" << __func__ << "]: " << "dividing by N: " << N << std::endl;
            else covariance_matrix /= static_cast<Scalar>( N );
        }

        return EXIT_SUCCESS;
    }

    template <class PointsPtrT, typename Scalar = float> inline int
    PCA( Eigen::Matrix<Scalar,4,4>  &frame,
         PointsPtrT                  cloud_ptr,
         pcl::PointIndices::ConstPtr indices_ptr = NULL ) //TODO: detach from PCL

    {
        Eigen::Matrix<Scalar,4,1> centroid;
        Eigen::Matrix<Scalar,3,3> covariance;
        if ( indices_ptr )
        {
            pcl::compute3DCentroid( /* cloud: */ *cloud_ptr, indices_ptr->indices, centroid );
            pcl::computeCovarianceMatrixNormalized( *cloud_ptr, *indices_ptr, centroid, covariance );
        }
        else
        {
            pcl::compute3DCentroid( /* cloud: */ *cloud_ptr, centroid );
            pcl::computeCovarianceMatrixNormalized( *cloud_ptr, centroid, covariance );
        }

        // eigen decomposition
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar,3,3> > eigen_solver( covariance, Eigen::ComputeEigenvectors );
        // sort decreasing
        Eigen::Matrix<Scalar,3,3> eigen_vectors = eigen_solver.eigenvectors();
        typedef std::pair<Scalar,Eigen::Matrix<Scalar,3,1> > PairT;
        struct SortFunctor { bool operator()( PairT const& a, PairT const& b ) { return a.first > b.first; } }; // > means biggest first
        std::vector<PairT> sorted( 3 );
        sorted[0] = PairT( eigen_solver.eigenvalues()(0), eigen_vectors.col(0) );
        sorted[1] = PairT( eigen_solver.eigenvalues()(1), eigen_vectors.col(1) );
        sorted[2] = PairT( eigen_solver.eigenvalues()(2), eigen_vectors.col(2) );
        std::sort( sorted.begin(), sorted.end(), SortFunctor() );

        // orthogonalize
        eigen_vectors.col(2) = eigen_vectors.col(0).cross( eigen_vectors.col(1) );

        // debug
        if ( eigen_vectors(0,0) != eigen_vectors(0,0)
             || eigen_vectors(1,1) != eigen_vectors(1,1)
             || eigen_vectors(2,2) != eigen_vectors(2,2) )
        {
            std::cerr << "nan eigen matrix" << std::endl;
            return EXIT_FAILURE;
        }

        frame = Eigen::Matrix<Scalar,4,4>::Identity();
        frame.template block<3,1>(0,0) = sorted[0].second; // biggest first
        frame.template block<3,1>(0,1) = sorted[1].second;
        frame.template block<3,1>(0,2) = sorted[2].second;
        frame.template block<3,1>(0,3) = centroid.template head<3>();

        return EXIT_SUCCESS;
    }

    // transforms to local coordinates by inverting a coordinate frame
    template <typename MyPointT, typename Scalar = float> int
    cloud2Local( boost::shared_ptr<       pcl::PointCloud<MyPointT> >        &cloud_ptr,
                 Eigen::Matrix<Scalar,4,4>                            const  &frame,
                 boost::shared_ptr<       pcl::PointCloud<MyPointT> > /* const */ in_cloud_ptr,
                 pcl::PointIndices::ConstPtr                                  indices_ptr  = NULL)
    {
        if ( !cloud_ptr )   cloud_ptr = boost::shared_ptr<pcl::PointCloud<MyPointT> >( new pcl::PointCloud<MyPointT>() );

        Eigen::Matrix<Scalar,4,4> transform( Eigen::Matrix<Scalar,4,4>::Identity() );
        transform.template block<3,3>(0,0) = frame.template block<3,3>(0,0).transpose();
        transform.template block<3,1>(0,3) = -1.f * (transform.template block<3,3>(0,0) * frame.template block<3,1>(0,3));

        if ( indices_ptr )
            pcl::transformPointCloud( *in_cloud_ptr, *indices_ptr, *cloud_ptr, transform );
        else
            pcl::transformPointCloud( *in_cloud_ptr, *cloud_ptr, transform );

        return EXIT_SUCCESS;
    }

    // transforms to local coordinates by inverting a coordinate frame
    template <typename MyPointT, typename Scalar = float> int
    local2World( boost::shared_ptr<       pcl::PointCloud<MyPointT> >       &cloud_ptr,
                 Eigen::Matrix<Scalar,4,4>                            const &frame,
                 boost::shared_ptr<       pcl::PointCloud<MyPointT> > /*const*/ in_cloud_ptr,
                 pcl::PointIndices::ConstPtr                                 indices_ptr   = NULL)
    {
        if ( !cloud_ptr )   cloud_ptr = boost::shared_ptr<pcl::PointCloud<MyPointT> >( new pcl::PointCloud<MyPointT>() );

        if ( indices_ptr )
            pcl::transformPointCloud( *in_cloud_ptr, *indices_ptr, *cloud_ptr, frame);
        else
            pcl::transformPointCloud( *in_cloud_ptr, *cloud_ptr, frame );

        return EXIT_SUCCESS;
    }

    namespace geometry
    {
        // Point2Primitive distance
        template <typename Scalar, int rows> Scalar
        pointPrimitiveDistance (Eigen::Matrix<Scalar,3,1> const& pnt,Eigen::Matrix<Scalar,rows,1> const& primitive);
        template<> inline float
        pointPrimitiveDistance<float,6> (Eigen::Matrix<float,3,1> const& pnt, Eigen::Matrix<float,6,1> const& line )
        {
            return (line.template head<3>() - pnt).cross( line.template segment<3>(3) ).norm();
        }
        template<> inline float
        pointPrimitiveDistance<float,4> (Eigen::Matrix<float,3,1> const& pnt, Eigen::Matrix<float,4,1> const& plane )
        {
            return plane.template head<3>().dot( pnt ) + plane(3);
        }

        // Primitive from point and normal
        template <typename Scalar, int rows> Eigen::Matrix<Scalar,rows,1>
        fromPointAndNormal( Eigen::Matrix<Scalar,3,1> const& pnt,Eigen::Matrix<Scalar,3,1> const& normal );
        template <> inline Eigen::Matrix<float,6,1>
        fromPointAndNormal<float,6>( Eigen::Matrix<float,3,1> const& pnt,Eigen::Matrix<float,3,1> const& normal )
        {
            return (Eigen::Matrix<float,6,1>() << pnt, normal).finished(); // TODO: this is bullshit, it's not the normal, but the direction...
        }
        template <> inline Eigen::Matrix<float,4,1>
        fromPointAndNormal<float,4>( Eigen::Matrix<float,3,1> const& pnt,Eigen::Matrix<float,3,1> const& normal )
        {
            //model_coefficients[3] = -1 * (model_coefficients.template head<4>().dot (p0.matrix ()));
            Eigen::Matrix<float,4,1> primitive;
            primitive.template segment<3>(0) = normal;
            primitive                    (3) = static_cast<float>(-1) * primitive.template head<3>().dot( pnt.template head<3>() ); // distance

            return primitive;
        }

        /**
         * @brief fitLine               [Re]Fits 3D line to a [part of a] pointcloud.
         * @param line                  Output line, and possibly input line to refit, if \param start_from_input_line is true.
         * @param cloud                 Points to fit to. Must have methods operator[] and getVector3fMap()->Eigen::Vector3f. If \param p_indices!=NULL, must have at least max(*p_indices) points.
         * @param scale                 Distance where point get's zero weight
         * @param p_indices             Indices to use from cloud. Can be NULL, in which case the whole cloud is used.
         * @param refit                 How many refit iterations. 0 means once, obviously (TODO to fix...).
         * @param start_from_input_line Assume, that \param line contains a meaningful input, and calculate weights on the 0th iteration already.
         */
        template <class PointsT, typename Scalar = float, int rows = 6> inline int
        fitLinearPrimitive( Eigen::Matrix<Scalar,rows,1>    & primitive
                            , PointsT                  const& cloud
                            , Scalar                          scale
                            , std::vector<int>              * p_indices             = NULL
                            , int                             refit                 = 0
                            , bool                            start_from_input      = false
                            , Scalar                       (*pointPrimitiveDistanceFunc)(Eigen::Matrix<Scalar,3,1> const& pnt, Eigen::Matrix<Scalar,rows,1> const& primitive) = &(pointPrimitiveDistance<Scalar,rows>)
                            , Eigen::Matrix<Scalar,rows,1> (*    fromPointAndNormalFunc)(Eigen::Matrix<Scalar,3,1> const& pnt, Eigen::Matrix<Scalar,3   ,1> const& normal   ) = &(fromPointAndNormal<Scalar,rows>)
                            , bool                            debug                 = false )
        {
            SG_STATIC_ASSERT( (rows == 4) || (rows == 6), smartgeometry_fit_linear_model_rows_not_4_or_6 );

            // number of points to take into account
            const int N = p_indices ? p_indices->size() : cloud.size();

            // skip, if not enought points found to fit to
            if ( N < 2 ) { std::cerr << "[" << __func__ << "]: " << "can't fit line to less then 2 points..." << std::endl; return EXIT_FAILURE; }

            int iteration = 0; // track refit iterations
            do
            {
                // LeastSquares weights
                std::vector<Scalar> weights( N, 1.f );

                // calculate weights, if value in "line" already meaningful
                if ( start_from_input || (iteration > 0) )
                {
                    // calculate distance from all points
                    for ( size_t point_id = 0; point_id != N; ++point_id )
                    {
                        // formula borrowed from PCL: (line_pt - point).cross3(line_dir).squaredNorm();
                        Eigen::Matrix<Scalar,3,1> pnt = cloud[ p_indices ? (*p_indices)[point_id] : point_id ].getVector3fMap();
//                        weights[point_id] = (*pointPrimitiveDistanceFunc)( pnt, primitive );
                        // TODO: template this
//                        switch (rows)
//                        {
//                            case 6:
//                                weights[point_id] = (primitive.template head<3>() - pnt).cross( primitive.template segment<3>(3) ).norm();
//                                break;
//                            case 4:
//                                weights[point_id] = primitive.template head<3>().dot( pnt ) + primitive(3);
//                                break;
//                            default:
//                                std::cerr << "[" << __func__ << "]: " << rows << " not implemented" << std::endl;
//                                return EXIT_FAILURE;
//                                break;
//                        }
                        weights[point_id] = pointPrimitiveDistanceFunc( pnt, primitive );
                    }

                    // the farther away, the smaller weight -->
                    // w_i = f( dist_i / scale ), dist_i < scale; f(x) = (x^2-1)^2
                    for ( size_t wi = 0; wi != weights.size(); ++wi )
                    {
                        if ( weights[wi] < scale )
                        {
                            weights[wi] /= scale;                                               // x = dist_i / scale
                            weights[wi] = (weights[wi] * weights[wi] - static_cast<Scalar>(1)); // x^2-1
                            weights[wi] *= weights[wi];                                         // (x^2-1)^2
                        }
                        else
                            weights[wi] = static_cast<Scalar>(0);                               // outside scale, truncated to 0
                    }
                }

                // compute centroid of cloud or selected points
                Eigen::Matrix<Scalar,4,1> centroid;
                smartgeometry::computeCentroid( centroid, cloud, p_indices );

                // compute neighbourhood covariance matrix
                Eigen::Matrix<Scalar,3,3> cov;
                smartgeometry::computeCovarianceMatrix( cov, cloud, p_indices, &centroid, &weights ); // weights might be all 1-s

                // solve for neighbourhood biggest eigen value
                Eigen::SelfAdjointEigenSolver< Eigen::Matrix<Scalar, 3, 3> > es;
                es.compute( cov );

                if ( rows == 6 ) // line -> dir ==
                {
                    // get eigen vector for biggest eigen value
                    const int max_eig_val_id = std::distance( es.eigenvalues().data(), std::max_element( es.eigenvalues().data(), es.eigenvalues().data()+3 ) );

                    // output line
                    primitive = fromPointAndNormalFunc( centroid.template head<3>(),
                                                        es.eigenvectors().col(max_eig_val_id).normalized() );
                }
                else if ( rows == 4 ) // plane
                {
                    // get eigen vector for biggest eigen value
                    const int min_eig_val_id = std::distance( es.eigenvalues().data(), std::min_element( es.eigenvalues().data(), es.eigenvalues().data()+3 ) );

                    // output line
                    primitive = fromPointAndNormalFunc( centroid.template head<3>(),
                                                        es.eigenvectors().col(min_eig_val_id).normalized() );
                }
                else
                    std::cerr << "[" << __func__ << "]: " << "lines(rows==6) or planes(rows==4), not rows == " << rows << std::endl;

#if DEBUG_FITLINE
                for ( int d = 0; d < 6; ++d )
                {
                    if ( primitive(d) != primitive(d) )
                    {
                        std::cerr << "centroid: \n" << centroid.transpose() << std::endl
                                  << "cov\n: " << cov << std::endl
                                  << "es.eigenvectors():\n" << es.eigenvectors() << std::endl;
                    }
                }

                if ( debug )
                {
                    std::cout << "[" << __func__ << "]: " << line.transpose() << std::endl;
                    std::cout << "[" << __func__ << "]: " << es.eigenvectors().col(0).transpose() << std::endl;
                    std::cout << "[" << __func__ << "]: " << es.eigenvectors().col(1).transpose() << std::endl;
                    std::cout << "[" << __func__ << "]: " << es.eigenvectors().col(2).transpose() << std::endl;

                    char tit[255]; sprintf( tit, "fitline_iteration%d", iteration );
                    pcl::visualization::PCLVisualizer::Ptr vptr( new pcl::visualization::PCLVisualizer(tit) );
                    vptr->setBackgroundColor( .5, .5, .6);
                    vptr->addPointCloud( cloud.makeShared() );
                    vptr->addArrow( toPointXYZ(line.template segment<3>(0) + line.template segment<3>(3) * .1f)
                                    , toPointXYZ(line.template segment<3>(0))
                                    , 1., 1., 0.
                                    , true
                                    , "line" );
                    if ( max_eig_val_id != 0 )
                    vptr->addArrow( toPointXYZ(centroid.template head<3>() + es.eigenvectors().col(0) * .1f)
                                    , toPointXYZ(centroid.template head<3>())
                                    , 1., 0., 0.
                                    , true
                                    , "e0" );
                    if ( max_eig_val_id != 1 )
                    vptr->addArrow( toPointXYZ(centroid.template head<3>() + es.eigenvectors().col(1) * .1f)
                                    , toPointXYZ(centroid.template head<3>())
                                    , 0., 1., 0.
                                    , true
                                    , "e1" );
                    if ( max_eig_val_id != 2 )
                    vptr->addArrow( toPointXYZ(centroid.template head<3>() + es.eigenvectors().col(2) * .1f)
                                    , toPointXYZ(centroid.template head<3>())
                                    , 0., 0., 1.
                                    , true
                                    , "e2" );
                    vptr->spin();
                }
#endif
            }
            while ( iteration++ < refit );

            return EXIT_SUCCESS;
        } // ... fitline


    } // ... ns geometry

} // ns smartgeometry

//template <typename Scalar, int rows> extern
//smartgeometry::geometry::pointPrimitiveDistance( Eigen::Matrix<float,3,1> const& pnt, Eigen::Matrix<float,6,1> const& line );


#endif // __SMARTGEOMETRY_UTIL_HPP__
