#include "pcltools/primitives.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace smartgeometry
{

    // resize normals cloud to match cloud_ptr, copy estimated ones to right positions, leave others as 0
    int
    Primitives::expandNormals( pcl::PointCloud<pcl::Normal>::Ptr      &cloud_normals_ptr,
                               pcl::PointIndices::ConstPtr             in_indices_ptr,
                               pcl::PointCloud<pcl::Normal>::ConstPtr  roi_cloud_normals_ptr,
                               int                                     output_size           )
    {
        int return_value = EXIT_SUCCESS;

        // allocate ptr
        if ( !cloud_normals_ptr ) cloud_normals_ptr = pcl::PointCloud<pcl::Normal>::Ptr( new pcl::PointCloud<pcl::Normal>() );
        // resize content
        cloud_normals_ptr->resize( output_size );

        // copy non-zeros
        for ( int index_id = 0;
                  index_id < in_indices_ptr->indices.size();
                ++index_id )
        {
            try
            {
                // copy the next normal to its place indexed by the values in "in_indices"
                cloud_normals_ptr->at( in_indices_ptr->indices[index_id] ) = roi_cloud_normals_ptr->at( index_id );
            }
            catch ( std::exception ex)
            {
                //std::cout << "index_id: " << index_id << std::endl;
                //std::cout << "in_indices_ptr->indices[index_id]: " << in_indices_ptr->indices[index_id] << std::endl;
                //std::cout << "roi_cloud_normals_ptr->at( index_id ): "<< roi_cloud_normals_ptr->at( index_id ) << std::endl;

                std::cerr << __func__ << ": probably indexing error..." << ex.what() << std::endl;

                return_value = EXIT_FAILURE;
            }
        }

        return return_value;
    }

    // TESTING //
    int
    Primitives::test( bool verbose, bool display )
    {
        // start
        if ( verbose )
            std::cout << "Testing Primitives class..." << std::endl;

        // init
        int return_value = EXIT_SUCCESS;

        // read
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud  ( new pcl::PointCloud<pcl::PointXYZRGB>() );
        pcl::PointIndices::Ptr                 indices( new pcl::PointIndices() );
        {
            return_value += pcl::io::loadPCDFile( "../test/data/double_sofa.pcd", *cloud );
            if ( return_value != EXIT_SUCCESS )
            {
                if ( verbose )
                    std::cerr << __func__ << ": could not read test data from ../test/data/double_sofa.pcd" << std::endl;
                return return_value;
            }

            // mockup indices with 90% of the points
            indices->indices.reserve( cloud->size() );
            srand(time(NULL));
            for ( int point_id = 0; point_id < cloud->size(); ++point_id )
            {
                if ( rand()/(float)RAND_MAX < .9f )
                    indices->indices.push_back( point_id );
            }
        }

        // extractPlane
        {
            pcl::ModelCoefficients::Ptr plane_coeffs( new pcl::ModelCoefficients() );
            pcl::PointIndices::Ptr      inliers     ( new pcl::PointIndices()      );
            return_value += extractPlane<pcl::PointXYZRGB>( /*          out_coeffs: */ plane_coeffs
                                                            , /*          in_cloud: */ cloud
                                                            , /*       out_indices: */ &inliers
                                                            , /*        in_indices: */ indices
                                                            , /* distanceThreshold: */ .08
                                                            , /*         SAC model: */ pcl::SACMODEL_NORMAL_PLANE
                                                            , /*            normal: */ NULL
                                                            , /*     neighbourhood: */ 10
                                                            , /*        SAC method: */ pcl::SAC_PROSAC
                                                            , /*          max_iter: */ 900 );

            // get a perpendicular plane
            Eigen::Vector3f plane1_normal;
            plane1_normal << plane_coeffs->values[0], plane_coeffs->values[1], plane_coeffs->values[2];

            pcl::ModelCoefficients::Ptr plane_coeffs2( new pcl::ModelCoefficients() );
            return_value += extractPlane<pcl::PointXYZRGB>( /*          out_coeffs: */ plane_coeffs2
                                                            , /*          in_cloud: */ cloud
                                                            , /*       out_indices: */ NULL
                                                            , /*        in_indices: */ indices
                                                            , /* distanceThreshold: */ .08
                                                            , /*         SAC model: */ pcl::SACMODEL_NORMAL_PARALLEL_PLANE
                                                            , /*            normal: */ &plane1_normal
                                                            , /*     neighbourhood: */ 15
                                                            , /*        SAC method: */ pcl::SAC_RANSAC
                                                            , /*          max_iter: */ 500 );

            if ( verbose )
                std::cout << "\t Primitives::extractPlane " << ((return_value == EXIT_SUCCESS) ? "PASSED" : "FAILED" ) << std::endl;
            if ( display && (return_value == EXIT_SUCCESS) )
            {
                pcl::visualization::PCLVisualizer::Ptr vptr(  new pcl::visualization::PCLVisualizer() );
                vptr->addPointCloud( cloud, "cloud", 0 );
                vptr->addPlane( *plane_coeffs, 0,0,0,"plane",0 );
                vptr->addPlane( *plane_coeffs2, 0,0,0,"plane2",0 );
                vptr->spin();
            }
        }

        // finish
        if ( verbose )
            std::cout << "Primitives class test " << ((return_value == EXIT_SUCCESS) ? "PASSED" : "FAILED" ) << std::endl;

        // return
        return return_value;
    }

} // namespace smartgeom
