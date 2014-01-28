#ifndef PRIMITIVES_HPP
#define PRIMITIVES_HPP
#include "Primitives.h"

#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include "pcl/common/angles.h"

namespace smartgeom
{
    template <typename MyPointT>
    int
    Primitives::extractPlane( pcl::ModelCoefficients::Ptr                           &out_coefficients
                              , boost::shared_ptr<const pcl::PointCloud<MyPointT> >  in_cloud_ptr
                              , pcl::PointIndices::Ptr                              *out_inlier_indices
                              , pcl::PointIndices::ConstPtr                          in_indices_ptr
                              , double                                               distanceThreshold
                              , int                                                  sac_model
                              , Eigen::Vector3f                                     *p_normal
                              , int                                                  normal_estimation_K
                              , int                                                  seg_sac_method
                              , int                                                  seg_max_iterations  )
    {
        // prepare output
        if ( !out_coefficients )
            out_coefficients = pcl::ModelCoefficients::Ptr( new pcl::ModelCoefficients() );

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
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr ( new pcl::PointCloud<pcl::Normal> );
        if ( in_indices_ptr )
        {
            Primitives::expandNormals( /*   out_cloud: */ cloud_normals_ptr,
                                       /*   in_indice: */ in_indices_ptr,
                                       /*  in_normals: */ roi_cloud_normals_ptr,
                                       /* output_size: */ in_cloud_ptr->size() );
        }
        else
        {
            cloud_normals_ptr = roi_cloud_normals_ptr;
        }

        // Segmentation
        pcl::PointIndices::Ptr inliers_indices( new pcl::PointIndices() );
        pcl::SACSegmentationFromNormals<MyPointT, pcl::Normal> seg;
        // in
        seg.setInputCloud           ( in_cloud_ptr       );
        seg.setInputNormals         ( cloud_normals_ptr  );
        if ( in_indices_ptr )
            seg.setIndices( in_indices_ptr ); // this step requires the normals to be sparse
        // props
        seg.setOptimizeCoefficients ( true               );
        seg.setMethodType           ( seg_sac_method     );
        seg.setDistanceThreshold    ( distanceThreshold  );
        seg.setMaxIterations        ( seg_max_iterations );

        switch ( sac_model )
        {
            case pcl::SACMODEL_NORMAL_PLANE:                                        // get a plane using the points AND normals provided
                seg.setModelType ( pcl::SACMODEL_NORMAL_PLANE    );
                seg.setNormalDistanceWeight ( 0.1                );
                break;
            case pcl::SACMODEL_NORMAL_PARALLEL_PLANE:                               // get a plane parallel to p_normal using the points AND normals provided
                seg.setModelType ( pcl::SACMODEL_NORMAL_PARALLEL_PLANE );
                seg.setAxis      ( *p_normal                     );
                seg.setEpsAngle  ( pcl::deg2rad(5.f)             );
                break;

                //        case pcl::SACMODEL_PERPENDICULAR_PLANE:
                //            seg.setModelType ( pcl::SACMODEL_PARALLEL_PLANE );
                //            seg.setAxis      ( *p_normal                    );
                //            seg.setEpsAngle  ( pcl::deg2rad(20.f)           );
                //            break;
                //        case pcl::SACMODEL_PARALLEL_PLANE:
                //            seg.setModelType ( pcl::SACMODEL_PARALLEL_PLANE );
                //            seg.setAxis      ( *p_normal                    );
                //            seg.setEpsAngle  ( pcl::deg2rad(5.f)           );
                //            break;
            default:
                std::cerr << __func__ << ": sac_model_plane unknown" << std::endl;
                return EXIT_FAILURE;
                break;
        }

        // work
        seg.segment ( *inliers_indices, *out_coefficients );

        // ouptut inlier indices, if needed
        if ( out_inlier_indices )
        {
            if ( !*out_inlier_indices )
                *out_inlier_indices = pcl::PointIndices::Ptr( new pcl::PointIndices() );

            (*out_inlier_indices)->indices = inliers_indices->indices;
        }

        // return status
        return (out_coefficients->values.size() > 0) ? EXIT_SUCCESS
                                                     : EXIT_FAILURE;
    }

} // namespace smartgeom

#endif // PRIMITIVES_HPP
