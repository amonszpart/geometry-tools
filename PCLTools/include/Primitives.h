#ifndef PRIMITIVES_H
#define PRIMITIVES_H

#include <stdlib.h>

#include <Eigen/Dense>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/features/normal_3d.h>

namespace smartgeom
{
    class Primitives
    {
        public:
            /// STATICs ///

            template <typename MyPointT> // TODO: remove cloud normal estimation to separate function
            static int
            extractPlane( pcl::ModelCoefficients::Ptr                         &out_coefficients,
                          boost::shared_ptr<const pcl::PointCloud<MyPointT> >  in_cloud,
                          pcl::PointIndices::Ptr                              *out_inlier_indices  = NULL,
                          pcl::PointIndices::ConstPtr                          in_indices          = pcl::PointIndices::ConstPtr(),
                          double                                               distanceThreshold   = .1,
                          int                                                  sacmodel            = pcl::SACMODEL_NORMAL_PLANE,
                          Eigen::Vector3f                                     *p_normal            = NULL,
                          int                                                  normal_estimation_K = 20,
                          int                                                  sac_method          = pcl::SAC_RANSAC,
                          int                                                  seg_max_iterations  = 500 );

            /**
             * @brief expandNormals Takes a dense cloud of normals and copies it to a sparse cloud of normals
             *
             *  Creates a cloud in "cloud_normals" with size "output_size"
             *  and copies normals from "roi_cloud_normals" to specific indices in "cloud_normals",
             *  where the indices come from "in_indices".
             *
             * @param[out] cloud_normals_ptr     Output sparse normals, has meaningful values at indices in "in_indices"
             * @param[in]  in_indices_ptr        Indices, where the output should have the normals stored
             * @param[in]  roi_cloud_normals_ptr Input dense normals
             * @param[in]  output_size           Output cloud size
             * @return EXIT_SUCCESS/EXIT_FAILURE
             */
            static int
            expandNormals( pcl::PointCloud<pcl::Normal>::Ptr      &cloud_normals_ptr,
                           pcl::PointIndices::ConstPtr             in_indices_ptr,
                           pcl::PointCloud<pcl::Normal>::ConstPtr  roi_cloud_normals_ptr,
                           int                                     output_size           );

            /// TESTING ///
            static int
            test( bool verbose = false, bool display = false );
    };
}

#ifndef INC_PRIMITIVES_HPP
#   define INC_PRIMITIVES_HPP
#   include "Primitives.hpp"
#endif // INC_PRIMITIVES_HPP

#endif // PRIMITIVES_H
