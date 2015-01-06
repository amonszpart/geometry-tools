#ifndef PRIMITIVES_H
#define PRIMITIVES_H

#include <stdlib.h>

#include <eigen3/Eigen/Dense>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/features/normal_3d.h>

namespace smartgeometry
{
    class Primitives
    {
        public:
            /// STATICs ///

            /**
             * @brief extractPlane Gets a ransac plane from a pointcloud
             *
             * @param[out] out_coefficients    A boost pointer to 4 coeffs: { Nx, Ny, Nz, d }, where N is the plane normal, and d is a distance from O
             * @param[in]  in_cloud            Input pointcloud of points typed MyPointT
             * @param[out] out_inlier_indices  Output inlier indices. Set to non NULL to request the output
             * @param[in]  in_indices          Input indices indexing "in_cloud"
             * @param[in]  distanceThreshold   Threshold to calcaulate inliers
             * @param[in]  sacmodel            NORMAL_PLANE estimates a plane using normals. NORMAL_PARALLEL_PLANE estimates using normals, and tries to be parallel to "*p_normal"
             * @param[in]  p_normal            A vector to be parallel to, NULL if unused.
             * @param[in]  normal_estimation_K Normals get estimated using this K-nearest neighbours, < 20 makes sense.
             * @param[in]  sac_method          RANSAC, PROSAC, etc.
             * @param[in]  seg_max_iterations  [RAN]SAC iteration cap, 500 seems ok.
             * @return EXIT_SUCCESS if "out_coefficients" contain meaningful results
             */
            template <typename MyPointT> // TODO: remove cloud normal estimation to separate function
            static int
            extractPlane( pcl::ModelCoefficients::Ptr                         &out_coefficients,
                          boost::shared_ptr<const pcl::PointCloud<MyPointT> >  in_cloud,
                          pcl::PointIndices::Ptr                              *out_inlier_indices  = NULL,
                          pcl::PointIndices::ConstPtr                          in_indices          = pcl::PointIndices::ConstPtr(),
                          double                                               distanceThreshold   = .1,
                          int                                                  sac_model           = pcl::SACMODEL_NORMAL_PLANE,
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
            static inline int
            expandNormals( pcl::PointCloud<pcl::Normal>::Ptr      &cloud_normals_ptr,
                           pcl::PointIndices::ConstPtr             in_indices_ptr,
                           pcl::PointCloud<pcl::Normal>::ConstPtr  roi_cloud_normals_ptr,
                           int                                     output_size           );

            /// TESTING ///
            static int
            test( bool verbose = false, bool display = false );
    };
} // namespace smartgeom

#ifndef INC_PRIMITIVES_HPP
#   define INC_PRIMITIVES_HPP
#   include "pcltools/primitives.hpp"
#endif // INC_PRIMITIVES_HPP

#endif // PRIMITIVES_H
