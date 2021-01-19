
#ifndef LIDAR_VEGETATION_FEATURE_GENERATION_ 
#define LIDAR_VEGETATION_FEATURE_GENERATION_ 

// Point Cloud Library includes
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/io/pcd_io.h>
#include <lidar_vegetation_features/TTVF.h>
#include <pcl/impl/instantiate.hpp>

// Eigen - Linear Math Library
#include <Eigen/Dense>
#include <cmath>

/* TODO
implement more statistics
- normals
- height
- density? 
- height distribution 

demeaning and remeaning (required before normals)

generate normals across cloud

remove erroneously high or low points (e.g. powerlines, subteranean noise) 

*/

namespace lidar_vegetation_features
{
    template <typename Keypoint, typename VegType, typename VegCloud, typename SearchTree>
    pcl::TTVF generateTTVFPoint(Keypoint point, VegType point_example, VegCloud cloud, pcl::PointCloud<pcl::PointNormal>::Ptr normals, SearchTree tree, pcl::KdTreeFLANN<pcl::PointNormal>::Ptr tree_normal);
    
    template <typename KeypointCloudType, typename VegCloudType, typename VegTree>
    void generateTTVFCloud(KeypointCloudType keypoints, VegCloudType veg_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr normals, VegTree tree, pcl::PointCloud<pcl::TTVF>::Ptr feature_cloud);

};

#endif // LIDAR_VEGETATION_FEATURE_GENERATION_ 