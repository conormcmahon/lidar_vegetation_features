
#ifndef LIDAR_VEGETATION_FEATURE_GENERATION_HPP_
#define LIDAR_VEGETATION_FEATURE_GENERATION_HPP_

#include "lidar_vegetation_features/feature_generation.h"

template <typename Keypoint, typename VegType, typename VegCloud, typename SearchTree>
pcl::TTVF  lidar_vegetation_features::generateTTVFPoint(Keypoint point, VegType veg_example, VegCloud cloud, SearchTree tree)
{
    pcl::TTVF output;
    int num_veg_neighbors = 0;
    std::vector<float> normal_altitudes;
    std::vector<float> heights;

    std::vector<int> neighbor_indices;
    std::vector<float> dists_squared;

    VegType point_veg;
    point_veg.x = point.x;
    point_veg.y = point.y;
    point_veg.z = point.z;
    tree->radiusSearch(point_veg, point.search_window, neighbor_indices, dists_squared);

    for(int i=0; i<neighbor_indices.size(); i++)
    {
        num_veg_neighbors++;
        heights.push_back(cloud->points[neighbor_indices[i]].height);
    }

    std::sort(heights.begin(), heights.end());
    
    output.x = point.x;
    output.y = point.y;
    output.z = point.z;
    output.height = point.height;
    output.search_window = point.search_window;
    output.point_density = num_veg_neighbors / (M_PI*pow(point.search_window,2));
    output.height_pct_100 = heights[heights.size()-1];
    output.height_pct_75 = heights[heights.size()*4/3];
    output.height_pct_50 = heights[heights.size()/2];
    output.height_pct_25 = heights[heights.size()/4];
    output.height_75_frac = output.height_pct_75 / output.height_pct_100;
    output.height_50_frac = output.height_pct_50 / output.height_pct_100;
    output.height_25_frac = output.height_pct_25 / output.height_pct_100;
    output.normal_angle_rmse = 0;

    return output;
}


template <typename KeypointCloudType, typename VegCloudType, typename VegTree>
void lidar_vegetation_features::generateTTVFCloud(KeypointCloudType keypoints, VegCloudType veg_cloud, VegTree tree, pcl::PointCloud<pcl::TTVF>::Ptr feature_cloud)
{
    std::cout << "Starting to generate TTVF features for input cloud with " << keypoints->points.size() << " points." << std::endl;
    for(std::size_t i=0; i<keypoints->points.size(); i++)
        feature_cloud->points.push_back(generateTTVFPoint(keypoints->points[i], veg_cloud->points[i], veg_cloud, tree)); 
    feature_cloud->width = 1;
    feature_cloud->height = feature_cloud->points.size();
}

#endif // LIDAR_VEGETATION_FEATURE_GENERATION_HPP_