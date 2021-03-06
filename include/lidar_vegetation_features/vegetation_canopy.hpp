
#ifndef VEGETATION_CANOPY_HPP_
#define VEGETATION_CANOPY_HPP_

#include "lidar_vegetation_features/vegetation_canopy.h"

template <typename VegCloud>
void vegetation_canopy::extractCanopyFirstReturns(VegCloud input, VegCloud canopy)
{
    for(int i=0; i<input->points.size(); i++)
    {
        if(checkCanopyStatusFirstReturns(input->points[i]))
            canopy->points.push_back(input->points[i]);
    }
}

template <typename VegPoint>
bool vegetation_canopy::checkCanopyStatusFirstReturns(VegPoint point)
{
    return (point.returnnumber == 1);
}

template <typename VegPoint, typename VegCloud>
void vegetation_canopy::statisticalFilter(VegCloud input, VegCloud output, int num_neighbors, float z_score_limit)
{
    typedef typename pcl::KdTreeFLANN<VegPoint> VegTree;
    typedef typename pcl::KdTreeFLANN<VegPoint>::Ptr VegTreePtr;
    VegTreePtr tree(new VegTree);
    tree->setInputCloud(input);
    for(int i=0; i<input->points.size(); i++)
        if(statisticalOutlierCheck(input->points[i], input, tree, num_neighbors, z_score_limit))
            output->points.push_back(input->points[i]);
}

template <typename VegPoint, typename VegCloud, typename VegTree>
bool vegetation_canopy::statisticalOutlierCheck(VegPoint point, VegCloud cloud, VegTree tree, int num_neighbors, float z_score_limit)
{
    std::vector<int> neighbor_indices; 
    std::vector<float> dist_squareds;
    tree->nearestKSearch(point, num_neighbors, neighbor_indices, dist_squareds);
    // Find Mean Z (height)
    float mean_z = 0;
    for(int i=0; i<neighbor_indices.size(); i++)
        mean_z += cloud->points[neighbor_indices[i]].z;
    mean_z /= neighbor_indices.size();
    // Find Standard Deviation of Z (height)
    float stdev_z = 0;
    for(int i=0; i<neighbor_indices.size(); i++)
        stdev_z += pow((cloud->points[neighbor_indices[i]].z - mean_z), 2);
    stdev_z = sqrt(stdev_z/neighbor_indices.size());
    // Statistical Thresholding
    std::cout << fabs((point.z - mean_z) / stdev_z) << " " << mean_z << " " << stdev_z << " " << point.x << " " << point.y << " " << point.z << std::endl;
    return (fabs((point.z - mean_z) / stdev_z) < z_score_limit);
}



template <typename VegPoint, typename VegCloud>
void vegetation_canopy::estimateNormals(VegCloud input, pcl::PointCloud<pcl::PointNormal>::Ptr output, int num_neighbors)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud3D(input, input_xyz);
    // Normal Estimation Object
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
    normal_estimation.setInputCloud(input_xyz);
    // KD Search Tree to find point neighbors
    //typedef typename pcl::search::KdTree<VegPoint> VegTree;
    //typedef typename pcl::search::KdTree<VegPoint>::Ptr VegTreePtr;
    //VegTreePtr tree (new VegTree);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation.setSearchMethod(tree);
    // Search size for neighbors to be used in normal estimation
    normal_estimation.setKSearch(num_neighbors);  
    // Compute the normals
    normal_estimation.compute(*output);
    pcl::copyPointCloud3D(input_xyz, output);
}

#endif //VEGETATION_CANOPY_HPP_