
#include "lidar_vegetation_features/maxima_locator.hpp"
#include <dirt_or_leaf/point_veg.h>
#include "lidar_vegetation_features/point_treetop.h"
#include "lidar_vegetation_features/TTVF.h"
#include "lidar_vegetation_features/feature_generation.hpp"
#include "lidar_vegetation_features/vegetation_canopy.hpp"


int main(int argc, char *argv[])
{
    std::string veg_decimated_filename = argv[1];
    std::string veg_filename = argv[2];
    std::string output_filename = argv[3];
    float coeff_first_order = std::atof(argv[4]);
    float coeff_second_order = std::atof(argv[5]);
    float coeff_third_order = std::atof(argv[6]);
    float stat_filter_neighbors = std::atof(argv[7]);
    float z_score_stat_filter = std::atof(argv[8]);

    //   Load Vegetation Point Cloud 
    MaximaLocator<pcl::PointVeg, pcl::PointTreetop> maxima_locator;
    maxima_locator.readVegCloudPCD(veg_decimated_filename);
    maxima_locator.setSearchPolynomial(coeff_first_order, coeff_second_order, coeff_third_order);
    maxima_locator.generateMaxima(output_filename + "treetops.pcd");
    maxima_locator.setOutputOptions(true, true);

    pcl::PointCloud<pcl::PointTreetop>::Ptr keypoints(new pcl::PointCloud<pcl::PointTreetop>);
    maxima_locator.getMaximaCloudCopy(keypoints);
    pcl::PointCloud<pcl::PointTreetop>::Ptr keypoints_filtered(new pcl::PointCloud<pcl::PointTreetop>);
    vegetation_canopy::statisticalFilter<pcl::PointTreetop, pcl::PointCloud<pcl::PointTreetop>::Ptr>(keypoints, keypoints_filtered, stat_filter_neighbors, z_score_stat_filter);
    pcl::PointCloud<pcl::PointVeg>::Ptr veg_cloud(new pcl::PointCloud<pcl::PointVeg>);
    pcl::io::loadPCDFile<pcl::PointVeg> (veg_filename, *veg_cloud);
    pcl::KdTreeFLANN<pcl::PointVeg>::Ptr veg_tree(new pcl::KdTreeFLANN<pcl::PointVeg>);
    veg_tree->setInputCloud(veg_cloud);

    pcl::PointCloud<pcl::PointVeg>::Ptr veg_first_returns(new pcl::PointCloud<pcl::PointVeg>);
    vegetation_canopy::extractCanopyFirstReturns(veg_cloud, veg_first_returns);
    pcl::PointCloud<pcl::PointVeg>::Ptr veg_filtered(new pcl::PointCloud<pcl::PointVeg>);
    vegetation_canopy::statisticalFilter<pcl::PointVeg, pcl::PointCloud<pcl::PointVeg>::Ptr>(veg_first_returns, veg_filtered, stat_filter_neighbors, z_score_stat_filter);
    pcl::PointCloud<pcl::PointNormal>::Ptr canopy_normals(new pcl::PointCloud<pcl::PointNormal>);
    vegetation_canopy::estimateNormals<pcl::PointVeg, pcl::PointCloud<pcl::PointVeg>::Ptr>(veg_filtered, canopy_normals, 10);

    pcl::PointCloud<pcl::TTVF>::Ptr TTVF_cloud(new pcl::PointCloud<pcl::TTVF>);
    lidar_vegetation_features::generateTTVFCloud(keypoints, veg_cloud, canopy_normals, veg_tree, TTVF_cloud);

    pcl::PCDWriter writer;
    writer.write<pcl::PointVeg>(output_filename + "_canopy.pcd", *veg_filtered, true);
    writer.write<pcl::PointNormal>(output_filename + "_canopy_normals.pcd", *canopy_normals, true);
    writer.write<pcl::TTVF>(output_filename + "_features.pcd", *TTVF_cloud, true);
}