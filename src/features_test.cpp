
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
    std::string maxima_filename = argv[3];
    std::string veg_canopy_filename = argv[4];
    std::string features_filename = argv[5];
    float coeff_first_order = std::atof(argv[6]);
    float coeff_second_order = std::atof(argv[7]);
    float coeff_third_order = std::atof(argv[8]);
    float stat_filter_neighbors = std::atof(argv[9]);
    float z_score_stat_filter = std::atof(argv[10]);

    //   Load Vegetation Point Cloud 
    MaximaLocator<pcl::PointVeg, pcl::PointTreetop> maxima_locator;
    maxima_locator.readVegCloudPCD(veg_decimated_filename);
    maxima_locator.setSearchPolynomial(coeff_first_order, coeff_second_order, coeff_third_order);
    maxima_locator.generateMaxima(maxima_filename);
    maxima_locator.setOutputOptions(true, true);

    pcl::PointCloud<pcl::PointTreetop>::Ptr keypoints(new pcl::PointCloud<pcl::PointTreetop>);
    maxima_locator.getMaximaCloudCopy(keypoints);
    pcl::PointCloud<pcl::PointVeg>::Ptr veg_cloud(new pcl::PointCloud<pcl::PointVeg>);
    pcl::io::loadPCDFile<pcl::PointVeg> (veg_filename, *veg_cloud);
    pcl::KdTreeFLANN<pcl::PointVeg>::Ptr veg_tree(new pcl::KdTreeFLANN<pcl::PointVeg>);
    veg_tree->setInputCloud(veg_cloud);

    pcl::PointCloud<pcl::PointVeg>::Ptr veg_first_returns(new pcl::PointCloud<pcl::PointVeg>);
    vegetation_canopy::extractCanopyFirstReturns(veg_cloud, veg_first_returns);
    pcl::PointCloud<pcl::PointVeg>::Ptr veg_filtered(new pcl::PointCloud<pcl::PointVeg>);
    vegetation_canopy::statisticalFilter<pcl::PointVeg, pcl::PointCloud<pcl::PointVeg>::Ptr>(veg_first_returns, veg_filtered, stat_filter_neighbors, z_score_stat_filter);

    pcl::PointCloud<pcl::TTVF>::Ptr TTVF_cloud(new pcl::PointCloud<pcl::TTVF>);
    lidar_vegetation_features::generateTTVFCloud(keypoints, veg_cloud, veg_tree, TTVF_cloud);

    pcl::PCDWriter writer;
    writer.write<pcl::PointVeg>(veg_canopy_filename, *veg_filtered, true);
    writer.write<pcl::TTVF>(features_filename, *TTVF_cloud, true);
}