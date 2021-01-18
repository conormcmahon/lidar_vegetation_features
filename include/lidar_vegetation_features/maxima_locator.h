
#ifndef MAXIMA_LOCATOR_
#define MAXIMA_LOCATOR_

// ** Point Cloud Library Includes **
//   Handling unordered clouds of points in XYZ space
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/io/pcd_io.h>
#include <dirt_or_leaf/las_point_types.h>
// Instantiate new PCL point types
#include <pcl/impl/instantiate.hpp>
// Eigen - Linear Math Library
#include <Eigen/Dense>
#include <cmath>


template <typename VegType, typename TreetopType>
class MaximaLocator{
public:
    // Defines for PCL PointCloud templates on custom point types
    typedef typename pcl::PointCloud<VegType> VC;
    typedef typename pcl::PointCloud<VegType>::Ptr VCP;
    typedef typename pcl::PointCloud<TreetopType> MC;
    typedef typename pcl::PointCloud<TreetopType>::Ptr MCP;
    // Defines for PCL KD Search Trees
    typedef typename pcl::KdTreeFLANN<VegType> VT;
    typedef typename pcl::KdTreeFLANN<VegType>::Ptr VTP;


    MaximaLocator();

    //   Load Vegetation Point Cloud 
    bool readVegCloudPCD(std::string filename);
    void setSearchPolynomial(float coeff_first_order, float coeff_second_order, float coeff_third_order);
    float getSearchWindow(VegType point);
    void generateMaxima(std::string filename);
    void setOutputOptions(bool demean, bool remean);
    void getMaximaCloudCopy(MCP cloud_copy);
    void getVegCloudCopy(VCP cloud_copy);

private:
    VCP veg_cloud_;
    VTP veg_tree_;
    MCP maxima_cloud_;
    std::vector<int> maxima_status_;

    float coeff_first_order_;
    float coeff_second_order_;
    float coeff_third_order_;

    bool demean_;
    bool remean_;


    //   Save Output Cloud
    bool writeMaximaCloud(std::string filename);
}; 

#endif // MAXIMA_LOCATOR_