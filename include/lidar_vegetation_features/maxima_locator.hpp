
#ifndef MAXIMA_LOCATOR_HPP_
#define MAXIMA_LOCATOR_HPP_

#include "lidar_vegetation_features/maxima_locator.h"

template <typename VegType, typename TreetopType>
MaximaLocator<VegType, TreetopType>::MaximaLocator():
    demean_(true),
    remean_(true), 
    coeff_first_order_(10),
    coeff_second_order_(0.5),
    coeff_third_order_(0)
{
    veg_cloud_.reset(new VC);
    veg_tree_.reset(new VT);
    maxima_cloud_.reset(new MC);
}


template <typename VegType, typename TreetopType>
void MaximaLocator<VegType, TreetopType>::setOutputOptions(bool demean, bool remean)
{
    demean_ = demean;
    remean_ = remean;
}




// Reads a Vegetation Cloud .PDC point cloud file (format from the Point Cloud Library)
// The recommended custom point type used (pcl::PointVeg) contains:
//  - Height (over local DEM height)
//  - Intensity (of LiDAR return)
//  - Roughness (measure of local height variability)
template <typename VegType, typename TreetopType>
bool MaximaLocator<VegType, TreetopType>::readVegCloudPCD(std::string filename)
{
    std::cout << "Reading an input vegetation cloud from file " << filename << std::endl;
    if (pcl::io::loadPCDFile<VegType> (filename, *veg_cloud_) == -1) 
    {
        std::cout << "\nCouldn't read file " << filename << std::endl;
        return false;
    }
    std::cout << "  Successfully read vegetation cloud with size " << veg_cloud_->points.size() << std::endl;
    // Build KD Search Tree on cloud
    veg_tree_->setInputCloud(veg_cloud_);

    maxima_status_ = std::vector<int>(veg_cloud_->points.size(), 0);
    return true;
}



// Polynomial search window around each target point for local maxima
// Function of vegetation height over local ground height:
//    search_radius = coeff_first_order + coeff_second_order*height + coeff_third_order*height^2
template <typename VegType, typename TreetopType>
void MaximaLocator<VegType, TreetopType>::setSearchPolynomial(float coeff_first_order, float coeff_second_order, float coeff_third_order)
{
    coeff_first_order_ = coeff_first_order;
    coeff_second_order_ = coeff_second_order;
    coeff_third_order_ = coeff_third_order;
}

template <typename VegType, typename TreetopType>
float MaximaLocator<VegType, TreetopType>::getSearchWindow(VegType point)
{
    return coeff_first_order_ + coeff_second_order_*point.height + coeff_third_order_*pow(point.height, 2);
}


template <typename VegType, typename TreetopType>
void MaximaLocator<VegType, TreetopType>::generateMaxima(std::string filename)
{
    for(int i=0; i<veg_cloud_->points.size(); i++)
    {
        if(maxima_status_[i] == -1)
        {
            continue;
        }
        float search_dist = getSearchWindow(veg_cloud_->points[i]);
        std::vector<int> neighbor_indices;
        std::vector<float> dists_squared;
        veg_tree_->radiusSearch(veg_cloud_->points[i], search_dist, neighbor_indices, dists_squared);
        for(int j=0; j<neighbor_indices.size(); j++)
        {
            // Compare height to neighbors... 
            if(veg_cloud_->points[i].height < veg_cloud_->points[neighbor_indices[j]].height)
            {
                // If the target point is lower than neighbors, it's not a maximum
                maxima_status_[i] = -1;
                break;
            }
            else 
            {
                // If the target point is higher, the neighbor might still be a local maximum if it is further than that point's search window
                float neighbor_search_dist = getSearchWindow(veg_cloud_->points[j]);
                if(sqrt(dists_squared[j]) < neighbor_search_dist)
                    maxima_status_[j] = -1;
            }
        }
        // If the status is still 0, the point is a maximum!
        if(maxima_status_[i] == 0)
        {
            std::cout << "maximum! " << i << " " << veg_cloud_->points[i].x << " " << veg_cloud_->points[i].y << " " << veg_cloud_->points[i].z << " " << veg_cloud_->points[i].height << std::endl;
            maxima_status_[i] = 1;
            TreetopType maximum;
            maximum.x = veg_cloud_->points[i].x;
            maximum.y = veg_cloud_->points[i].y;
            maximum.z = veg_cloud_->points[i].z;
            maximum.height = veg_cloud_->points[i].height;
            maximum.search_window = search_dist;
            maxima_cloud_->points.push_back(maximum);
        }
    }
    pcl::PCDWriter writer;
    writer.write<TreetopType>(filename, *maxima_cloud_, true);
}

template <typename VegType, typename TreetopType>
void MaximaLocator<VegType, TreetopType>::getMaximaCloudCopy(MCP cloud_copy)
{
    *cloud_copy = *maxima_cloud_;
}
template <typename VegType, typename TreetopType>
void MaximaLocator<VegType, TreetopType>::getVegCloudCopy(VCP cloud_copy)
{
    *cloud_copy = *veg_cloud_;
}

#endif // MAXIMA_LOCATOR_HPP_