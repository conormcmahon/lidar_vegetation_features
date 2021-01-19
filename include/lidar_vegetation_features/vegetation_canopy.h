
#ifndef VEGETATION_CANOPY_
#define VEGETATION_CANOPY_

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/search.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <dirt_or_leaf/las_conversions.hpp>

namespace vegetation_canopy
{
    template <typename VegCloud>
    void extractCanopyFirstReturns(VegCloud input, VegCloud canopy);

    template <typename VegPoint>
    bool checkCanopyStatusFirstReturns(VegPoint point);

    template <typename VegCloud>
    void statisticalFilter(VegCloud input, VegCloud output, int num_neighbors, float z_score_limit);

    template <typename VegPoint, typename VegCloud, typename VegTree>
    bool statisticalOutlierCheck(VegPoint point, VegCloud output, VegTree tree, int num_neighbors, float z_score_limit);

    template <typename VegCloud>
    void estimateNormals(VegCloud input, pcl::PointCloud<pcl::PointNormal>::Ptr  output, int num_neighbors);
};

#endif //VEGETATION_CANOPY_