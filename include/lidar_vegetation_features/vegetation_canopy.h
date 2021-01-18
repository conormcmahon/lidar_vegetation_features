
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
};