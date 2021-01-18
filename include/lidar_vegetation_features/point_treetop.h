
#ifndef POINT_TREETOP_
#define POINT_TREETOP_
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/point_representation.h>

namespace pcl{

struct PointTreetop
  {
    PCL_ADD_POINT4D                     // Macro quad-word XYZ
    int point_id;                       // Unique point ID assigned within entire flow system
    float height;                       // Treetop Height over Ground
    float search_window;                // Search Window in which to look for other potential maxima
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // Ensure proper alignment
  } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointTreetop,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, height, height)
                                  (float, search_window, search_window)
)

namespace pcl {
template <>
class DefaultPointRepresentation<PointTreetop> : public PointRepresentation<PointTreetop>
{
public:
  DefaultPointRepresentation ()
  {
    nr_dimensions_ = 2;
  }
  
  virtual void
  copyToFloatArray (const PointTreetop &p, float * out) const
  {
    out[0] = p.x;
    out[1] = p.y;
  }
};
}
#endif // POINT_TREETOP_
