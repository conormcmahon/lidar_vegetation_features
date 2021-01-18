
#ifndef TTVF_
#define TTVF_
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/point_representation.h>

namespace pcl{

struct TTVF
  {
    PCL_ADD_POINT4D                     // Macro quad-word XYZ
    int point_id;                       // Unique point ID assigned within entire flow system
    float height;                       // Treetop Height over Ground
    float search_window;                // Search Window in which to look for other potential maxima
    float point_density; 
    float height_pct_100;
    float height_pct_75;
    float height_pct_50;
    float height_pct_25;
    float height_75_frac;
    float height_50_frac;
    float height_25_frac;
    float normal_angle_rmse;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // Ensure proper alignment
  } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::TTVF,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, height, height)
                                  (float, search_window, search_window)
                                  (float, point_density, point_density)
                                  (float, height_pct_100, height_pct_100)
                                  (float, height_pct_75, height_pct_75)
                                  (float, height_pct_50, height_pct_50)
                                  (float, height_pct_25, height_pct_25)
                                  (float, height_75_frac, height_75_frac)
                                  (float, height_50_frac, height_50_frac)
                                  (float, height_25_frac, height_25_frac)
                                  (float, normal_angle_rmse, normal_angle_rmse)
)

namespace pcl {
template <>
class DefaultPointRepresentation<TTVF> : public PointRepresentation<TTVF>
{
public:
  DefaultPointRepresentation ()
  {
    nr_dimensions_ = 2;
  }
  
  virtual void
  copyToFloatArray (const TTVF &p, float * out) const
  {
    out[0] = p.x;
    out[1] = p.y;
  }
};
}
#endif // TTVF_
