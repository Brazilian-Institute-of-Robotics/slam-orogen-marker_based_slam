#ifndef marker_based_mapping_TYPES_HPP
#define marker_based_mapping_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */
#include <base/samples/RigidBodyState.hpp>

namespace marker_based_slam {

struct RelativeMarkerPoses{
    int pair1;
    int pair2;
    base::samples::RigidBodyState rbs;
};

struct MapPose{
    int id;
    base::samples::RigidBodyState rbs;
};

struct PoseTransformation
{
    double x;
    double y;
    double z;
    double rot_x;
    double rot_y;
    double rot_z;
};

}

#endif

