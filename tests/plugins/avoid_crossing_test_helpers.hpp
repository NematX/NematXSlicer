#ifndef test_plugins_avoid_crossing_test_helpers_hpp_
#define test_plugins_avoid_crossing_test_helpers_hpp_

#include "libslic3r/TriangleMesh.hpp"

/*
Routing tests share a shallow pair of 10 mm squares joined by a 5 mm gap's
bridge. The bridge overlaps each square by 1 mm so slicing merges the mesh.
The caller selects its width in millimeters and owns the returned mesh.
Example: TriangleMesh mesh = Slic3r::Test::Plugins::dumbbell(0.4);
*/
namespace Slic3r::Test::Plugins {

// Join two shallow squares with a narrow neck of the requested width in millimeters.
inline TriangleMesh dumbbell(double width)
{
    TriangleMesh mesh = make_cube(10., 10., 0.6);
    TriangleMesh right = make_cube(10., 10., 0.6); right.translate(15., 0., 0.); mesh.merge(right);
    TriangleMesh neck = make_cube(7., width, 0.6); neck.translate(9., 5. - width / 2., 0.); mesh.merge(neck);
    return mesh;
}

}
#endif
