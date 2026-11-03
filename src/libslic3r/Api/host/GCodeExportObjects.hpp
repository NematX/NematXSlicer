#ifndef slic3r_GCodeExportObjects_hpp_
#define slic3r_GCodeExportObjects_hpp_
#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"

/*
GCodeExportDescription is a copied catalogue of the final plan's object entries
and generated helper geometry. A real entry refers to a physical model instance;
other entries can represent generated items without a source object. A footprint
measures the area covered by deposited paths, not just their centerlines.

During EXPORT, call copy_gcode_export_objects(), then use the returned indices
with the public data-tree views when source slices are needed. Keep that source
Print alive while reading slices; the copied description itself has no such
lifetime dependency.

Owned catalog and first-layer geometry for integrated G-code exporters.
Call copy_gcode_export_objects during EXPORT, then resolve real objects and
instances through SDK indices. No returned value borrows the Print or plan.
Footprints are deposited-width coverage in plateau mm, not machine positions
adjusted for tool offsets. An absent footprint has present == false.
Object and instance indices are meaningful only on real catalog entries.
Footprint areas are union coverage in mm2; tower thickness is in mm.
*/
namespace Slic3r::ApiHost {
struct GCodeExportFootprint
{
    bool present = false;
    std::array<double, 4> bbox{};
    double area = 0.;
    double layer_height = 0.;
};
struct GCodeExportObject
{
    uint32_t id = 0;
    std::string name;
    bool real = false;
    uint32_t object_index = 0;
    uint32_t instance_index = 0;
    GCodeExportFootprint brim;
    GCodeExportFootprint support;
};
struct GCodeExportDescription
{
    std::vector<GCodeExportObject> objects;
    std::vector<uint32_t> used_tools;
    uint32_t initial_tool = 0;
    GCodeExportFootprint global_brim;
    GCodeExportFootprint wipe_tower;
};
// Copy catalog, plan tool visits and generated first-layer footprints; absent plans or stale real indices throw.
GCodeExportDescription copy_gcode_export_objects(const print_handle *print);
}
#endif
