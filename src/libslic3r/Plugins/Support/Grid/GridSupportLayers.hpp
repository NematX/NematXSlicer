///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Generator layers are temporary slabs: print_z is their top, bottom_z their
   geometric bottom, and height their printable thickness. Contacts meet the
   object, interfaces extend contacts, and base slabs form sparse columns.
   A deque owns the slabs until generation ends, so vectors may safely point
   into it. Only the final auxiliary layers are published to the object. */
#ifndef plugins_support_grid_layers_hpp_
#define plugins_support_grid_layers_hpp_

#include <deque>
#include <memory>
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"

namespace Slic3r::SupportGrid {
enum class LayerType { TopContact, BottomContact, TopInterface, BottomInterface, Base, Intermediate };

struct GeneratorLayer {
    LayerType type = LayerType::Intermediate;
    coord_t print_z = 0, bottom_z = 0, height = 0, height_block = 0;
    size_t object_above = size_t(-1), object_below = size_t(-1);
    bool bridging = false;
    Polygons polygons;
    std::unique_ptr<Polygons> contact_polygons, overhang_polygons;

    // Order slabs by top Z, then decreasing height, with bridge slabs first.
    bool operator<(const GeneratorLayer &rhs) const {
        return print_z < rhs.print_z || (print_z == rhs.print_z &&
            (height > rhs.height || (height == rhs.height && bridging && !rhs.bridging)));
    }
    // Return the top-contact bottom or bottom-contact top used to divide Z intervals.
    coord_t extreme_z() const { return type == LayerType::TopContact ? bottom_z : print_z; }
    // Return the printable bottom, excluding any separation gap.
    coord_t bottom_print_z() const { return print_z - height; }
};
using GeneratorLayers = std::vector<GeneratorLayer *>;
using LayerStorage = std::deque<GeneratorLayer>;
} // namespace Slic3r::SupportGrid
#endif
