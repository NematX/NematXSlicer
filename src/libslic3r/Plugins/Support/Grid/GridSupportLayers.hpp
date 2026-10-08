///|/ SuperSlicer is released under the terms of the AGPLv3 or higher.
/* Generator layers are temporary slabs: print_z is their top, bottom_z their
   geometric bottom, and height their printable thickness. Contacts meet the
   object, interfaces extend contacts, and base slabs form sparse columns.
   A deque owns the slabs until generation ends, so vectors may safely point
   into it. Only the final auxiliary layers are published to the object.

   The geometry of a slab is a polygon collection kept in the plugin storage
   given at construction (StoredPolygonCollection): it is released when the
   slab is destroyed. Top contacts also keep their contact and overhang areas,
   which the other slabs do not have:

       GeneratorLayer slab(storage);
       slab.contact_polygons.emplace(storage); // only for a top contact */
#ifndef plugins_support_grid_layers_hpp_
#define plugins_support_grid_layers_hpp_

#include <deque>
#include <optional>
#include <vector>

#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

namespace slic3r_api::Support::Grid {
enum class LayerType { TopContact, BottomContact, TopInterface, BottomInterface, Base, Intermediate };

struct GeneratorLayer {
    // Create an empty intermediate slab whose geometry lives in storage.
    explicit GeneratorLayer(storage_handle *storage) : polygons(storage) {}

    LayerType type = LayerType::Intermediate;
    coord_t print_z = 0, bottom_z = 0, height = 0, height_block = 0;
    size_t object_above = size_t(-1), object_below = size_t(-1);
    bool bridging = false;
    StoredPolygonCollection polygons;
    std::optional<StoredPolygonCollection> contact_polygons, overhang_polygons;

    // Order slabs by top Z, then decreasing height, with bridge slabs first.
    bool operator<(const GeneratorLayer &rhs) const {
        return print_z < rhs.print_z || (print_z == rhs.print_z &&
            (height > rhs.height || (height == rhs.height && bridging && !rhs.bridging)));
    }
    // Return the top-contact bottom or bottom-contact top used to divide Z intervals.
    coord_t extreme_z() const { return type == LayerType::TopContact ? bottom_z : print_z; }
    // Return the printable bottom, excluding any separation gap.
    coord_t bottom_print_z() const { return print_z - height; }
    // Give the slab back its initial values: no geometry, intermediate type, zero Z.
    void reset();
};
using GeneratorLayers = std::vector<GeneratorLayer *>;
using LayerStorage = std::deque<GeneratorLayer>;

inline void GeneratorLayer::reset()
{
    type = LayerType::Intermediate;
    print_z = bottom_z = height = height_block = 0;
    object_above = object_below = size_t(-1);
    bridging = false;
    polygons.clear();
    contact_polygons.reset();
    overhang_polygons.reset();
}
} // namespace slic3r_api::Support::Grid
#endif
