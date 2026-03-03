///|/ Copyright (c) Prusa Research 2017 - 2023 Lukáš Matěna @lukasmatena, Vojtěch Bubník @bubnikv
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GCode_WipeTower2_hpp_
#define slic3r_GCode_WipeTower2_hpp_

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <tcbspan/span.hpp>

#include "libslic3r/Point.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"

namespace Slic3r
{

class PrintConfig;
class PrintObjectConfig;
class PrintRegionConfig;
class FullPrintConfig;
class Print;
enum GCodeFlavor : unsigned char;

class Layer;
class PrintObject;

// For const correctness: Wrapping a vector of non-const pointers as a span of const pointers.
template<class T>
using SpanOfConstPtrs           = tcb::span<const T* const>;

/**
 * WipeTower:
 * 1) in print, plan(layers) is called. This will compute the needed space for toolchanges, purges.
 * Whiththat, it ahs the max bb per layer.
 * 2) in print, it can get the max boundingbox from the toolchanges, purges
 * 4) in gcode, when a new layer start, it gets the info if a purge is needed (todo, currently not possible as
 * retractions are done after) and the toolchange order
 * 3) in gcode, when a toolchange occur: it gets the paths from
 * infill, it goes to wp, ram & unload, changetool, [go to infill] unretract & wipe (wp/infill).
 * It print the border & infill of the wipetower if needed (before going into infill).
 *
 * */
class WipeTowerLayer;
class WipeTower2
{
protected:
    // config for us
    const PrintConfig *m_config;
    // if global wipetower, this is the default object_config
    const PrintObjectConfig *m_object_config;
    const PrintRegionConfig *m_region_config;

    // position relative to object coordinates or platter coordinates
    Point m_position; //cached from m_object_config

    coord_t max_line_width = -1;
    coord_t min_line_width = -1;

public:
    //std::vector<std::shared_ptr<WipeTowerLayer>> wipe_tower_layers;
    //std::map<coord_t, std::shared_ptr<WipeTowerLayer>> print_z_to_wipe_tower_layer;

    // ===== for print =====
    void set_config(const PrintConfig *config, const PrintObjectConfig *object_config, const PrintRegionConfig *region_config);

    coord_t width() const;
    Vec2d position() const;
    coord_t extra_spacing() const;
    double rotation_angle() const;
    //((ConfigOptionFloat,                wipe_tower_per_color_wipe))
    //((ConfigOptionFloat,                wipe_tower_cone_angle))
    //((ConfigOptionFloat,                wipe_tower_bridging))
    //((ConfigOptionInt,                  wipe_tower_extruder))

    // general info about a filament, computed from config.
    struct FilamentToolchangeInfo
    {
        uint16_t tool_id = -1;
        // purge the nozzle before retracting
        double purge_volume; // mm3
        double purge_flow_mm3_sec;
        coord_t purge_width;
        coord_t purge_spacing;
        //Flow purge_flow;
        // wipe after unretracting (also used for unretracting)
        // when going into this tool, what's the worst wipe dist?
        //distf_t wipe_volume_max_from; // mm3
        //// when going from this tool, what's the worst wipe length?
        //distf_t wipe_volume_max_to; // mm3
        double wipe_volume_min; // mm3
        coord_t wipe_width;
        coord_t wipe_spacing;
        double wipe_speed;
        //Flow wipe_flow;
    };
    // one per extruder
    std::vector<FilamentToolchangeInfo> m_filament_change_data;

    // fused layers (with same z & height) info (general layer, can have multiple object or only one)
    struct ObjectLayerData
    {
        //WipeTowerLayer *wipe_tower_layer;
        coord_t real_z;
        coord_t real_height;
        // when the real_height is too low, it's merged with previous layer(s).
        // this vector stores the layer order (this is present inside)
        std::vector<const Layer *> my_layers;
        static int64_t compute_layer_key(coord_t z, coord_t height);
        //std::unique_ptr<WipeTowerLayer> create_wipe_tower_layer();
    };
    // layer_key (z & height) to LayerData
    std::map<int64_t, std::unique_ptr<ObjectLayerData>> m_layer_data;
    
    // fused layers (same z, can have different height) info
    // as we can use the same tool at the same z for different height before switching to another one.
    struct ZLayerData
    {
        coord_t real_z;
        std::vector<uint16_t> extruders; // from Toolordering (should be ordered correctly)
        coord_t estimated_wipe_tower_length; // from extruder toolchange
    };

    // data to construct a wipe tower layer.
    struct WipeTowerLayerData
    {
        coord_t extrusion_z;
        coord_t extrusion_height;
        std::vector<ObjectLayerData *> fused_with;
        std::map<coord_t, ZLayerData> extruders_data;
        coord_t estimated_wipe_tower_length; // from extruder toolchange
        //distf_t worst_needed_length;
        //distf_t worst_needed_length_with_fused;
        std::vector<const Layer *> layers() const;
        std::vector<const PrintObject *> objects() const;
    };
    std::vector<std::unique_ptr<WipeTowerLayerData>> m_WTLayer_data;
    std::map<coord_t, WipeTowerLayerData*> m_printz_to_WTLayer_data;

    // init m_filament_change_data and m_layer_data, m_printz_to_WTLayer_data
    void init(const Print *print, const SpanOfConstPtrs<PrintObject> &objects, ToolOrdering &ordering);
    std::map<coord_t, std::shared_ptr<WipeTowerLayer>> create_layers() const;

    // get tower width at this z (if nothing at this width, return 0)
    //distf_t get_max_length(coord_t z);

    // rough estimation
    static std::vector<std::vector<float>> extract_wipe_volumes(const ConfigBase &config);

    static coord_t floatz_tolayer_coord(double z);

    ExtrusionEntityCollection prime(const std::vector<uint16_t> &tools, coord_t first_layer_height);

    bool has_toolchange() const;

    static std::pair<double, double> get_wipe_tower_cone_base(double width,
                                                              double height,
                                                              double depth,
                                                              double angle_deg);

    Flow get_ramming_flow(const uint16_t tool_id, const bool first_layer, const double layer_height);
    //TODO
    // in mm3
    double get_loading_volume(uint16_t tool_idx) {
        return 1;
    }
    double get_loading_speed(uint16_t tool_idx) {
        return 1;
    }
    double get_prime_volume(uint16_t tool_idx) {
        return 1;
    }
    double get_prime_speed(uint16_t tool_idx) {
        return 1;
    }
    double get_unloading_volume(uint16_t tool_idx) {
        return 1;
    }
    double get_unloading_speed(uint16_t tool_idx) {
        return 1;
    }
    double get_first_layer_acceleration(uint16_t tool_idx) {
        return 1;
    }
    double get_first_layer_pa(uint16_t tool_idx) {
        return 1;
    }
    double get_first_layer_fan_speed(uint16_t tool_idx) {
        return 1;
    }
    friend class WipeTowerLayer;
};

// ===== for gcode =====
// plan 
class WipeTowerLayer
{
protected:
public:
    // config for us
    const PrintConfig *m_config;
    // if global wipetower, this is the default object_config
    const PrintObjectConfig *m_object_config;

    const WipeTower2* m_wipe_tower_info;
    //const WipeTower2::WipeTowerLayerData* m_layer_data;

    // all layer z purged in this layer; 
    std::set<coord_t> layers_z;
    std::vector<const Layer *> layers;

    // this layer extrusion information
    coord_t extrusion_z;
    coord_t extrusion_height;
    std::vector<coord_t> uninitialized_z;
    std::vector<coord_t> initialized_z;

    bool m_is_finished = false;
    coord_t m_current_y_pos = 0;
    coord_t m_max_y_pos = 0;
    
    Flow brim_flow;
    Polylines brim;
    bool brim_done = false;
    Flow tower_perimeter_flow;
    Polylines tower_perimeters;
    uint16_t perimeter_tool_idx = uint16_t(-1);
    bool perimeter_done = false;

    struct Toolchange
    {
        const Layer *layer;
        uint16_t from_tool_id;
        Flow purge_flow;
        Polyline purge_lines;
        uint16_t to_tool_id;
        Flow wipe_flow;
        Polyline wipe_lines;
        bool done = false;
    };
    // print_z & old_tool & new_tool => toolchange
    std::map<std::tuple<coord_t, uint16_t, uint16_t>, Toolchange> m_toolchanges;

    struct Purge
    {
        uint16_t tool_id;
        Flow purge_flow;
        Polyline purge_lines;
        bool done = false;
    };
    std::vector<Purge> m_purges;

    // ===== for both =====

public:

    WipeTowerLayer(const WipeTower2 *wipetower)
        : m_wipe_tower_info(wipetower), m_config(wipetower->m_config), m_object_config(wipetower->m_object_config) {}


    // gcode

    // plan the whole layer toolchanges & purges.
    void init(const std::vector<const Layer *> layers,
                    const std::vector<uint16_t> &ordered_extruders,
                    const std::vector<uint16_t> &purges);

    // ask to create the next extrusions
    //ExtrusionEntityCollection create_toolchange(uint16_t from, uint16_t to, Point current_pos);
    //ExtrusionEntityCollection create_purge(uint16_t toolid, Point current_pos);

    ExtrusionEntityCollection tool_change(const Layer *layer, uint16_t old_tool, uint16_t new_tool);
    bool finish_layer(ExtrusionEntityCollection &collection, uint16_t current_extruder, bool force = false);

protected:
    void toolchange_load(ExtrusionEntityCollection &collection,
                            const Polyline &loading_lines,
                            const uint16_t tool_id);
    bool toolchange_Unload(ExtrusionEntityCollection &collection,
                            const Polyline &ramming_lines,
                            const uint16_t tool_id,
                            const Flow &ramming_flow);
    void toolchange_Wipe(ExtrusionEntityCollection &collection,
                         Polyline wipe_lines,
                         const Flow wipe_flow,
                         const uint16_t tool_id);
    void toolchange_Change(ExtrusionEntityCollection &collection, const uint16_t new_tool);

    bool print_perimeter(ExtrusionEntityCollection &collection);


};




} // namespace Slic3r

#endif // slic3r_GCode_WipeTower2_hpp_ 
