///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/ Copyright (c) Prusa Research 2017 - 2021 Vojtěch Bubník @bubnikv
///|/
///|/ ported from lib/Slic3r/GCode/SpiralVase.pm:
///|/ Copyright (c) Prusa Research 2017 Vojtěch Bubník @bubnikv
///|/ Copyright (c) Slic3r 2013 - 2014 Alessandro Ranellucci @alranel
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_SpiralVase_hpp_
#define slic3r_SpiralVase_hpp_

#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/libslic3r.h"

namespace Slic3r {


class SpiralVase {
public:
    SpiralVase(const PrintConfig &config, GCodeFlavor flavor) : m_config(config)
    {
        m_reader.z() = (float)m_config.z_offset;
        m_reader.apply_config(m_config, flavor);
    };

    void 		enable(bool en) {
   		m_transition_layer = en && ! m_enabled;
    	m_enabled 		   = en;
    }

    std::string process_layer(const std::string& gcode);

    bool is_transition_layer() { return m_transition_layer; }

private:
    const PrintConfig  &m_config;
    GCodeReader 		m_reader;

    bool 				m_enabled = false;
    // First spiral vase layer. Layer height has to be ramped up from zero to the target layer height.
    bool 				m_transition_layer = false;
};

}

#endif // slic3r_SpiralVase_hpp_
