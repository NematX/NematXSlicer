///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/ Copyright (c) Prusa Research 2017 - 2020 Vojtěch Bubník @bubnikv, Oleksandra Iushchenko @YuSanka
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_GUI_ConfigExceptions_hpp_
#define slic3r_GUI_ConfigExceptions_hpp_

#include <exception>

#include "libslic3r/Exception.hpp"

namespace Slic3r {

class ConfigError : public Slic3r::RuntimeError { 
	using Slic3r::RuntimeError::RuntimeError;
};

namespace GUI {

class ConfigGUITypeError : public ConfigError { 
	using ConfigError::ConfigError;
};

} // namespace GUI
} // namespace Slic3r

#endif // slic3r_GUI_ConfigExceptions_hpp_
