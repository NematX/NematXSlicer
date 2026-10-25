///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2018 - 2023 Oleksandra Iushchenko @YuSanka, Pavel Mikuš @Godrak, Vojtěch Bubník @bubnikv, Roman Beránek @zavorka, Lukáš Matěna @lukasmatena, Enrico Turri @enricoturri1966
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
// PrintBase prepares model placeholders and output paths shared by print implementations.
// Output names preserve late variables (brace-delimited names resolved from print statistics)
// while filtering the surrounding text before the extension is appended.
#include "PrintBase.hpp"

#include <regex>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "Exception.hpp"
#include "I18N.hpp"

namespace Slic3r
{

void PrintTryCancel::operator()() const
{
    m_print->throw_if_canceled();
}

size_t PrintStateBase::g_last_timestamp = 0;

// Update "scale", "input_filename", "input_filename_base" placeholders from the current m_objects.
void PrintBase::update_object_placeholders(DynamicConfig &config, const std::string & /* default_output_ext */) const
{
    // get the first input file name
    std::string input_file;
    std::vector<std::string> v_scale;
    int num_objects = 0;
    int num_instances = 0;
	for (const ModelObject &model_object : m_model.objects()) {
		ModelInstance *printable = nullptr;
		for (ModelInstance *model_instance : model_object.instances)
			if (model_instance->is_printable()) {
				printable = model_instance;
				++ num_instances;
			}
		if (printable) {
            ++ num_objects;
	        // CHECK_ME -> Is the following correct ?
			v_scale.push_back("x:" + boost::lexical_cast<std::string>(printable->get_scaling_factor(X) * 100) +
				"% y:" + boost::lexical_cast<std::string>(printable->get_scaling_factor(Y) * 100) +
				"% z:" + boost::lexical_cast<std::string>(printable->get_scaling_factor(Z) * 100) + "%");
	        if (input_file.empty())
	            input_file = model_object.name.empty() ? model_object.input_file : model_object.name;
	    }
    }
    
    config.set_key_value("num_objects", new ConfigOptionInt(num_objects));
    config.set_key_value("num_instances", new ConfigOptionInt(num_instances));

    config.set_key_value("scale", new ConfigOptionStrings(v_scale));
    if (! input_file.empty()) {
        // get basename with and without suffix
        const std::string input_filename = boost::filesystem::path(input_file).filename().string();
        const std::string input_filename_base = input_filename.substr(0, input_filename.find_last_of("."));
//        config.set_key_value("input_filename", new ConfigOptionString(input_filename_base + default_output_ext));
        config.set_key_value("input_filename_base", new ConfigOptionString(input_filename_base));
    }
}

// Expand borrowed format and configuration inputs, filter text around late variables, and return an owned name;
// preserve explicit extensions, use 0 for absent or non-integer initial_extruder options, propagate std::invalid_argument
// for integer values outside [-1, 32767], and wrap runtime formatting failures as PlaceholderParserError.
std::string PrintBase::output_filename(const std::string &format, const std::string &default_ext, const std::string &filename_base, const DynamicConfig *config_override) const
{
    DynamicConfig cfg;
    if (config_override != nullptr)
    	cfg = *config_override;
    cfg.set_key_value("version", new ConfigOptionString(std::string(SLIC3R_VERSION)));
    PlaceholderParser::update_timestamp(cfg);
    this->update_object_placeholders(cfg, default_ext);
    if (! filename_base.empty()) {
//		cfg.set_key_value("input_filename", new ConfigOptionString(filename_base + default_ext));
		cfg.set_key_value("input_filename_base", new ConfigOptionString(filename_base));
    }
    try {
        const int64_t raw_initial_extruder = config_override != nullptr &&
                config_override->option("initial_extruder") != nullptr &&
                config_override->option("initial_extruder")->type() == coInt ?
            config_override->option("initial_extruder")->get_int() : 0;
        if (raw_initial_extruder < EXTRUDER_ID_UNKNOWN ||
            raw_initial_extruder > (std::numeric_limits<extruder_index_t>::max)())
            throw std::invalid_argument("initial_extruder does not fit in extruder_index_t");
        const extruder_index_t extruder_initial = static_cast<extruder_index_t>(raw_initial_extruder);
        boost::filesystem::path filepath = format.empty() ?
            cfg.opt_string("input_filename_base") + default_ext :
            this->placeholder_parser().process(format, extruder_initial, &cfg);
        //remove unwanted characters
        std::string forbidden_base;
        if (const ConfigOptionString* opt = this->placeholder_parser().external_config()->option<ConfigOptionString>("gcode_filename_illegal_char")) {
            forbidden_base = opt->value;
        }
        if (!forbidden_base.empty()) {
            const std::string filename_init = filepath.stem().string();
            std::string filename;
            std::string extension = filepath.extension().string();
            const std::regex placeholder("\\{[a-z_]+\\}");
            std::regex forbidden_regex;
            bool regexp_used = false;
            if (forbidden_base.front() == '(' || forbidden_base.front() == '[') {
                try {
                    forbidden_regex = std::regex(forbidden_base);
                    regexp_used = true;
                }catch(std::exception){}
            }

            // Filter each text segment independently: replacements may change its length, but late variables stay intact.
            std::smatch matches;
            std::string::const_iterator searchStart = filename_init.cbegin();
            bool has_placeholder;
            do {
                has_placeholder = std::regex_search(searchStart, filename_init.cend(), matches, placeholder);
                std::string segment(searchStart, has_placeholder ? matches[0].first : filename_init.cend());
                if (regexp_used) {
                    segment = std::regex_replace(segment, forbidden_regex, "_");
                } else {
                    for (size_t i = 0; i < forbidden_base.size(); ++i)
                        std::replace(segment.begin(), segment.end(), forbidden_base.at(i), '_');
                }
                filename += segment;
                if (has_placeholder) {
                    filename += matches.str();
                    searchStart = matches.suffix().first;
                }
            } while (has_placeholder);
            // set the path var
            filepath = filename + extension;
        }
        if (filepath.extension().empty())
            filepath.replace_extension(default_ext);
        return filepath.string();
    } catch (std::runtime_error &err) {
        throw Slic3r::PlaceholderParserError(_u8L("Failed processing of the output_filename_format template.") + "\n" + err.what());
    }
}

std::string PrintBase::output_filepath(const std::string &path, const std::string &filename_base) const
{
    // if we were supplied no path, generate an automatic one based on our first object's input file
    if (path.empty())
        // get the first input file name
        return (boost::filesystem::path(m_model.propose_export_file_name_and_path()).parent_path() / this->output_filename(filename_base)).make_preferred().string();
    
    // if we were supplied a directory, use it and append our automatically generated filename
    boost::filesystem::path p(path);
    if (boost::filesystem::is_directory(p))
        return (p / this->output_filename(filename_base)).make_preferred().string();
    
    // if we were supplied a file which is not a directory, use it
    return path;
}

void PrintBase::status_update_warnings(int step, PrintStateBase::WarningLevel /* warning_level */, const std::string &message, const PrintObjectBase* print_object)
{
    SlicingStatus status = print_object ? SlicingStatus(*print_object, step) : SlicingStatus(*this, step);
    const std::string console_message = message.empty() ? std::string() :
        std::string(print_object != nullptr ? "print_object" : "print") +
            " warning: " + message;
    m_status.publish(std::move(status), console_message);
}

std::mutex& PrintObjectBase::state_mutex(PrintBase *print)
{ 
	return print->state_mutex();
}

std::function<void()> PrintObjectBase::cancel_callback(PrintBase *print)
{ 
	return print->cancel_callback();
}

void PrintObjectBase::status_update_warnings(PrintBase *print, int step, PrintStateBase::WarningLevel warning_level, const std::string &message)
{
    print->status_update_warnings(step, warning_level, message, this);
}

} // namespace Slic3r
