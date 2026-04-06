#include "TemperatureMover.hpp"

#include "GCodeReader.hpp"
#include "LocalesUtils.hpp"

#include <iomanip>

#include <boost/log/trivial.hpp>


namespace Slic3r {

const std::string& TemperatureMover::process_gcode(const std::string& gcode, bool flush)
{
    m_process_output = "";

    if (m_max_seconds_delay > 0) {
        // recompute buffer time to recover from rounding
        m_buffer_time_size = 0;
        for (auto &data : m_buffer) {
            assert(data.time >= 0 && data.time < 1000000 && !std::isnan(data.time));
            m_buffer_time_size += data.time;
        }

        if (!gcode.empty())
            m_parser.parse_buffer(gcode, [this](GCodeReader &reader, const GCodeReader::GCodeLine &line) {
                /*m_process_output += line.raw() + "\n";*/
                this->_process_gcode_line(reader, line);
            });

        if (flush) {
            while (!m_buffer.empty()) {
                write_buffer_data();
            }
        }

        return m_process_output;
    } else {
        // not activated, skip processing and return the gcode as is.
        return gcode;
    }
}

namespace TemperatureMover_func {

bool is_end_of_word(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == 0; }

float get_axis_value(const std::string &line, char axis)
{
    char match[3] = " X";
    match[1]      = axis;

    size_t pos = line.find(match) + 2;
    // size_t end = std::min(line.find(' ', pos + 1), line.find(';', pos + 1));
    // Try to parse the numeric value.
    const char *c    = line.c_str();
    char *      pend = nullptr;
    errno            = 0;
    double v         = strtod(c + pos, &pend);
    if (pend != nullptr && errno == 0 && pend != c) {
        // The axis value has been parsed correctly.
        return float(v);
    }
    return NAN;
}
float get_nematx_axis_value(const std::string &line)
{

    size_t pos = line.find('=');
    if (pos == std::string::npos) {
        return -1.f;
    }
    pos++;
    // size_t end = std::min(line.find(' ', pos + 1), line.find(';', pos + 1));
    // Try to parse the numeric value.
    const char *c    = line.c_str();
    char *      pend = nullptr;
    errno            = 0;
    double v         = strtod(c + pos, &pend);
    if (pend != nullptr && errno == 0 && pend != c) {
        // The axis value has been parsed correctly.
        return float(v);
    }
    return NAN;
}

void change_axis_value(std::string &line, char axis, const float new_value, const int decimal_digits)
{
    char match[3] = " X";
    match[1]      = axis;

    size_t pos = line.find(match) + 2;
    size_t end = std::min(line.find(' ', pos + 1), line.find(';', pos + 1));
    line       = line.replace(pos, end - pos, to_string_nozero(new_value, decimal_digits));
}

bool parse_number(const std::string_view sv, int& out)
{
    // Legacy conversion, which is costly due to having to make a copy of the string before conversion.
    try {
        assert(sv.size() < 1024);
        assert(sv.data() != nullptr);
        std::string str{ sv };
        size_t read = 0;
        out = std::stoi(str, &read);
        return str.size() == read;
    }
    catch (...) {
        return false;
    }
}
} // namespace TemperatureMover_func

std::tuple<int16_t, uint16_t> TemperatureMover::_get_temperature_with_extruder_idx(const std::string &line,
                                                                                   GCodeFlavor flavor) {
    if (flavor == (gcfNematX)) {
        if (line.compare(0, 4, "M104") == 0 || line.compare(0, 4, "M109") == 0) {
            return std::tuple<int16_t, uint16_t>((int16_t) TemperatureMover_func::get_nematx_axis_value(line),
                                                 uint16_t(0));
        } else if (line.compare(0, 4, "M124") == 0 || line.compare(0, 4, "M129") == 0) {
            return std::tuple<int16_t, uint16_t>((int16_t) TemperatureMover_func::get_nematx_axis_value(line),
                                                 uint16_t(1));
        }
        return std::tuple<int16_t, uint16_t>(int16_t(-1), uint16_t(0));
    } else {
        if (line.compare(0, 4, "M104") == 0 || line.compare(0, 4, "M109") == 0) {
            return std::tuple<int16_t, uint16_t>((int16_t) TemperatureMover_func::get_nematx_axis_value(line),
                                                 m_current_extruder);
        }
        return std::tuple<int16_t, uint16_t>(int16_t(-1), uint16_t(0));
    }
}

void TemperatureMover::_put_in_middle_G1(std::list<BufferData>::reverse_iterator item_to_split, float nb_sec_since_itemtosplit_start, BufferData &&line_to_write, float max_time) {
    assert(item_to_split != m_buffer.rend());
    // if the fan is at the end of the g1 and the diff is less than 10% of the delay, then don't bother
    if (nb_sec_since_itemtosplit_start > item_to_split->time * 0.9 && (item_to_split->time - nb_sec_since_itemtosplit_start) < max_time * 0.1) {
        // doesn't really need to be split, print it after
        m_buffer.insert(next(item_to_split).base(), line_to_write);
    } else 
        // if it's almost at the start of the g1, and the time "lost" is less than 10%
        if (nb_sec_since_itemtosplit_start < item_to_split->time * 0.1 && nb_sec_since_itemtosplit_start < max_time * 0.1) {
        // doesn't really need to be split, print it before
        m_buffer.insert(item_to_split.base(), line_to_write);
    } else if (item_to_split->raw.size() > 2
        && item_to_split->raw[0] == 'G' && item_to_split->raw[1] == '1' && item_to_split->raw[2] == ' ') {
        assert(nb_sec_since_itemtosplit_start < item_to_split->time);
        float after_percent = nb_sec_since_itemtosplit_start / item_to_split->time;
        BufferData before = *item_to_split;
        before.time *= (1 - after_percent);
        item_to_split->time *= after_percent;
        if (item_to_split->dx != 0) {
            before.dx = item_to_split->dx * (1 - after_percent);
            item_to_split->x += before.dx;
            item_to_split->dx = item_to_split->dx * after_percent;
            TemperatureMover_func::change_axis_value(before.raw, 'X', before.x + before.dx, 3);
        }
        if (item_to_split->dy != 0) {
            before.dy = item_to_split->dy * (1 - after_percent);
            item_to_split->y += before.dy;
            item_to_split->dy = item_to_split->dy * after_percent;
            TemperatureMover_func::change_axis_value(before.raw, 'Y', before.y + before.dy, 3);
        }
        if (item_to_split->dz != 0) {
            before.dz = item_to_split->dz * (1 - after_percent);
            item_to_split->z += before.dz;
            item_to_split->dz = item_to_split->dz * after_percent;
            TemperatureMover_func::change_axis_value(before.raw, 'Z', before.z + before.dz, 3);
        }
        if (item_to_split->de != 0) {
            if (relative_e) {
                before.de = item_to_split->de * (1 - after_percent);
                TemperatureMover_func::change_axis_value(before.raw, before.e_axis_char, before.de, 5);
                item_to_split->de = item_to_split->de * after_percent;
                TemperatureMover_func::change_axis_value(item_to_split->raw, item_to_split->e_axis_char, item_to_split->de, 5);
            } else {
                before.de = item_to_split->de * (1 - after_percent);
                item_to_split->e += before.de;
                item_to_split->de = item_to_split->de * after_percent;
                TemperatureMover_func::change_axis_value(before.raw, before.e_axis_char, before.e + before.de, 5);
            }
        }
        // add before then line_to_write, then there is the modified data.
        before.raw += " ; before";
        m_buffer.insert(next(item_to_split).base(), line_to_write);
        m_buffer.insert(next(item_to_split).base(), before);
        item_to_split->raw += " ; after";

    } else {
        //not a G1, print it before or after
        if (nb_sec_since_itemtosplit_start > item_to_split->time * 0.5f) {
            m_buffer.insert(next(item_to_split).base(), line_to_write);
        } else {
            m_buffer.insert(item_to_split.base(), line_to_write);
        }
    }
}

void TemperatureMover::_remove_low_heat(int16_t min_temp, uint16_t extr_idx, float past_sec) {
    // erase in the buffer -> don't cooldown if you are in the process of heating.
    // we began at the "recent" side , and remove as long as we don't push past_sec to 
    auto it = m_buffer.begin();
    while (it != m_buffer.end() && past_sec > 0) {
        past_sec -= it->time;
        if (it->temperature_extruder_idx == extr_idx && it->temperature >= 0 &&
            it->temperature < min_temp) {
            //found something that is lower than us
            it = remove_from_buffer(it);
        } else {
            ++it;
        }
    }
}

//FIXME: add other firmware
// or just create that damn new gcode writer arch
void TemperatureMover::_process_T(const std::string_view command)
{
    if (command.length() > 1 && command[1] >= '0' && command[1] <= '9') {
        int eid = 0;
        if (!TemperatureMover_func::parse_number(command.substr(1), eid) || eid < 0 || eid > 255) {
            GCodeFlavor flavor = m_writer.config.gcode_flavor;
            // Specific to the MMU2 V2 (see https://www.help.prusa3d.com/en/article/prusa-specific-g-codes_112173):
            if ((flavor == gcfMarlinLegacy || flavor == gcfMarlinFirmware) && (command == "Tx" || command == "Tc" || command == "T?"))
                return;

            // T-1 is a valid gcode line for RepRap Firmwares (used to deselects all tools) see https://github.com/prusa3d/PrusaSlicer/issues/5677
            if ((flavor != gcfRepRap && flavor != gcfSprinter) || eid != -1)
                m_current_extruder = static_cast<uint16_t>(0);
        } else {
            m_current_extruder = static_cast<uint16_t>(eid);
        }
    }
}


void TemperatureMover::_process_ACTIVATE_EXTRUDER(const std::string_view cmd)
{
    if (size_t cmd_end = cmd.find("ACTIVATE_EXTRUDER"); cmd_end != std::string::npos) {
        size_t extruder_pos_start = cmd.find("EXTRUDER", cmd_end + std::string_view("ACTIVATE_EXTRUDER").size()) + std::string_view("EXTRUDER").size();
        assert(cmd[extruder_pos_start - 1] == 'R');
        if (extruder_pos_start != std::string::npos) {
            //remove next char until '-' or [0-9]
            while (extruder_pos_start < cmd.size() && (cmd[extruder_pos_start] == ' ' || cmd[extruder_pos_start] == '=' || cmd[extruder_pos_start] == '\t'))
                ++extruder_pos_start;
            size_t extruder_pos_end = extruder_pos_start + 1;
            while (extruder_pos_end < cmd.size() && cmd[extruder_pos_end] != ' ' && cmd[extruder_pos_end] != '\t' && cmd[extruder_pos_end] != '\r' && cmd[extruder_pos_end] != '\n')
                ++extruder_pos_end;
            std::string_view extruder_name = cmd.substr(extruder_pos_start, extruder_pos_end-extruder_pos_start);
            // we have a "name". It may be whatever or "extruder" + X
            for (const Extruder &extruder : m_writer.extruders()) {
                if (m_writer.config.tool_name.get_at(extruder.id()) == extruder_name) {
                    m_current_extruder = static_cast<uint16_t>(extruder.id());
                    return;
                }
            }
            std::string extruder_str("extruder");
            if (extruder_str == extruder_name) {
                m_current_extruder = static_cast<uint16_t>(0);
                return;
            }
            for (const Extruder &extruder : m_writer.extruders()) {
                if (extruder_str + std::to_string(extruder.id()) == extruder_name) {
                    m_current_extruder = static_cast<uint16_t>(extruder.id());
                    return;
                }
            }
        }
        BOOST_LOG_TRIVIAL(error) << "invalid ACTIVATE_EXTRUDER gcode command: '" << cmd << "', ignored by the fam mover post-process.";
    }
}

void TemperatureMover::_process_gcode_line(GCodeReader& reader, const GCodeReader::GCodeLine& line)
{
    // processes 'normal' gcode lines
    bool need_flush = false;
    std::string cmd(line.cmd());
    double time = 0;
    int16_t new_temperature = -1;
    uint16_t temperature_extruder_idx = 0;
    if (cmd.length() > 1) {
        if (::toupper(cmd[0]) == 'G') {
            assert(!line.has_f() || line.f() > 0);
            if (line.has_f() && line.f() > 0) {
                m_current_speed = line.f() / 60.0f;
            }
        }
        switch (::toupper(cmd[0])) {
        case 'A':
            _process_ACTIVATE_EXTRUDER(line.raw());
                break;
        case 'T':
        case 't':
            _process_T(cmd);
                break;
        case 'G':
        {
            if (::atoi(&cmd[1]) == 1 || ::atoi(&cmd[1]) == 0) {
                double distx = line.dist_X(reader);
                double disty = line.dist_Y(reader);
                double distz = line.dist_Z(reader);
                double dist = distx * distx + disty * disty + distz * distz;
                if (dist > 0) {
                    dist = std::sqrt(dist);
                    assert(m_current_speed > 0 && m_current_speed < 1000000 && !std::isnan(m_current_speed));
                    time = dist / m_current_speed;
                    assert(time >= 0 && time < 1000000 && !std::isnan(time));
                }
            } else if (::atoi(&cmd[1]) == 2 || ::atoi(&cmd[1]) == 3) {
                // TODO: compute real dist
                double distx = line.dist_X(reader);
                double disty = line.dist_Y(reader);
                double dist = distx * distx + disty * disty;
                if (dist > 0) {
                    dist = std::sqrt(dist);
                    assert(m_current_speed > 0 && m_current_speed < 1000000 && !std::isnan(m_current_speed));
                    time = dist / m_current_speed;
                    assert(time >= 0 && time < 1000000 && !std::isnan(time));
                }
            }
            break;
        }
        case 'M':
        {
            std::tie(new_temperature, temperature_extruder_idx) =
                this->_get_temperature_with_extruder_idx(line.raw(), m_writer.config.gcode_flavor);
            if (new_temperature >= 0) {
                if (!m_is_custom_gcode) {
                    // if slow down => put in the queue. if not =>
                    if (m_back_buffer_temperatures[temperature_extruder_idx] >= 0 &&
                        m_back_buffer_temperatures[temperature_extruder_idx] < new_temperature) {
                        if (temperature_extruder_idx < heating_speed.size() && heating_speed[temperature_extruder_idx] > 0) {
                            //compute delay
                            double nb_seconds_delay = (new_temperature - m_back_buffer_temperatures[temperature_extruder_idx]) / heating_speed[temperature_extruder_idx];
                            //don't put this command in the queue
                            time = -1;
                            // this M104 need to go in the past
                            // first erase everything lower than that value
                            //_remove_low_heat(new_temperature, temperature_extruder_idx, nb_seconds_delay);
                            // then write the heat command
                            if (!m_buffer.empty() && (m_buffer_time_size - m_buffer.front().time * 0.1) > nb_seconds_delay) {
                                //_print_in_middle_G1(m_buffer.front(), float(m_buffer_time_size - nb_seconds_delay), line.raw());
                                //remove_from_buffer(m_buffer.begin());
                                float time_count = nb_seconds_delay;
                                auto rit = m_buffer.rbegin();
                                while (rit != m_buffer.rend() && time_count > 0) {
                                    time_count -= rit->time;
                                    if (time_count< 0) {
                                        //found something that is lower than us
                                        _put_in_middle_G1(rit, rit->time + time_count, BufferData(std::string(line.raw()), line.e_char() ,0, new_temperature, temperature_extruder_idx), nb_seconds_delay);
                                        //found, stop
                                        break;
                                    } else if (rit->temperature >= 0 && rit->temperature_extruder_idx == temperature_extruder_idx &&
                                            rit->temperature <= new_temperature) {
                                            //found something that is lower than us
                                        auto it = rit.base();
                                        ++rit;
                                        remove_from_buffer(it);
                                    } else {
                                        ++rit;
                                    }
                                }
                            } else {
                                // buffer not enough, write it before the buffer.
                                m_process_output += line.raw() + "\n";
                            }
                        }
                    }
                    m_back_buffer_temperatures[temperature_extruder_idx] = new_temperature;
                } else {
                    put_in_buffer(BufferData(std::string("; m_is_custom_gcode, so not touched: ") + line.raw(), line.e_char(), 0, -1, 0));
                    // have to flush the buffer to avoid erasing a temperature command.
                    need_flush = true;
                }
            }
            break;
        }
        }
    } else {
        if(!line.raw().empty() && line.raw().front() == ';')
        {
            if (line.raw().size() > 10 && line.raw().rfind(";TYPE:", 0) == 0) {
                // get the type of the next extrusions
                std::string extrusion_string = line.raw().substr(6, line.raw().size() - 6);
                current_role                 = string_to_gcode_extrusion_role(extrusion_string);
                assert(current_role != GCodeExtrusionRole::None);
            }
            if (line.raw().size() > 16) {
                if (line.raw().rfind("; custom gcode", 0) != std::string::npos) {
                    if (line.raw().rfind("; custom gcode end", 0) != std::string::npos) {
                        m_is_custom_gcode = false;
                    } else {
                        m_is_custom_gcode = true;
                    }
                }
            }
        }
    }

    if (time >= 0) {
        BufferData& new_data = put_in_buffer(BufferData(line.raw(), line.e_char(), time, new_temperature, temperature_extruder_idx));
        if (line.has(Axis::X)) {
            new_data.x = reader.x();
            new_data.dx = line.dist_X(reader);
        }
        if (line.has(Axis::Y)) {
            new_data.y = reader.y();
            new_data.dy = line.dist_Y(reader);
        }
        if (line.has(Axis::Z)) {
            new_data.z = reader.z();
            new_data.dz = line.dist_Z(reader);
        }
        if (line.has(Axis::E)) {
            new_data.e = reader.e();
            if (relative_e) {
                new_data.de = line.e();
                // GCode reader doesn't know it's relative extrusion, we have to do it ourself.
                //assert(new_data.e == 0);
                new_data.e = 0;
            } else
                new_data.de = line.dist_E(reader);
        }
        assert(new_data.dx == 0 || reader.x() == new_data.x);
        assert(new_data.dx == 0 || std::abs(reader.x() + new_data.dx - line.x()) < 0.00002f);
        assert(new_data.dy == 0 || reader.y() == new_data.y);
        assert(new_data.dy == 0 || std::abs(reader.y() + new_data.dy - line.y()) < 0.00002f);
        assert(new_data.de == 0 || (relative_e?0:reader.e()) == new_data.e);
        assert(new_data.de == 0 || std::abs((relative_e?0.f:reader.e()) + new_data.de - line.e()) < 0.00001f);
        //assert(new_data.de == 0 ||(relative_e?0.f:reader.e()) + new_data.de == line.e());
    }
    // puts the line back into the gcode
    //if buffer too big, flush it.
    if (time >= 0) {
        // Add EPSILON to allow to have a buffer even with 0 m_buffer_time_size, so multiple consecutive M106 can be culled.
        while (!m_buffer.empty() && (need_flush || m_buffer_time_size - m_buffer.front().time > m_max_seconds_delay + EPSILON) ){
            write_buffer_data();
        }
    }
#if _DEBUG
    double sum = 0;
    for (auto& data : m_buffer) sum += data.time;
    assert( std::abs(m_buffer_time_size - sum) < 0.01);
#endif
}

void TemperatureMover::write_buffer_data()
{
    BufferData &frontdata = m_buffer.front();
    if (frontdata.temperature < 0 || frontdata.temperature != m_front_buffer_temperatures[frontdata.temperature_extruder_idx] ) {
        m_process_output += frontdata.raw + "\n";
        if (frontdata.temperature >= 0) {
            m_front_buffer_temperatures[frontdata.temperature_extruder_idx] = frontdata.temperature;
        }
    } else {
        assert(frontdata.temperature == m_front_buffer_temperatures[frontdata.temperature_extruder_idx]);
        m_process_output += "; skip temp, as it's already set: " + frontdata.raw + "\n";
    }
    remove_from_buffer(m_buffer.begin());
}

} // namespace Slic3r

