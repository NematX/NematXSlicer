#include <catch2/catch.hpp>

#include <atomic>
#include <cmath>
#include <string>
#include <vector>

#include "plugin_e2e_test_helpers.hpp"
#include "support_grid_test_helpers.hpp"
#include "support_pattern_test_context.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Plugins/Support/Filled/FilledSupportGenerator.hpp"
#include "libslic3r/Plugins/Support/Tree/TreeSupportGenerator.hpp"

/* The contact XY gap (support_material_contact_xy_spacing), checked on the slabs of the support families of small sliced
   models. Each fixture runs once with the setting disabled and once with it enabled, with a 1.5-mm XY gap and a 0.1-mm contact
   gap on 0.4-mm nozzles:
   - small zone: a tab narrower than the XY gap beside a post;
   - steep slope: a tilted plate whose underside overhangs by less than the XY gap per layer;
   - sharp angle: an inverted prism whose lower edge is a sharp angle between two such slopes;
   - normal support: the overhanging roof of SG::slab(), whose sparse body far from the contacts must not change.
   In contact mode every slab keeps the contact gap from the object layers beside it, and the part of a non-contact slab nearer
   than the XY gap lies under a contact shrunk by at least one cone step. The settings that do not reduce the gap give the same
   slabs as the disabled setting, and the tree family ignores the setting.
   Growth variants compare the fixed-step reference with the default, measure clearance at other steps, and export a small STL
   through the normal pipeline. Contact areas are measured before toolpaths can merge them into interfaces; paths and G-code
   are checked after slicing. The shared support gap-fill catalogue checks coverage of all declared geometry keys. */

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
namespace Common = slic3r_api::Support::Common;

namespace {
constexpr const char *xy_gap = "1.5";
constexpr const char *contact_gap = "0.1";

// The slabs of one family run on the first object of a sliced print.
struct Generated {
    PluginStorage storage;
    Common::GeneratedSupport support{SG::handle(storage)};
    Common::SupportParams params;
    Generated(Print &print, const std::string &style, const Common::SupportParams *reference = nullptr)
        : params(reference == nullptr ? SG::params(print, print.object(0)) : *reference)
    {
        const PrintObject &object = print.object(0);
        if (style == "filled")
            slic3r_api::Support::Filled::generate_support(nullptr, SG::print_view(print), SG::object_view(object), params, support);
        else if (style == "tree" || style == "organic") {
            const Slic3r::Test::SupportTree::SupportPatternContext patterns(print, SG::handle(storage));
            Slic3r::TreeFamily::generate_tree_support(patterns.context(), SG::print_view(print), SG::object_view(object), params, style == "organic",
                                                      support);
        } else
            slic3r_api::Support::Grid::generate_support(nullptr, SG::print_view(print), SG::object_view(object), params, support);
    }
};

// A sliced print of mesh with the family style, the XY gap and the given contact gap ("!..." disables it).
struct Sliced {
    Print print;
    Model model;
    Sliced(const TriangleMesh &mesh, const std::string &style, const char *contact, const char *xy = xy_gap, int interfaces = 3,
           const char *growth = nullptr, const char *resolution = nullptr)
    {
        DynamicPrintConfig config = SG::config(style.c_str(), style == "filled" || style == "grid_merged", interfaces);
        config.set_deserialize_strict({{"support_material_xy_spacing", xy}, {"support_material_contact_xy_spacing", contact},
                                        {"support_material_threshold", "60"}, {"raft_layers", "0"}});
        if (growth != nullptr) config.set_deserialize_strict("support_material_contact_cone_growth", growth);
        if (resolution != nullptr) config.set_deserialize_strict("resolution_internal", resolution);
        Test::init_print({mesh}, print, model, config);
        SG::prefix(print);
    }
};

// A 10 x 10 x 6 mm post with a 1 x 4 x 1 mm tab on its +x side, 5 mm above the bed.
TriangleMesh small_zone()
{
    TriangleMesh post = make_cube(10., 10., 6.);
    TriangleMesh tab = make_cube(1., 4., 1.);
    tab.translate(10.f, 3.f, 5.f);
    post.merge(tab);
    return post;
}

// A 12 x 8 x 1 mm plate tilted by 15 degrees, whose high end rests on a 2 x 8 mm post; its low end is 3 mm above the bed.
TriangleMesh steep_slope()
{
    TriangleMesh plate = make_cube(12., 8., 1.);
    plate.rotate_y(-float(15. * PI / 180.));
    plate.translate(0.f, 0.f, 3.f - plate.bounding_box().min.z());
    TriangleMesh post = make_cube(2., 8., plate.bounding_box().max.z());
    post.translate(float(plate.bounding_box().max.x()) - 2.f, 0.f, 0.f);
    plate.merge(post);
    return plate;
}

// A 10-mm wide, 1.3-mm high inverted prism along y whose lower edge lies 4 mm above the bed, held by a post at its +y end.
TriangleMesh sharp_angle()
{
    TriangleMesh prism(its_make_prism(10.f, 8.f, 1.3f));
    prism.rotate_y(float(PI));
    prism.translate(5.f, 4.f, 5.3f);
    TriangleMesh post = make_cube(10., 2., 5.3);
    post.translate(0.f, 8.f, 0.f);
    prism.merge(post);
    return prism;
}

// The union of the object layers whose span meets the printed height of the slab.
Polygons beside(const PrintObject &object, const Common::GeneratorLayer &slab)
{
    Polygons result;
    for (const Layer &layer : object.layers())
        if (layer.scaled_print_z() > slab.print_z - slab.height && layer.scaled_bottom_z() < slab.print_z)
            append(result, to_polygons(layer.lslices()));
    return union_(result);
}

// The area (mm2) of polygons nearer than distance to the object layers beside the slab. Round joins measure the true distance;
// the production square joins keep at least as much.
double near_area(const PrintObject &object, const Common::GeneratorLayer &slab, coord_t distance)
{
    const Polygons near = intersection(SG::native(slab.polygons),
        offset(beside(object, slab), double(distance - SCALED_EPSILON), ClipperLib::jtRound, scale_d(0.01)));
    return unscaled(unscaled(area(near)));
}

double contact_area(const Generated &generated)
{
    Polygons all;
    for (const Common::GeneratorLayer *contact : generated.support.top)
        append(all, SG::native(contact->polygons));
    return unscaled(unscaled(area(union_(all))));
}

// Every slab keeps the contact gap; the part of a non-contact slab nearer than the XY gap lies under a contact shrunk by one step.
void check_gaps(const PrintObject &object, const Generated &generated)
{
    const Common::SupportParams &p = generated.params;
    REQUIRE(p.contact_mode);
    Polygons contacts;
    for (const Common::GeneratorLayers *list : {&generated.support.top, &generated.support.bottom})
        for (const Common::GeneratorLayer *contact : *list)
            append(contacts, SG::native(contact->polygons));
    const Polygons cone_bound = p.contact_cone_layers == 0 ? Polygons{} :
        offset(union_(contacts), -double(p.contact_cone_step - SCALED_EPSILON), ClipperLib::jtSquare);
    for (const Common::GeneratorLayers *list : {&generated.support.top, &generated.support.bottom, &generated.support.base,
                                                &generated.support.interfaces})
        for (const Common::GeneratorLayer *slab : *list) {
            if (slab->polygons.empty() || slab->height <= 0) continue;
            INFO("slab z=" << unscaled(slab->print_z) << " type=" << int(slab->type));
            CHECK(near_area(object, *slab, p.gap_xy_contact) == Approx(0.).margin(1e-4));
            if (list == &generated.support.top || list == &generated.support.bottom) continue;
            const Polygons near = intersection(SG::native(slab->polygons),
                offset(beside(object, *slab), double(p.gap_xy - SCALED_EPSILON), ClipperLib::jtRound, scale_d(0.01)));
            CHECK(unscaled(unscaled(area(diff(near, cone_bound)))) == Approx(0.).margin(1e-3));
        }
}

// Whether two runs give the same slabs, list by list, at the same Z and with the same area.
bool same_slabs(const Generated &a, const Generated &b)
{
    const Common::GeneratorLayers *lists_a[] = {&a.support.top, &a.support.bottom, &a.support.base, &a.support.interfaces};
    const Common::GeneratorLayers *lists_b[] = {&b.support.top, &b.support.bottom, &b.support.base, &b.support.interfaces};
    for (size_t list = 0; list < 4; ++list) {
        if (lists_a[list]->size() != lists_b[list]->size()) return false;
        for (size_t i = 0; i < lists_a[list]->size(); ++i) {
            const Common::GeneratorLayer &sa = *(*lists_a[list])[i], &sb = *(*lists_b[list])[i];
            if (sa.print_z != sb.print_z || sa.height != sb.height) return false;
            const Polygons &pa = SG::native(sa.polygons), &pb = SG::native(sb.polygons);
            if (!diff(pa, pb).empty() || !diff(pb, pa).empty()) return false;
        }
    }
    return true;
}

// Run a fixture in both modes, check the gaps of the contact mode and return the contact areas (disabled, enabled).
std::pair<double, double> both_modes(const TriangleMesh &mesh, const std::string &style)
{
    Sliced off(mesh, style, (std::string("!") + contact_gap).c_str());
    Sliced on(mesh, style, contact_gap);
    const Generated without(off.print, style), with(on.print, style);
    CHECK_FALSE(without.params.contact_mode);
    check_gaps(on.print.object(0), with);
    return {contact_area(without), contact_area(with)};
}
} // namespace

TEST_CASE("Contact gap supports a tab narrower than the XY gap", "[plugins][support-contact-xy]")
{
    const std::string style = GENERATE(as<std::string>{}, "grid", "snug");
    CAPTURE(style);
    const std::pair<double, double> areas = both_modes(small_zone(), style);
    CHECK(areas.second > areas.first + 1.);
}

TEST_CASE("Contact gap supports the top of a steep slope", "[plugins][support-contact-xy]")
{
    const std::pair<double, double> areas = both_modes(steep_slope(), "grid");
    CHECK(areas.second > areas.first + 1.);
}

TEST_CASE("Contact gap supports a sharp lower edge", "[plugins][support-contact-xy]")
{
    const std::pair<double, double> areas = both_modes(sharp_angle(), "grid");
    CHECK(areas.second > areas.first + 1.);
}

TEST_CASE("Contact gap keeps the normal sparse support away from the contacts", "[plugins][support-contact-xy]")
{
    Sliced off(SG::slab(), "grid", "!0.1");
    Sliced on(SG::slab(), "grid", contact_gap);
    const Generated without(off.print, "grid"), with(on.print, "grid");
    check_gaps(on.print.object(0), with);
    const Common::SupportParams &p = with.params;
    // The roof is 4 mm high: its contact keeps only the contact gap from the post, and the interface slabs below it come back
    // to the XY gap by cone steps.
    CHECK(contact_area(with) >= contact_area(without));
    for (const Common::GeneratorLayer *slab : with.support.interfaces) {
        INFO("interface z=" << unscaled(slab->print_z));
        CHECK(near_area(on.print.object(0), *slab, p.gap_xy_contact + p.contact_cone_step) == Approx(0.).margin(1e-4));
    }
    // The sparse slabs further than the cone below the contact are unchanged.
    const coord_t contact_bottom = with.support.top.front()->bottom_z;
    const coord_t far = contact_bottom - coord_t(p.contact_cone_layers + 1) * scale_to_layer_coord(0.3);
    REQUIRE(with.support.base.size() == without.support.base.size());
    size_t compared = 0;
    for (size_t i = 0; i < with.support.base.size(); ++i) {
        const Common::GeneratorLayer &a = *with.support.base[i], &b = *without.support.base[i];
        REQUIRE(a.print_z == b.print_z);
        if (a.print_z >= far) continue;
        INFO("base z=" << unscaled(a.print_z));
        const Polygons &pa = SG::native(a.polygons), &pb = SG::native(b.polygons);
        CHECK((diff(pa, pb).empty() && diff(pb, pa).empty()));
        ++compared;
    }
    CHECK(compared > 0);
}

TEST_CASE("Contact gap settings that do not reduce the XY gap change nothing", "[plugins][support-contact-xy]")
{
    for (const char *style : {"grid", "snug"}) {
        CAPTURE(style);
        Sliced disabled(small_zone(), style, "!0.1");
        Sliced wide(small_zone(), style, xy_gap);
        const Generated reference(disabled.print, style), same(wide.print, style);
        CHECK_FALSE(same.params.contact_mode);
        CHECK(same_slabs(reference, same));
        Sliced no_gap_off(small_zone(), style, "!0", "0");
        Sliced no_gap_on(small_zone(), style, "0", "0");
        const Generated a(no_gap_off.print, style), b(no_gap_on.print, style);
        CHECK_FALSE(b.params.contact_mode);
        CHECK(same_slabs(a, b));
    }
}

TEST_CASE("Filled support contacts keep only the contact gap", "[plugins][support-contact-xy][support-filled]")
{
    const std::pair<double, double> areas = both_modes(small_zone(), "filled");
    CHECK(areas.second > areas.first + 1.);
}

TEST_CASE("Tree support ignores the contact gap", "[plugins][support-contact-xy][support-tree]")
{
    Sliced off(small_zone(), "tree", "!0.1");
    Sliced on(small_zone(), "tree", contact_gap);
    const Generated without(off.print, "tree"), with(on.print, "tree");
    REQUIRE(with.params.contact_mode);
    CHECK(same_slabs(without, with));
}

// Reconstruct the fixed 0.67-nozzle reference independently, and compare the complete generated slabs with the default setting.
TEST_CASE("Default cone growth preserves fixed-step support slabs", "[plugins][support-contact-xy]")
{
    for (const char *style : {"grid", "snug", "grid_merged", "filled"}) {
        CAPTURE(style);
        Sliced sliced(small_zone(), style, contact_gap);
        const Generated current(sliced.print, style);
        Common::SupportParams fixed = current.params;
        REQUIRE(fixed.contact_mode);
        fixed.contact_cone_step = std::max(coord_t(1), scale_i(0.67 * fixed.interface_flow_mm.nozzle_diameter));
        fixed.contact_cone_layers = size_t((fixed.gap_xy - fixed.gap_xy_contact + fixed.contact_cone_step - 1) / fixed.contact_cone_step);
        const Generated reference(sliced.print, style, &fixed);
        CHECK(same_slabs(current, reference));
        check_gaps(sliced.print.object(0), current);
    }
}

// The contact stays at the reduced gap; only its transition changes, and zero leaves every ordinary slab at the normal gap.
TEST_CASE("Cone growth changes transition slabs without moving support contacts", "[plugins][support-contact-xy]")
{
    for (const char *style : {"grid", "snug", "grid_merged", "filled"}) {
        CAPTURE(style);
        // A wide roof retains Filled's lower projection; fine resolution isolates cone bounds from polygon simplification.
        Sliced normal(SG::slab(), style, contact_gap, xy_gap, 3, nullptr, "0.000001");
        const Generated reference(normal.print, style);
        for (const char *growth : {"25%", "100%", "0"}) {
            CAPTURE(growth);
            Sliced varied(SG::slab(), style, contact_gap, xy_gap, 3, growth, "0.000001");
            const Generated result(varied.print, style);
            CHECK(contact_area(result) == Approx(contact_area(reference)));
            CHECK_FALSE(same_slabs(result, reference));
            check_gaps(varied.print.object(0), result);
        }
    }
}

// Ignored input values must not leak into tree geometry or into the ordinary-gap fallback.
TEST_CASE("Cone growth changes nothing outside contact mode or in tree families", "[plugins][support-contact-xy]")
{
    for (const char *style : {"grid", "snug", "tree", "organic"}) {
        for (const char *contact : {"!0.1", "1.5"}) {
            CAPTURE(style, contact);
            Sliced a(small_zone(), style, contact, xy_gap, 3, "25%");
            Sliced b(small_zone(), style, contact, xy_gap, 3, "100%");
            const Generated first(a.print, style), second(b.print, style);
            CHECK_FALSE(first.params.contact_mode);
            CHECK(same_slabs(first, second));
        }
    }
    for (const char *style : {"tree", "organic"}) {
        CAPTURE(style);
        Sliced a(small_zone(), style, contact_gap, xy_gap, 3, "0");
        Sliced b(small_zone(), style, contact_gap, xy_gap, 3, "100%");
        const Generated first(a.print, style), second(b.print, style);
        REQUIRE(first.params.contact_mode);
        CHECK(same_slabs(first, second));
    }
}

// The imported tab really publishes contact surfaces and prints support-interface moves for every independent growth variation.
TEST_CASE("Contact cone growth exports a linear STL matrix", "[plugins][support-contact-xy][plugin-e2e][contact-cone-e2e]")
{
    namespace E2E = Test::PluginE2E;
    DynamicPrintConfig baseline = SG::config();
    baseline.set_deserialize_strict({{"support_material_xy_spacing", xy_gap}, {"support_material_contact_xy_spacing", contact_gap},
        {"support_material_threshold", "60"}, {"gcode_flavor", "marlin2"}, {"gcode_comments", "1"},
        {"filament_diameter", "1.75,1.75"},
        {"gcode_exporter_plugin", "gcode.exporter.simple"}});
    const std::vector<E2E::Case> cases = E2E::linear_cases(Orchestrator::instance(), {}, baseline,
        {{{"support_material_contact_cone_growth"}, {{"gentle", {{"support_material_contact_cone_growth", "25%"}}},
            {"short", {{"support_material_contact_cone_growth", "100%"}}},
            {"absolute", {{"support_material_contact_cone_growth", "0.268"}}},
            {"no cones", {{"support_material_contact_cone_growth", "0"}}}}}});
    const TriangleMesh mesh = E2E::stl_fixture(small_zone());
    double default_contact = 0., default_volume = 0.;
    for (const E2E::Case &c : cases) {
        CAPTURE(c.name);
        DynamicPrintConfig config = E2E::apply_case(baseline, c);
        Print print;
        Model model;
        Test::init_print({mesh}, print, model, config);
        double contact = 0., support_volume = 0.;
        std::atomic<bool> observed{false};
        // The first toolpath status precedes dispatch. Contacts still have their marker before automatic interface fusion.
        print.set_status_callback([&](const PrintBase::SlicingStatus &) {
            if (print.running_step() != STEP_SUPPORT_TOOLPATH || observed.exchange(true)) return;
            for (const Layer &layer : print.object(0).auxiliary_layers())
                for (const LayerSliceIsland &island : layer.islands())
                    for (const LayerRegionIsland &region : island.regions_islands())
                        for (const Surface &surface : region.fill_surfaces()) {
                            const double area = unscaled(unscaled(std::abs(surface.expolygon.area())));
                            if ((surface.surface_type & stModContact) != 0) contact += area;
                            if (surface.has_mod_support()) support_volume += area * unscaled(layer.scaled_height());
                        }
        });
        print.process();
        print.set_status_silent();
        REQUIRE(observed.load());
        CHECK(contact > 0.);
        CHECK(support_volume > 0.);
        if (c.name == "default") {
            default_contact = contact;
            default_volume = support_volume;
        } else {
            CHECK(contact == Approx(default_contact));
            if (c.name == "absolute") CHECK(support_volume == Approx(default_volume));
            else CHECK(support_volume != Approx(default_volume));
        }
        double interface_length = 0.;
        for (const Layer &layer : print.object(0).auxiliary_layers()) {
            for (const auto &[role, measure] : SG::measure_layer(layer))
                if (is_support_interface(static_cast<ExtrusionRoleModifier>(role.second)))
                    interface_length += measure.length;
        }
        CHECK(interface_length > 0.);
        const std::string code = Test::gcode(print);
        size_t interface_moves = 0;
        std::string role;
        GCodeReader reader;
        reader.apply_config(config);
        reader.parse_buffer(code, [&](GCodeReader &state, const GCodeReader::GCodeLine &line) {
            if (line.comment().rfind("TYPE:", 0) == 0) role = line.comment().substr(5);
            if (role == "Support material interface" && line.extruding(state) && line.dist_XY(state) > 0.) ++interface_moves;
        });
        CHECK(interface_moves > 0);
        REQUIRE(print.step_state_with_warnings(STEP_SUPPORT).is_done());
        config.set_deserialize_strict("support_material_contact_cone_growth", c.name == "gentle" ? "67%" : "25%");
        CHECK(print.apply(model, config) == Print::APPLY_STATUS_INVALIDATED);
        CHECK_FALSE(print.step_state_with_warnings(STEP_SUPPORT).is_done());
        // Support changes resume from the shared Slicing checkpoint, which also replays perimeters but keeps slices.
        CHECK(print.step_state_with_warnings(STEP_POST_SLICING).is_done());
    }
}
