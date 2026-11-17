#include <catch2/catch.hpp>

#include <cmath>
#include <string>
#include <vector>

#include "support_grid_test_helpers.hpp"
#include "support_pattern_test_context.hpp"
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
   slabs as the disabled setting, and the tree family ignores the setting. */

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
    Generated(Print &print, const std::string &style)
        : params(SG::params(print, print.object(0)))
    {
        const PrintObject &object = print.object(0);
        if (style == "filled")
            slic3r_api::Support::Filled::generate_support(nullptr, SG::print_view(print), SG::object_view(object), params, support);
        else if (style == "tree") {
            const Slic3r::Test::SupportTree::SupportPatternContext patterns(print, SG::handle(storage));
            Slic3r::TreeFamily::generate_tree_support(patterns.context(), SG::print_view(print), SG::object_view(object), params, false,
                                                      support);
        } else
            slic3r_api::Support::Grid::generate_support(nullptr, SG::print_view(print), SG::object_view(object), params, support);
    }
};

// A sliced print of mesh with the family style, the XY gap and the given contact gap ("!..." disables it).
struct Sliced {
    Print print;
    Model model;
    Sliced(const TriangleMesh &mesh, const std::string &style, const char *contact, const char *xy = xy_gap, int interfaces = 3)
    {
        DynamicPrintConfig config = SG::config(style.c_str(), style == "filled", interfaces);
        config.set_deserialize_strict({{"support_material_xy_spacing", xy}, {"support_material_contact_xy_spacing", contact},
                                       {"support_material_threshold", "60"}, {"raft_layers", "0"}});
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
    const Polygons cone_bound = offset(union_(contacts), -double(p.contact_cone_step - SCALED_EPSILON), ClipperLib::jtSquare);
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
