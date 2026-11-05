///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include <catch2/catch.hpp>

#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleSelector.hpp"

/*
These tests check scene edits used by the GUI or callers without a GUI.
They compare copied geometry, placement and configuration with the source
values. libslic3r_tests initializes plugin option definitions at startup.

Multipart merge cases exercise Model::merge_objects_into_multipart().
The new object keeps each source's first-instance placement, volume IDs,
settings, layer-height ranges and printability; the sources remain owned
by the same Model until its caller removes them.

Painting cases query ModelVolume::facets_annotation_keys(), check the
alphabetical non-empty keys and remove one entry before querying again.

Representative direct calls (branches are not a timeline):

    TEST_CASE
        |-- model.add_object()
        |-- make_cube()
        |-- first->add_instance()
        |-- set_offset()
        |-- set_rotation()
        `-- model.merge_objects_into_multipart()
*/

using namespace Slic3r;

TEST_CASE("Merging objects keeps the platter geometry of their first instance", "[model_edit]")
{
    Model model;
    ModelObject *first = model.add_object("first", "first.stl", make_cube(10, 20, 30));
    first->add_instance()->set_offset(Vec3d(12, 13, 14));
    first->instances[0]->set_rotation(Z, 0.4);
    ModelObject *second = model.add_object("second", "second.stl", make_cube(5, 6, 7));
    second->add_instance()->set_offset(Vec3d(-8, 21, 2));
    second->instances[0]->set_scaling_factor(Vec3d(1.2, 0.8, 1.5));

    const Transform3d before_first = first->instances[0]->get_matrix() * first->volumes[0]->get_matrix();
    const Transform3d before_second = second->instances[0]->get_matrix() * second->volumes[0]->get_matrix();
    const ObjectID first_volume_id = first->volumes[0]->id();
    const ObjectID second_volume_id = second->volumes[0]->id();
    ModelObject *merged = model.merge_objects_into_multipart({first, second}, "Merged", true);

    REQUIRE(merged->volumes.size() == 2);
    REQUIRE(merged->instances.size() == 1);
    CHECK(merged->name == "Merged");
    CHECK(merged->origin_translation.isZero());
    CHECK(merged->volumes[0]->id() == first_volume_id);
    CHECK(merged->volumes[1]->id() == second_volume_id);
    const Transform3d after_first = merged->instances[0]->get_matrix() * merged->volumes[0]->get_matrix();
    const Transform3d after_second = merged->instances[0]->get_matrix() * merged->volumes[1]->get_matrix();
    CHECK(after_first.matrix().isApprox(before_first.matrix(), 1e-6));
    CHECK(after_second.matrix().isApprox(before_second.matrix(), 1e-6));
    CHECK(model.objects().size() == 3);
    CHECK((first->instances[0]->get_matrix() * first->volumes[0]->get_matrix()).matrix().isApprox(before_first.matrix(), 1e-6));
    CHECK((second->instances[0]->get_matrix() * second->volumes[0]->get_matrix()).matrix().isApprox(before_second.matrix(), 1e-6));
}

TEST_CASE("Merging objects merges settings, layer ranges and printability", "[model_edit]")
{
    Model model;
    ModelObject *first = model.add_object("first", "first.stl", make_cube(10, 10, 10));
    first->add_instance()->printable = false;
    first->config.set_key_value("extruder", new ConfigOptionInt(2));
    first->config.set_key_value("perimeters", new ConfigOptionInt(3));
    first->layer_config_ranges[{0., 1.}].set_key_value("perimeters", new ConfigOptionInt(5));
    ModelObject *second = model.add_object("second", "second.stl", make_cube(5, 5, 5));
    second->add_instance()->printable = true;
    second->config.set_key_value("perimeters", new ConfigOptionInt(7));
    second->config.set_key_value("top_solid_layers", new ConfigOptionInt(4));
    second->layer_config_ranges[{0., 1.}].set_key_value("perimeters", new ConfigOptionInt(8));
    second->layer_config_ranges[{1., 2.}].set_key_value("perimeters", new ConfigOptionInt(6));

    ModelObject *merged = model.merge_objects_into_multipart({first, second}, "Merged", true);
    REQUIRE(merged->instances.size() == 1);
    CHECK(merged->instances[0]->printable);
    CHECK(merged->config.opt_int("perimeters") == 3);
    CHECK(merged->config.opt_int("top_solid_layers") == 4);
    REQUIRE(merged->volumes.size() == 2);
    CHECK(merged->volumes[0]->config.opt_int("extruder") == 2);
    REQUIRE(merged->layer_config_ranges.size() == 2);
    CHECK(merged->layer_config_ranges.at({0., 1.}).opt_int("perimeters") == 5);
    CHECK(merged->layer_config_ranges.at({1., 2.}).opt_int("perimeters") == 6);
}

TEST_CASE("Merging objects assigns the single-part extruder after volume sorting", "[model_edit]")
{
    Model model;
    ModelObject *first = model.add_object("first", "first.stl", make_cube(10, 10, 10));
    first->add_instance();
    ModelVolume *modifier = first->add_volume(make_cube(2, 2, 2));
    modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    ModelObject *second = model.add_object("second", "second.stl", make_cube(5, 5, 5));
    second->add_instance();
    second->config.set_key_value("extruder", new ConfigOptionInt(3));
    const ObjectID part_id = second->volumes[0]->id();
    const ObjectID modifier_id = modifier->id();

    ModelObject *merged = model.merge_objects_into_multipart({first, second}, "Merged", true);
    REQUIRE(merged->volumes.size() == 3);
    CHECK(merged->volumes[1]->id() == part_id);
    CHECK(merged->volumes[1]->config.opt_int("extruder") == 3);
    CHECK(merged->volumes[2]->id() == modifier_id);
    CHECK_FALSE(merged->volumes[2]->config.has("extruder"));
}

TEST_CASE("Volumes list their non-empty facet annotation keys", "[model_edit]")
{
    Model model;
    ModelObject *object = model.add_object("cube", "cube.stl", make_cube(10, 10, 10));
    ModelVolume *volume = object->volumes[0];
    CHECK(volume->facets_annotation_keys().empty());
    volume->facets_annotation_mutable("empty");
    TriangleSelector selector(volume->mesh());
    selector.set_facet(0, EnforcerBlockerType::ENFORCER);
    volume->facets_annotation_mutable("plugin:paint").set(selector);
    selector.set_facet(0, EnforcerBlockerType::BLOCKER);
    volume->facets_annotation_mutable("builtin:seam").set(selector);
    CHECK(volume->facets_annotation_keys() == std::vector<std::string>{"builtin:seam", "plugin:paint"});
    volume->reset_facets_annotation("plugin:paint");
    CHECK(volume->facets_annotation_keys() == std::vector<std::string>{"builtin:seam"});
}
