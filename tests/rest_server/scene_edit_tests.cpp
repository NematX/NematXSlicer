// Headless scene-edit tests exercise model rules before HTTP and session publication.
#include <catch2/catch.hpp>

#include "scene_edit.hpp"
#include "session.hpp"
#include "libslic3r/EmbossShape.hpp"
#include "libslic3r/TextConfiguration.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "rest_models_test_helpers.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Rest;

TEST_CASE("Scene edits follow the GUI part type rules", "[scene_edit]")
{
    Model model;
    ModelObject *object = model.add_object("object", "object.stl", make_cube(10, 10, 10));
    object->add_instance();
    ModelVolume *second = object->add_volume(make_cube(5, 5, 5));
    REQUIRE(set_part_type(*object, *second, ModelVolumeType::PARAMETER_MODIFIER).changed);
    CHECK(second->type() == ModelVolumeType::PARAMETER_MODIFIER);
    CHECK_THROWS_AS(set_part_type(*object, *object->volumes.front(), ModelVolumeType::NEGATIVE_VOLUME), SceneEditError);
    second->emboss_shape = EmbossShape{};
    second->text_configuration = TextConfiguration{};
    CHECK_THROWS_AS(set_part_type(*object, *second, ModelVolumeType::SUPPORT_ENFORCER), SceneEditError);
    CHECK_THROWS_AS(rename_volume(*second, "text"), SceneEditError);
    CHECK_FALSE(rename_object(*object, "object").changed);
}

TEST_CASE("Scene edits delete entities with GUI cut and last-entity rules", "[scene_edit]")
{
    Model model;
    ModelObject *object = model.add_object("object", "object.stl", make_cube(10, 10, 10));
    object->add_instance();
    ModelVolume *modifier = object->add_volume(make_cube(5, 5, 5), ModelVolumeType::PARAMETER_MODIFIER);
    const std::uint64_t retained = object->volumes.front()->id().id;
    const SceneChange change = delete_volume(*object, *modifier, false);
    REQUIRE(change.rebindings.size() == 1);
    CHECK(change.rebindings.front().from_engine_id == retained);
    CHECK_THROWS_AS(delete_volume(*object, *object->volumes.front(), false), SceneEditError);
    CHECK_THROWS_AS(delete_instance(*object, 0), SceneEditError);

    ModelObject *cut_a = model.add_object("cut-a", "cut-a.stl", make_cube(4, 4, 4));
    cut_a->add_instance();
    cut_a->cut_id.init();
    ModelObject *cut_b = model.add_object("cut-b", "cut-b.stl", make_cube(4, 4, 4));
    cut_b->add_instance();
    cut_b->cut_id = cut_a->cut_id;
    CHECK_THROWS_AS(delete_object(model, *cut_a, false), SceneEditError);
    REQUIRE(delete_object(model, *cut_a, true).changed);
    CHECK_FALSE(cut_b->is_cut());
}

TEST_CASE("Scene edits add parts and shapes placed as in the GUI", "[scene_edit]")
{
    Model model;
    ModelObject *target = model.add_object("target", "target.stl", make_cube(10, 10, 10));
    target->add_instance();
    Model imported;
    ModelObject *source = imported.add_object("source", "client.stl", make_cube(4, 4, 4));
    source->add_instance();
    const SceneChange part = add_part(*target, std::move(imported), ModelVolumeType::PARAMETER_MODIFIER, "client.stl");
    CHECK(part.bindings.size() == 1);
    const ShapeRequest request{ModelVolumeType::SUPPORT_ENFORCER, "box", Vec3d(20, 30, 40), Vec3d(50, 60, 70)};
    const SceneChange shape = add_shape(*target, 0, request);
    CHECK(shape.bindings.size() == 1);
    CHECK(target->volumes.back()->source.is_from_builtin_objects);
}

TEST_CASE("Scene edits split objects and parts with new entity IDs", "[scene_edit]")
{
    RestServerTest::ensure_engine_initialized();
    TriangleMesh shells = make_cube(10, 10, 10);
    TriangleMesh other = make_cube(5, 5, 5);
    other.translate(Vec3f(20, 0, 0));
    shells.merge(other);
    Model model;
    ModelObject *object = model.add_object("shells", "shells.stl", std::move(shells));
    object->add_instance();
    const std::uint64_t old_id = object->volumes.front()->id().id;
    const SceneChange change = split_volume(*object, *object->volumes.front(), 2);
    CHECK(change.unbindings.front().engine_id == old_id);
    CHECK(change.bindings.size() == 2);
    CHECK(change.bindings.front().engine_id != old_id);
}

TEST_CASE("Scene shape validation leaves the target untouched before adding a volume", "[scene_edit][rest_shape_bounds]")
{
    Model model;
    ModelObject *object = model.add_object("object", "object.stl", make_cube(10, 10, 10));
    object->add_instance();
    const ModelVolume *solid = object->volumes.front();
    for (double x : {1e100, 1e-300}) {
        CAPTURE(x);
        const ShapeRequest request{ModelVolumeType::PARAMETER_MODIFIER, "box", Vec3d(x, 20, 20)};
        CHECK_THROWS_AS(add_shape(*object, 0, request), SceneEditError);
        REQUIRE(object->volumes.size() == 1);
        CHECK(object->volumes.front() == solid);
    }
    ModelObject *flat = model.add_object("flat", "flat.stl", make_cube(10, 10, 0));
    flat->add_instance();
    const ShapeRequest slab{ModelVolumeType::PARAMETER_MODIFIER, "slab"};
    CHECK_THROWS_AS(add_shape(*flat, 0, slab), SceneEditError);
    CHECK(flat->volumes.size() == 1);
}

TEST_CASE("Scene edits merge single-instance objects and keep part IDs", "[scene_edit]")
{
    Model model;
    ModelObject *first = model.add_object("first", "first.stl", make_cube(10, 10, 10));
    first->add_instance();
    ModelObject *second = model.add_object("second", "second.stl", make_cube(5, 5, 5));
    second->add_instance()->set_offset(Vec3d(20, 0, 0));
    const std::set<std::uint64_t> part_ids{first->volumes.front()->id().id, second->volumes.front()->id().id};
    const SceneChange change = merge_objects(model, {first, second}, "Merged");
    REQUIRE(model.objects().size() == 1);
    CHECK(part_ids.count(model.get_object(0).volumes[0]->id().id) == 1);
    CHECK(part_ids.count(model.get_object(0).volumes[1]->id().id) == 1);
    CHECK(change.unbindings.size() == 4);
}

TEST_CASE("Scene edit rebinding preserves REST IDs and rejects stale commits", "[scene_edit][rest_entities]")
{
    SessionRegistry registry(std::chrono::hours(24));
    const std::shared_ptr<SessionContext> session = registry.create("alice", SessionContext::TimePoint{});
    const std::string id = session->bind_entity(EntityKind::volume, 11).value();
    const SessionContext::MutationToken stale = session->begin_mutation().value();
    REQUIRE(session->commit_mutation(stale, SessionData{}, false,
        SessionContext::TimePoint{} + std::chrono::hours(1)));

    std::vector<std::string> ignored;
    const std::vector<EntityRebinding> move{{EntityKind::volume, 11, 12}};
    CHECK_FALSE(session->commit_mutation(stale, SessionData{}, false,
        SessionContext::TimePoint{} + std::chrono::hours(2), {}, ignored, false, {}, move));
    CHECK(session->resolve_entity(id)->engine_id == 11);

    const SessionContext::MutationToken current = session->begin_mutation().value();
    REQUIRE(session->commit_mutation(current, SessionData{}, false,
        SessionContext::TimePoint{} + std::chrono::hours(3), {}, ignored, false, {}, move));
    REQUIRE(session->resolve_entity(id).has_value());
    CHECK(session->resolve_entity(id)->engine_id == 12);
    CHECK_FALSE(session->bind_entity(EntityKind::volume, 11).value() == id);
}
