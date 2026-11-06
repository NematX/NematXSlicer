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
