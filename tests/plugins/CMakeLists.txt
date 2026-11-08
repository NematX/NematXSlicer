get_filename_component(_TEST_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)

add_executable(${_TEST_NAME}_tests
    ${_TEST_NAME}_tests.cpp
    ../../src/plugins_cpp/DenseInfill/DenseInfill.cpp
    ../../src/plugins_cpp/ExtraPerimeterOverhangWave/ExtraPerimeterOverhangWave.cpp
    ../../src/plugins_cpp/FlatAreaLayerHeight/FlatAreaLayerHeight.cpp
    ../../src/plugins_cpp/NozzleCleaning/NozzleCleaning.cpp
    ../../src/plugins_cpp/Polyholes/Polyholes.cpp
    clipper_offset_test_helpers.cpp
    gcode_test_helpers.cpp
    perimeter_test_helpers.cpp
    plugin_e2e_test_helpers.cpp
    plugin_test_helpers.cpp
    test_adaptive_cubic_pattern.cpp
    test_lightning_pattern.cpp
    test_auxiliary_layer_helpers.cpp
    test_bridge_detector_cancellation.cpp
    test_bridge_ordering.cpp
    test_bridge_pipeline.cpp
    test_bridge_preparation.cpp
    mesh_queries_c_fixture.c
    plugin_api_headers_c_fixture.c
    support_pattern_c_fixture.c
    mesh_query_visibility_reference.cpp
    test_mesh_queries.cpp
    test_triangle_mesh_storage.cpp
    test_public_kdtree.cpp
    test_clipper_views.cpp
    test_geometry_cleanup_views.cpp
    test_clipper_offset2_parity.cpp
    test_clipper_offset_contacts.cpp
    test_clipper_offset_parity.cpp
    test_clipper_offset_strategies.cpp
    test_bambu_config_export.cpp
    test_config_translations.cpp
    test_deferred_callback_exception.cpp
    test_extrusion_continuity.cpp
    test_extrusion_linear_points.cpp
    test_extrusion_ordered_leaf.cpp
    test_geometry_primitives.cpp
    test_extrusion_split_by_areas.cpp
    test_extrusion_views.cpp
    test_fff_print_gui_rules.cpp
    test_plugin_activation_config.cpp
    test_plugin_api_compatibility.cpp
    test_plugin_config.cpp
    test_plugin_e2e_matrix.cpp
    test_plugin_loader_diagnostics.cpp
    test_plugin_property.cpp
    test_print_records.cpp
    test_printing_entity_property_traversal.cpp
    test_plugin_repository.cpp
    test_plugin_translation_catalogs.cpp
    test_python_plugin_clear.cpp
    test_repository_updater.cpp
    test_seam_placer_cost.cpp
    test_seam_placer_fixed_direction.cpp
    test_seam_placer_service.cpp
    test_simple_infill_patterns.cpp
    test_line_pattern.cpp
    test_arachne_infill.cpp
    test_concentric_pattern.cpp
    test_smooth_pattern.cpp
    test_post_full_fill_volume.cpp
    test_rectilinear_edge_grid.cpp
    test_rectilinear_clipper.cpp
    test_rectilinear_work_buffers.cpp
    test_rectilinear_connect.cpp
    test_rectilinear_fills.cpp
    test_rectilinear_sawtooth.cpp
    test_rectilinear_with_perimeter.cpp
    test_gcode_default_firmware_session.cpp
    test_gcode_exporter.cpp
    test_step_export.cpp
    test_gcode_statistics.cpp
    test_gcode_custom_gcode_per_print_z.cpp
    test_gcode_feature_gcode.cpp
    test_gcode_firmware_dialects.cpp
    test_arc_geometry.cpp
    test_gcode_firmware_state.cpp
    test_gcode_line_parser.cpp
    test_gcode_layer_height_slider.cpp
    test_gcode_reader_compatibility.cpp
    test_gcode_reader.cpp
    test_gcode_post_process.cpp
    test_gcode_printing_plan_file_writer.cpp
    test_gcode_script_processor.cpp
    test_gcode_settings_scripts.cpp
    test_gui_rule_explanations.cpp
    test_gui_rule_helpers.cpp
    test_gyroid_pattern.cpp
    test_heater_temperature.cpp
    test_infill_pattern_init.cpp
    test_infill_pattern_lists.cpp
    test_layer_extrusion_edit_entry_state.cpp
    test_layer_extrusion_edit_lift.cpp
    test_layer_extrusion_edit_retraction.cpp
    test_layer_extrusion_edit_retraction_wipe.cpp
    test_layer_extrusion_edit_toolchange.cpp
    test_layer_extrusion_edit_transition_scope.cpp
    test_layer_extrusion_edit_travel.cpp
    test_layer_extrusion_edit_travel_deceleration.cpp
    test_layer_extrusion_edit_speed.cpp
    test_layer_region_support_flow.cpp
    test_layer_extrusion_edit_temperature.cpp
    test_support_filled_family.cpp
    test_support_grid_family.cpp
    test_support_grid_legacy_compare.cpp
    test_support_tree_dump.cpp
    test_support_tree_routing.cpp
    test_support_tree_settings.cpp
    test_support_tree_volumes.cpp
test_support_tree_family.cpp
    test_support_tree_legacy_compare.cpp
    test_support_publication.cpp
    test_support_slab_placement.cpp
    test_flat_area_layer_height.cpp
    test_nozzle_cleaning.cpp
    test_ordering_advanced_extrusion_tree.cpp
    test_ordering_engine.cpp
    test_ordering_printing_extrusion_presort.cpp
    test_ordering_wipe_into_first.cpp
    test_ordering_xy_serpentine.cpp
    test_ordering_extrusion_tree.cpp
    test_orchestrator_plugin_execution.cpp
    test_max_overhang_threshold.cpp
    test_overhang_slope_fill.cpp
    test_perimeter_empty_steps.cpp
    test_perimeter_arachne_generator.cpp
    test_perimeter_classic_generator.cpp
    test_perimeter_detect_overhang.cpp
    test_perimeter_extra_perimeter_below_area.cpp
    test_perimeter_extra_perimeter_count.cpp
    test_perimeter_extra_perimeter_odd_layer.cpp
    test_perimeter_extra_perimeters_on_overhangs.cpp
    test_perimeter_fuzzy_skin.cpp
    test_perimeter_hairy_object.cpp
    test_perimeter_mark_first_loop.cpp
    test_perimeter_only_one_perimeter_first_layer.cpp
    test_perimeter_only_one_perimeter_on_top.cpp
    test_perimeter_python_simple_generator.cpp
    test_perimeter_remove_gap_fill_on_overhangs.cpp
    test_perimeter_separate_hole_contour.cpp
    test_perimeter_simple_generator.cpp
    test_plugin_storage.cpp
    test_plugin_run_context.cpp
    test_polygon_simplify.cpp
    test_polyholes.cpp
    test_output_filename.cpp
    test_printing_plan.cpp
    test_print_status_queue.cpp
    test_skirt_brim_default_generator.cpp
    test_hole_size_compensation.cpp
    test_slice_volume.cpp
    test_xy_size_compensation_geometry.cpp
    test_xy_size_compensation_integration.cpp
    test_step_pipeline_invalidation.cpp
    test_step_selector_snapshot.cpp
    test_step_selector_traits.cpp
    test_string_helpers.cpp
    test_dense_infill.cpp
    test_support_demand_buildplate_only.cpp
    test_support_demand_modifiers.cpp
    test_support_demand_overhangs.cpp
    test_support_toolpath_step.cpp
    test_support_toolpath_legacy.cpp
    test_support_toolpath_branch.cpp
    test_support_branch_classify.cpp
    test_support_common.cpp
    test_support_step.cpp
    test_layer_extrusion_edit_support_acceleration.cpp
    test_support_pattern_service.cpp
    test_support_tool_ordering.cpp
    test_surface_clean_infill_surfaces.cpp
    test_surface_initial_typed_surface_builder.cpp
    test_surface_infill_region_compatibility_splitter.cpp
    test_surface_solid_shells.cpp
    test_surface_top_surface_expansion.cpp
    test_tool_change_wipe.cpp
    test_tree_geometry.cpp
    test_vase_multi_island_connector.cpp
    test_vendor_config.cpp
    test_wipe_tower_footprint.cpp
    test_wipe_tower_layout.cpp
    test_wipe_tower_manager.cpp
    test_wipe_tower_toolchange_unretract.cpp
    )

add_library(plugin_loader_missing_register_fixture SHARED plugin_loader_fixture.cpp)
target_compile_definitions(plugin_loader_missing_register_fixture PRIVATE
    SLIC3R_PLUGIN_EXPORTS
    SLIC3R_TEST_PLUGIN_MISSING_REGISTER)
target_include_directories(plugin_loader_missing_register_fixture PRIVATE ../..)
set_target_properties(plugin_loader_missing_register_fixture PROPERTIES FOLDER "tests/plugins")

add_library(plugin_loader_missing_abi_fixture SHARED plugin_loader_fixture.cpp)
target_compile_definitions(plugin_loader_missing_abi_fixture PRIVATE
    SLIC3R_PLUGIN_EXPORTS
    SLIC3R_TEST_PLUGIN_MISSING_ABI)
target_include_directories(plugin_loader_missing_abi_fixture PRIVATE ../..)
set_target_properties(plugin_loader_missing_abi_fixture PROPERTIES FOLDER "tests/plugins")

add_library(plugin_loader_bad_api_major_fixture SHARED plugin_loader_fixture.cpp)
target_compile_definitions(plugin_loader_bad_api_major_fixture PRIVATE
    SLIC3R_PLUGIN_EXPORTS
    SLIC3R_TEST_PLUGIN_BAD_API_MAJOR
    SLIC3R_TEST_PLUGIN_THROW_REGISTER)
target_include_directories(plugin_loader_bad_api_major_fixture PRIVATE ../..)
set_target_properties(plugin_loader_bad_api_major_fixture PROPERTIES FOLDER "tests/plugins")

add_library(plugin_loader_bad_api_minor_fixture SHARED plugin_loader_fixture.cpp)
target_compile_definitions(plugin_loader_bad_api_minor_fixture PRIVATE
    SLIC3R_PLUGIN_EXPORTS
    SLIC3R_TEST_PLUGIN_BAD_API_MINOR)
target_include_directories(plugin_loader_bad_api_minor_fixture PRIVATE ../..)
set_target_properties(plugin_loader_bad_api_minor_fixture PROPERTIES FOLDER "tests/plugins")

add_library(plugin_loader_short_api_table_fixture SHARED plugin_loader_fixture.cpp)
target_compile_definitions(plugin_loader_short_api_table_fixture PRIVATE
    SLIC3R_PLUGIN_EXPORTS
    SLIC3R_TEST_PLUGIN_SHORT_API_TABLE)
target_include_directories(plugin_loader_short_api_table_fixture PRIVATE ../..)
set_target_properties(plugin_loader_short_api_table_fixture PROPERTIES FOLDER "tests/plugins")

add_library(plugin_loader_long_api_table_fixture SHARED plugin_loader_fixture.cpp)
target_compile_definitions(plugin_loader_long_api_table_fixture PRIVATE
    SLIC3R_PLUGIN_EXPORTS
    SLIC3R_TEST_PLUGIN_LONG_API_TABLE)
target_include_directories(plugin_loader_long_api_table_fixture PRIVATE ../..)
set_target_properties(plugin_loader_long_api_table_fixture PROPERTIES FOLDER "tests/plugins")

add_library(plugin_loader_throw_register_fixture SHARED plugin_loader_fixture.cpp)
target_compile_definitions(plugin_loader_throw_register_fixture PRIVATE
    SLIC3R_PLUGIN_EXPORTS
    SLIC3R_TEST_PLUGIN_THROW_REGISTER)
target_include_directories(plugin_loader_throw_register_fixture PRIVATE ../..)
set_target_properties(plugin_loader_throw_register_fixture PROPERTIES FOLDER "tests/plugins")

# Exercise Python callbacks from a native thread whose temporary interpreter
# state ends before the returned diagnostic is inspected.
add_library(python_seam_placer_native_fixture SHARED python_seam_placer_native_fixture.cpp)
target_include_directories(python_seam_placer_native_fixture PRIVATE ../..)
set_target_properties(python_seam_placer_native_fixture PROPERTIES FOLDER "tests/plugins")

add_library(python_gcode_firmware_native_fixture SHARED python_gcode_firmware_native_fixture.cpp)
target_include_directories(python_gcode_firmware_native_fixture PRIVATE ../..)
set_target_properties(python_gcode_firmware_native_fixture PROPERTIES FOLDER "tests/plugins")

add_dependencies(${_TEST_NAME}_tests
    plugin_loader_missing_register_fixture
    plugin_loader_missing_abi_fixture
    plugin_loader_bad_api_major_fixture
    plugin_loader_bad_api_minor_fixture
    plugin_loader_short_api_table_fixture
    plugin_loader_long_api_table_fixture
    plugin_loader_throw_register_fixture
    python_seam_placer_native_fixture
    python_gcode_firmware_native_fixture)
target_compile_definitions(${_TEST_NAME}_tests PRIVATE
    SLIC3R_FILESYSTEM_TRANSACTION_TESTING
    SLIC3R_TEST_PLUGIN_MISSING_REGISTER_DLL=R"\($<TARGET_FILE:plugin_loader_missing_register_fixture>\)"
    SLIC3R_TEST_PLUGIN_MISSING_ABI_DLL=R"\($<TARGET_FILE:plugin_loader_missing_abi_fixture>\)"
    SLIC3R_TEST_PLUGIN_BAD_API_MAJOR_DLL=R"\($<TARGET_FILE:plugin_loader_bad_api_major_fixture>\)"
    SLIC3R_TEST_PLUGIN_BAD_API_MINOR_DLL=R"\($<TARGET_FILE:plugin_loader_bad_api_minor_fixture>\)"
    SLIC3R_TEST_PLUGIN_SHORT_API_TABLE_DLL=R"\($<TARGET_FILE:plugin_loader_short_api_table_fixture>\)"
    SLIC3R_TEST_PLUGIN_LONG_API_TABLE_DLL=R"\($<TARGET_FILE:plugin_loader_long_api_table_fixture>\)"
    SLIC3R_TEST_PLUGIN_THROW_REGISTER_DLL=R"\($<TARGET_FILE:plugin_loader_throw_register_fixture>\)")

# Malformed protocol fixtures must fail before their throwing registration runs.
foreach(_variant IN ITEMS abi3 legacy_abi unknown_api zero_api_table missing_vtable invalid_api_size changing_api_size)
    string(TOUPPER "${_variant}" _variant_upper)
    add_library(plugin_loader_${_variant}_fixture SHARED plugin_loader_fixture.cpp)
    target_compile_definitions(plugin_loader_${_variant}_fixture PRIVATE
        SLIC3R_PLUGIN_EXPORTS
        SLIC3R_TEST_PLUGIN_${_variant_upper}
        SLIC3R_TEST_PLUGIN_THROW_REGISTER)
    target_include_directories(plugin_loader_${_variant}_fixture PRIVATE ../..)
    set_target_properties(plugin_loader_${_variant}_fixture PROPERTIES FOLDER "tests/plugins")
    add_dependencies(${_TEST_NAME}_tests plugin_loader_${_variant}_fixture)
    target_compile_definitions(${_TEST_NAME}_tests PRIVATE
        SLIC3R_TEST_PLUGIN_${_variant_upper}_DLL=R"\($<TARGET_FILE:plugin_loader_${_variant}_fixture>\)")
endforeach()

target_link_libraries(${_TEST_NAME}_tests test_common test_common_data libslic3r)
# Scoped to the strict-freeze tests so toggling the option rebuilds only them.
if (SLIC3R_STRICT_FREEZE)
    set_source_files_properties(test_plugin_property.cpp test_python_plugin_clear.cpp
        PROPERTIES COMPILE_DEFINITIONS SLIC3R_TEST_STRICT_FREEZE=1)
endif()
set_property(TARGET ${_TEST_NAME}_tests PROPERTY FOLDER "tests")
# The C fixtures include the plugin API headers as a C plugin does: compile them as strict C11, without compiler extensions.
set_target_properties(${_TEST_NAME}_tests PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

if (WIN32)
    target_sources(${_TEST_NAME}_tests PRIVATE plugin_version_metadata.rc)
    prusaslicer_copy_dlls(${_TEST_NAME}_tests)
    set_target_properties(${_TEST_NAME}_tests PROPERTIES ENABLE_EXPORTS ON)
endif()

# Exercise the metadata generated for an actual external plugin target. The
# same test follows plugin.dll, plugin.so or plugin.dylib through the package
# cache without duplicating version constants.
if (TARGET polyholes_plugin)
    add_dependencies(${_TEST_NAME}_tests polyholes_plugin)
    get_target_property(_polyholes_package_version
        polyholes_plugin SLIC3R_PLUGIN_PACKAGE_VERSION)
    get_target_property(_polyholes_slicer_version
        polyholes_plugin SLIC3R_PLUGIN_SLICER_VERSION)
    target_compile_definitions(${_TEST_NAME}_tests PRIVATE
        SLIC3R_TEST_POLYHOLES_PLUGIN_DLL=R"\($<TARGET_FILE:polyholes_plugin>\)"
        SLIC3R_TEST_POLYHOLES_PACKAGE_VERSION=R"\(${_polyholes_package_version}\)"
        SLIC3R_TEST_POLYHOLES_SLICER_VERSION=R"\(${_polyholes_slicer_version}\)"
        SLIC3R_TEST_BUILD_SLICER_VERSION=R"\(${SLIC3R_VERSION_FULL}\)"
        SLIC3R_TEST_BUILD_RC_VERSION=R"\(${SLIC3R_RC_VERSION_DOTS}\)"
    )
endif()

if (TARGET flat_area_layer_height_plugin)
    get_target_property(_flat_area_package_version
        flat_area_layer_height_plugin SLIC3R_PLUGIN_PACKAGE_VERSION)
    get_target_property(_flat_area_slicer_version
        flat_area_layer_height_plugin SLIC3R_PLUGIN_SLICER_VERSION)
    target_compile_definitions(${_TEST_NAME}_tests PRIVATE
        SLIC3R_TEST_FLAT_AREA_PACKAGE_VERSION=R"\(${_flat_area_package_version}\)"
        SLIC3R_TEST_FLAT_AREA_SLICER_VERSION=R"\(${_flat_area_slicer_version}\)"
    )
endif()

if (TARGET python_plugin_loader)
    set(_python_plugin_package_targets
        python_example_noop_package
        python_hairy_object_package
        python_initial_typed_surface_builder_package
        python_polyholes_high_level_package
        python_polyholes_package
        python_simple_perimeter_generator_package
    )
    target_compile_definitions(${_TEST_NAME}_tests PRIVATE SLIC3R_TEST_PYTHON_PLUGINS)

    # The production loader discovers pure Python packages as siblings of its
    # own package. Recreate that installed layout around the test executable so
    # tests exercise the same path instead of relying on scripts embedded in
    # the bridge package.
    set(_python_test_runtime_directory "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
    set(_python_plugin_package_ids
        python.example.noop
        python.perimeter.post_process.hairy_object
        python.surface.initial_typed_surface_builder
        python.polyholes.high_level
        python.polyholes
        python.perimeter.generator.simple
    )
    set(_python_plugin_copy_commands)
    list(LENGTH _python_plugin_package_targets _python_plugin_package_count)
    math(EXPR _python_plugin_package_last "${_python_plugin_package_count} - 1")
    foreach(_python_plugin_package_idx RANGE ${_python_plugin_package_last})
        list(GET _python_plugin_package_targets ${_python_plugin_package_idx} _python_plugin_package_target)
        list(GET _python_plugin_package_ids ${_python_plugin_package_idx} _python_plugin_package_id)
        get_target_property(_python_plugin_package_directory
            ${_python_plugin_package_target} SLIC3R_PLUGIN_PACKAGE_DIRECTORY)
        list(APPEND _python_plugin_copy_commands
            COMMAND "${CMAKE_COMMAND}" -E remove_directory
                "${_python_test_runtime_directory}/plugins/${_python_plugin_package_id}"
            COMMAND "${CMAKE_COMMAND}" -E copy_directory
                "${_python_plugin_package_directory}"
                "${_python_test_runtime_directory}/plugins/${_python_plugin_package_id}"
        )
    endforeach()

    add_custom_target(${_TEST_NAME}_python_runtime
        COMMAND "${CMAKE_COMMAND}" -E remove_directory
            "${_python_test_runtime_directory}/plugins/python"
        COMMAND "${CMAKE_COMMAND}" -E make_directory
            "${_python_test_runtime_directory}/plugins/python"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "$<TARGET_FILE:python_plugin_loader>"
            "${_python_test_runtime_directory}/plugins/python/$<TARGET_FILE_NAME:python_plugin_loader>"
        COMMAND "${CMAKE_COMMAND}" -E remove_directory
            "${_python_test_runtime_directory}/plugins/python/python"
        COMMAND "${CMAKE_COMMAND}" -E copy_directory
            "$<TARGET_FILE_DIR:python_plugin_loader>/python"
            "${_python_test_runtime_directory}/plugins/python/python"
        COMMAND "${CMAKE_COMMAND}" -E remove_directory
            "${_python_test_runtime_directory}/plugins/python.external.package_root"
        COMMAND "${CMAKE_COMMAND}" -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/pure_python_package"
            "${_python_test_runtime_directory}/plugins/python.external.package_root"
        ${_python_plugin_copy_commands}
        COMMAND "${CMAKE_COMMAND}" -E remove_directory
            "${_python_test_runtime_directory}/plugins/python.test.priority.low"
        COMMAND "${CMAKE_COMMAND}" -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/python_priority_low_package"
            "${_python_test_runtime_directory}/plugins/python.test.priority.low"
        COMMAND "${CMAKE_COMMAND}" -E remove_directory
            "${_python_test_runtime_directory}/plugins/python.test.priority.high"
        COMMAND "${CMAKE_COMMAND}" -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/python_priority_high_package"
            "${_python_test_runtime_directory}/plugins/python.test.priority.high"
        COMMAND "${CMAKE_COMMAND}" -E remove_directory
            "${_python_test_runtime_directory}/plugins/python.test.clear.invalid_registration"
        COMMAND "${CMAKE_COMMAND}" -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/invalid_python_clear_package"
            "${_python_test_runtime_directory}/plugins/python.test.clear.invalid_registration"
        COMMENT "Copy Python plugin runtime to plugin tests"
        VERBATIM
    )
    add_dependencies(${_TEST_NAME}_python_runtime python_plugin_loader ${_python_plugin_package_targets})
    add_dependencies(${_TEST_NAME}_tests ${_TEST_NAME}_python_runtime)

    # The test executable exports the host ABI itself. Remove the application
    # DLL after the normal runtime-copy commands so the Python bridge resolves
    # registration functions from the test process.
    add_custom_command(TARGET ${_TEST_NAME}_tests POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E rm -f
            "$<TARGET_FILE_DIR:${_TEST_NAME}_tests>/Slic3r.dll"
        VERBATIM
    )
endif()

add_test(${_TEST_NAME}_tests ${_TEST_NAME}_tests "[plugins]" ${CATCH_EXTRA_ARGS})

# Hidden probes intentionally stop their process, so a separate driver checks
# the guard's status and diagnostic without aborting the ordinary plugin suite.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME plugins_seam_visibility_guard
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_seam_visibility_guard.py"
        "$<TARGET_FILE:${_TEST_NAME}_tests>")
set_tests_properties(plugins_seam_visibility_guard PROPERTIES TIMEOUT 180 LABELS "plugins;seam-visibility")

# The measured parent must not share its timing destination with isolated registries.
add_test(NAME plugins_exporter_child_timings
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_plugins_test_timings.py"
        "$<TARGET_FILE:${_TEST_NAME}_tests>" ExporterChildTimingTests)
set_tests_properties(plugins_exporter_child_timings PROPERTIES TIMEOUT 780 LABELS "plugins;exporter;timings")
