# Build the analytical motion solver directly into its standalone tests.
# This target needs only the standard library and registers no runtime plugin.
add_executable(pressure_kinematics_tests
    src/PressureKinematics.cpp
    tests/kinematics_tests.cpp
)
target_include_directories(pressure_kinematics_tests PRIVATE src)
add_test(NAME pressure_kinematics COMMAND pressure_kinematics_tests)

# Build the movement planner without a runtime plugin or SDK dependency.
add_executable(pressure_planner_tests
    src/PressureKinematics.cpp
    src/PressurePlanner.cpp
    tests/planner_tests.cpp
)
target_include_directories(pressure_planner_tests PRIVATE src)
add_test(NAME pressure_planner COMMAND pressure_planner_tests)

# Use only the inline SDK quantizer, without linking a host library or plugin.
add_executable(pressure_extrusion_tests
    src/PressureExtrusion.cpp
    tests/extrusion_tests.cpp
)
target_include_directories(pressure_extrusion_tests PRIVATE src "${SUPERSLICER_SOURCE_DIR}/src")
add_test(NAME pressure_extrusion COMMAND pressure_extrusion_tests)

# Exercise spatial projection and the real planner/extrusion bridge using inline SDK helpers only.
add_executable(pressure_geometry_tests
    src/PressureExtrusion.cpp
    src/PressureGeometry.cpp
    src/PressureKinematics.cpp
    src/PressurePlanner.cpp
    tests/geometry_tests.cpp
)
target_include_directories(pressure_geometry_tests PRIVATE src "${SUPERSLICER_SOURCE_DIR}/src")
add_test(NAME pressure_geometry COMMAND pressure_geometry_tests)

# Test the Reader client against three local C stubs, without a host library.
add_executable(pressure_reader_tests
    src/PressureReader.cpp
    tests/reader_tests.cpp
)
target_include_directories(pressure_reader_tests PRIVATE src "${SUPERSLICER_SOURCE_DIR}/src")
add_test(NAME pressure_reader COMMAND pressure_reader_tests)

# Exercise lexical doubles with the real SDK parser, without a host archive.
add_executable(pressure_tokens_tests
    src/PressureTokens.cpp
    tests/tokens_tests.cpp
    "${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Api/plugin/cpp/gcode/GCodeLineParser.cpp"
)
target_include_directories(pressure_tokens_tests PRIVATE src "${SUPERSLICER_SOURCE_DIR}/src")
add_test(NAME pressure_tokens COMMAND pressure_tokens_tests)

# Join original bytes and validated events with inline SDK state helpers only.
add_executable(pressure_source_tests
    src/PressureSource.cpp
    tests/source_tests.cpp
)
target_include_directories(pressure_source_tests PRIVATE src "${SUPERSLICER_SOURCE_DIR}/src")
add_test(NAME pressure_source COMMAND pressure_source_tests)

# Follow nominal doubles with the real SDK lexer and header-only NematX syntax.
add_executable(pressure_nominal_tests
    src/PressureNominal.cpp
    src/PressureTokens.cpp
    src/PressureSource.cpp
    tests/nominal_tests.cpp
    "${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Api/plugin/cpp/gcode/GCodeLineParser.cpp"
)
target_include_directories(pressure_nominal_tests PRIVATE src "${SUPERSLICER_SOURCE_DIR}/src")
add_test(NAME pressure_nominal COMMAND pressure_nominal_tests)

# Classify owned lines using nominal doubles and the real SDK lexical parser.
add_executable(pressure_classification_tests
    src/PressureClassification.cpp
    src/PressureNominal.cpp
    src/PressureSource.cpp
    src/PressureTokens.cpp
    tests/classification_tests.cpp
    "${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Api/plugin/cpp/gcode/GCodeLineParser.cpp"
)
target_include_directories(pressure_classification_tests PRIVATE src "${SUPERSLICER_SOURCE_DIR}/src")
add_test(NAME pressure_classification COMMAND pressure_classification_tests)

# Rewrite prepared groups using only the necessary SDK lexical/number sources.
add_executable(pressure_rewrite_tests
    src/PressureClassification.cpp
    src/PressureExtrusion.cpp
    src/PressureGeometry.cpp
    src/PressureKinematics.cpp
    src/PressureNominal.cpp
    src/PressurePlanner.cpp
    src/PressureRewrite.cpp
    src/PressureSource.cpp
    src/PressureTokens.cpp
    tests/rewrite_tests.cpp
    "${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Api/plugin/cpp/gcode/GCodeFormatter.cpp"
    "${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Api/plugin/cpp/gcode/GCodeLineParser.cpp"
)
target_include_directories(pressure_rewrite_tests PRIVATE src "${SUPERSLICER_SOURCE_DIR}/src" "${Boost_INCLUDE_DIR}")
add_test(NAME pressure_rewrite COMMAND pressure_rewrite_tests)
add_test(NAME pressure_rewrite_annotations COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/rewrite_annotation_test.py" "$<TARGET_FILE:pressure_rewrite_tests>")

# Decode model comments with the header-only decimal parser, without host linkage.
add_executable(pressure_controls_tests
    src/PressureControls.cpp
    tests/controls_tests.cpp
)
target_include_directories(pressure_controls_tests PRIVATE src "${SUPERSLICER_SOURCE_DIR}/src")
add_test(NAME pressure_controls COMMAND pressure_controls_tests)

# Assemble existing components with local owned Reader-event sequences only.
add_executable(pressure_flow_tests
    src/PressureClassification.cpp
    src/PressureControls.cpp
    src/PressureExtrusion.cpp
    src/PressureFlow.cpp
    src/PressureGeometry.cpp
    src/PressureKinematics.cpp
    src/PressureNominal.cpp
    src/PressurePlanner.cpp
    src/PressureRewrite.cpp
    src/PressureSource.cpp
    src/PressureTokens.cpp
    tests/flow_tests.cpp
    "${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Api/plugin/cpp/gcode/GCodeFormatter.cpp"
    "${SUPERSLICER_SOURCE_DIR}/src/libslic3r/Api/plugin/cpp/gcode/GCodeLineParser.cpp"
)
target_include_directories(pressure_flow_tests PRIVATE src "${SUPERSLICER_SOURCE_DIR}/src" "${Boost_INCLUDE_DIR}")
add_test(NAME pressure_flow COMMAND pressure_flow_tests)
set_tests_properties(pressure_flow PROPERTIES TIMEOUT 60)
