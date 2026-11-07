// Exact fingerprints of native/base role-query traces on deterministic scratch trees.
// The test alpha-renames IDs and matches each trace name and snapshot count before
// comparing its digest. These values impose no order on independent Clipper islands.

#pragma once

#include <cstddef>

namespace Slic3r::Test {

struct NativeExtrusionPublicationGolden {
    const char *name;
    std::size_t snapshots;
    const char *digest;
};

inline constexpr NativeExtrusionPublicationGolden native_extrusion_publication_goldens[]{
    {"role-empty", 1, "db2755a7ea25880aebd72a9d20051b8d2a82da4b"},
    {"role-empty-attributed", 1, "e8354573910e0d41bd62adba6ed04aaf646950a8"},
    {"role-homogeneous", 1, "b10e8fb2dea7024ca388c795540cda264a8aa9e7"},
    {"role-mixed", 1, "f4537b0e45fe93c23167fdc81f544ecb992eac93"},
    {"role-root-opposite", 1, "fb18e891bd0725b7788a112e145ba4dad8ad88a6"},
    {"role-root-concordant", 1, "460b858a062dbd75e42892cbef76690e072cd19c"},
    {"role-none-first", 1, "9e92c86fe057f0122aed1267d9038232debf5385"},
    {"role-none-last", 1, "d4cdb53ed262a13f64c0c1c0c1fb9305a88645b4"},
    {"role-attributed-nesting", 1, "bfb60cd64144419bd4be4cf947c6d13ebf25de71"}
};

} // namespace Slic3r::Test
