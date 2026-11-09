// Exact fingerprints of role-query and clear/append/swap/copy/move traces on deterministic scratch trees.
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
    {"clear-empty", 6, "7a381175461d2533df45d2c8053e62ec1dda58a6"},
    {"clear-full", 6, "e9264fc855d425bc252af4ea3dd50a1d981571f2"},
    {"append-00", 6, "67a9ecbdbf858287c82f2c1d327be63b62494f8f"},
    {"append-01", 6, "59fe47a6789ddf39b4d66965d53f156c463a5b5e"},
    {"append-10", 6, "b5790634d79d685a097527672ae094d68274cdc4"},
    {"append-11", 6, "68ec817c55c4d0df32a8d83544d3a7c2ffab4fc7"},
    {"swap-00", 6, "bca8a1f12e1b9dbdd9823a84292f100baef99be7"},
    {"swap-01", 6, "96e0111c922531841fb3b0b790163ab0ba35dae7"},
    {"swap-10", 6, "25a348d49b2caa55193ff621afc6736ac04e265a"},
    {"swap-11", 6, "31fe3d67d9a53527035eb626d4e39502dae431d6"},
    {"clear-append-empty", 6, "e9264fc855d425bc252af4ea3dd50a1d981571f2"},
    {"clear-append-full", 6, "5a75fd185ff6ecad4346ad65bf11a128f576ee48"},
    {"control-copy", 5, "c0221e3e40032d1f11f94796cd8e98538d0c33b8"},
    {"control-move", 5, "e97b24d23e22208320af290b77a828e0c7d5c7c2"},
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
