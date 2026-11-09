// Native fill-cache references cover complete stored states and copy/transfer relationships.
// Match each fixture name to the digest of its geometry, direct scopes, permissions and identities.

#ifndef slic3r_test_native_fill_cache_goldens_hpp_
#define slic3r_test_native_fill_cache_goldens_hpp_

namespace Slic3r::Test {

struct NativeFillCacheGolden {
    const char *name;
    const char *digest;
};

inline constexpr NativeFillCacheGolden native_fill_cache_goldens[] = {
    {"storage-0-0-0", "c9d474571014f20b28bfc073dfabdd340644bc20"},
    {"storage-0-1-0", "787b1c6032f8f26ba46196bb65c758fdce41294d"},
    {"storage-0-2-0", "c9d474571014f20b28bfc073dfabdd340644bc20"},
    {"storage-1-0-0", "0b3cec8f39d51c6c814f69b6a5d4fced115af194"},
    {"storage-1-1-0", "4c27998d11d02d59b5e3d6d6f6375c1331a4905f"},
    {"storage-1-2-0", "0b3cec8f39d51c6c814f69b6a5d4fced115af194"},
    {"storage-2-0-0", "4534d6699694a5a18e0d93f2dcecc843a12a5af7"},
    {"storage-2-1-0", "c8452de79ee7842c5d696cca5a5d70d55fdcb053"},
    {"storage-2-2-0", "4bd0eb634c13b269eefc24ec2067b2127f9fe6ff"},
    {"storage-3-0-0", "73279098645c0ba2266b218e88d152318295966a"},
    {"storage-3-1-0", "ebe1cc2cdb0ba954072eba2563ec40d7648091ae"},
    {"storage-3-2-0", "13854b677c1d877f9306592aa57cb6657b739af2"},
    {"storage-4-0-0", "e3345727081e78e6f3b79781d24b1f03bc05a465"},
    {"storage-4-1-0", "e08682952f1684663bc157109be12f7c76578378"},
    {"storage-4-2-0", "f103c35f0bb75fe77faac17fad8a593ac943f3ba"},
    {"storage-5-0-0", "960b61cd838655463ea174b35b085d684f707987"},
    {"storage-5-1-0", "699989ac4032f4f2df508c6b32b522c2850796f0"},
    {"storage-5-2-0", "960b61cd838655463ea174b35b085d684f707987"},
    {"storage-6-0-0", "78905d8f89a0ad2e5cc242f4f638f13ae5434dc2"},
    {"storage-6-1-0", "89bdad92b9ed9cca2abd15c860f31373b8bb8fe1"},
    {"storage-6-2-0", "219ac8221607df209a2171fb643bddc30f13bfef"},
    {"storage-7-0-0", "98aece39b53f70830a0682eb9a45b2fad279c2d1"},
    {"storage-7-1-0", "a66a58f485da61844251978b8832eef9ddb0a775"},
    {"storage-7-2-0", "288b6af030179237c2bff169b487d31d699b11b2"},
    {"storage-8-0-0", "5d2a87a0fac83228c8a4649748c92878b4a505fc"},
    {"storage-8-1-0", "34d279ff247f7b6e2d4c9e24e48971e6c82404ad"},
    {"storage-8-2-0", "5ac1e76ecb90c5ab4a9482e1c6834bc228498b05"},
    {"storage-9-0-0", "1b6e1e2dd6f1c64e176f16dc3946f143aadf8a27"},
    {"storage-9-1-0", "d4a091e9e3de130fec7c921a252cc255f1b60097"},
    {"storage-9-2-0", "0672bc82a2715cd634a363b37c3beb6dccdf96d2"},
    {"storage-0-0-1", "c9d474571014f20b28bfc073dfabdd340644bc20"},
    {"storage-0-1-1", "787b1c6032f8f26ba46196bb65c758fdce41294d"},
    {"storage-0-2-1", "c9d474571014f20b28bfc073dfabdd340644bc20"},
    {"storage-1-0-1", "0b3cec8f39d51c6c814f69b6a5d4fced115af194"},
    {"storage-1-1-1", "4c27998d11d02d59b5e3d6d6f6375c1331a4905f"},
    {"storage-1-2-1", "0b3cec8f39d51c6c814f69b6a5d4fced115af194"},
    {"storage-2-0-1", "4534d6699694a5a18e0d93f2dcecc843a12a5af7"},
    {"storage-2-1-1", "c8452de79ee7842c5d696cca5a5d70d55fdcb053"},
    {"storage-2-2-1", "89eb91118eda6a851c81514a9fd8ee573fb48acf"},
    {"storage-3-0-1", "73279098645c0ba2266b218e88d152318295966a"},
    {"storage-3-1-1", "ebe1cc2cdb0ba954072eba2563ec40d7648091ae"},
    {"storage-3-2-1", "1e2f364a75edacb50e998a14c3fd014474e63ef1"},
    {"storage-4-0-1", "e3345727081e78e6f3b79781d24b1f03bc05a465"},
    {"storage-4-1-1", "e08682952f1684663bc157109be12f7c76578378"},
    {"storage-4-2-1", "5a6ee7fed6c6b6bc1276ec0a0eb64e572510cea5"},
    {"storage-5-0-1", "960b61cd838655463ea174b35b085d684f707987"},
    {"storage-5-1-1", "699989ac4032f4f2df508c6b32b522c2850796f0"},
    {"storage-5-2-1", "960b61cd838655463ea174b35b085d684f707987"},
    {"storage-6-0-1", "78905d8f89a0ad2e5cc242f4f638f13ae5434dc2"},
    {"storage-6-1-1", "89bdad92b9ed9cca2abd15c860f31373b8bb8fe1"},
    {"storage-6-2-1", "e60d57ce5e46c5d0867d85793a46120d565bac30"},
    {"storage-7-0-1", "98aece39b53f70830a0682eb9a45b2fad279c2d1"},
    {"storage-7-1-1", "a66a58f485da61844251978b8832eef9ddb0a775"},
    {"storage-7-2-1", "eecb498a16e96ba9138802ecb34cf308b332ad15"},
    {"storage-8-0-1", "5d2a87a0fac83228c8a4649748c92878b4a505fc"},
    {"storage-8-1-1", "34d279ff247f7b6e2d4c9e24e48971e6c82404ad"},
    {"storage-8-2-1", "567a0b7afc10f3d484c02fc9acdf243c5d619a18"},
    {"storage-9-0-1", "1b6e1e2dd6f1c64e176f16dc3946f143aadf8a27"},
    {"storage-9-1-1", "d4a091e9e3de130fec7c921a252cc255f1b60097"},
    {"storage-9-2-1", "82b2bd81b9ee6076c168eb5e7dfaf9999c919fdc"},
    {"public-fill-0", "e31386a439eb4edb0f6fed9e203da06e41b0a05a"},
    {"public-fill-1", "07d5382a288cf067e2d78ae1c9907bf5fecd6e9d"},
    {"public-fill-2", "8e6eafb2a5f24d1849257c7da72e1e01d267dcc2"},
    {"public-fill-3", "39719b63ce20b9d009c4b4a083c344b49b742074"},
    {"public-fill-4", "0bdd332cc3a2bb6bdd938343db57d240ceaacb53"},
    {"public-fill-5", "e8fde503b3ea9724376bff6b0b9dbab43cc81c5b"},
    {"anchoring-0", "2e058ee106a8704adafe44b2c4c27a816ed020f5"},
    {"anchoring-18", "18343a782b0d09f68ae094f4fef0c1436d7e1d0d"},
    {"ironing-0", "da39a3ee5e6b4b0d3255bfef95601890afd80709"},
    {"ironing-1", "8eff5e906aeb25f2cc6a84545e993be9cdd2667a"},
    {"ironing-2", "2e3f8e0e734b0360bf6deb1f92257a3d1ba71567"},
    {"ironing-3", "242264c38cccb74b1a4b5e9be2555939f0f88d1e"},
};

} // namespace Slic3r::Test

#endif
