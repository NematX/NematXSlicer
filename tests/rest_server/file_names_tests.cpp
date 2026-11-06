// Pure tests of session name validation and portable download file names.
#include <catch2/catch.hpp>
#include <stdexcept>
#include <string>
#include "file_names.hpp"

using namespace Slic3r::Rest;

TEST_CASE("Attachment names include an ASCII fallback and RFC5987 UTF-8", "[rest_file_names]")
{
    CHECK(content_disposition_attachment("Bracket.3mf") ==
        "attachment; filename=\"Bracket.3mf\"; filename*=UTF-8''Bracket.3mf");
    CHECK(content_disposition_attachment(u8"pi\u00e8ce.3mf") ==
        "attachment; filename=\"pi_ce.3mf\"; filename*=UTF-8''pi%C3%A8ce.3mf");
    CHECK(content_disposition_attachment(u8"part\U0001F600.3mf") ==
        "attachment; filename=\"part_.3mf\"; filename*=UTF-8''part%F0%9F%98%80.3mf");
    CHECK(content_disposition_attachment("a b'%.3mf") ==
        "attachment; filename=\"a b'_.3mf\"; filename*=UTF-8''a%20b%27%25.3mf");
    CHECK(content_disposition_attachment("part%20name.3mf") ==
        "attachment; filename=\"part_20name.3mf\"; filename*=UTF-8''part%2520name.3mf");
}

TEST_CASE("Session names are bounded UTF-8 without control characters", "[rest_file_names]")
{
    CHECK_NOTHROW(validate_session_name("Support bracket"));
    CHECK_NOTHROW(validate_session_name(u8"pi\u00e8ce \u2014 v2"));
    CHECK_NOTHROW(validate_session_name("a/b:c*?"));
    CHECK_NOTHROW(validate_session_name(std::string(255, 'a')));
    CHECK_THROWS_AS(validate_session_name(""), std::invalid_argument);
    CHECK_THROWS_AS(validate_session_name(std::string(256, 'a')), std::invalid_argument);
    CHECK_THROWS_AS(validate_session_name("tab\tname"), std::invalid_argument);
    CHECK_THROWS_AS(validate_session_name(std::string("nul\0x", 5)), std::invalid_argument);
    CHECK_THROWS_AS(validate_session_name("del\x7f"), std::invalid_argument);
    CHECK_THROWS_AS(validate_session_name("bad\xff"), std::invalid_argument);
    // Overlong form, UTF-16 surrogate and truncated sequence.
    CHECK_THROWS_AS(validate_session_name("\xc0\xaf"), std::invalid_argument);
    CHECK_THROWS_AS(validate_session_name("\xed\xa0\x80"), std::invalid_argument);
    CHECK_THROWS_AS(validate_session_name("\xe2\x82"), std::invalid_argument);
}

TEST_CASE("A loaded project file gives the session name without its extension", "[rest_file_names]")
{
    CHECK(session_name_from_file("support.3mf") == "support");
    CHECK(session_name_from_file("support.v2.3MF") == "support.v2");
    CHECK(session_name_from_file("noextension") == "noextension");
    CHECK(session_name_from_file(".3mf") == ".3mf");
    CHECK_FALSE(session_name_from_file("bad\xff.3mf").has_value());
    CHECK_FALSE(session_name_from_file(std::string(256, 'a') + ".3mf").has_value());
}

TEST_CASE("Portable file names are valid on Windows, Linux and macOS", "[rest_file_names]")
{
    CHECK(portable_file_name("project", ".3mf") == "project.3mf");
    CHECK(portable_file_name("part.3mf", ".3mf") == "part.3mf");
    CHECK(portable_file_name("Part.3MF", ".3mf") == "Part.3MF");
    CHECK(portable_file_name("part.stl", ".3mf") == "part.stl.3mf");
    CHECK(portable_file_name(u8"pi\u00e8ce", ".3mf") == u8"pi\u00e8ce.3mf");
    CHECK(portable_file_name("a<b>c:d\"e/f\\g|h?i*j", ".3mf") == "a_b_c_d_e_f_g_h_i_j.3mf");
    CHECK(portable_file_name("tab\there\x7f", ".3mf") == "tab_here_.3mf");
    CHECK(portable_file_name("bad\xff\xfe", ".3mf") == "bad__.3mf");
    CHECK(portable_file_name("../etc/passwd", ".3mf") == ".._etc_passwd.3mf");

    // Windows refuses trailing dots and spaces, and device names.
    CHECK(portable_file_name("name. .", ".3mf") == "name.3mf");
    CHECK(portable_file_name("name .3mf", ".3mf") == "name.3mf");
    CHECK(portable_file_name("nul.3mf", ".3mf") == "_nul.3mf");
    CHECK(portable_file_name("Con", ".3mf") == "_Con.3mf");
    CHECK(portable_file_name("com1.tar", ".3mf") == "_com1.tar.3mf");
    CHECK(portable_file_name("LPT9 ", ".3mf") == "_LPT9.3mf");
    CHECK(portable_file_name("COM0", ".3mf") == "COM0.3mf");
    CHECK(portable_file_name("console", ".3mf") == "console.3mf");

    // Nothing usable remains: the caller falls back to another name.
    CHECK_FALSE(portable_file_name("", ".3mf").has_value());
    CHECK_FALSE(portable_file_name(".", ".3mf").has_value());
    CHECK_FALSE(portable_file_name("..", ".3mf").has_value());
    CHECK_FALSE(portable_file_name(".3mf", ".3mf").has_value());
    CHECK_FALSE(portable_file_name(" . ", ".3mf").has_value());

    // The limit counts bytes and never splits a multi-byte character.
    const std::string long_ascii = portable_file_name(std::string(400, 'a'), ".3mf").value();
    CHECK(long_ascii.size() == 255);
    CHECK(long_ascii == std::string(251, 'a') + ".3mf");
    std::string accented;
    for (int index = 0; index < 200; ++index)
        accented += u8"\u00e9";
    const std::string long_utf8 = portable_file_name(accented, ".3mf").value();
    CHECK(long_utf8.size() == 254);
    CHECK_NOTHROW(validate_session_name(long_utf8));
}
