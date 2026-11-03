"""Offline checks; direct invocation: python test_host_identity.py --output-dir <binary-dir>/identity-checks/Debug --cmake <cmake-executable>."""
import argparse
import binascii
import hashlib
import pathlib
import plistlib
import struct
import subprocess
import unittest
import zlib

ROOT = pathlib.Path(__file__).resolve().parents[2]
OUT = None
CMAKE = None


def asset_digest(name, data):
    """Return SHA-256 of asset bytes, normalizing only SVG CRLF line endings to LF."""
    if name.endswith(".svg"):
        data = data.replace(b"\r\n", b"\n")
    return hashlib.sha256(data).hexdigest()


def write_mo(path, prefix):
    """Write a deterministic French catalogue with singular, plural and context messages."""
    messages = {
        "": "Content-Type: text/plain; charset=UTF-8\nLanguage: fr\nPlural-Forms: nplurals=2; plural=(n > 1);\n",
        "item": prefix + "one",
        "item\0items": prefix + "one\0" + prefix + "many",
        "menu\x04item": prefix + "context-one",
        "menu\x04item\0items": prefix + "context-one\0" + prefix + "context-many",
    }
    keys = sorted(messages)
    originals = [key.encode() for key in keys]
    translations = [messages[key].encode() for key in keys]
    offset = 28 + 16 * len(keys)
    tables = []
    strings = b""
    for values in (originals, translations):
        for value in values:
            tables.append(struct.pack("<II", len(value), offset + len(strings)))
            strings += value + b"\0"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(struct.pack("<7I", 0x950412DE, 0, len(keys), 28, 28 + 8 * len(keys), 0, 0) + b"".join(tables) + strings)


class IdentityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        OUT.mkdir(parents=True, exist_ok=True)
        script = 'include("' + (ROOT / 'version.inc').as_posix() + '")\n'
        script += 'set(SLIC3R_RESOURCES_DIR "' + (ROOT / 'resources').as_posix() + '")\n'
        cls.generated = {}
        for source in ("src/libslic3r/libslic3r_version.h.in", "src/platform/msw/Slic3r.rc.in",
                       "src/platform/msw/gcodeviewer.rc.in", "src/platform/unix/Slic3r.desktop.in",
                       "src/platform/unix/Gcodeviewer.desktop.in", "src/platform/osx/Info.plist.in"):
            destination = OUT / pathlib.Path(source).name.removesuffix(".in")
            cls.generated[source] = destination
            script += f'configure_file("{(ROOT / source).as_posix()}" "{destination.as_posix()}" @ONLY)\n'
        (OUT / "generate.cmake").write_text(script, encoding="utf-8")
        subprocess.run([CMAKE, "-P", str(OUT / "generate.cmake")], cwd=ROOT, check=True)
        for domain, prefix in (("NematXSlicer", "app-"), ("thirdparty", "third-")):
            write_mo(OUT / "catalogues" / "fr" / "LC_MESSAGES" / (domain + ".mo"), prefix)

    def test_generated_metadata(self):
        header = self.generated["src/libslic3r/libslic3r_version.h.in"].read_text()
        for key, value in (("SLIC3R_APP_KEY", "NematXSlicer"), ("SLIC3R_APP_CMD", "nematxslicer"),
                           ("SLIC3R_VERSION", "3.0.0-alpha+UNKNOWN"), ("SLIC3R_VERSION_FULL", "3.0.0.0-alpha+UNKNOWN"),
                           ("SLIC3R_APP_PROG_ID", "NematXSlicer.1"), ("SLIC3R_APP_W_PROG_ID", "NematXSlicer.1"),
                           ("GCODEVIEWER_APP_PROG_ID", "NematXSlicer.GCodeViewer.1"),
                           ("GCODEVIEWER_APP_W_PROG_ID", "NematXSlicer.GCodeViewer.1")):
            self.assertRegex(header, '#define ' + key + r' L?"' + value.replace("+", r"\+").replace(".", r"\.") + '"')
        self.assertNotIn("@", header)
        plist = plistlib.loads(self.generated["src/platform/osx/Info.plist.in"].read_bytes())
        self.assertEqual(plist["CFBundleIdentifier"], "com.nematx.NematXSlicer")
        for key in ("CFBundleVersion", "CFBundleShortVersionString"):
            self.assertEqual(plist[key], "3.0.0")
        self.assertEqual(plist["CFBundleURLTypes"][0]["CFBundleURLSchemes"], ["slic3r"])
        for source, filename in (("Slic3r", "nematxslicer"), ("gcodeviewer", "nematxslicer-gcodeviewer")):
            rc = self.generated[f"src/platform/msw/{source}.rc.in"].read_text()
            self.assertIn('"OriginalFilename", "' + filename + '.exe"', rc)
            self.assertIn("FILEVERSION 3,0,0,0", rc)
        desktop = self.generated["src/platform/unix/Gcodeviewer.desktop.in"].read_text()
        self.assertIn("Comment=G-code viewer of NematXSlicer", desktop)
        version = (ROOT / "version.inc").read_text()
        self.assertIn('set(SLIC3R_GUID "ee58a199-3c77-4865-bbe9-52b9cbb2bc32")', version)
        cmake = (ROOT / "CMakeLists.txt").read_text()
        for key, value in (("MAJOR", 3), ("MINOR", 0), ("PATCH", 0)):
            self.assertIn(f'set(CPACK_PACKAGE_VERSION_{key} "{value}")', cmake)

    def test_nematx_skips_automatic_downloader_registration(self):
        source = (ROOT / "src/slic3r/GUI/GUI_App.cpp").read_text(encoding="utf-8")
        startup = source.split("bool GUI_App::config_wizard_startup()", 1)[1]
        startup = startup.split("void GUI_App::prepare_plugin_updates_on_startup()", 1)[0]
        startup = " ".join(startup.split())
        self.assertIn(
            'if constexpr (std::string_view(SLIC3R_APP_KEY) != "NematXSlicer") { '
            'if (is_editor() && m_last_app_conf_lower_version && app_config->get_bool("downloader_url_registered")) { '
            'show_downloader_registration_dialog(); return true; } }',
            startup,
        )
        self.assertNotIn("app_config->set(", startup)

    def test_png_and_icns(self):
        icons = ROOT / "resources/icons"
        for name in ("NematXSlicer", "NematXSlicer-gcodeviewer"):
            formats = [("_32px", 32), ("_128px", 128), ("_192px", 192)]
            formats += [("", 192), ("_256_icns", 256)] if name == "NematXSlicer" else [("-mac_128px", 128)]
            for suffix, size in formats:
                data = (icons / (name + suffix + ".png")).read_bytes()
                self.assertEqual(data[:8], b"\x89PNG\r\n\x1a\n")
                self.assertEqual(struct.unpack_from(">II", data, 16), (size, size))
                offset = 8
                while offset < len(data):
                    length = struct.unpack_from(">I", data, offset)[0]
                    body = data[offset + 4:offset + 8 + length]
                    self.assertEqual(binascii.crc32(body) & 0xffffffff, struct.unpack_from(">I", data, offset + 8 + length)[0])
                    offset += 12 + length
                self.assertEqual(offset, len(data))
        data = (icons / "NematXSlicer.icns").read_bytes()
        self.assertEqual(data[:4], b"icns")
        self.assertEqual(struct.unpack_from(">I", data, 4)[0], len(data))
        offset = 8
        for tag, suffix in ((b"ic07", "_128px"), (b"ic08", "_256_icns")):
            self.assertEqual(data[offset:offset + 4], tag)
            length = struct.unpack_from(">I", data, offset + 4)[0]
            self.assertEqual(data[offset + 8:offset + length], (icons / f"NematXSlicer{suffix}.png").read_bytes())
            offset += length
        self.assertEqual(offset, len(data))
        self.assertEqual((icons / "NematXSlicer.ico").read_bytes(), (icons / "NematXSlicer-gcodeviewer.ico").read_bytes())
        self.assertEqual((icons / "NematXSlicer.png").read_bytes(), (icons / "NematXSlicer_192px.png").read_bytes())

    def test_asset_provenance_and_exact_dib_pixels(self):
        icons = ROOT / "resources/icons"
        # Source asset digests keep this check independent of locally imported Git references.
        digests = {
            "NematXSlicer.ico": "881161c9c8d53415ba03c17ba014220152b8a3d908be099c219341d285741992",
            "NematXSlicer-gcodeviewer.ico": "881161c9c8d53415ba03c17ba014220152b8a3d908be099c219341d285741992",
            "NematXSlicer.svg": "00155a96ac9bfe40b20f551d87562eca6f51312683b7d8687b8cbd048e713e58",
            "NematXSlicer-gcodeviewer.svg": "00155a96ac9bfe40b20f551d87562eca6f51312683b7d8687b8cbd048e713e58",
            "NematXSlicer_192px.png": "d48cc2723ee89c23fa0b08a204df4478cb74e2d9f37f926f7c49ed5d09e6228a",
            "NematXSlicer_192px_transparent.png": "df29772c4388b7df4f81527cbb8ba296a8ca07d996f72aef44f1a2c3a69339d3",
        }
        for name, expected in digests.items():
            data = (icons / name).read_bytes()
            self.assertEqual(asset_digest(name, data), expected)
            if name.endswith(".svg"):
                lf = data.replace(b"\r\n", b"\n")
                crlf = lf.replace(b"\n", b"\r\n")
                self.assertNotEqual(lf, crlf)
                self.assertEqual(asset_digest(name, lf), expected)
                self.assertEqual(asset_digest(name, crlf), expected)
                self.assertEqual(asset_digest(name + ".bin", crlf), hashlib.sha256(crlf).hexdigest())
                self.assertNotEqual(asset_digest(name + ".bin", crlf), expected)
                self.assertNotEqual(asset_digest(name, lf + b" "), expected)
        ico = (icons / "NematXSlicer.ico").read_bytes()
        sizes = set()
        for i in range(struct.unpack_from("<H", ico, 4)[0]):
            w, h, _, _, _, bits, length, offset = struct.unpack_from("<BBBBHHII", ico, 6 + 16 * i)
            w, h = w or 256, h or 256
            if w not in (32, 128, 256):
                continue
            sizes.add(w)
            self.assertEqual((w, bits), (h, 32))
            dib = ico[offset:offset + length]
            suffix = "_256_icns" if w == 256 else f"_{w}px"
            png = (icons / f"NematXSlicer{suffix}.png").read_bytes()
            chunks, pos = [], 8
            while pos < len(png):
                size = struct.unpack_from(">I", png, pos)[0]
                if png[pos + 4:pos + 8] == b"IDAT":
                    chunks.append(png[pos + 8:pos + 8 + size])
                pos += size + 12
            rgba = zlib.decompress(b"".join(chunks))
            for y in range(h):
                self.assertEqual(rgba[y * (1 + w * 4)], 0)
                for x in range(w):
                    sy = h - 1 - y
                    b, g, r, a = dib[40 + (sy * w + x) * 4:44 + (sy * w + x) * 4]
                    if dib[40 + w * h * 4 + sy * ((w + 31) // 32 * 4) + x // 8] & (128 >> (x % 8)):
                        a = 0
                    start = y * (1 + w * 4) + 1 + x * 4
                    self.assertEqual(rgba[start:start + 4], bytes((r, g, b, a)))
        self.assertEqual(sizes, {32, 128, 256})

    def test_palette_splash_and_preserved_resources(self):
        colors = dict(line.split(" = ", 1) for line in (ROOT / "resources/ui_layout/default/colors.ini").read_text().splitlines() if " = " in line)
        for key, value in (("Gui_color_dark", "279fad"), ("Gui_color", "29bbcc"), ("Gui_color_light", "3ddbed")):
            self.assertEqual(colors[key], value)
        for key in ("splash_screen_editor", "splash_screen_gcodeviewer"):
            self.assertNotEqual(colors[key], "random")
            self.assertTrue((ROOT / "resources/splashscreen" / colors[key]).is_file())
        photos = {
            "200902_Overview6713_1200.jpg": "6134b3517eb0bb1dbc860d7608b10492e1f591d572108a25bedb08cfbb3fe7a3",
            "200902_Overview6720_1200.jpg": "a3bd4feb72ab7a38425f4d841a979c000d8a3b79f23037892bd5885a1c851b3b",
            "230419_NEX01_1200.jpg": "d021f74f175e657eb75181ee4b0c43f3d22e69f3e98bee2e98ef0af53390b2ab",
        }
        for photo, expected in photos.items():
            data = (ROOT / "resources/splashscreen" / photo).read_bytes()
            self.assertEqual(data[:2], b"\xff\xd8")
            self.assertEqual(hashlib.sha256(data).hexdigest(), expected)
        for name in ("Slic3r.ico", "Slic3r.svg", "Slic3r.png", "Slic3r.icns", "Slic3r_32px.png",
                     "Slic3r_128px.png", "Slic3r_192px.png", "Slic3r_192px_transparent.png", "Slic3r_256_icns.png",
                     "Slic3r-mac_128px.png", "Slic3r-gcodeviewer.ico", "Slic3r-gcodeviewer.svg",
                     "Slic3r-gcodeviewer_32px.png", "Slic3r-gcodeviewer_128px.png", "Slic3r-gcodeviewer_192px.png",
                     "Slic3r-gcodeviewer-mac_128px.png"):
            self.assertTrue((ROOT / "resources/icons" / name).is_file(), name)
        for name in ("vector_finesse.jpg", "prusa-gcodepreview.jpg"):
            self.assertTrue((ROOT / "resources/splashscreen" / name).is_file(), name)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=pathlib.Path, required=True)
    parser.add_argument("--cmake", required=True)
    args, unittest_args = parser.parse_known_args()
    OUT = args.output_dir.resolve()
    CMAKE = args.cmake
    unittest.main(argv=[__file__] + unittest_args)
