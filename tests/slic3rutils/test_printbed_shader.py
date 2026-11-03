"""Check printbed shader source consistency without executing or compiling GLSL."""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[2]


class PrintbedShaderSources(unittest.TestCase):
    # Inspect each shipped dialect for the shared alpha expressions and uniform type.
    def test_shader_sources(self):
        for dialect in ("110", "140", "ES"):
            with self.subTest(dialect=dialect):
                source = (ROOT / "resources" / "shaders" / dialect / "printbed.fs").read_text(encoding="utf-8")
                source = re.sub(r"//[^\n]*|/\*.*?\*/", "", source, flags=re.S)
                source = " ".join(source.split())
                self.assertEqual(re.findall(r"uniform\s+(\w+)\s+solid_color\s*;", source), ["bool"])
                self.assertEqual(source.count("solid_color"), 2)
                self.assertEqual(source.count("return vec4(mix(back_color, fore_color.rgb, fore_color.a), (transparent_background || !solid_color) ? fore_color.a : 1.0);"), 1)
                self.assertEqual(source.count("return vec4(color.rgb, transparent_background ? color.a * 0.25 : color.a);"), 1)
                self.assertEqual(source.count("color.a = transparent_background ? color.a * 0.5 : color.a;"), 1)

    # Inspect the single printbed activation for explicit false before grid and triangle draws.
    def test_host_uniform_order(self):
        source = (ROOT / "src" / "slic3r" / "GUI" / "3DBed.cpp").read_text(encoding="utf-8")
        source = source.split("void Bed3D::render_texture(", 1)[1].split("void Bed3D::render_model(", 1)[0]
        source = re.sub(r"//[^\n]*|/\*.*?\*/", "", source, flags=re.S)
        self.assertEqual(source.count('get_shader("printbed")'), 1)
        self.assertEqual(source.count('set_uniform("solid_color"'), 1)
        markers = (
            'get_shader("printbed")',
            "shader->start_using();",
            'shader->set_uniform("transparent_background", bottom);',
            'shader->set_uniform("solid_color", false);',
            "render_grid(bottom, true);",
            "m_triangles.render();",
        )
        positions = [source.index(marker) for marker in markers]
        self.assertEqual(positions, sorted(positions))


if __name__ == "__main__":
    unittest.main()
