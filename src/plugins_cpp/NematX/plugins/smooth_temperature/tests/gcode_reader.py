"""Read the heater state during Marlin extrusions for the package tests.

parse_gcode tracks absolute/relative E, resets, tools and object/copy labels.
Result.series requires one unambiguous instance and checks that all expected
printed layers exist. Synthetic extrusions are retained separately, so an empty
or partly unread file cannot stand in for a successful export.

    parse_gcode -> Result.series / wait_sites / commands
"""
from __future__ import annotations

import re
from dataclasses import dataclass

WORD = re.compile(r"([A-Z])([+-]?(?:\d+(?:\.\d*)?|\.\d+))")
OBJECT = re.compile(r"; printing object (.*?) id:(\d+) copy (\d+)$")


@dataclass
class Result:
    # (global layer, tool, name, object id, copy id) -> setpoints in extrusion order.
    setpoints: dict[tuple[int, int, str, int, int], list[int]]
    # (global layer, tool, temperature) for every wait, including startup.
    waits: list[tuple[int, int, int]]
    gcode: list[str]
    # (global layer, extrusion type, temperature) outside object labels.
    synthetic: list[tuple[int, str, int]]
    synthetic_heaters: list[str]

    def series(self, name: str, expected_layers: int, tool: int = 0, copy: int = 0,
               positions: list[int] | None = None) -> list[list[int]]:
        """Read one instance and fail on missing layers or ambiguous object names."""
        keys = sorted(key for key in self.setpoints if key[1:3] == (tool, name) and key[4] == copy)
        if len(keys) != expected_layers or len({key[3] for key in keys}) != 1:
            raise AssertionError(f"{name}/copy {copy}/T{tool}: expected {expected_layers} layers of one object, got {keys}")
        actual = [key[0] for key in keys]
        wanted = positions if positions is not None else list(range(actual[0], actual[0] + expected_layers))
        if actual != wanted:
            raise AssertionError(f"{name}/copy {copy}/T{tool}: wrong printed layer positions: {actual}, expected {wanted}")
        return [self.setpoints[key] for key in keys]

    def wait_sites(self) -> list[tuple[int, int]]:
        """Locations of waits, without their changed setpoints."""
        return [(layer, tool) for layer, tool, _ in self.waits]

    def commands(self) -> list[str]:
        """All executable lines, excluding comments and blank lines."""
        return [line.split(";", 1)[0].rstrip() for line in self.gcode if line.split(";", 1)[0].strip()]


def parse_gcode(lines: list[str]) -> Result:
    """Track setpoints on positive XY extrusions, rejecting an unknown heater state."""
    setpoint: dict[int, int] = {}
    tool, layer, e = 0, -1, 0.0
    absolute_e = True
    instance = None
    kind = ""
    out: dict[tuple[int, int, str, int, int], list[int]] = {}
    waits, synthetic, synthetic_heaters = [], [], []
    for raw in lines:
        if raw.startswith(";LAYER_CHANGE"):
            layer += 1
            kind = ""
        elif match := OBJECT.fullmatch(raw):
            instance = (match[1], int(match[2]), int(match[3]))
        elif raw.startswith("; printing object "):
            raise AssertionError(f"Unreadable object label: {raw}")
        elif raw.startswith("; stop printing object"):
            instance = None
        elif raw.startswith(";TYPE:"):
            kind = raw[len(";TYPE:"):].strip()
        code = raw.split(";", 1)[0].strip().upper()
        if not code:
            continue
        command = code.split(" ", 1)[0]
        fields = {key: float(value) for key, value in WORD.findall(code[len(command):])}
        if command in ("M104", "M109") and ("S" in fields or "R" in fields):
            heater = int(fields.get("T", tool))
            temperature = int(fields.get("S", fields.get("R")))
            setpoint[heater] = temperature
            if command == "M109":
                waits.append((layer, heater, temperature))
            if instance is None and kind.lower() in ("skirt", "brim", "skirt/brim"):
                synthetic_heaters.append(code)
        elif re.fullmatch(r"T\d+", command):
            tool = int(command[1:])
        elif command in ("M82", "M83"):
            absolute_e = command == "M82"
        elif command == "G92" and "E" in fields:
            e = fields["E"]
        elif command in ("G1", "G2", "G3") and "E" in fields:
            delta = fields["E"] - e if absolute_e else fields["E"]
            e = fields["E"] if absolute_e else e + fields["E"]
            if delta <= 0 or not ("X" in fields or "Y" in fields) or layer < 0:
                continue
            if tool not in setpoint:
                raise AssertionError(f"Extrusion without a known setpoint: layer {layer}, T{tool}")
            if instance is None:
                synthetic.append((layer, kind, setpoint[tool]))
            else:
                name, object_id, copy_id = instance
                values = out.setdefault((layer, tool, name, object_id, copy_id), [])
                if not values or values[-1] != setpoint[tool]:
                    values.append(setpoint[tool])
    return Result(out, waits, lines, synthetic, synthetic_heaters)
