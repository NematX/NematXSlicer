#!/usr/bin/env python3
"""Activate an isolated external package while retaining host plugin defaults."""

import argparse
import re
from pathlib import Path


def ensure_entry(lines, section_name, key, value):
    header = re.compile(r"^\s*\[([^\]]+)\]\s*(?:[;#].*)?$")
    section_start = None
    section_end = len(lines)

    for index, line in enumerate(lines):
        match = header.match(line)
        if not match:
            continue
        if section_start is not None:
            section_end = index
            break
        if match.group(1) == section_name:
            section_start = index

    if section_start is None:
        if lines and lines[-1].strip():
            lines.append("")
        lines.append(f"[{section_name}]")
        section_start = len(lines) - 1
        section_end = len(lines)

    key_pattern = re.compile(r"^\s*([^=;#]+?)\s*=")
    for index in range(section_start + 1, section_end):
        match = key_pattern.match(lines[index])
        if match and match.group(1).strip() == key:
            lines[index] = f"{key} = {value}"
            return

    lines.insert(section_end, f"{key} = {value}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--defaults", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    # Repeat --plugin-id when one package registers several plugins.
    parser.add_argument("--plugin-id", required=True, action="append")
    parser.add_argument("--plugin-version", required=True)
    parser.add_argument("--package-id")
    parser.add_argument("--slicer-version", required=True)
    args = parser.parse_args()
    package_id = args.package_id or args.plugin_id[0]

    if not args.defaults.is_file():
        raise SystemExit(f"Host default activation file not found: {args.defaults}")

    content = args.defaults.read_text(encoding="utf-8-sig")
    newline = "\r\n" if "\r\n" in content else "\n"
    lines = content.splitlines()

    ensure_entry(lines, "installed", package_id, args.plugin_version)
    ensure_entry(
        lines,
        "installed",
        f"{package_id}.slicer_version",
        args.slicer_version,
    )
    for plugin_id in args.plugin_id:
        ensure_entry(lines, "activated", plugin_id, "1")
        ensure_entry(lines, "plugin_packages", plugin_id, package_id)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(newline.join(lines) + newline, encoding="utf-8")


if __name__ == "__main__":
    main()
