#!/usr/bin/env python3
"""Create a lightweight OBJ visual mesh that matches the lunar DEM collision."""

import argparse
from pathlib import Path

from PIL import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_image", type=Path)
    parser.add_argument("output_obj", type=Path)
    parser.add_argument("--samples", type=int, default=129)
    args = parser.parse_args()

    if args.samples < 2:
        raise ValueError("--samples must be at least 2")

    image = Image.open(args.input_image).convert("L")
    width, height = image.size
    pixels = image.load()
    output = args.output_obj
    output.parent.mkdir(parents=True, exist_ok=True)
    material = output.with_suffix(".mtl")

    # These values intentionally match lunar_heightmap/model.sdf.
    terrain_size = 513.0
    terrain_height = 20.0
    terrain_z_offset = -13.0
    samples = args.samples

    heights = []
    for row in range(samples):
        source_y = round(row * (height - 1) / (samples - 1))
        heights.append([
            pixels[round(column * (width - 1) / (samples - 1)), source_y]
            / 255.0 * terrain_height + terrain_z_offset
            for column in range(samples)
        ])

    spacing = terrain_size / (samples - 1)
    normals = []
    for row in range(samples):
        for column in range(samples):
            left = heights[row][max(column - 1, 0)]
            right = heights[row][min(column + 1, samples - 1)]
            down = heights[max(row - 1, 0)][column]
            up = heights[min(row + 1, samples - 1)][column]
            dzdx = (right - left) / (spacing * (2 if 0 < column < samples - 1 else 1))
            dzdy = (up - down) / (spacing * (2 if 0 < row < samples - 1 else 1))
            length = (dzdx * dzdx + dzdy * dzdy + 1.0) ** 0.5
            normals.append((-dzdx / length, -dzdy / length, 1.0 / length))

    with output.open("w", encoding="utf-8") as stream:
        stream.write("# Lightweight visual mesh generated from the lunar DEM\n")
        stream.write(f"mtllib {material.name}\n")
        for row in range(samples):
            y = terrain_size * (row / (samples - 1) - 0.5)
            for column in range(samples):
                x = terrain_size * (column / (samples - 1) - 0.5)
                z = heights[row][column]
                stream.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")

        for normal_x, normal_y, normal_z in normals:
            stream.write(f"vn {normal_x:.6f} {normal_y:.6f} {normal_z:.6f}\n")

        stream.write("usemtl lunar_regolith\n")
        for row in range(samples - 1):
            for column in range(samples - 1):
                first = row * samples + column + 1
                stream.write(
                    f"f {first}//{first} {first + 1}//{first + 1} "
                    f"{first + samples + 1}//{first + samples + 1}\n"
                )
                stream.write(
                    f"f {first}//{first} {first + samples + 1}//{first + samples + 1} "
                    f"{first + samples}//{first + samples}\n"
                )

    # This is intentionally simple: the SDF material remains the primary
    # material, while this standard OBJ material prevents an importer fallback
    # to bright white if it does not apply that SDF override.
    with material.open("w", encoding="utf-8") as stream:
        stream.write("newmtl lunar_regolith\n")
        stream.write("Ka 0.08 0.085 0.09\n")
        stream.write("Kd 0.20 0.21 0.23\n")
        stream.write("Ks 0 0 0\n")
        stream.write("Ns 1\n")
        stream.write("d 1\n")
        stream.write("illum 1\n")


if __name__ == "__main__":
    main()
