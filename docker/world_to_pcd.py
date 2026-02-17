#!/usr/bin/env python3
"""
Convert Gazebo .world (SDF) files to PCD point cloud files.

Parses cylinder obstacles from the SDF XML and samples points on their
surfaces and caps. Optionally adds ground plane points.

Usage:
    python3 world_to_pcd.py <input.world> <output.pcd> [--resolution 0.05] [--ground]
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np


def parse_pose(pose_str):
    """Parse SDF pose string 'x y z roll pitch yaw' into (x, y, z)."""
    parts = pose_str.strip().split()
    return float(parts[0]), float(parts[1]), float(parts[2])


def parse_cylinders(world_path):
    """
    Parse a Gazebo .world file and extract cylinder obstacles.

    Returns a list of dicts: {x, y, z, radius, length}
    where (x, y, z) is the cylinder center from the model <pose>.
    """
    tree = ET.parse(world_path)
    root = tree.getroot()

    cylinders = []

    # Find all <model> elements
    for model in root.iter("model"):
        name = model.get("name", "")
        # Skip non-obstacle models (ground_plane, sun, etc.)
        # Obstacle models are typically named with numbers
        if not name.isdigit():
            continue

        # Get model pose
        pose_elem = model.find("pose")
        if pose_elem is None:
            continue
        cx, cy, cz = parse_pose(pose_elem.text)

        # Find cylinder geometry (check both collision and visual)
        cyl_elem = None
        for geom in model.iter("geometry"):
            cyl = geom.find("cylinder")
            if cyl is not None:
                cyl_elem = cyl
                break

        if cyl_elem is None:
            continue

        radius = float(cyl_elem.find("radius").text)
        length = float(cyl_elem.find("length").text)

        cylinders.append({
            "x": cx, "y": cy, "z": cz,
            "radius": radius, "length": length,
        })

    return cylinders


def parse_ground_plane(world_path):
    """
    Parse the ground plane from the .world file.

    Returns (center_x, center_y, size_x, size_y) or None.
    """
    tree = ET.parse(world_path)
    root = tree.getroot()

    for model in root.iter("model"):
        name = model.get("name", "")
        if "ground" not in name.lower():
            continue

        pose_elem = model.find("pose")
        if pose_elem is None:
            cx, cy = 0.0, 0.0
        else:
            cx, cy, _ = parse_pose(pose_elem.text)

        for geom in model.iter("geometry"):
            plane = geom.find("plane")
            if plane is not None:
                size_elem = plane.find("size")
                if size_elem is not None:
                    parts = size_elem.text.strip().split()
                    sx, sy = float(parts[0]), float(parts[1])
                    return cx, cy, sx, sy

    return None


def sample_cylinder(cx, cy, cz, radius, length, resolution):
    """
    Sample points on a cylinder surface and top/bottom caps.

    The cylinder is centered at (cx, cy, cz) with its axis along Z.
    """
    points = []
    half_len = length / 2.0

    # Number of angular samples
    circumference = 2.0 * math.pi * radius
    n_angle = max(8, int(circumference / resolution))

    # Number of height samples along the surface
    n_height = max(2, int(length / resolution))

    angles = np.linspace(0, 2 * math.pi, n_angle, endpoint=False)
    heights = np.linspace(-half_len, half_len, n_height)

    # Surface points
    for h in heights:
        for a in angles:
            px = cx + radius * math.cos(a)
            py = cy + radius * math.sin(a)
            pz = cz + h
            points.append((px, py, pz))

    # Top and bottom caps
    n_radial = max(1, int(radius / resolution))
    radii = np.linspace(0, radius, n_radial + 1)
    for r in radii:
        if r == 0:
            # Center point on each cap
            points.append((cx, cy, cz + half_len))
            points.append((cx, cy, cz - half_len))
        else:
            n_a = max(4, int(2 * math.pi * r / resolution))
            cap_angles = np.linspace(0, 2 * math.pi, n_a, endpoint=False)
            for a in cap_angles:
                px = cx + r * math.cos(a)
                py = cy + r * math.sin(a)
                points.append((px, py, cz + half_len))
                points.append((px, py, cz - half_len))

    return points


def sample_ground(cx, cy, sx, sy, resolution):
    """Sample points on a ground plane at z=0."""
    points = []
    x_min = cx - sx / 2.0
    x_max = cx + sx / 2.0
    y_min = cy - sy / 2.0
    y_max = cy + sy / 2.0

    xs = np.arange(x_min, x_max + resolution, resolution)
    ys = np.arange(y_min, y_max + resolution, resolution)

    for x in xs:
        for y in ys:
            points.append((x, y, 0.0))

    return points


def write_pcd(points, output_path):
    """Write points to a PCD v0.7 ASCII file."""
    n = len(points)
    with open(output_path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {n}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {n}\n")
        f.write("DATA ascii\n")
        for px, py, pz in points:
            f.write(f"{px:.6f} {py:.6f} {pz:.6f}\n")


def main():
    parser = argparse.ArgumentParser(
        description="Convert Gazebo .world (SDF) files to PCD point clouds"
    )
    parser.add_argument("input", help="Input .world file path")
    parser.add_argument("output", help="Output .pcd file path")
    parser.add_argument("--resolution", type=float, default=0.05,
                        help="Point spacing in meters (default: 0.05)")
    parser.add_argument("--ground", action="store_true",
                        help="Include ground plane points")

    args = parser.parse_args()

    print(f"Parsing {args.input}...")
    cylinders = parse_cylinders(args.input)
    print(f"  Found {len(cylinders)} cylinder obstacles")

    all_points = []

    for cyl in cylinders:
        pts = sample_cylinder(
            cyl["x"], cyl["y"], cyl["z"],
            cyl["radius"], cyl["length"],
            args.resolution
        )
        all_points.extend(pts)

    print(f"  Sampled {len(all_points)} obstacle points")

    if args.ground:
        ground = parse_ground_plane(args.input)
        if ground:
            cx, cy, sx, sy = ground
            # Use coarser resolution for ground to avoid excessive points
            ground_res = max(args.resolution, 0.5)
            ground_pts = sample_ground(cx, cy, sx, sy, ground_res)
            all_points.extend(ground_pts)
            print(f"  Added {len(ground_pts)} ground plane points "
                  f"(resolution={ground_res:.2f}m)")
        else:
            print("  Warning: No ground plane found in .world file")

    print(f"  Total points: {len(all_points)}")
    write_pcd(all_points, args.output)
    print(f"  Written to {args.output}")


if __name__ == "__main__":
    main()
