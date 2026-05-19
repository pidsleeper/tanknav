#!/usr/bin/env python3
"""Convert PCD point cloud map to Nav2 2D occupancy grid map."""

import struct
import sys
from pathlib import Path

import numpy as np

GRID_RESOLUTION = 0.05  # meters per pixel
HEIGHT_MIN = -0.15      # below robot base (ground)
HEIGHT_MAX = 0.5        # above robot base (obstacles)
OCCUPIED_THRESH = 3     # min points per cell to mark occupied


def read_pcd(filepath: str) -> np.ndarray:
    """Read XYZ points from a PCD file (binary or ASCII)."""
    with open(filepath, "rb") as f:
        header = b""
        while True:
            line = f.readline()
            header += line
            if line.startswith(b"DATA"):
                data_fmt = line.strip().split()[-1].decode()
                break

        field_line = [
            l for l in header.split(b"\n") if l.startswith(b"FIELDS")
        ][0]
        fields = field_line.decode().strip().split()[1:]

        count_line = [
            l for l in header.split(b"\n") if l.startswith(b"COUNT")
        ]
        counts = [1] * len(fields)
        if count_line:
            counts = [int(x) for x in count_line[0].decode().strip().split()[1:]]

        size_line = [
            l for l in header.split(b"\n") if l.startswith(b"SIZE")
        ][0]
        sizes = [int(x) for x in size_line.decode().strip().split()[1:]]

        has_xyz = [f in ("x", "y", "z") for f in fields]

        if data_fmt == "binary":
            raw = f.read()
            offset = 0
            pts = []
            for _ in range(len(raw) // sum(sizes)):
                pt = [0.0, 0.0, 0.0]
                idx = 0
                for fi, (sz, cnt, need) in enumerate(
                    zip(sizes, counts, has_xyz)
                ):
                    for _ in range(cnt):
                        val = struct.unpack_from("f" if sz == 4 else "d", raw, offset)[0]
                        if need:
                            pt[idx] = val
                            idx += 1
                        offset += sz
                pts.append(pt)
            return np.array(pts)
        else:
            pts = []
            for line in f:
                parts = line.decode().strip().split()
                pt = [float(parts[i]) for i, fld in enumerate(fields) if fld in ("x", "y", "z")]
                if len(pt) == 3:
                    pts.append(pt)
            return np.array(pts)


def pcd_to_pgm(pcd_path: str, pgm_path: str, yaml_path: str):
    """Convert PCD to PGM occupancy grid + YAML."""
    print(f"Reading {pcd_path}...")
    points = read_pcd(pcd_path)
    print(f"  {len(points)} points loaded")

    # Filter by height
    mask = (points[:, 2] >= HEIGHT_MIN) & (points[:, 2] <= HEIGHT_MAX)
    filtered = points[mask]
    print(f"  {len(filtered)} points after Z filter [{HEIGHT_MIN}, {HEIGHT_MAX}]")

    if len(filtered) == 0:
        print("ERROR: No points after Z filtering!")
        sys.exit(1)

    # Compute map bounds
    x_min, x_max = filtered[:, 0].min(), filtered[:, 0].max()
    y_min, y_max = filtered[:, 1].min(), filtered[:, 1].max()
    print(f"  X range: [{x_min:.2f}, {x_max:.2f}]")
    print(f"  Y range: [{y_min:.2f}, {y_max:.2f}]")

    # Grid dimensions
    width = int((x_max - x_min) / GRID_RESOLUTION) + 3  # +2 border
    height = int((y_max - y_min) / GRID_RESOLUTION) + 3
    origin_x = x_min - GRID_RESOLUTION
    origin_y = y_min - GRID_RESOLUTION

    print(f"  Grid: {width} x {height} @ {GRID_RESOLUTION}m/px")

    # Rasterize: count points per cell
    ix = ((filtered[:, 0] - origin_x) / GRID_RESOLUTION).astype(int)
    iy = ((filtered[:, 1] - origin_y) / GRID_RESOLUTION).astype(int)
    valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    ix, iy = ix[valid], iy[valid]

    counts = np.zeros((height, width), dtype=np.int32)
    np.add.at(counts, (iy, ix), 1)

    # Occupancy grid (invert Y for PGM coordinate system)
    # PGM: 0=occupied(black), 255=free(white)
    # Nav2: 0=free, 100=occupied, -1=unknown
    grid = np.full((height, width), 205, dtype=np.uint8)  # default: unknown → light gray
    grid[counts == 0] = 254                                # free → near white
    grid[counts >= OCCUPIED_THRESH] = 0                     # occupied → black

    # Flip Y: PGM origin is top-left, map origin is bottom-left
    grid = np.flipud(grid)

    # Write PGM (binary)
    with open(pgm_path, "wb") as f:
        f.write(f"P5\n{width} {height}\n255\n".encode())
        f.write(grid.tobytes())
    print(f"  Wrote {pgm_path}")

    # Write YAML
    yaml_content = f"""image: {Path(pgm_path).name}
resolution: {GRID_RESOLUTION}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    with open(yaml_path, "w") as f:
        f.write(yaml_content)
    print(f"  Wrote {yaml_path}")

    print("Done!")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: pcd_to_navmap.py <input.pcd> [output_prefix]")
        sys.exit(1)

    pcd_path = sys.argv[1]
    prefix = sys.argv[2] if len(sys.argv) > 2 else Path(pcd_path).stem
    out_dir = Path(pcd_path).parent

    pcd_to_pgm(
        str(pcd_path),
        str(out_dir / f"{prefix}.pgm"),
        str(out_dir / f"{prefix}.yaml"),
    )
