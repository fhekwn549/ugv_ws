#!/usr/bin/env python3
"""
Convert ROS 2 map (PGM + YAML) to Gazebo SDF wall model.

Reads map.pgm and map.yaml, extracts occupied pixels, merges adjacent
occupied cells into greedy rectangles, and generates an SDF model
(model.config + model.sdf) that can be included in a Gazebo world.

Usage:
    python3 map_to_gazebo_world.py [--map-yaml PATH] [--output-dir PATH]
                                   [--wall-height H] [--downsample N]
"""

import argparse
import os
import shutil
import struct
import sys
from pathlib import Path


def parse_pgm(pgm_path: str) -> tuple:
    """Parse a binary PGM (P5) file. Returns (width, height, max_val, pixels)."""
    with open(pgm_path, 'rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError(f'Expected P5 PGM, got {magic}')

        # Skip comments
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()

        width, height = map(int, line.split())
        max_val = int(f.readline().strip())
        data = f.read()

    if max_val <= 255:
        pixels = list(data)
    else:
        pixels = list(struct.unpack(f'>{width * height}H', data))

    return width, height, max_val, pixels


def parse_map_yaml(yaml_path: str) -> dict:
    """Parse map.yaml without PyYAML dependency."""
    config = {}
    with open(yaml_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if ':' in line:
                key, val = line.split(':', 1)
                key = key.strip()
                val = val.strip()
                # Parse origin as list
                if val.startswith('['):
                    val = [float(x.strip()) for x in val.strip('[]').split(',')]
                else:
                    try:
                        val = float(val)
                    except ValueError:
                        pass
                config[key] = val
    return config


def extract_occupied(pixels, width, height, max_val, occupied_thresh, negate,
                     downsample=1):
    """Return set of (row, col) for occupied cells on a downsampled grid.

    When downsample > 1, each NxN block is treated as occupied if ANY cell
    in the block is occupied. Returns coordinates in the downsampled grid.
    """
    occupied = set()
    threshold = int((1.0 - occupied_thresh) * max_val) if not negate else int(occupied_thresh * max_val)

    ds = downsample
    ds_height = (height + ds - 1) // ds
    ds_width = (width + ds - 1) // ds

    for dr in range(ds_height):
        for dc in range(ds_width):
            block_occupied = False
            for sr in range(dr * ds, min((dr + 1) * ds, height)):
                for sc in range(dc * ds, min((dc + 1) * ds, width)):
                    pixel = pixels[sr * width + sc]
                    if negate:
                        block_occupied = pixel > threshold
                    else:
                        block_occupied = pixel < threshold
                    if block_occupied:
                        break
                if block_occupied:
                    break
            if block_occupied:
                occupied.add((dr, dc))

    return occupied, ds_width, ds_height


def greedy_merge(occupied: set, height: int, width: int) -> list:
    """Merge occupied cells into minimal rectangles using greedy algorithm."""
    remaining = set(occupied)
    rectangles = []

    while remaining:
        # Pick a cell
        r, c = min(remaining)

        # Extend right
        max_col = c
        while (r, max_col + 1) in remaining and max_col + 1 < width:
            max_col += 1

        # Extend down
        max_row = r
        can_extend = True
        while can_extend and max_row + 1 < height:
            for cc in range(c, max_col + 1):
                if (max_row + 1, cc) not in remaining:
                    can_extend = False
                    break
            if can_extend:
                max_row += 1

        # Record rectangle and remove cells
        rectangles.append((r, c, max_row, max_col))
        for rr in range(r, max_row + 1):
            for cc in range(c, max_col + 1):
                remaining.discard((rr, cc))

    return rectangles


def generate_sdf(rectangles, resolution, origin, wall_height, wall_thickness,
                 map_width_m, map_height_m, map_image_name):
    """Generate SDF model string from merged rectangles.

    All wall boxes are placed as separate collision/visual elements inside
    a single <link> to keep entity count low and avoid Gazebo slowdowns.
    """
    ox, oy, _ = origin

    collisions = []
    visuals = []
    for i, (r1, c1, r2, c2) in enumerate(rectangles):
        # Rectangle dimensions in meters
        w = (c2 - c1 + 1) * resolution
        h = (r2 - r1 + 1) * resolution

        # Center position in map frame
        # PGM row 0 is top of image = highest Y in map frame
        cx = ox + (c1 + (c2 - c1 + 1) / 2.0) * resolution
        cy = oy + (map_height_m - (r1 + (r2 - r1 + 1) / 2.0) * resolution)
        cz = wall_height / 2.0

        collisions.append(f'''      <collision name="collision_{i}">
        <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>
        <geometry>
          <box><size>{w:.4f} {h:.4f} {wall_height}</size></box>
        </geometry>
      </collision>''')

        visuals.append(f'''      <visual name="visual_{i}">
        <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>
        <geometry>
          <box><size>{w:.4f} {h:.4f} {wall_height}</size></box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.6 0.6 0.6 1</diffuse>
        </material>
      </visual>''')

    # Floor plane with map texture
    floor_cx = ox + map_width_m / 2.0
    floor_cy = oy + map_height_m / 2.0

    floor_visual = f'''      <visual name="map_floor">
        <pose>{floor_cx:.4f} {floor_cy:.4f} 0.001 0 0 0</pose>
        <geometry>
          <plane><normal>0 0 1</normal><size>{map_width_m:.4f} {map_height_m:.4f}</size></plane>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <script>
            <uri>model://map_walls/materials</uri>
            <name>MapFloor/Image</name>
          </script>
        </material>
      </visual>'''

    all_elements = '\n'.join(collisions) + '\n' + '\n'.join(visuals) + '\n' + floor_visual

    sdf = f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="map_walls">
    <static>true</static>
    <link name="walls">
{all_elements}
    </link>
  </model>
</sdf>
'''
    return sdf


def generate_model_config():
    return '''<?xml version="1.0"?>
<model>
  <name>map_walls</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>
    Auto-generated walls from ROS 2 occupancy grid map.
  </description>
</model>
'''


def generate_ogre_material(map_image_name):
    """Generate Ogre material script for map floor texture."""
    return f'''material MapFloor/Image
{{
  technique
  {{
    pass
    {{
      texture_unit
      {{
        texture {map_image_name}
      }}
    }}
  }}
}}
'''


def main():
    # Locate ugv_nav/maps relative to this script
    script_dir = Path(__file__).resolve().parent
    pkg_dir = script_dir.parent  # ugv_roarm_description
    ugv_main_dir = pkg_dir.parent  # ugv_main
    default_map_yaml = ugv_main_dir / 'ugv_nav' / 'maps' / 'map.yaml'
    default_output = pkg_dir / 'worlds' / 'map_walls'

    parser = argparse.ArgumentParser(description='Convert ROS 2 map to Gazebo SDF model')
    parser.add_argument('--map-yaml', default=str(default_map_yaml),
                        help='Path to map.yaml')
    parser.add_argument('--output-dir', default=str(default_output),
                        help='Output directory for SDF model')
    parser.add_argument('--wall-height', type=float, default=0.5,
                        help='Wall height in meters (default: 0.5)')
    parser.add_argument('--downsample', type=int, default=3,
                        help='Downsample factor for collision grid (default: 3)')
    args = parser.parse_args()

    # Parse map yaml
    map_yaml = parse_map_yaml(args.map_yaml)
    map_dir = os.path.dirname(os.path.abspath(args.map_yaml))

    image_file = map_yaml.get('image', 'map.pgm')
    pgm_path = os.path.join(map_dir, image_file)
    resolution = float(map_yaml.get('resolution', 0.05))
    origin = map_yaml.get('origin', [-19, -5.9, 0])
    negate = int(map_yaml.get('negate', 0))
    occupied_thresh = float(map_yaml.get('occupied_thresh', 0.65))

    print(f'Map YAML: {args.map_yaml}')
    print(f'PGM file: {pgm_path}')
    print(f'Resolution: {resolution} m/px')
    print(f'Origin: {origin}')
    print(f'Occupied threshold: {occupied_thresh}')

    # Parse PGM
    width, height, max_val, pixels = parse_pgm(pgm_path)
    print(f'Image size: {width}x{height}, max_val: {max_val}')

    map_width_m = width * resolution
    map_height_m = height * resolution
    print(f'Map size: {map_width_m:.2f}m x {map_height_m:.2f}m')

    # Extract occupied cells with downsampling
    ds = args.downsample
    occupied, ds_width, ds_height = extract_occupied(
        pixels, width, height, max_val, occupied_thresh, negate, downsample=ds)
    ds_resolution = resolution * ds
    print(f'Downsample: {ds}x (effective resolution: {ds_resolution} m/px)')
    print(f'Downsampled grid: {ds_width}x{ds_height}')
    print(f'Occupied cells: {len(occupied)}')

    if not occupied:
        print('No occupied cells found. Check thresholds.')
        sys.exit(1)

    # Merge into rectangles
    rectangles = greedy_merge(occupied, ds_height, ds_width)
    print(f'Merged into {len(rectangles)} rectangles (from {len(occupied)} cells)')

    # Generate output
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # model.config
    with open(output_dir / 'model.config', 'w') as f:
        f.write(generate_model_config())

    # model.sdf — use downsampled resolution for wall geometry
    sdf = generate_sdf(rectangles, ds_resolution, origin, args.wall_height,
                        ds_resolution, map_width_m, map_height_m, image_file)
    with open(output_dir / 'model.sdf', 'w') as f:
        f.write(sdf)

    # Copy map image for floor texture
    materials_dir = output_dir / 'materials'
    materials_dir.mkdir(exist_ok=True)
    shutil.copy2(pgm_path, materials_dir / image_file)

    # Generate Ogre material script
    scripts_mat_dir = materials_dir / 'scripts'
    scripts_mat_dir.mkdir(exist_ok=True)
    with open(scripts_mat_dir / 'map_floor.material', 'w') as f:
        f.write(generate_ogre_material(image_file))

    # Copy texture to textures dir
    textures_dir = materials_dir / 'textures'
    textures_dir.mkdir(exist_ok=True)
    shutil.copy2(pgm_path, textures_dir / image_file)

    print(f'\nGenerated SDF model at: {output_dir}')
    print(f'  model.config')
    print(f'  model.sdf ({len(rectangles)} wall segments)')
    print(f'  materials/textures/{image_file}')
    print(f'\nDone!')


if __name__ == '__main__':
    main()
